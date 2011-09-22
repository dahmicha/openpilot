/**
 ******************************************************************************
 * @addtogroup OpenPilotModules OpenPilot Modules
 * @{
 * @addtogroup GSPModule GPS Module
 * @brief Process GPS information
 * @{
 *
 * @file       GTOP_BIN.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @brief      GPS module, handles GPS and NMEA stream
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include "openpilot.h"
#include "pios.h"
#include "GTOP_BIN_CUSTOM.h"
#include "gpsposition.h"
#include "gpstime.h"
#include "gpssatellites.h"

#include <string.h>	// memmove
#include <stdbool.h> /* Boolean variables */

#ifdef ENABLE_GPS_BINARY_CUSTOM_GTOP

// ************
// the structure of the binary packet

typedef struct
{
	uint8_t payload; ///< Number of payload bytes
	int32_t latitude;  ///< Latitude in degrees * 10^7
	int32_t longitude; ///< Longitude in degrees * 10^7
	int32_t msl_altitude;  ///< MSL altitude in meters * 10^2
	uint32_t ground_speed; ///< FIXME SPEC UNCLEAR
	int32_t heading;
	uint8_t satellites;
	uint8_t fix_type;
	uint32_t date;
	uint32_t utc_time;
	uint16_t hdop;
	uint8_t ck_a;
	uint8_t ck_b;
}  __attribute__((__packed__)) type_gps_bin_custom_packet;

typedef type_gps_bin_custom_packet gps_bin_custom_packet_t;

enum MTK_DECODE_STATES
{
	MTK_DECODE_UNINIT = 0,
	MTK_DECODE_GOT_CK_A = 1,
	MTK_DECODE_GOT_CK_B = 2
};

typedef struct
{
	union {
		uint16_t ck;
		struct {
		    uint8_t ck_a;
		    uint8_t ck_b;
		};
	};
    uint8_t decode_state;
//    bool new_data;
//    uint8_t fix;
    bool print_errors;
    int16_t rx_count;
}  __attribute__((__packed__)) type_gps_bin_custom_state;

typedef type_gps_bin_custom_state gps_bin_custom_state_t;

// ************

// buffer that holds the incoming binary packet
static uint8_t gps_rx_buffer[sizeof(gps_bin_custom_packet_t)] __attribute__ ((aligned(4)));

// number of bytes currently in the rx buffer
static gps_bin_custom_state_t mtk_state;

// ************
// endian swapping functions

//static uint16_t swap2Bytes(uint16_t data)
//{
//	return (((data >> 8) & 0x00ff) |
//			((data << 8) & 0xff00));
//}
//
//static uint32_t swap4Bytes(uint32_t data)
//{
//	return (((data >> 24) & 0x000000ff) |
//			((data >>  8) & 0x0000ff00) |
//			((data <<  8) & 0x00ff0000) |
////			((data << 24) & 0xff000000));
////}
//
///****************************************************************
// *
// ****************************************************************/
// // Join 4 bytes into a long
//long join_4_bytes(unsigned char Buffer[])
//{
//  union long_union {
//        int32_t dword;
//        uint8_t  byte[4];
//} longUnion;
//
//  longUnion.byte[3] = *Buffer;
//  longUnion.byte[2] = *(Buffer+1);
//  longUnion.byte[1] = *(Buffer+2);
//  longUnion.byte[0] = *(Buffer+3);
//  return(longUnion.dword);
//}

void mtk_decode_init(gps_bin_custom_state_t* mtk_state)
{
	mtk_state->ck_a = 0;
	mtk_state->ck_b = 0;
	mtk_state->rx_count = 0;
	mtk_state->decode_state = MTK_DECODE_UNINIT;
}

void mtk_checksum(uint8_t b, uint8_t* ck_a, uint8_t* ck_b)
{
  *(ck_a) = *(ck_a) + b;
  *(ck_b) = *(ck_b) + *(ck_a);
}


// ************
/**
 * Parses a complete binary packet and update the GPSPosition and GPSTime UAVObjects
 *
 * param[in] .. b = a new received byte from the GPS
 *
 * return '0' if we have found a valid binary packet
 * return <0 if any errors were encountered with the packet or no packet found
 */

int GTOP_BIN_CUSTOM_update_position(uint8_t b, volatile uint32_t *chksum_errors, volatile uint32_t *parsing_errors)
{
	if (mtk_state.decode_state == MTK_DECODE_UNINIT)
	{
		if (b == 0xd0) mtk_state.decode_state = MTK_DECODE_GOT_CK_A;
	}
	else if (mtk_state.decode_state == MTK_DECODE_GOT_CK_A)
	{
		if (b == 0xdd)
		{
			mtk_state.decode_state = MTK_DECODE_GOT_CK_B;
		}
		else
		{
			// Second start symbol was wrong, reset state machine
			mtk_decode_init(&mtk_state);
		}
	}
	else if (mtk_state.decode_state == MTK_DECODE_GOT_CK_B)
	{
		// Add to checksum
		if (mtk_state.rx_count < 33) mtk_checksum(b, &(mtk_state.ck_a), &(mtk_state.ck_b));
		// Fill packet buffer
		gps_rx_buffer[mtk_state.rx_count] = b;
		mtk_state.rx_count++;
		uint8_t ret = 0;

		// Packet size minus checksum
		if (mtk_state.rx_count >= 35)
		{
			gps_bin_custom_packet_t* packet = (gps_bin_custom_packet_t*) gps_rx_buffer;
			// Check if checksum is valid
			if (mtk_state.ck_a == packet->ck_a && mtk_state.ck_b == packet->ck_b)
			{
				GPSPositionData	GpsData;
				GPSPositionGet(&GpsData);
				switch (packet->fix_type)
				{
					case 1: GpsData.Status = GPSPOSITION_STATUS_NOFIX; break;
					case 2: GpsData.Status = GPSPOSITION_STATUS_FIX2D; break;
					case 3: GpsData.Status = GPSPOSITION_STATUS_FIX3D; break;
					default: GpsData.Status = GPSPOSITION_STATUS_NOGPS; break;
				}
				GpsData.Latitude        = packet->latitude;   // degrees * 10e6
				GpsData.Longitude       = packet->longitude;	// degrees * 10e6
				GpsData.Altitude        = packet->msl_altitude;                                       // meters
				GpsData.GeoidSeparation = 5000;                                 // meters
				GpsData.Heading         = packet->heading / 1000;                                 // degrees
				GpsData.Groundspeed     = packet->ground_speed / 3600;                                  // m/s
				GpsData.Satellites      = packet->satellites;                                                  //
				GpsData.PDOP            = packet->hdop / 100.0f;                                                                            // not available in binary mode
				GpsData.HDOP            = packet->hdop / 100.0f;                                                //
				GpsData.VDOP            = 99.99;                                                                            // not available in binary mode
				//				mtk_state.new_data = true;
				GPSPositionSet(&GpsData);

				// set the gps time object
				GPSTimeData GpsTime;
				GPSTimeGet(&GpsTime);
				//			uint32_t utc_time = rx_packet->data.utc_time / 1000;
				//			GpsTime.Second = utc_time % 100;          // seconds
				//			GpsTime.Minute = (utc_time / 100) % 100;  // minutes
				//			GpsTime.Hour = utc_time / 10000;          // hours
				//			GpsTime.Day = rx_packet->data.day;        // day
				//			GpsTime.Month = rx_packet->data.month;    // month
				//			GpsTime.Year = rx_packet->data.year;      // year
				GPSTimeSet(&GpsTime);


				// set the number of satellites
				GPSSatellitesData SattelliteData;
				GPSSatellitesGet(&SattelliteData);
				memset(&SattelliteData, 0, sizeof(SattelliteData));
				SattelliteData.SatsInView = packet->satellites;                                                    //
				GPSSatellitesSet(&SattelliteData);

				ret = 0;
			}
			else
			{
				ret = -1;
			}
			// Reset state machine to decode next packet
			mtk_decode_init(&mtk_state);
			return ret;
		}
	}
	return -1;     // no valid packet found
}

// ************

void GTOP_BIN_CUSTOM_init(void)
{
	mtk_decode_init(&mtk_state);
//    mtk_state.new_data = false;
//    mtk_state.fix = 0;
    mtk_state.print_errors = false;
}

// ************

#endif // ENABLE_GPS_BINARY_GTOP

