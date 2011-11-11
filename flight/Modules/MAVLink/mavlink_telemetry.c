/**
 ******************************************************************************
 * @addtogroup OpenPilotModules OpenPilot Modules
 * @{ 
 * @addtogroup TelemetryModule Telemetry Module
 * @brief Main telemetry module
 * Starts three tasks (RX, TX, and priority TX) that watch event queues
 * and handle all the telemetry of the UAVobjects
 * @{ 
 *
 * @file       telemetry.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @brief      Telemetry module, handles telemetry and UAVObject updates
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
#include <stdbool.h>
#include "mavlink_telemetry.h"
#include "flighttelemetrystats.h"
#include "gcstelemetrystats.h"
#include "telemetrysettings.h"
#include "CoordinateConversions.h"

/* Include all UAVObjects that need to be translated to MAVLink */
#include "manualcontrolcommand.h"  /* Remote control / manual commands */
#include "baroaltitude.h"          /* Pressure sensor */
#include "attitudeactual.h"        /* Estimated attitude */
#include "attitudematrix.h"        /* Estimated attitude */
#include "attituderaw.h"           /* Raw attitude sensor measurements */
#include "gpsposition.h"		   /* GPS position */
#include "gpssatellites.h"		   /* GPS satellites */
#include "flightstatus.h"          /* Main system state machine */
#include "globalpositiondesired.h" /* Set global position desired */
#include "positiondesired.h"       /* Set local position desired */
#include "positionactual.h"
#include "actuatorcommand.h"
#include "flightbatterystate.h"
#include "flightbatterysettings.h"


#include "actuatorsettings.h"

// Private constants
#define MAX_QUEUE_SIZE   MAVLINK_QUEUE_SIZE
#define STACK_SIZE_BYTES (PIOS_MAVLINK_STACK_SIZE)
#define TASK_PRIORITY_RX (tskIDLE_PRIORITY + 2)
#define TASK_PRIORITY_TX (tskIDLE_PRIORITY + 2)
#define TASK_PRIORITY_STATE_MACHINE (tskIDLE_PRIORITY + 2)
#define TASK_PRIORITY_TXPRI (tskIDLE_PRIORITY + 2)
#define REQ_TIMEOUT_MS 250
#define MAX_RETRIES 2
#define STATS_UPDATE_PERIOD_MS 4000
#define CONNECTION_TIMEOUT_MS 8000

// Private types

// Private variables
static uint32_t telemetryPort; // FIXME INIT
static xQueueHandle queue;

#if defined(PIOS_TELEM_PRIORITY_QUEUE)
static xQueueHandle priorityQueue;
static xTaskHandle telemetryTxPriTaskHandle;
static void telemetryTxPriTask(void *parameters);
#else
#define priorityQueue queue
#endif

static xTaskHandle telemetryTxTaskHandle;
static xTaskHandle telemetryRxTaskHandle;
static xTaskHandle mavlinkStateMachineTaskHandle;
static uint32_t txErrors;
static uint32_t txRetries;
static TelemetrySettingsData settings;
static uint32_t timeOfLastObjectUpdate;

// Private functions
static void telemetryTxTask(void *parameters);
static void telemetryRxTask(void *parameters);
static void mavlinkStateMachineTask(void *parameters);
//static int32_t transmitData(uint8_t * data, int32_t length);
static void registerObject(UAVObjHandle obj);
static void updateObject(UAVObjHandle obj);
static int32_t addObject(UAVObjHandle obj);
static int32_t setUpdatePeriod(UAVObjHandle obj, int32_t updatePeriodMs);
static void processObjEvent(UAVObjEvent * ev);
static void updateTelemetryStats();
static void gcsTelemetryStatsUpdated();
static void updateSettings();
/** \brief Enable hardware in the loop simulation */
static bool enableHil(bool enable);

//#include "mavlink_types.h"
//mavlink_system_t mavlink_system;
uint16_t next_param = 0;

//#include "mavlink_send_bridge.h"
#include "mavlink_debug.h"
#include "mavlink_parameters_openpilot.h"

/* Struct that stores the communication settings of this system.
   you can also define / alter these settings elsewhere, as long
   as they're included BEFORE mavlink.h.
   So you can set the

   mavlink_system.sysid = 100; // System ID, 1-255
   mavlink_system.compid = 50; // Component/Subsystem ID, 1-255

   Lines also in your main.c, e.g. by reading these parameter from EEPROM.
 */
//#include "mavlink_types.h"
//mavlink_system_t mavlink_system;
//
//#include "mavlink_send_bridge.h"
//#include "mavlink_debug.h"

static mavlink_message_t rx_msg;
//static mavlink_message_t tx_msg;
static mavlink_status_t rx_status;
static uint8_t mavlinkTxBuf[MAVLINK_MAX_PACKET_LEN];

//#include "common/mavlink.h"
#include "uavobjectmavlinksettings.h"

/* 3: Define waypoint helper functions */
void mavlink_missionlib_send_message(mavlink_message_t* msg);
void mavlink_missionlib_send_gcs_string(const char* string);
uint64_t mavlink_missionlib_get_system_timestamp();
void mavlink_missionlib_current_waypoint_changed(uint16_t index, float param1,
		float param2, float param3, float param4, float param5_lat_x,
		float param6_lon_y, float param7_alt_z, uint8_t frame, uint16_t command);

/* 4: Include waypoint protocol */
#include <waypoints.h>
mavlink_wpm_storage wpm;

#include "mavlink_missionlib_data.h"
#include "mavlink_parameters.h"


/* Provide the interface functions for the waypoint manager */

/*
 *  @brief Sends a MAVLink message over UDP
 */
void mavlink_missionlib_send_message(mavlink_message_t* msg)
{
	uint16_t len = mavlink_msg_to_send_buffer(mavlinkTxBuf, msg);
	// Send buffer
	PIOS_COM_SendBufferNonBlocking(telemetryPort, mavlinkTxBuf, len);
}

void mavlink_missionlib_send_gcs_string(const char* string)
{
	const int len = 50;
	mavlink_statustext_t status;
	int i = 0;
	while (i < len - 1)
	{
		status.text[i] = string[i];
		if (string[i] == '\0')
			break;
		i++;
	}
	status.text[i] = '\0'; // Enforce null termination
	mavlink_message_t msg;

	mavlink_msg_statustext_encode(mavlink_system.sysid, mavlink_system.compid, &msg, &status);
	mavlink_missionlib_send_message(&msg);
}

uint64_t mavlink_missionlib_get_system_timestamp()
{
	//	struct timeval tv;
	//	gettimeofday(&tv, NULL);
	//	return ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;
	return xTaskGetTickCount() * portTICK_RATE_MS;
}

void mavlink_missionlib_current_waypoint_changed(uint16_t index, float param1,
		float param2, float param3, float param4, float param5_lat_x,
		float param6_lon_y, float param7_alt_z, uint8_t frame, uint16_t command)
{
	if (frame == MAV_FRAME_GLOBAL || frame == MAV_FRAME_GLOBAL_RELATIVE_ALT)
	{
		GlobalPositionDesiredData pos;
		GlobalPositionDesiredGet(&pos);
		pos.Latitude = param5_lat_x*1E7;
		pos.Longitude = param6_lon_y*1E7;
		pos.Altitude = param7_alt_z;
		pos.AltitudeIsAbsoluteMSL = (frame == MAV_FRAME_GLOBAL) ? 1 : 0;
		GlobalPositionDesiredSet(&pos);
	}
	else if (frame == MAV_FRAME_LOCAL_NED)
	{
		PositionDesiredData pos;
		PositionDesiredGet(&pos);
		pos.North = param5_lat_x*1E2;  // Convert from meter to cm
		pos.East = param6_lon_y*1E2;   // Convert from meter to cm
		pos.Down = param7_alt_z*1E2;   // Convert from meter to cm
		PositionDesiredSet(&pos);
	}
}

/**
 * Initialise the telemetry module
 * \return -1 if initialisation failed
 * \return 0 on success
 */
int32_t MAVLinkStart(void)
{
	// Process all registered objects and connect queue for updates
	UAVObjIterate(&registerObject);

	// Listen to objects of interest
	GCSTelemetryStatsConnectQueue(priorityQueue);
	TelemetrySettingsConnectQueue(priorityQueue);

	// Start telemetry tasks
	xTaskCreate(telemetryTxTask, (signed char *)"MAVLinkTx", STACK_SIZE_BYTES/4, NULL, TASK_PRIORITY_TX, &telemetryTxTaskHandle);
	xTaskCreate(telemetryRxTask, (signed char *)"MAVLinkRx", STACK_SIZE_BYTES/4, NULL, TASK_PRIORITY_RX, &telemetryRxTaskHandle);
	xTaskCreate(mavlinkStateMachineTask, (signed char*) "MAVLinkSM", STACK_SIZE_BYTES/4, NULL, TASK_PRIORITY_STATE_MACHINE, &mavlinkStateMachineTaskHandle);
	TaskMonitorAdd(TASKINFO_RUNNING_TELEMETRYTX, telemetryTxTaskHandle);
	TaskMonitorAdd(TASKINFO_RUNNING_TELEMETRYRX, telemetryRxTaskHandle);

#if defined(PIOS_TELEM_PRIORITY_QUEUE)
	xTaskCreate(telemetryTxPriTask, (signed char *)"MAVLPriTx", STACK_SIZE_BYTES/4, NULL, TASK_PRIORITY_TXPRI, &telemetryTxPriTaskHandle);
	TaskMonitorAdd(TASKINFO_RUNNING_TELEMETRYTXPRI, telemetryTxPriTaskHandle);
#endif

	return 0;
}

/**
 * Initialise the telemetry module
 * \return -1 if initialisation failed
 * \return 0 on success
 */
int32_t MAVLinkInitialize(void)
{
	UAVObjEvent ev;

	FlightTelemetryStatsInitialize();
	GCSTelemetryStatsInitialize();
	TelemetrySettingsInitialize();

	// Initialize vars
	timeOfLastObjectUpdate = 0;

	// Create object queues
	queue = xQueueCreate(MAX_QUEUE_SIZE, sizeof(UAVObjEvent));
#if defined(PIOS_TELEM_PRIORITY_QUEUE)
	priorityQueue = xQueueCreate(MAX_QUEUE_SIZE, sizeof(UAVObjEvent));
#endif

	// Get telemetry settings object
	updateSettings();

	// Initialize waypoint protocol
	mavlink_wpm_init(&wpm);

	// Start specific update rates


	// Create periodic event that will be used to update the telemetry stats
	txErrors = 0;
	txRetries = 0;
	memset(&ev, 0, sizeof(UAVObjEvent));
	EventPeriodicQueueCreate(&ev, priorityQueue, STATS_UPDATE_PERIOD_MS);


	return 0;
}

MODULE_INITCALL(MAVLinkInitialize, MAVLinkStart)

/**
 * Register a new object, adds object to local list and connects the queue depending on the object's
 * telemetry settings.
 * \param[in] obj Object to connect
 */
static void registerObject(UAVObjHandle obj)
{
	// Setup object for periodic updates
	addObject(obj);

	// Setup object for telemetry updates
	updateObject(obj);
}

/**
 * Update object's queue connections and timer, depending on object's settings
 * \param[in] obj Object to updates
 */
static void updateObject(UAVObjHandle obj)
{
	UAVObjMetadata metadata;
	int32_t eventMask;

	// Get metadata
	UAVObjGetMetadata(obj, &metadata);

	// Setup object depending on update mode
	if (metadata.telemetryUpdateMode == UPDATEMODE_PERIODIC) {
		// Set update period
		setUpdatePeriod(obj, metadata.telemetryUpdatePeriod);
		// Connect queue
		eventMask = EV_UPDATED_MANUAL | EV_UPDATE_REQ;
		if (UAVObjIsMetaobject(obj)) {
			eventMask |= EV_UNPACKED;	// we also need to act on remote updates (unpack events)
		}
		UAVObjConnectQueue(obj, priorityQueue, eventMask);
	} else if (metadata.telemetryUpdateMode == UPDATEMODE_ONCHANGE) {
		// Set update period
		setUpdatePeriod(obj, 0);
		// Connect queue
		eventMask = EV_UPDATED | EV_UPDATED_MANUAL | EV_UPDATE_REQ;
		if (UAVObjIsMetaobject(obj)) {
			eventMask |= EV_UNPACKED;	// we also need to act on remote updates (unpack events)
		}
		UAVObjConnectQueue(obj, priorityQueue, eventMask);
	} else if (metadata.telemetryUpdateMode == UPDATEMODE_MANUAL) {
		// Set update period
		setUpdatePeriod(obj, 0);
		// Connect queue
		eventMask = EV_UPDATED_MANUAL | EV_UPDATE_REQ;
		if (UAVObjIsMetaobject(obj)) {
			eventMask |= EV_UNPACKED;	// we also need to act on remote updates (unpack events)
		}
		UAVObjConnectQueue(obj, priorityQueue, eventMask);
	} else if (metadata.telemetryUpdateMode == UPDATEMODE_NEVER) {
		// Set update period
		setUpdatePeriod(obj, 0);
		// Disconnect queue
		UAVObjDisconnectQueue(obj, priorityQueue);
	}
}

// FIXME XXX

#include "systemstats.h"
#include "flighttelemetrystats.h"
#include "gcstelemetrystats.h"

FlightTelemetryStatsData flightStats;
GCSTelemetryStatsData gcsTelemetryStatsData;
static AttitudeActualData attitudeActual;
static AttitudeMatrixData attitudeMatrix;
static AttitudeRawData attitudeRaw;
static BaroAltitudeData baroAltitude;
static GPSPositionData gpsPosition;
//static ManualControlCommandData manualControl;
static mavlink_raw_imu_t attitude_raw;
static mavlink_attitude_t attitude;
static mavlink_scaled_pressure_t pressure;
static mavlink_gps_raw_int_t gps_raw;
//static mavlink_rc_channels_raw_t rc_channels;
//static mavlink_debug_vect_t debug;
static uint32_t lastOperatorHeartbeat = 0;
static bool hilEnabled = false;


/**
 * Processes queue events
 */
static void processObjEvent(UAVObjEvent * ev)
{
	UAVObjMetadata metadata;
	//	FlightTelemetryStatsData flightStats;
	//	GCSTelemetryStatsData gcsTelemetryStatsData;
	//	int32_t retries;
	//	int32_t success;

	if (ev->obj == 0) {
		updateTelemetryStats();
	} else if (ev->obj == GCSTelemetryStatsHandle()) {
		gcsTelemetryStatsUpdated();
	} else if (ev->obj == TelemetrySettingsHandle()) {
		updateSettings();
	} else {

		// Get object metadata
		UAVObjGetMetadata(ev->obj, &metadata);

		// If this is a metaobject then make necessary telemetry updates
		if (UAVObjIsMetaobject(ev->obj)) {
			updateObject(UAVObjGetLinkedObj(ev->obj));	// linked object will be the actual object the metadata are for
		}

		mavlink_message_t msg;

		mavlink_system.sysid = 20;
		mavlink_system.compid = MAV_COMP_ID_IMU;
		mavlink_system.type = MAV_TYPE_FIXED_WING;
		uint8_t mavClass = MAV_AUTOPILOT_OPENPILOT;

		AlarmsClear(SYSTEMALARMS_ALARM_TELEMETRY);

		// Setup type and object id fields
		uint32_t objId = UAVObjGetID(ev->obj);

		//		uint64_t timeStamp = 0;
		switch(objId) {
		case BAROALTITUDE_OBJID:
		{
			BaroAltitudeGet(&baroAltitude);
			pressure.press_abs = baroAltitude.Pressure*10.0f;
			pressure.temperature = baroAltitude.Temperature*100.0f;
			mavlink_msg_scaled_pressure_encode(mavlink_system.sysid, mavlink_system.compid, &msg, &pressure);
			// Copy the message to the send buffer
			uint16_t len = mavlink_msg_to_send_buffer(mavlinkTxBuf, &msg);
			// Send buffer
			PIOS_COM_SendBufferNonBlocking(telemetryPort, mavlinkTxBuf, len);
			break;
		}
		case FLIGHTTELEMETRYSTATS_OBJID:
		{
			//				FlightTelemetryStatsData flightTelemetryStats;
			FlightTelemetryStatsGet(&flightStats);

			// XXX this is a hack to make it think it got a confirmed
			// connection
			flightStats.Status = FLIGHTTELEMETRYSTATS_STATUS_CONNECTED;
			GCSTelemetryStatsGet(&gcsTelemetryStatsData);
			gcsTelemetryStatsData.Status = GCSTELEMETRYSTATS_STATUS_CONNECTED;
			//
			//
			//				//mavlink_msg_heartbeat_send(MAVLINK_COMM_0,mavlink_system.type,mavClass);
			//				mavlink_msg_heartbeat_pack(mavlink_system.sysid, mavlink_system.compid, &msg, mavlink_system.type, mavClass);
			//				// Copy the message to the send buffer
			//				uint16_t len = mavlink_msg_to_send_buffer(mavlinkTxBuf, &msg);
			//				// Send buffer
			//				PIOS_COM_SendBufferNonBlocking(telemetryPort, mavlinkTxBuf, len);
			break;
		}
		case SYSTEMSTATS_OBJID:
		{
			FlightStatusData flightStatus;
			FlightStatusGet(&flightStatus);

			uint8_t system_state = MAV_STATE_UNINIT;
			uint8_t base_mode = 0;
			uint8_t custom_mode = 0;

			// Set flight mode
			switch (flightStatus.FlightMode)
			{
			case FLIGHTSTATUS_FLIGHTMODE_MANUAL:
				base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
				break;
			case FLIGHTSTATUS_FLIGHTMODE_POSITIONHOLD:
				base_mode |= MAV_MODE_FLAG_GUIDED_ENABLED;
				break;
			case FLIGHTSTATUS_FLIGHTMODE_STABILIZED1:
				base_mode |= MAV_MODE_FLAG_STABILIZE_ENABLED;
				break;
			case FLIGHTSTATUS_FLIGHTMODE_STABILIZED2:
				base_mode |= MAV_MODE_FLAG_GUIDED_ENABLED;
				break;
			case FLIGHTSTATUS_FLIGHTMODE_STABILIZED3:
				base_mode |= MAV_MODE_FLAG_AUTO_ENABLED;
				break;
			case FLIGHTSTATUS_FLIGHTMODE_VELOCITYCONTROL:
				base_mode |= MAV_MODE_FLAG_GUIDED_ENABLED;
				break;
			default:
				base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
				break;
			}

			// Set arming state
			switch (flightStatus.Armed)
			{
			case FLIGHTSTATUS_ARMED_ARMING:
			case FLIGHTSTATUS_ARMED_ARMED:
				system_state = MAV_STATE_ACTIVE;
				base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
				break;
			case FLIGHTSTATUS_ARMED_DISARMED:
				system_state = MAV_STATE_STANDBY;
				base_mode &= !MAV_MODE_FLAG_SAFETY_ARMED;
				break;
			}

			// Set HIL
			if (hilEnabled) base_mode |= MAV_MODE_FLAG_HIL_ENABLED;

			mavlink_msg_heartbeat_send(MAVLINK_COMM_0, mavlink_system.type, mavClass, base_mode, custom_mode, system_state);

			SystemStatsData stats;
			SystemStatsGet(&stats);
			FlightBatteryStateData flightBatteryData;
			FlightBatteryStateGet(&flightBatteryData);
			FlightBatterySettingsData flightBatterySettings;
			FlightBatterySettingsGet(&flightBatterySettings);

			uint16_t batteryVoltage = (uint16_t)(flightBatteryData.Voltage*1000.0f);
			int16_t batteryCurrent = -1; // -1: Not present / not estimated
			int8_t batteryPercent = -1; // -1: Not present / not estimated
			//			if (flightBatterySettings.SensorCalibrations[FLIGHTBATTERYSETTINGS_SENSORCALIBRATIONS_CURRENTFACTOR] == 0)
			//			{
			// Factor is zero, sensor is not present
			// Estimate remaining capacity based on lipo curve
			batteryPercent = 100.0f*((flightBatteryData.Voltage - 9.6f)/(12.6f - 9.6f));
			//			}
			//			else
			//			{
			//				// Use capacity and current
			//				batteryPercent = 100.0f*((flightBatterySettings.Capacity - flightBatteryData.ConsumedEnergy) / flightBatterySettings.Capacity);
			//				batteryCurrent = flightBatteryData.Current*100;
			//			}

				mavlink_msg_sys_status_send(MAVLINK_COMM_0, 0xFF, 0xFF, 0xFF, ((uint16_t)stats.CPULoad*10), batteryVoltage, batteryCurrent, batteryPercent, 0, 0, 0, 0, 0, 0);
//				// Copy the message to the send buffer
//				uint16_t len = mavlink_msg_to_send_buffer(mavlinkTxBuf, &msg);
//				// Send buffer
//				PIOS_COM_SendBufferNonBlocking(telemetryPort, mavlinkTxBuf, len);
			break;
		}
		case ATTITUDERAW_OBJID:
		{
			AttitudeRawGet(&attitudeRaw);

			// Copy data
			attitude_raw.xacc = attitudeRaw.accels[ATTITUDERAW_ACCELS_X];
			attitude_raw.yacc = attitudeRaw.accels[ATTITUDERAW_ACCELS_Y];
			attitude_raw.zacc = attitudeRaw.accels[ATTITUDERAW_ACCELS_Z];
			attitude_raw.xgyro = attitudeRaw.gyros[ATTITUDERAW_GYROS_X];
			attitude_raw.ygyro = attitudeRaw.gyros[ATTITUDERAW_GYROS_Y];
			attitude_raw.zgyro = attitudeRaw.gyros[ATTITUDERAW_GYROS_Z];
			attitude_raw.xmag = attitudeRaw.magnetometers[ATTITUDERAW_MAGNETOMETERS_X];
			attitude_raw.ymag = attitudeRaw.magnetometers[ATTITUDERAW_MAGNETOMETERS_Y];
			attitude_raw.zmag = attitudeRaw.magnetometers[ATTITUDERAW_MAGNETOMETERS_Z];

			mavlink_msg_raw_imu_encode(mavlink_system.sysid, mavlink_system.compid, &msg, &attitude_raw);
			// Copy the message to the send buffer
			uint16_t len = mavlink_msg_to_send_buffer(mavlinkTxBuf, &msg);
			// Send buffer
			PIOS_COM_SendBufferNonBlocking(telemetryPort, mavlinkTxBuf, len);

			if (hilEnabled)
			{
				mavlink_hil_controls_t controls;

				// Copy data
				controls.roll_ailerons = 0.1;
				controls.pitch_elevator = 0.1;
				controls.yaw_rudder = 0.0;
				controls.throttle = 0.8;

				mavlink_msg_hil_controls_encode(mavlink_system.sysid, mavlink_system.compid, &msg, &controls);
				// Copy the message to the send buffer
				len = mavlink_msg_to_send_buffer(mavlinkTxBuf, &msg);
				// Send buffer
				PIOS_COM_SendBufferNonBlocking(telemetryPort, mavlinkTxBuf, len);
			}
			break;
		}
		case ATTITUDEMATRIX_OBJID:
		{
			AttitudeMatrixGet(&attitudeMatrix);

			// Copy data
			attitude.roll = attitudeMatrix.Roll;
			attitude.pitch = attitudeMatrix.Pitch;
			attitude.yaw = attitudeMatrix.Yaw;

			attitude.rollspeed = attitudeMatrix.AngularRates[0];
			attitude.pitchspeed = attitudeMatrix.AngularRates[1];
			attitude.yawspeed = attitudeMatrix.AngularRates[2];

			mavlink_msg_attitude_encode(mavlink_system.sysid,
					mavlink_system.compid, &msg, &attitude);
			// Copy the message to the send buffer
			uint16_t len = mavlink_msg_to_send_buffer(mavlinkTxBuf, &msg);
			// Send buffer
			PIOS_COM_SendBufferNonBlocking(telemetryPort, mavlinkTxBuf, len);
			break;
		}
		case GPSPOSITION_OBJID:
		{
			GPSPositionGet(&gpsPosition);
			gps_raw.time_usec = 0;
			gps_raw.lat = gpsPosition.Latitude*10;
			gps_raw.lon = gpsPosition.Longitude*10;
			gps_raw.alt = gpsPosition.Altitude*10;
			gps_raw.eph = gpsPosition.HDOP*100;
			gps_raw.epv = gpsPosition.VDOP*100;
			gps_raw.cog = gpsPosition.Heading*100;
			gps_raw.satellites_visible = gpsPosition.Satellites;
			gps_raw.fix_type = gpsPosition.Status;
			mavlink_msg_gps_raw_int_encode(mavlink_system.sysid, mavlink_system.compid, &msg, &gps_raw);
			// Copy the message to the send buffer
			uint16_t len = mavlink_msg_to_send_buffer(mavlinkTxBuf, &msg);
			// Send buffer
			PIOS_COM_SendBufferNonBlocking(telemetryPort, mavlinkTxBuf, len);

			//			mavlink_msg_gps_raw_int_send(MAVLINK_COMM_0, gps_raw.usec, gps_raw.lat, gps_raw.lon, gps_raw.alt, gps_raw.eph, gps_raw.epv, gps_raw.hdg, gps_raw.satellites_visible, gps_raw.fix_type, 0);

			break;
		}
		case POSITIONACTUAL_OBJID:
		{
			PositionActualData pos;
			PositionActualGet(&pos);
			mavlink_local_position_ned_t m_pos;
			m_pos.time_boot_ms = 0;
			m_pos.x = pos.North;
			m_pos.y = pos.East;
			m_pos.z = pos.Down;
			m_pos.vx = 0.0f;
			m_pos.vy = 0.0f;
			m_pos.vz = 0.0f;

			mavlink_msg_local_position_ned_encode(mavlink_system.sysid, mavlink_system.compid, &msg, &m_pos);

			// Copy the message to the send buffer
			uint16_t len = mavlink_msg_to_send_buffer(mavlinkTxBuf, &msg);
			// Send buffer
			PIOS_COM_SendBufferNonBlocking(telemetryPort, mavlinkTxBuf, len);
		}
		break;
		case ACTUATORCOMMAND_OBJID:
		{
			mavlink_rc_channels_scaled_t rc;
			float val;
			ManualControlCommandRollGet(&val);
			rc.chan1_scaled = val*1000;
			ManualControlCommandPitchGet(&val);
			rc.chan2_scaled = val*1000;
			ManualControlCommandYawGet(&val);
			rc.chan3_scaled = val*1000;
			ManualControlCommandThrottleGet(&val);
			rc.chan4_scaled = val*1000;

			ActuatorCommandData act;
			ActuatorCommandGet(&act);

			rc.chan5_scaled = act.Channel[0];
			rc.chan6_scaled = act.Channel[1];
			rc.chan7_scaled = act.Channel[2];
			rc.chan8_scaled = act.Channel[3];

			ManualControlCommandData cmd;
			ManualControlCommandGet(&cmd);

			rc.rssi = ((uint8_t)(cmd.Connected == MANUALCONTROLCOMMAND_CONNECTED_TRUE))*255;
			rc.port = 0;

			mavlink_msg_rc_channels_scaled_encode(mavlink_system.sysid, mavlink_system.compid, &msg, &rc);


			// Copy the message to the send buffer
			uint16_t len = mavlink_msg_to_send_buffer(mavlinkTxBuf, &msg);
			// Send buffer
			PIOS_COM_SendBufferNonBlocking(PIOS_COM_TELEM_RF, mavlinkTxBuf, len);
			break;
		}
		case MANUALCONTROLCOMMAND_OBJID:
		{
			mavlink_rc_channels_scaled_t rc;
			float val;
			ManualControlCommandRollGet(&val);
			rc.chan1_scaled = val*1000;
			ManualControlCommandPitchGet(&val);
			rc.chan2_scaled = val*1000;
			ManualControlCommandYawGet(&val);
			rc.chan3_scaled = val*1000;
			ManualControlCommandThrottleGet(&val);
			rc.chan4_scaled = val*1000;

			rc.chan5_scaled = 0;
			rc.chan6_scaled = 0;
			rc.chan7_scaled = 0;
			rc.chan8_scaled = 0;

			ManualControlCommandData cmd;
			ManualControlCommandGet(&cmd);

			rc.rssi = ((uint8_t)(cmd.Connected == MANUALCONTROLCOMMAND_CONNECTED_TRUE))*255;
			rc.port = 0;

			mavlink_msg_rc_channels_scaled_encode(mavlink_system.sysid, mavlink_system.compid, &msg, &rc);


			// Copy the message to the send buffer
			uint16_t len = mavlink_msg_to_send_buffer(mavlinkTxBuf, &msg);
			// Send buffer
			PIOS_COM_SendBufferNonBlocking(PIOS_COM_TELEM_RF, mavlinkTxBuf, len);
			break;
		}
		default:
		{
			//printf("unknown object: %x\n",(unsigned int)objId);
			break;
		}
		}
	}
}

/**
 * Telemetry transmit task, regular priority
 */
static void telemetryTxTask(void *parameters)
{
	UAVObjEvent ev;

	// Loop forever
	while (1) {
		// Wait for queue message
		if (xQueueReceive(queue, &ev, portMAX_DELAY) == pdTRUE) {
			// Process event
			processObjEvent(&ev);
		}
	}
}

/**
 * Telemetry transmit task, high priority
 */
#if defined(PIOS_TELEM_PRIORITY_QUEUE)
static void telemetryTxPriTask(void *parameters)
{
	UAVObjEvent ev;

	// Loop forever
	while (1) {
		// Wait for queue message
		if (xQueueReceive(priorityQueue, &ev, portMAX_DELAY) == pdTRUE) {
			// Process event
			processObjEvent(&ev);
		}
	}
}
#endif

static void mavlinkStateMachineTask(void* parameters)
{
	// MAVLink protocol state machine
	while (1) {
		// Run handlers, sleep for 20 ms
		// Send setpoints, time out
		// Send one param
		mavlink_pm_queued_send();
		vTaskDelay(5);
		mavlink_wpm_loop();
		vTaskDelay(5);
		// Send one text message
		debug_message_send_one();
		// Wait 20 ms
		vTaskDelay(20);
	}
}

static bool enableHil(bool enable)
{
	// READ-ONLY flag write to ActuatorCommand
	UAVObjHandle handle = ActuatorCommandHandle();

	UAVObjMetadata commandMeta;
	UAVObjGetMetadata(handle, &commandMeta);

	if (enable)
	{
		// Disable actuators
		commandMeta.access = ACCESS_READONLY;
	}
	else
	{
		// Enable actuators
		commandMeta.access = ACCESS_READWRITE;
	}

	return (UAVObjSetMetadata(handle, &commandMeta) == 0);
}

/**
 * Telemetry transmit task. Processes queue events and periodic updates.
 */
static void telemetryRxTask(void *parameters)
{
	uint32_t inputPort;
	uint8_t	c;

	// Task loop
	while (1) {
#if defined(PIOS_INCLUDE_USB_HID)
		// Determine input port (USB takes priority over telemetry port)
		if (PIOS_USB_HID_CheckAvailable(0)) {
			inputPort = PIOS_COM_TELEM_USB;
		} else
#endif /* PIOS_INCLUDE_USB_HID */
		{
			inputPort = telemetryPort;
		}

		mavlink_channel_t mavlink_chan = MAVLINK_COMM_0;

		// Block until a byte is available
		PIOS_COM_ReceiveBuffer(inputPort, &c, 1, portMAX_DELAY);

		// And process it

		if (mavlink_parse_char(mavlink_chan, c, &rx_msg, &rx_status))
		{

			// Handle packet with waypoint component
			mavlink_wpm_message_handler(&rx_msg);

			// Handle packet with parameter component
			mavlink_pm_message_handler(mavlink_chan, &rx_msg);

			switch (rx_msg.msgid)
			{
			case MAVLINK_MSG_ID_HEARTBEAT:
			{
				// Check if this is the gcs
				mavlink_heartbeat_t beat;
				mavlink_msg_heartbeat_decode(&rx_msg, &beat);
				if (beat.type == MAV_TYPE_GCS)
				{
					// Got heartbeat from the GCS, we're good!
					lastOperatorHeartbeat = xTaskGetTickCount() * portTICK_RATE_MS;
				}
			}
			break;
			case MAVLINK_MSG_ID_SET_MODE:
			{
				mavlink_set_mode_t mode;
				mavlink_msg_set_mode_decode(&rx_msg, &mode);
				// Check if this system should change the mode
				if (mode.target_system == mavlink_system.sysid)
				{
					FlightStatusData flightStatus;
					FlightStatusGet(&flightStatus);

					switch (mode.base_mode)
					{
					case MAV_MODE_MANUAL_ARMED:
					{
						flightStatus.FlightMode = FLIGHTSTATUS_FLIGHTMODE_MANUAL;
						flightStatus.Armed = FLIGHTSTATUS_ARMED_ARMED;
					}
					break;
					case MAV_MODE_MANUAL_DISARMED:
					{
						flightStatus.FlightMode = FLIGHTSTATUS_FLIGHTMODE_MANUAL;
						flightStatus.Armed = FLIGHTSTATUS_ARMED_DISARMED;
					}
					break;
					case MAV_MODE_PREFLIGHT:
					{
						flightStatus.Armed = FLIGHTSTATUS_ARMED_DISARMED;
					}
					break;
					case MAV_MODE_STABILIZE_ARMED:
					{
						flightStatus.FlightMode = FLIGHTSTATUS_FLIGHTMODE_STABILIZED1;
						flightStatus.Armed = FLIGHTSTATUS_ARMED_ARMED;
					}
					break;
					case MAV_MODE_GUIDED_ARMED:
					{
						flightStatus.FlightMode = FLIGHTSTATUS_FLIGHTMODE_STABILIZED2;
						flightStatus.Armed = FLIGHTSTATUS_ARMED_ARMED;
					}
					break;
					case MAV_MODE_AUTO_ARMED:
					{
						flightStatus.FlightMode = FLIGHTSTATUS_FLIGHTMODE_STABILIZED3;
						flightStatus.Armed = FLIGHTSTATUS_ARMED_ARMED;
					}
					break;
					}

					bool newHilEnabled = (mode.base_mode & MAV_MODE_FLAG_DECODE_POSITION_HIL);
					if (newHilEnabled != hilEnabled)
					{
						if (newHilEnabled)
						{
							// READ-ONLY flag write to ActuatorCommand
							UAVObjMetadata meta;
							UAVObjHandle handle = ActuatorCommandHandle();
							UAVObjGetMetadata(handle, &meta);
							meta.access = ACCESS_READONLY;
							UAVObjSetMetadata(handle, &meta);

							mavlink_missionlib_send_gcs_string("ENABLING HIL SIMULATION");
							mavlink_missionlib_send_gcs_string("+++++++++++++++++++++++");
							mavlink_missionlib_send_gcs_string("BLOCKING ALL ACTUATORS");
						}
						else
						{
							// READ-ONLY flag write to ActuatorCommand
							UAVObjMetadata meta;
							UAVObjHandle handle = ActuatorCommandHandle();
							UAVObjGetMetadata(handle, &meta);
							meta.access = ACCESS_READWRITE;
							UAVObjSetMetadata(handle, &meta);

							mavlink_missionlib_send_gcs_string("DISABLING HIL SIMULATION");
							mavlink_missionlib_send_gcs_string("+++++++++++++++++++++++");
							mavlink_missionlib_send_gcs_string("ACTIVATING ALL ACTUATORS");
						}
					}
					hilEnabled = newHilEnabled;

					FlightStatusSet(&flightStatus);

					// Check HIL
					bool hilEnabled = (mode.base_mode & MAV_MODE_FLAG_DECODE_POSITION_HIL);
					enableHil(hilEnabled);
				}
			}
			break;
			case MAVLINK_MSG_ID_HIL_STATE:
			{
				if (hilEnabled)
				{
					mavlink_hil_state_t hil;
					mavlink_msg_hil_state_decode(&rx_msg, &hil);

					// Write GPSPosition
					GPSPositionData gps;
					GPSPositionGet(&gps);
					gps.Altitude = hil.alt/10;
					gps.Latitude = hil.lat/10;
					gps.Longitude = hil.lon/10;
					GPSPositionSet(&gps);

					// Write PositionActual
					PositionActualData pos;
					PositionActualGet(&pos);
					// FIXME WRITE POSITION HERE
					PositionActualSet(&pos);

					// Write AttitudeActual
					AttitudeActualData att;
					AttitudeActualGet(&att);
					att.Roll = hil.roll;
					att.Pitch = hil.pitch;
					att.Yaw = hil.yaw;
					// FIXME
					//att.RollSpeed = hil.rollspeed;
					//att.PitchSpeed = hil.pitchspeed;
					//att.YawSpeed = hil.yawspeed;

					// Convert to quaternion formulation
					RPY2Quaternion(&attitudeActual.Roll, &attitudeActual.q1);
					// Write AttitudeActual
					AttitudeActualSet(&att);

					// Write AttitudeRaw
					AttitudeRawData raw;
					AttitudeRawGet(&raw);
					raw.gyros[0] = hil.rollspeed;
					raw.gyros[1] = hil.pitchspeed;
					raw.gyros[2] = hil.yawspeed;
					raw.accels[0] = hil.xacc;
					raw.accels[1] = hil.yacc;
					raw.accels[2] = hil.zacc;
					//				raw.magnetometers[0] = hil.xmag;
					//				raw.magnetometers[0] = hil.ymag;
					//				raw.magnetometers[0] = hil.zmag;
					AttitudeRawSet(&raw);
				}
			}
			break;
			case MAVLINK_MSG_ID_COMMAND_LONG:
			{
				// FIXME Implement
			}
			break;
			}
		}
	}
}

///**
// * Transmit data buffer to the modem or USB port.
// * \param[in] data Data buffer to send
// * \param[in] length Length of buffer
// * \return 0 Success
// */
//static int32_t transmitData(uint8_t * data, int32_t length)
//{
//	uint32_t outputPort;
//
//	// Determine input port (USB takes priority over telemetry port)
//#if defined(PIOS_INCLUDE_USB_HID)
//	if (PIOS_USB_HID_CheckAvailable(0)) {
//		outputPort = PIOS_COM_TELEM_USB;
//	} else
//#endif /* PIOS_INCLUDE_USB_HID */
//	{
//		outputPort = telemetryPort;
//	}
//
//	return PIOS_COM_SendBufferNonBlocking(outputPort, data, length);
//}


/**
 * Setup object for periodic updates.
 * \param[in] obj The object to update
 * \return 0 Success
 * \return -1 Failure
 */
static int32_t addObject(UAVObjHandle obj)
{
	UAVObjEvent ev;

	// Add object for periodic updates
	ev.obj = obj;
	ev.instId = UAVOBJ_ALL_INSTANCES;
	ev.event = EV_UPDATED_MANUAL;
	return EventPeriodicQueueCreate(&ev, queue, 0);
}

/**
 * Set update period of object (it must be already setup for periodic updates)
 * \param[in] obj The object to update
 * \param[in] updatePeriodMs The update period in ms, if zero then periodic updates are disabled
 * \return 0 Success
 * \return -1 Failure
 */
static int32_t setUpdatePeriod(UAVObjHandle obj, int32_t updatePeriodMs)
{
	UAVObjEvent ev;

	// Add object for periodic updates
	ev.obj = obj;
	ev.instId = UAVOBJ_ALL_INSTANCES;
	ev.event = EV_UPDATED_MANUAL;
	return EventPeriodicQueueUpdate(&ev, queue, updatePeriodMs);
}

/**
 * Called each time the GCS telemetry stats object is updated.
 * Trigger a flight telemetry stats update if a connection is not
 * yet established.
 */
static void gcsTelemetryStatsUpdated()
{
	FlightTelemetryStatsData flightStats;
	GCSTelemetryStatsData gcsStats;
	FlightTelemetryStatsGet(&flightStats);
	GCSTelemetryStatsGet(&gcsStats);
	if (flightStats.Status != FLIGHTTELEMETRYSTATS_STATUS_CONNECTED || gcsStats.Status != GCSTELEMETRYSTATS_STATUS_CONNECTED) {
		updateTelemetryStats();
	}
}

/**
 * Update telemetry statistics and handle connection handshake
 */
static void updateTelemetryStats()
{
	//	flightStats.Status = FLIGHTTELEMETRYSTATS_STATUS_CONNECTED;
	//	gcsStats.Status = GCSTELEMETRYSTATS_STATUS_CONNECTED;

	FlightTelemetryStatsData flightStats;
	GCSTelemetryStatsData gcsStats;
	uint8_t forceUpdate;
	uint8_t connectionTimeout;
	uint32_t timeNow;

	// Get object data
	FlightTelemetryStatsGet(&flightStats);
	GCSTelemetryStatsGet(&gcsStats);

	// Update stats object
	if (flightStats.Status == FLIGHTTELEMETRYSTATS_STATUS_CONNECTED) {
		//flightStats.RxDataRate = (float)utalkStats.rxBytes / ((float)STATS_UPDATE_PERIOD_MS / 1000.0);
		//flightStats.TxDataRate = (float)utalkStats.txBytes / ((float)STATS_UPDATE_PERIOD_MS / 1000.0);
		//flightStats.RxFailures += utalkStats.rxErrors;
		flightStats.TxFailures += txErrors;
		flightStats.TxRetries += txRetries;
		txErrors = 0;
		txRetries = 0;
	} else {
		flightStats.RxDataRate = 0;
		flightStats.TxDataRate = 0;
		flightStats.RxFailures = 0;
		flightStats.TxFailures = 0;
		flightStats.TxRetries = 0;
		txErrors = 0;
		txRetries = 0;
	}

	// Check for connection timeout
	timeNow = xTaskGetTickCount() * portTICK_RATE_MS;
	if ((timeNow - lastOperatorHeartbeat) > CONNECTION_TIMEOUT_MS) {
		connectionTimeout = 1;
	} else {
		connectionTimeout = 0;
	}

	// Update connection state
	forceUpdate = 1;

	// FIXME Hardcode value for now
	//	connectionTimeout = 0;

	// If the GCS heartbeat has been received in the last five seconds
	// set as connected
	if (flightStats.Status == FLIGHTTELEMETRYSTATS_STATUS_DISCONNECTED) {
		if (connectionTimeout == 0) {
			// Switching from disconnected to connected
			flightStats.Status = FLIGHTTELEMETRYSTATS_STATUS_CONNECTED;
			gcsStats.Status = GCSTELEMETRYSTATS_STATUS_CONNECTED;
		} else {
			// Not switching, no update needed
			forceUpdate = 0;
		}
	} else {
		// We were connected, checking if we're still connected
		if (connectionTimeout == 1) {
			flightStats.Status = FLIGHTTELEMETRYSTATS_STATUS_DISCONNECTED;
			gcsStats.Status = GCSTELEMETRYSTATS_STATUS_DISCONNECTED;
		} else {
			// We were connected and still are, no update needed
			forceUpdate = 0;
		}
	}

	// FIXME HARDCODED VALUES
	flightStats.Status = FLIGHTTELEMETRYSTATS_STATUS_CONNECTED;
	gcsStats.Status = GCSTELEMETRYSTATS_STATUS_CONNECTED;
	forceUpdate = 1;

	//	if (flightStats.Status == FLIGHTTELEMETRYSTATS_STATUS_DISCONNECTED) {
	//		// Wait for connection request
	//		if (gcsStats.Status == GCSTELEMETRYSTATS_STATUS_HANDSHAKEREQ) {
	//			flightStats.Status = FLIGHTTELEMETRYSTATS_STATUS_HANDSHAKEACK;
	//		}
	//	} else if (flightStats.Status == FLIGHTTELEMETRYSTATS_STATUS_HANDSHAKEACK) {
	//		// Wait for connection
	//		if (gcsStats.Status == GCSTELEMETRYSTATS_STATUS_CONNECTED) {
	//			flightStats.Status = FLIGHTTELEMETRYSTATS_STATUS_CONNECTED;
	//		} else if (gcsStats.Status == GCSTELEMETRYSTATS_STATUS_DISCONNECTED) {
	//			flightStats.Status = FLIGHTTELEMETRYSTATS_STATUS_DISCONNECTED;
	//		}
	//	} else if (flightStats.Status == FLIGHTTELEMETRYSTATS_STATUS_CONNECTED) {
	//		if (gcsStats.Status != GCSTELEMETRYSTATS_STATUS_CONNECTED || connectionTimeout) {
	//			flightStats.Status = FLIGHTTELEMETRYSTATS_STATUS_DISCONNECTED;
	//		} else {
	//			forceUpdate = 0;
	//		}
	//	} else {
	//		flightStats.Status = FLIGHTTELEMETRYSTATS_STATUS_DISCONNECTED;
	//	}

	// Update the telemetry alarm
	if (flightStats.Status == FLIGHTTELEMETRYSTATS_STATUS_CONNECTED) {
		AlarmsClear(SYSTEMALARMS_ALARM_TELEMETRY);
	} else {
		AlarmsSet(SYSTEMALARMS_ALARM_TELEMETRY, SYSTEMALARMS_ALARM_ERROR);
	}

	// Update object
	FlightTelemetryStatsSet(&flightStats);

	// Force telemetry update if not connected
	if (forceUpdate) {
		FlightTelemetryStatsUpdated();
	}
}

/**
 * Update the telemetry settings, called on startup and
 * each time the settings object is updated
 */
static void updateSettings()
{
	// Set port
	telemetryPort = PIOS_COM_TELEM_RF;

	// Retrieve settings
	TelemetrySettingsGet(&settings);

	//    if (telemetryPort) {
	//	// Set port speed
	//	if (settings.Speed == TELEMETRYSETTINGS_SPEED_2400) PIOS_COM_ChangeBaud(telemetryPort, 2400);
	//	else if (settings.Speed == TELEMETRYSETTINGS_SPEED_4800) PIOS_COM_ChangeBaud(telemetryPort, 4800);
	//	else if (settings.Speed == TELEMETRYSETTINGS_SPEED_9600) PIOS_COM_ChangeBaud(telemetryPort, 9600);
	//	else if (settings.Speed == TELEMETRYSETTINGS_SPEED_19200) PIOS_COM_ChangeBaud(telemetryPort, 19200);
	//	else if (settings.Speed == TELEMETRYSETTINGS_SPEED_38400) PIOS_COM_ChangeBaud(telemetryPort, 38400);
	//	else if (settings.Speed == TELEMETRYSETTINGS_SPEED_57600) PIOS_COM_ChangeBaud(telemetryPort, 57600);
	//	else if (settings.Speed == TELEMETRYSETTINGS_SPEED_115200) PIOS_COM_ChangeBaud(telemetryPort, 115200);
	//    }
}

/**
 * @}
 * @}
 */
