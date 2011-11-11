/*******************************************************************************

 Copyright (C) 2011 Lorenz Meier lm ( a t ) inf.ethz.ch

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.

 ****************************************************************************/

#include "mavlink_parameters_openpilot.h"
#include "uavobjectmavlinksettings.h"
#include "math.h" /* isinf / isnan checks */

#include "openpilot.h"
#include "objectpersistence.h"

#include "flightstatus.h"

extern mavlink_system_t mavlink_system;
extern uint16_t next_param;

extern void mavlink_missionlib_send_message(mavlink_message_t* msg);
extern void mavlink_missionlib_send_gcs_string(const char* string);

void mavlink_pm_start_sending_params()
{
	next_param = 0;
	mavlink_missionlib_send_gcs_string("PM# Sending list");
}

void mavlink_pm_stop_sending_params()
{
	next_param = getParamCount();
	mavlink_missionlib_send_gcs_string("PM# Aborting list transmission");
}

void mavlink_pm_message_handler(const mavlink_channel_t chan, const mavlink_message_t* msg)
{
	switch (msg->msgid)
	{
	case MAVLINK_MSG_ID_COMMAND_LONG:
	{
		mavlink_command_long_t cmd;
		mavlink_msg_command_long_decode(msg, &cmd);
		if (cmd.command == MAV_CMD_PREFLIGHT_STORAGE)
		{
			FlightStatusData status;
			FlightStatusGet(&status);
			if (status.Armed == FLIGHTSTATUS_ARMED_DISARMED/* || status.Armed == FLIGHTSTATUS_ARMED_HIL*/)
			{
				// WRITE command (to SDCARD / EEPROM)
				if (cmd.param1 == 1)
				{
					uint32_t ret = writeParametersToStorage();
					// FIXME Evaluate return value
					if (ret != 1000)
					{
						mavlink_missionlib_send_gcs_string("PM# stored params from RAM to ROM");
					}
					//mavlink_missionlib_send_gcs_string("PM# FAILED storing params");
				}
				// READ command (from SDCARD / EEPROM)
				else if (cmd.param1 == 0)
				{
					uint32_t ret = readParametersFromStorage();
					// FIXME Evaluate return value
					if (ret != 1000)
					{
						mavlink_missionlib_send_gcs_string("PM# read params from ROM to RAM");
						// Emit all params fresh
						mavlink_pm_start_sending_params();
					}
				}
			}
			else
			{
				mavlink_missionlib_send_gcs_string("PM# IGN read/write when ARMED");
			}
		}
	}
	break;
	case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
	{
		// Start sending parameters
		mavlink_pm_start_sending_params();
	}
	break;
	case MAVLINK_MSG_ID_PARAM_SET:
	{
		mavlink_param_set_t set;
		mavlink_msg_param_set_decode(msg, &set);

		// Check if this message is for this system
		if (set.target_system == mavlink_system.sysid && set.target_component == mavlink_system.compid)
		{
			char* key = set.param_id;

			int16_t index = getParamIndexByName(key);

			if (index == -1)
			{
				// Param not found
				mavlink_missionlib_send_gcs_string("PM# ERR: Param does not exist");
			}
			else
			{
				mavlink_param_union_t paramValue;
				// Only write and emit changes if there is actually a difference
				// AND only write if new value is NOT "not-a-number"
				// AND is NOT infinity
				if (1 == 1)
					//							&& !isnan(set.param_value) // FIXME Need to check NaN only for floats
					//							&& !isinf(set.param_value))
				{
					paramValue.param_float = set.param_value;
					paramValue.type = set.param_type;

					// Report back new value
					// Read back from storage
					uint8_t ret = setParamByIndex(index, &paramValue);
					if (ret == MAVLINK_RET_VAL_PARAM_SUCCESS)
					{
						// Write back new value

						ret = getParamByIndex(index, &paramValue);

#ifndef MAVLINK_USE_CONVENIENCE_FUNCTIONS
						mavlink_message_t tx_msg;
						mavlink_msg_param_value_pack_chan(mavlink_system.sysid,
								mavlink_system.compid,
								MAVLINK_COMM_0,
								&tx_msg,
								getParamNameByIndex(index),
								paramValue.param_float,
								paramValue.type,
								getParamCount(),
								index);
						mavlink_missionlib_send_message(&tx_msg);
#else
						mavlink_msg_param_value_send(MAVLINK_COMM_0,
								getParamNameByIndex(index),
								paramValue.param_float,
								paramValue.type,
								getParamCount(),
								index);
#endif
					}

					// Send back verbose status code
					switch (ret)
					{
					case MAVLINK_RET_VAL_PARAM_SUCCESS:
						mavlink_missionlib_send_gcs_string("PM# OK: Wrote param");
						break;
					case MAVLINK_RET_VAL_PARAM_NAME_DOES_NOT_EXIST:
						mavlink_missionlib_send_gcs_string("PM# ERR: Param does not exist");
						break;
					case MAVLINK_RET_VAL_PARAM_TYPE_MISMATCH:
						mavlink_missionlib_send_gcs_string("PM# ERR: Data type mismatch");
						break;
					case MAVLINK_RET_VAL_PARAM_VALUE_TOO_LOW:
						mavlink_missionlib_send_gcs_string("PM# ERR: Val too low");
						break;
					case MAVLINK_RET_VAL_PARAM_VALUE_TOO_HIGH:
						mavlink_missionlib_send_gcs_string("PM# ERR: Val too high");
						break;
					case MAVLINK_RET_VAL_PARAM_VALUE_NOT_SUPPORTED:
						mavlink_missionlib_send_gcs_string("PM# ERR: Val not supported");
						break;
					case MAVLINK_RET_VAL_PARAM_WRITE_ERROR:
						mavlink_missionlib_send_gcs_string("PM# ERR: Write error");
						break;
					default:
						mavlink_missionlib_send_gcs_string("PM# ERR: Unknown error");
						break;
					}

				} // End unchanged value check
				else
				{
					mavlink_missionlib_send_gcs_string("PM# OK: Param unchanged");
				}
			} // End match check
		} // End system ID check

	} // End case
	break;

	} // End switch

}

/**
 * @brief Send low-priority messages at a maximum rate of xx Hertz
 *
 * This function sends messages at a lower rate to not exceed the wireless
 * bandwidth. It sends one message each time it is called until the buffer is empty.
 * Call this function with xx Hertz to increase/decrease the bandwidth.
 */
void mavlink_pm_queued_send(void)
{
	//send parameters one by one
	if (next_param < getParamCount())
	{
		mavlink_param_union_t param;
		//for (int i.. all active comm links)
		uint8_t ret = getParamByIndex(next_param, &param);
		if (ret != MAVLINK_RET_VAL_PARAM_SUCCESS)
		{
			mavlink_missionlib_send_gcs_string("PM# ERR: Param index not found");
			return;
		}

		mavlink_msg_param_value_send(MAVLINK_COMM_0,
				getParamNameByIndex(next_param),
				param.param_float,
				param.type,
				getParamCount(),
				next_param);
		next_param++;
	}
}
