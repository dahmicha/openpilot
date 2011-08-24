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

#include "testing/mavlink_missionlib_data.h"
#include "mavlink_parameters_openpilot.h"
#include "mavlink_settings_adapter.h"
#include "math.h" /* isinf / isnan checks */

extern mavlink_system_t mavlink_system;
extern uint16_t next_param;

extern void mavlink_missionlib_send_message(mavlink_message_t* msg);
extern void mavlink_missionlib_send_gcs_string(const char* string);

void mavlink_pm_message_handler(const mavlink_channel_t chan, const mavlink_message_t* msg)
{
	switch (msg->msgid)
	{
		case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
			{
				// Start sending parameters
				next_param = 0;
				mavlink_missionlib_send_gcs_string("PM SENDING LIST");
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

				// Check if matched
				if (index != -1)
				{
					mavlink_param_union_t paramValue;
					if (!getParamByIndex(index, &paramValue)) return;
					// Only write and emit changes if there is actually a difference
					// AND only write if new value is NOT "not-a-number"
					// AND is NOT infinity
					if (paramValue.param_float != set.param_value)
						//							&& !isnan(set.param_value) // FIXME Need to check NaN only for floats
						//							&& !isinf(set.param_value))
					{
						paramValue.param_float = set.param_value;
						paramValue.type = set.param_type;

						if (setParamByName(set.param_id, &paramValue))
						{

						// Report back new value
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

	mavlink_missionlib_send_gcs_string("PM set param");
						}
						else
						{
							mavlink_missionlib_send_gcs_string("PM rejected param");
						}
					} // End valid value check
				} // End match check
			} // End system ID check

	} // End case
			break;
			
	} // End switch
	
}
	
//	/**
//	 * @brief Send low-priority messages at a maximum rate of xx Hertz
//	 *
//	 * This function sends messages at a lower rate to not exceed the wireless
//	 * bandwidth. It sends one message each time it is called until the buffer is empty.
//	 * Call this function with xx Hertz to increase/decrease the bandwidth.
//	 */
//	void mavlink_pm_queued_send(void)
//	{
//		//send parameters one by one
//		if (next_param < getParamCount())
//		{
//			//for (int i.. all active comm links)
//#ifndef MAVLINK_USE_CONVENIENCE_FUNCTIONS
//			mavlink_message_t tx_msg;
//			mavlink_msg_param_value_pack_chan(mavlink_system.sysid,
//											  mavlink_system.compid,
//											  MAVLINK_COMM_0,
//											  &tx_msg,
//											  pm.param_names[pm.next_param],
//											  pm.param_values[pm.next_param],
//											  MAV_DATA_TYPE_FLOAT,
//											  getParamCount(),
//											  next_param);
//			mavlink_missionlib_send_message(&tx_msg);
//#else
//			mavlink_msg_param_value_send(MAVLINK_COMM_0,
//										 pm.param_names[pm.next_param],
//										 pm.param_values[pm.next_param],
//										 MAV_DATA_TYPE_FLOAT,
//										 pm.size,
//										 pm.next_param);
//#endif
//			pm.next_param++;
//		}
//	}
