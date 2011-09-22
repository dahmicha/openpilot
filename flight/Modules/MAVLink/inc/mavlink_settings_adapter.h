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
 * @file       telemetry.h
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @brief      Include file of the telemetry module.
 * 	       As with all modules only the initialize function is exposed all other
 * 	       interactions with the module take place through the event queue and
 *             objects.
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

#ifndef MAVLINK_ADAPTER_H
#define MAVLINK_ADAPTER_H

#include "mavlink_types.h"

#define MAX_ACTUATOR_PARAMS 2
#define ONBOARD_PARAM_NAME_LENGTH 16

#include <string.h>
#include <stdbool.h>

enum MAVLINK_RET_VAL {
	MAVLINK_RET_VAL_PARAM_SUCCESS = 0,
	MAVLINK_RET_VAL_PARAM_NAME_DOES_NOT_EXIST = 1,
	MAVLINK_RET_VAL_PARAM_TYPE_MISMATCH = 2,
	MAVLINK_RET_VAL_PARAM_VALUE_TOO_LOW = 3,
	MAVLINK_RET_VAL_PARAM_VALUE_TOO_HIGH = 4,
	MAVLINK_RET_VAL_PARAM_VALUE_NOT_SUPPORTED = 5,
	MAVLINK_RET_VAL_PARAM_WRITE_ERROR = 6,
};

uint16_t getParamCount();

uint8_t getParamByName(const char* name, mavlink_param_union_t* param);

uint8_t setParamByName(const char* name, mavlink_param_union_t* param);

int16_t getParamIndexByName(const char* name);

uint8_t getParamByIndex(uint16_t index, mavlink_param_union_t* param);
uint8_t setParamByIndex(uint16_t index, const mavlink_param_union_t* param);

const char* getParamNameByIndex(uint16_t index);

int32_t writeParametersToStorage();

int32_t readParametersFromStorage();

#endif /* MAVLINK_ADAPTER_H */

/**
 * @}
 * @}
 */
