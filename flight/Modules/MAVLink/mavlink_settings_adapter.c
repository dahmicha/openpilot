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

#include <string.h>
#include "mavlink_settings_adapter.h"

#include "openpilot.h"
#include "mavlink.h"

// START INCLUSION OF SETTINGS HEADERS
#include "actuatorsettings.h"
// END INCLUSION OF SETTINGS HEADERS

// START INCLUSION OF INDIVIDUAL ADAPTER HEADERS
int16_t getActuatorSettingsParamIndexByName(const char* name);

const char* getActuatorSettingsParamNameByIndex(uint16_t index);

uint8_t getActuatorSettingsParamByIndex(uint16_t index, mavlink_param_union_t* param);

uint8_t setActuatorSettingsParamByIndex(uint16_t index, const mavlink_param_union_t* param);
// END INCLUSION OF INDIVIDUAL ADAPTER HEADERS


int16_t getParamIndexByName(const char* name)
{
	return getActuatorSettingsParamIndexByName(name);
}

uint8_t getParamByIndex(uint16_t index, mavlink_param_union_t* param)
{
	return getActuatorSettingsParamByIndex(index, param);
}

uint8_t setParamByIndex(uint16_t index, const mavlink_param_union_t* param)
{
	return setActuatorSettingsParamByIndex(index, param);
}

const char* getParamNameByIndex(uint16_t index)
{
	return getActuatorSettingsParamNameByIndex(index);
}

uint16_t getParamCount()
{
	return MAX_ACTUATOR_PARAMS;
}


uint8_t getParamByName(const char* name, mavlink_param_union_t* param)
{
	int16_t index = -1;
	uint8_t ret;

	// Search for index as long as it stays not found (-1)

	// START INDEX FOUND SECTION
	if (index == -1) index = getActuatorSettingsParamIndexByName(name);
	// END INDEX FOUND SECTION
	
	if (index > -1)
	{
		// Break on first match

		// START VALUE FOUND SECTION
		ret = getActuatorSettingsParamByIndex(index, param);
		if (ret == MAVLINK_RET_VAL_PARAM_SUCCESS) return ret; // Else continue with other sub-sections
		// END VALUE FOUND SECTION
	}
	
	// No match, return false
	return MAVLINK_RET_VAL_PARAM_NAME_DOES_NOT_EXIST;
}

uint8_t setParamByName(const char* name, mavlink_param_union_t* param)
{
	int16_t index = -1;
	uint8_t ret;

	// Search for index as long as it stays not found (-1)

	// START INDEX FOUND SECTION
	if (index == -1) index = getActuatorSettingsParamIndexByName(name);
	// END INDEX FOUND SECTION

	if (index > -1)
	{
		// Break on first match

		// START VALUE FOUND SECTION
		ret = setActuatorSettingsParamByIndex(index, param);
		if (ret == MAVLINK_RET_VAL_PARAM_SUCCESS) return ret; // Else continue with other sub-sections
		// END VALUE FOUND SECTION
	}
	
	// No match, return false
	return MAVLINK_RET_VAL_PARAM_NAME_DOES_NOT_EXIST;
}


// CONTENT OF C-FILES - TO BE REMOVED

int16_t getActuatorSettingsParamIndexByName(const char* name)
{
	for (int i = 0; i < MAX_ACTUATOR_PARAMS; ++i)
	{
		bool match = true;
		const char* storage_name = getActuatorSettingsParamNameByIndex(i);
		for (uint8_t j = 0; j < ONBOARD_PARAM_NAME_LENGTH; ++j)
		{
			// Compare
			if (storage_name[j] != name[j])
			{
				match = false;
			}
			
			// End matching if null termination is reached
			if (storage_name[j] == '\0')
			{
				break;
			}
		}
		if (match) return i;
	}
	return -1;
}

const char* getActuatorSettingsParamNameByIndex(uint16_t index)
{
	switch (index)
	{
		case 0:
			return "fixedwing_roll1";
		case 1:
			return "fixedwing_roll2";
	}
	return 0;
}

uint8_t getActuatorSettingsParamByIndex(uint16_t index, mavlink_param_union_t* param)
{
	ActuatorSettingsData settings;
	ActuatorSettingsGet(&settings);
	switch (index)
	{
		case 0:
		{
			param->param_uint32 = settings.FixedWingRoll1;
			param->type = MAVLINK_TYPE_UINT32_T;
		}
			break;
		case 1:
		{
			param->param_uint32 = settings.FixedWingRoll2;
			param->type = MAVLINK_TYPE_UINT32_T;
		}
			break;
		default:
		{
			return MAVLINK_RET_VAL_PARAM_NAME_DOES_NOT_EXIST;
		}
			break;
	}
	// Not returned in default case, return true
	return MAVLINK_RET_VAL_PARAM_SUCCESS;
}

uint8_t setActuatorSettingsParamByIndex(uint16_t index, const mavlink_param_union_t* param)
{
	ActuatorSettingsData settings;
	ActuatorSettingsGet(&settings);
	switch (index)
	{
		case 0:
		{
			if (param->type == MAVLINK_TYPE_UINT32_T)
			{
				settings.FixedWingRoll1 = param->param_uint32;
			}
			else
			{
				return MAVLINK_RET_VAL_PARAM_TYPE_MISMATCH;
			}
		}
			break;
		case 1:
		{
			if (param->type == MAVLINK_TYPE_UINT32_T)
			{
				settings.FixedWingRoll2 = param->param_uint32;
			}
			else
			{
				return MAVLINK_RET_VAL_PARAM_TYPE_MISMATCH;
			}
		}
			break;
		default:
		{
			return MAVLINK_RET_VAL_PARAM_NAME_DOES_NOT_EXIST;
		}
			break;
	}
	
	// Not returned in default case, try to write (ok == 0) and return result of
	// write operation
	if (ActuatorSettingsSet(&settings) == 0)
	{
		return MAVLINK_RET_VAL_PARAM_SUCCESS;
	}
	else
	{
		return MAVLINK_RET_VAL_PARAM_WRITE_ERROR;
	}
}

int32_t writeParametersToStorage()
{
	//					// WORKING
	//					ObjectPersistenceData objper;
	//
	//					// Write all objects individually
	//					// for;;
	//					ObjectPersistenceGet(&objper);
	//
	//					ActuatorSettingsData settings;
	//					ActuatorSettingsGet(&settings);
	//
	//					objper.Selection = OBJECTPERSISTENCE_SELECTION_SINGLEOBJECT;
	//					objper.Operation = OBJECTPERSISTENCE_OPERATION_SAVE;
	//					objper.ObjectID = ACTUATORSETTINGS_OBJID;
	//					objper.InstanceID = 0;
	//					ObjectPersistenceSet(&objper);
	//					// END WORKING



	UAVObjHandle handle = ActuatorSettingsHandle();
	return UAVObjSave(handle, 0);
}

int32_t readParametersFromStorage()
{
	UAVObjHandle handle = ActuatorSettingsHandle();
	return UAVObjLoad(handle, 0);
}

/**
 * @}
 * @}
 */
