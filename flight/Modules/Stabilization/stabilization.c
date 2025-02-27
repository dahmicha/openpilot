/**
 ******************************************************************************
 * @addtogroup OpenPilotModules OpenPilot Modules
 * @{
 * @addtogroup StabilizationModule Stabilization Module
 * @brief Stabilization PID loops in an airframe type independent manner
 * @note This object updates the @ref ActuatorDesired "Actuator Desired" based on the
 * PID loops on the @ref AttitudeDesired "Attitude Desired" and @ref AttitudeActual "Attitude Actual"
 * @{
 *
 * @file       stabilization.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @brief      Attitude stabilization module.
 *
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
#include "stabilization.h"
#include "stabilizationsettings.h"
#include "actuatordesired.h"
#include "ratedesired.h"
#include "stabilizationdesired.h"
#include "attitudeactual.h"
#ifndef PX2MODE
		// Attitude should not be filtered by a controller
		// instead, the attitude filter should provide
		// speed estimates
#include "attituderaw.h"
#endif
#include "flightstatus.h"
#include "manualcontrol.h" // Just to get a macro
#include "CoordinateConversions.h"
//#include "mavlink_debug.h"

// Private constants
#define MAX_QUEUE_SIZE 1

#if defined(PIOS_STABILIZATION_STACK_SIZE)
#define STACK_SIZE_BYTES PIOS_STABILIZATION_STACK_SIZE
#else
#define STACK_SIZE_BYTES 724
#endif

#define TASK_PRIORITY (tskIDLE_PRIORITY+4)
#define FAILSAFE_TIMEOUT_MS 30

enum {PID_RATE_ROLL, PID_RATE_PITCH, PID_RATE_YAW, PID_ROLL, PID_PITCH, PID_YAW, PID_MAX};

enum {ROLL,PITCH,YAW,MAX_AXES};


// Private types
typedef struct {
	float p;
	float i;
	float d;
	float iLim;
	float iAccumulator;
	float lastErr;
} pid_type;

// Private variables
static xTaskHandle taskHandle;
static StabilizationSettingsData settings;
static xQueueHandle queue;
float dT = 1;
float gyro_alpha = 0;
float gyro_filtered[3] = {0,0,0};
float axis_lock_accum[3] = {0,0,0};
uint8_t max_axis_lock = 0;
uint8_t max_axislock_rate = 0;
float weak_leveling_kp = 0;
uint8_t weak_leveling_max = 0;
bool lowThrottleZeroIntegral;

pid_type pids[PID_MAX];

// Private functions
static void stabilizationTask(void* parameters);
static float ApplyPid(pid_type * pid, const float err);
static float bound(float val);
static void ZeroPids(void);
static void SettingsUpdatedCb(UAVObjEvent * ev);

/**
 * Module initialization
 */
int32_t StabilizationStart()
{
	// Initialize variables

	// Start main task
	xTaskCreate(stabilizationTask, (signed char*)"Stabilization", STACK_SIZE_BYTES/4, NULL, TASK_PRIORITY, &taskHandle);
	TaskMonitorAdd(TASKINFO_RUNNING_STABILIZATION, taskHandle);
	PIOS_WDG_RegisterFlag(PIOS_WDG_STABILIZATION);
	return 0;
}

/**
 * Module initialization
 */
int32_t StabilizationInitialize()
{
	// Initialize variables
	StabilizationSettingsInitialize();
	ActuatorDesiredInitialize();
#if defined(DIAGNOSTICS)
	RateDesiredInitialize();
#endif

	// Create object queue
	queue = xQueueCreate(MAX_QUEUE_SIZE, sizeof(UAVObjEvent));

	// Listen for updates.
#ifdef PX2MODE
	//	 Attitude should not be filtered by a controller
	//	 instead, the attitude filter should provide
	//	 speed estimates
	AttitudeActualConnectQueue(queue);
#else
	AttitudeRawConnectQueue(queue);
#endif

	StabilizationSettingsConnectCallback(SettingsUpdatedCb);
	SettingsUpdatedCb(StabilizationSettingsHandle());
	// Start main task

	return 0;
}

MODULE_INITCALL(StabilizationInitialize, StabilizationStart)

/**
 * Module task
 */
static void stabilizationTask(void* parameters)
{
	portTickType lastSysTime;
	portTickType thisSysTime;
	UAVObjEvent ev;


	ActuatorDesiredData actuatorDesired;
	StabilizationDesiredData stabDesired;
	RateDesiredData rateDesired;
	AttitudeActualData attitudeActual;
#ifndef PX2MODE
		// Attitude should not be filtered by a controller
		// instead, the attitude filter should provide
		// speed estimates
	AttitudeRawData attitudeRaw;
#endif
	FlightStatusData flightStatus;

	SettingsUpdatedCb((UAVObjEvent *) NULL);

	// Main task loop
	lastSysTime = xTaskGetTickCount();
	ZeroPids();
	while(1) {
		PIOS_WDG_UpdateFlag(PIOS_WDG_STABILIZATION);

		// Wait until the AttitudeRaw object is updated, if a timeout then go to failsafe
		if ( xQueueReceive(queue, &ev, FAILSAFE_TIMEOUT_MS / portTICK_RATE_MS) != pdTRUE )
		{
			AlarmsSet(SYSTEMALARMS_ALARM_STABILIZATION,SYSTEMALARMS_ALARM_WARNING);
			continue;
		}

		// Check how long since last update
		thisSysTime = xTaskGetTickCount();
		if(thisSysTime > lastSysTime) // reuse dt in case of wraparound
			dT = (thisSysTime - lastSysTime) / portTICK_RATE_MS / 1000.0f;
		lastSysTime = thisSysTime;

		FlightStatusGet(&flightStatus);
		StabilizationDesiredGet(&stabDesired);
		AttitudeActualGet(&attitudeActual);

#ifndef PX2MODE
		// Attitude should not be filtered by a controller
		// instead, the attitude filter should provide
		// speed estimates
		AttitudeRawGet(&attitudeRaw);
#endif

#if defined(DIAGNOSTICS)
		RateDesiredGet(&rateDesired);
#endif

#if defined(PIOS_QUATERNION_STABILIZATION)
		// Quaternion calculation of error in each axis.  Uses more memory.
		float rpy_desired[3];
		float q_desired[4];
		float q_error[4];
		float local_error[3];

		// Essentially zero errors for anything in rate or none
		if(stabDesired.StabilizationMode[STABILIZATIONDESIRED_STABILIZATIONMODE_ROLL] == STABILIZATIONDESIRED_STABILIZATIONMODE_ATTITUDE)
			rpy_desired[0] = stabDesired.Roll;
		else
			rpy_desired[0] = attitudeActual.Roll;

		if(stabDesired.StabilizationMode[STABILIZATIONDESIRED_STABILIZATIONMODE_PITCH] == STABILIZATIONDESIRED_STABILIZATIONMODE_ATTITUDE)
			rpy_desired[1] = stabDesired.Pitch;
		else
			rpy_desired[1] = attitudeActual.Pitch;

		if(stabDesired.StabilizationMode[STABILIZATIONDESIRED_STABILIZATIONMODE_YAW] == STABILIZATIONDESIRED_STABILIZATIONMODE_ATTITUDE)
			rpy_desired[2] = stabDesired.Yaw;
		else
			rpy_desired[2] = attitudeActual.Yaw;

		RPY2Quaternion(rpy_desired, q_desired);
		quat_inverse(q_desired);
		quat_mult(q_desired, &attitudeActual.q1, q_error);
		quat_inverse(q_error);
		Quaternion2RPY(q_error, local_error);

#else
		// Simpler algorithm for CC, less memory
		float local_error[3] = {stabDesired.Roll - attitudeActual.Roll,
			stabDesired.Pitch - attitudeActual.Pitch,
			stabDesired.Yaw - attitudeActual.Yaw};
		local_error[2] = fmod(local_error[2] + 180, 360) - 180;
#endif

#ifdef PX2MODE
		gyro_filtered[0] = attitudeActual.RollRate;
		gyro_filtered[1] = attitudeActual.PitchRate;
		gyro_filtered[2] = attitudeActual.YawRate;
#else
		// Attitude should not be filtered by a controller
		// instead, the attitude filter should provide
		// speed estimates
		for(uint8_t i = 0; i < MAX_AXES; i++) {
			gyro_filtered[i] = gyro_filtered[i] * gyro_alpha + attitudeRaw.gyros[i] * (1 - gyro_alpha);
		}
#endif

//		debug_vect("des_e", stabDesired.Roll, stabDesired.Pitch, stabDesired.Yaw);
//		debug_vect("pos_e", local_error[0], local_error[1], local_error[2]);
//		debug_vect("rate_e", gyro_filtered[0], gyro_filtered[1], gyro_filtered[2]);


		float *attitudeDesiredAxis = &stabDesired.Roll;
		float *actuatorDesiredAxis = &actuatorDesired.Roll;
		float *rateDesiredAxis = &rateDesired.Roll;

		//Calculate desired rate
		for(uint8_t i=0; i< MAX_AXES; i++)
		{
			switch(stabDesired.StabilizationMode[i])
			{
				case STABILIZATIONDESIRED_STABILIZATIONMODE_RATE:
					rateDesiredAxis[i] = attitudeDesiredAxis[i];
					axis_lock_accum[i] = 0;
					break;

				case STABILIZATIONDESIRED_STABILIZATIONMODE_WEAKLEVELING:
				{
					float weak_leveling = local_error[i] * weak_leveling_kp;

					if(weak_leveling > weak_leveling_max)
						weak_leveling = weak_leveling_max;
					if(weak_leveling < -weak_leveling_max)
						weak_leveling = -weak_leveling_max;

					rateDesiredAxis[i] = attitudeDesiredAxis[i] + weak_leveling;

					axis_lock_accum[i] = 0;
					break;
				}
				case STABILIZATIONDESIRED_STABILIZATIONMODE_ATTITUDE:
					rateDesiredAxis[i] = ApplyPid(&pids[PID_ROLL + i], local_error[i]);
					axis_lock_accum[i] = 0;
					break;

				case STABILIZATIONDESIRED_STABILIZATIONMODE_AXISLOCK:
					if(fabs(attitudeDesiredAxis[i]) > max_axislock_rate) {
						// While getting strong commands act like rate mode
						rateDesiredAxis[i] = attitudeDesiredAxis[i];
						axis_lock_accum[i] = 0;
					} else {
						// For weaker commands or no command simply attitude lock (almost) on no gyro change
						axis_lock_accum[i] += (attitudeDesiredAxis[i] - gyro_filtered[i]) * dT;
						if(axis_lock_accum[i] > max_axis_lock)
							axis_lock_accum[i] = max_axis_lock;
						else if(axis_lock_accum[i] < -max_axis_lock)
							axis_lock_accum[i] = -max_axis_lock;

						rateDesiredAxis[i] = ApplyPid(&pids[PID_ROLL + i], axis_lock_accum[i]);
					}
					break;
			}
		}

		uint8_t shouldUpdate = 1;
#if defined(DIAGNOSTICS)
		RateDesiredSet(&rateDesired);
#endif
		ActuatorDesiredGet(&actuatorDesired);
		//Calculate desired command
		for(int8_t ct=0; ct< MAX_AXES; ct++)
		{
			if(rateDesiredAxis[ct] > settings.MaximumRate[ct])
				rateDesiredAxis[ct] = settings.MaximumRate[ct];
			else if(rateDesiredAxis[ct] < -settings.MaximumRate[ct])
				rateDesiredAxis[ct] = -settings.MaximumRate[ct];

			switch(stabDesired.StabilizationMode[ct])
			{
				case STABILIZATIONDESIRED_STABILIZATIONMODE_RATE:
				case STABILIZATIONDESIRED_STABILIZATIONMODE_ATTITUDE:
				case STABILIZATIONDESIRED_STABILIZATIONMODE_AXISLOCK:
				case STABILIZATIONDESIRED_STABILIZATIONMODE_WEAKLEVELING:
				{
					float command = ApplyPid(&pids[PID_RATE_ROLL + ct],  rateDesiredAxis[ct] - gyro_filtered[ct]);
					actuatorDesiredAxis[ct] = bound(command);
					break;
				}
				case STABILIZATIONDESIRED_STABILIZATIONMODE_NONE:
					switch (ct)
				{
					case ROLL:
						actuatorDesiredAxis[ct] = bound(attitudeDesiredAxis[ct]);
						shouldUpdate = 1;
						break;
					case PITCH:
						actuatorDesiredAxis[ct] = bound(attitudeDesiredAxis[ct]);
						shouldUpdate = 1;
						break;
					case YAW:
						actuatorDesiredAxis[ct] = bound(attitudeDesiredAxis[ct]);
						shouldUpdate = 1;
						break;
				}
					break;

			}
		}

		// Save dT
		actuatorDesired.UpdateTime = dT * 1000;

		if(PARSE_FLIGHT_MODE(flightStatus.FlightMode) == FLIGHTMODE_MANUAL)
			shouldUpdate = 0;

		if(shouldUpdate)
		{
			actuatorDesired.Throttle = stabDesired.Throttle;
			if(dT > 15)
				actuatorDesired.NumLongUpdates++;
			ActuatorDesiredSet(&actuatorDesired);
		}

		if(flightStatus.Armed != FLIGHTSTATUS_ARMED_ARMED ||
		   (lowThrottleZeroIntegral && stabDesired.Throttle < 0) ||
		   !shouldUpdate)
		{
			ZeroPids();
		}


		// Clear alarms
		AlarmsClear(SYSTEMALARMS_ALARM_STABILIZATION);
	}
}

float ApplyPid(pid_type * pid, const float err)
{
	float diff = (err - pid->lastErr);
	pid->lastErr = err;

	// Scale up accumulator by 1000 while computing to avoid losing precision
	pid->iAccumulator += err * (pid->i * dT * 1000);
	if(pid->iAccumulator > (pid->iLim * 1000)) {
		pid->iAccumulator = pid->iLim * 1000;
	} else if (pid->iAccumulator < -(pid->iLim * 1000)) {
		pid->iAccumulator = -pid->iLim * 1000;
	}
	return ((err * pid->p) + pid->iAccumulator / 1000 + (diff * pid->d / dT));
}


static void ZeroPids(void)
{
	for(int8_t ct = 0; ct < PID_MAX; ct++) {
		pids[ct].iAccumulator = 0;
		pids[ct].lastErr = 0;
	}
	for(uint8_t i = 0; i < 3; i++)
		axis_lock_accum[i] = 0;
}


/**
 * Bound input value between limits
 */
static float bound(float val)
{
	if(val < -1) {
		val = -1;
	} else if(val > 1) {
		val = 1;
	}
	return val;
}


static void SettingsUpdatedCb(UAVObjEvent * ev)
{
	memset(pids,0,sizeof (pid_type) * PID_MAX);
	StabilizationSettingsGet(&settings);

	// Set the roll rate PID constants
	pids[0].p = settings.RollRatePID[STABILIZATIONSETTINGS_ROLLRATEPID_KP];
	pids[0].i = settings.RollRatePID[STABILIZATIONSETTINGS_ROLLRATEPID_KI];
	pids[0].d = settings.RollRatePID[STABILIZATIONSETTINGS_ROLLRATEPID_KD];
	pids[0].iLim = settings.RollRatePID[STABILIZATIONSETTINGS_ROLLRATEPID_ILIMIT];

	// Set the pitch rate PID constants
	pids[1].p = settings.PitchRatePID[STABILIZATIONSETTINGS_PITCHRATEPID_KP];
	pids[1].i = settings.PitchRatePID[STABILIZATIONSETTINGS_PITCHRATEPID_KI];
	pids[1].d = settings.PitchRatePID[STABILIZATIONSETTINGS_PITCHRATEPID_KD];
	pids[1].iLim = settings.PitchRatePID[STABILIZATIONSETTINGS_PITCHRATEPID_ILIMIT];

	// Set the yaw rate PID constants
	pids[2].p = settings.YawRatePID[STABILIZATIONSETTINGS_YAWRATEPID_KP];
	pids[2].i = settings.YawRatePID[STABILIZATIONSETTINGS_YAWRATEPID_KI];
	pids[2].d = settings.YawRatePID[STABILIZATIONSETTINGS_YAWRATEPID_KD];
	pids[2].iLim = settings.YawRatePID[STABILIZATIONSETTINGS_YAWRATEPID_ILIMIT];

	// Set the roll attitude PI constants
	pids[3].p = settings.RollPI[STABILIZATIONSETTINGS_ROLLPI_KP];
	pids[3].i = settings.RollPI[STABILIZATIONSETTINGS_ROLLPI_KI];
	pids[3].iLim = settings.RollPI[STABILIZATIONSETTINGS_ROLLPI_ILIMIT];

	// Set the pitch attitude PI constants
	pids[4].p = settings.PitchPI[STABILIZATIONSETTINGS_PITCHPI_KP];
	pids[4].i = settings.PitchPI[STABILIZATIONSETTINGS_PITCHPI_KI];
	pids[4].iLim = settings.PitchPI[STABILIZATIONSETTINGS_PITCHPI_ILIMIT];

	// Set the yaw attitude PI constants
	pids[5].p = settings.YawPI[STABILIZATIONSETTINGS_YAWPI_KP];
	pids[5].i = settings.YawPI[STABILIZATIONSETTINGS_YAWPI_KI];
	pids[5].iLim = settings.YawPI[STABILIZATIONSETTINGS_YAWPI_ILIMIT];

	// Maximum deviation to accumulate for axis lock
	max_axis_lock = settings.MaxAxisLock;
	max_axislock_rate = settings.MaxAxisLockRate;

	// Settings for weak leveling
	weak_leveling_kp = settings.WeakLevelingKp;
	weak_leveling_max = settings.MaxWeakLevelingRate;

	// Whether to zero the PID integrals while throttle is low
	lowThrottleZeroIntegral = settings.LowThrottleZeroIntegral == STABILIZATIONSETTINGS_LOWTHROTTLEZEROINTEGRAL_TRUE;

	// The dT has some jitter iteration to iteration that we don't want to
	// make thie result unpredictable.  Still, it's nicer to specify the constant
	// based on a time (in ms) rather than a fixed multiplier.  The error between
	// update rates on OP (~300 Hz) and CC (~475 Hz) is negligible for this
	// calculation
	const float fakeDt = 0.0025f;
	if(settings.GyroTau < 0.0001f)
		gyro_alpha = 0;   // not trusting this to resolve to 0
	else
		gyro_alpha = exp(-fakeDt  / settings.GyroTau);
}


/**
 * @}
 * @}
 */
