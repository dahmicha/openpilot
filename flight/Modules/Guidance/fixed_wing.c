/*
 * fixed_wing.c
 *
 *  Created on: 16.10.2011
 *      Author: user
 */

#include "fixed_wing.h"
#include "openpilot.h"
#include "globalpositiondesired.h"
#include "globalpositionactual.h"
#include "guidancesettings.h"
#include "stabilizationdesired.h"
#include "manualcontrolcommand.h"
//#include "CoordinateConversions.h"

static float bound(float val, float min, float max);

void updateFixedWingDesiredVelocity()
{
#ifdef PX2MODE
	GuidanceSettingsData guidanceSettings;
	GlobalPositionDesiredData globalPositionDesired;
	GlobalPositionActualData globalPosition;
	StabilizationDesiredData stabDesired;
//	StabilizationDesiredGet(&stabDesired);
	GlobalPositionDesiredGet(&globalPosition);
//	GlobalPositionActualGet(&globalPosition);
	GuidanceSettingsGet(&guidanceSettings);

	float throttleDesired;


	// Calculate desired throttle based on speed error
	// feed-forward pitch angle
	// FIXME Philippe
	float kp = 0.001f;
	float minThrottle = 0.2f; // Should be a setting, prevent motor from stopping to turn
	float maxThrottle = 1.0f; // Spec'ed to be 0-1 range

	// It doesn't make sense to just build the difference of the angles
	// this line should just explain what roughly needs to be done
	// regarding the involved data structures.
	throttleDesired = bound((globalPositionDesired.Latitude-globalPosition.Latitude)*kp, minThrottle, maxThrottle);


	if(guidanceSettings.ThrottleControl == GUIDANCESETTINGS_THROTTLECONTROL_FALSE) {
		// Override throttle with manual setting
		ManualControlCommandData manualControl;
		ManualControlCommandGet(&manualControl);
		stabDesired.Throttle = manualControl.Throttle;
	} else {
		stabDesired.Throttle = throttleDesired;
	}


	StabilizationDesiredSet(&stabDesired);
#endif
}

void updateFixedWingDesiredAttitude()
{
#ifdef PX2MODE
	GlobalPositionDesiredData globalPositionDesired;
	GlobalPositionActualData globalPosition;
	StabilizationDesiredData stabDesired;
	StabilizationDesiredGet(&stabDesired);
	GlobalPositionDesiredGet(&globalPosition);
	GlobalPositionActualGet(&globalPosition);

	float rollDesired = 0.0f;
	float pitchDesired = 0.0f;
	float yawDesired = 0.0f;

	// Calculate heading and altitude error
	// set new attitude
	// FIXME Philippe

	// LOITERING WAYPOINT
	if (globalPositionDesired.LoiterEnabled) {
		rollDesired = 123 * (globalPositionDesired.Latitude - globalPosition.Latitude);
	} else {
	// NORMAL NAVIGATION WAYPOINT
		rollDesired = 567 * (globalPositionDesired.Latitude - globalPosition.Latitude);
	}

	// Write values back in stabilization structure
	// the attitude control loop will try to reach
	// these setpoints
	stabDesired.Roll = rollDesired;
	stabDesired.Pitch = pitchDesired;
	stabDesired.Yaw = yawDesired;

	stabDesired.StabilizationMode[STABILIZATIONDESIRED_STABILIZATIONMODE_ROLL] = STABILIZATIONDESIRED_STABILIZATIONMODE_ATTITUDE;
	stabDesired.StabilizationMode[STABILIZATIONDESIRED_STABILIZATIONMODE_PITCH] = STABILIZATIONDESIRED_STABILIZATIONMODE_ATTITUDE;
	stabDesired.StabilizationMode[STABILIZATIONDESIRED_STABILIZATIONMODE_YAW] = STABILIZATIONDESIRED_STABILIZATIONMODE_ATTITUDE;

	// ENABLE ONCE FIXED WING IS REALLY ACTIVES
	//StabilizationDesiredSet(&stabDesired);
#else
	float a = bound(0.1f, 0.2f, 0.5f);
	a = a;
#endif
}





// LATER USE

void manualSetFixedWingDesiredVelocity()
{

}

/**
 * Bound input value between limits
 */
static float bound(float val, float min, float max)
{
	if (val < min) {
		val = min;
	} else if (val > max) {
		val = max;
	}
	return val;
}
