// This file is part of MatrixPilot.
//
//    http://code.google.com/p/gentlenav/
//
// Copyright 2009-2011 MatrixPilot Team
// See the AUTHORS.TXT file for a list of authors of MatrixPilot.
//
// MatrixPilot is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// MatrixPilot is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with MatrixPilot.  If not, see <http://www.gnu.org/licenses/>.


#include "defines.h"
#include "navigate.h"
#include "behaviour.h"
#include "servoPrepare.h"
#include "config.h"
#include "states.h"
#include "altitudeCntrl.h"
#include "alt_agl.h"
#include "../libDCM/rmat.h"
#include "../libDCM/gpsData.h"
#include "../libDCM/estWind.h"
#include "../libDCM/deadReckoning.h"
#include "../libUDB/servoOut.h"
#include "options_mavlink.h"

#if (ALTITUDE_GAINS_VARIABLE != 1)

union longww throttleFiltered = { 0 };

#define THROTTLEFILTSHIFT   12

#define THROTTLE_DEADBAND            150
#define ELEVATOR_DEADBAND             15
#define MAX_ELEV_OFFSET_FBW          512
#define MAXTHROTTLE         (2.0*SERVORANGE*altit.AltHoldThrottleMax)
#define FIXED_WP_THROTTLE   (2.0*SERVORANGE*RACING_MODE_WP_THROTTLE)

#define THROTTLEHEIGHTGAIN  (((altit.AltHoldThrottleMax \
                            - altit.AltHoldThrottleMin)*2.0 \
                            *SERVORANGE)/(altit.HeightMargin*2.0))

#define PITCHATMAX          (altit.AltHoldPitchMax*(RMAX/57.3))
#define PITCHATMIN          (altit.AltHoldPitchMin*(RMAX/57.3))
#define PITCHATZERO         (altit.AltHoldPitchHigh*(RMAX/57.3))
#define PITCHHEIGHTGAIN     ((PITCHATMAX - PITCHATMIN) / \
                                (altit.HeightMargin*2.0))

#define HEIGHTTHROTTLEGAIN  ((1.5*(altit.HeightTargetMax-altit.HeightTargetMin) * \
                                1024.0) / SERVORANGE)

static void normalAltitudeCntrl(void);
static void manualThrottle(int16_t throttleIn);
static void hoverAltitudeCntrl(void);

int16_t pitchAltitudeAdjust = 0;
int16_t elevator_override = 0;
boolean filterManual = false;
union longww desiredHeight32;
union longww desiredHeightAGL32;
#if (USE_RANGER_INPUT != 0)
static int32_t terrain_height_change;
static int32_t terrain_height;
static int32_t previous_terrain_height;
#endif

// Variables required for mavlink.  Used in AltitudeCntrlVariable and airspeedCntrl
int16_t height_target_min;
int16_t height_target_max;
int16_t height_margin;
fractional alt_hold_throttle_min;
fractional alt_hold_throttle_max;
int16_t alt_hold_pitch_min;
int16_t alt_hold_pitch_max;
int16_t alt_hold_pitch_high;
int16_t rtl_pitch_down;
int16_t desiredSpeed;

void init_altitudeCntrl(void)
{
	height_target_min     = altit.HeightTargetMin;
	height_target_max     = altit.HeightTargetMax;
	height_margin         = altit.HeightMargin;
	alt_hold_throttle_min = altit.AltHoldThrottleMin * RMAX;
	alt_hold_throttle_max = altit.AltHoldThrottleMax * RMAX;
	alt_hold_pitch_min    = altit.AltHoldPitchMin;
	alt_hold_pitch_max    = altit.AltHoldPitchMax;
	alt_hold_pitch_high   = altit.AltHoldPitchHigh;
	rtl_pitch_down        = gains.RtlPitchDown;
	desiredSpeed          = altit.DesiredSpeed * 10; // Stored in 10ths of meters per second
}

void save_altitudeCntrl(void)
{
//	altit.YawKDAileron = yawkdail / (SCALEGYRO*RMAX);
	altit.HeightTargetMax = height_target_max;
	altit.HeightTargetMin = height_target_min;
	altit.AltHoldThrottleMin = alt_hold_throttle_min / RMAX;
	altit.AltHoldThrottleMax = alt_hold_throttle_max / RMAX;
	altit.AltHoldPitchMin = alt_hold_pitch_min;
	altit.AltHoldPitchMax = alt_hold_pitch_max;
	altit.AltHoldPitchHigh = alt_hold_pitch_high;
}

#if (SPEED_CONTROL == 1)  // speed control loop

static int32_t excess_energy_height(void) // computes (1/2gravity)*(actual_speed^2 - desired_speed^2)
{
	int16_t speedAccum = 6 * desiredSpeed;
	int32_t equivalent_energy_air_speed = -(__builtin_mulss(speedAccum, speedAccum));
	int32_t equivalent_energy_ground_speed = equivalent_energy_air_speed;
	int16_t speed_component;
	union longww accum;

	speed_component = IMUvelocityx._.W1 - estimatedWind[0];
	accum.WW = __builtin_mulsu(speed_component, 37877);
	equivalent_energy_air_speed += __builtin_mulss(accum._.W1, accum._.W1);

	speed_component = IMUvelocityy._.W1 - estimatedWind[1];
	accum.WW = __builtin_mulsu(speed_component, 37877);
	equivalent_energy_air_speed += __builtin_mulss(accum._.W1, accum._.W1);

	speed_component = IMUvelocityz._.W1 - estimatedWind[2];
	accum.WW = __builtin_mulsu(speed_component, 37877);
	equivalent_energy_air_speed += __builtin_mulss(accum._.W1, accum._.W1);

	// if we are going forward, add the energy, otherwise, subract it
	accum.WW = __builtin_mulsu(forward_ground_speed, 37877);
	if (forward_ground_speed > 0)
	{
		equivalent_energy_ground_speed += __builtin_mulss(accum._.W1, accum._.W1);
	}
	else
	{
		equivalent_energy_ground_speed -= __builtin_mulss(accum._.W1, accum._.W1);
	}

	// return the smaller of the energies of ground and air speed
	// to keep both of them from getting too small
	if (equivalent_energy_ground_speed < equivalent_energy_air_speed)
	{
		return equivalent_energy_ground_speed;
	}
	else
	{
		return equivalent_energy_air_speed;
	}
}
#else

static int32_t excess_energy_height(void)
{
	return 0;
}

#if (USE_MAVLINK == 1)
// Initialize to the value from options.h.  Allow updating this value from LOGO/MavLink/etc.
// Stored in 10ths of meters per second
int16_t desiredSpeed = (DESIRED_SPEED*10);
#endif // (USE_MAVLINK == 1)

#endif //(SPEED_CONTROL == 1)  // speed control loop

void altitudeCntrl(void)
{
#if (USE_RANGER_INPUT != 0)
	calculate_height_above_ground_level();
#endif
	if (canStabilizeHover() && current_orientation == F_HOVER)
	{
		hoverAltitudeCntrl();
	}
	else
	{
		normalAltitudeCntrl();
	}
}

static void set_throttle_control(int16_t throttle)
{
	int16_t throttleIn;
	int16_t temp;

	if (state_flags._.altitude_hold_throttle || state_flags._.altitude_hold_pitch || filterManual)
	{
		if (udb_flags._.radio_on == 1)
		{
			throttleIn = udb_pwIn[THROTTLE_INPUT_CHANNEL];
		}
		else
		{
			throttleIn = udb_pwTrim[THROTTLE_INPUT_CHANNEL];
		}
		temp = throttleIn + REVERSE_IF_NEEDED(THROTTLE_CHANNEL_REVERSED, throttle);
		if (THROTTLE_CHANNEL_REVERSED)
		{
			if (temp > udb_pwTrim[THROTTLE_INPUT_CHANNEL]) throttle = throttleIn - udb_pwTrim[THROTTLE_INPUT_CHANNEL];
		}
		else
		{
			if (temp < udb_pwTrim[THROTTLE_INPUT_CHANNEL]) throttle = udb_pwTrim[THROTTLE_INPUT_CHANNEL] - throttleIn;
		}
		throttle_control = throttle;
	}
	else
	{
		throttle_control = 0;
	}
}

void setTargetAltitude(int16_t targetAlt)
{
	desiredHeight32._.W1 = targetAlt;
}

static int16_t get_elevInOffset(void)
{
    int16_t elevInOffset;
    if (udb_flags._.radio_on == 1)
	{
		elevInOffset = udb_pwIn[ELEVATOR_INPUT_CHANNEL] - udb_pwTrim[ELEVATOR_INPUT_CHANNEL];
	}
	else
	{
		elevInOffset = 0;
	}
	if (ELEVATOR_CHANNEL_REVERSED)
	{
		elevInOffset = -elevInOffset;
	}

    if ((elevInOffset > (int16_t) -ELEVATOR_DEADBAND) && (elevInOffset < (int16_t) ELEVATOR_DEADBAND))
    {
        elevInOffset = 0;
    }
    else 
    {
        if (elevInOffset < 0)
        {
            elevInOffset = elevInOffset + (int16_t) ELEVATOR_DEADBAND;
        }
        else if (elevInOffset > 0)
        {
            elevInOffset = elevInOffset - (int16_t) ELEVATOR_DEADBAND;
        }
        else
        {
            elevInOffset = 0;
        }
    }
    return(elevInOffset);
}

static int32_t incremental_height_from_elevator_control(int16_t elevInOffset)
{
    int32_t height_increment32;
    if (elevInOffset == 0)
    {
        height_increment32 = 0;
    }
    else
    {
        if ( elevInOffset > (int16_t) MAX_ELEV_OFFSET_FBW )
        {
            elevInOffset = (int16_t) MAX_ELEV_OFFSET_FBW;
        }
        if ( elevInOffset < (int16_t) -MAX_ELEV_OFFSET_FBW ) 
        {
            elevInOffset = (int16_t) -MAX_ELEV_OFFSET_FBW;
        }
        // 65536 represents 1.0
        // 1024 represents a full offset (normally 1000))
        // Usually FBW is using 1/2 the offset i.e. 512
        // 65536/512 * 128  creates a fraction of 1.0
        height_increment32 = __builtin_mulus((uint16_t)((REFERENCE_SPEED) * 128), elevInOffset) ;
    }  
    return(height_increment32); //  elevator_override is external variable  
}

int16_t calculate_elevator_override(int16_t elevInOffset)
{ 
    if (elevInOffset == 0)
    {
        elevator_override = 0;
    }
    else
    {
        if ( elevInOffset > (int16_t) MAX_ELEV_OFFSET_FBW )
        {
            elevator_override = elevInOffset -  (int16_t) MAX_ELEV_OFFSET_FBW;
        }
        if ( elevInOffset < (int16_t) -MAX_ELEV_OFFSET_FBW ) 
        {
            elevator_override = elevInOffset + (int16_t) MAX_ELEV_OFFSET_FBW;
        }
    }
    return(elevator_override);
}


union longww calculate_throttle(int16_t throttleInOffset,int32_t speed_height )
{
    union longww throttleAccum;
    union longww heightError = { 0 };
    if (throttleInOffset < (int16_t)(THROTTLE_DEADBAND) && udb_flags._.radio_on)
    {
        throttleAccum.WW  = 0;
    }
    else
    { 
#if (USE_RANGER_INPUT != 0)
        if ( state_flags._.terrain_follow == 0 )
#endif
        {
            heightError.WW = - desiredHeight32.WW ;
            heightError.WW = (heightError.WW + IMUlocationz.WW + speed_height) >> 13;
        }
#if (USE_RANGER_INPUT != 0)
        else
        { 
            heightError.WW = - desiredHeightAGL32.WW ;
            heightError.WW = (heightError.WW + height_above_ground_meters.WW + speed_height) >> 13;
        }
#endif
        if (heightError._.W0 < (-(int16_t)(altit.HeightMargin*8.0)))
        {
            throttleAccum.WW = (int16_t)(MAXTHROTTLE);
        }
        else if (heightError._.W0 > (int16_t)(altit.HeightMargin*8.0))
        {
            throttleAccum.WW = 0; // Note this is not MINTHROTTLE 
        }
        else
        {
            throttleAccum.WW = (int16_t)(MAXTHROTTLE) + (__builtin_mulss((int16_t)(THROTTLEHEIGHTGAIN), (-heightError._.W0 - (int16_t)(altit.HeightMargin*8.0))) >> 3);
            if (throttleAccum.WW > (int16_t)(MAXTHROTTLE))throttleAccum.WW = (int16_t)(MAXTHROTTLE);
        }
        if (settings._.RacingMode == 1)
        {
            if (state_flags._.GPS_steering)
            {
                throttleAccum.WW = (int32_t)(FIXED_WP_THROTTLE);
            }
        }
    }
    return(throttleAccum);
}

#if (USE_RANGER_INPUT != 0)

//	Implementation of state machine to transition between normal flight and terrain follow
static void ent_desired_height_control_S(void);
static void desired_height_control_S(void);
static void ent_terrain_follow_height_control_S(void);
static void terrain_follow_height_control_S(void);

static void (*manage_TerrainFollow_transition_S)(void) = &ent_desired_height_control_S;

static void ent_desired_height_control_S(void)
{
    state_flags._.terrain_follow = 0;
    desiredHeight32.WW = IMUlocationz.WW;
    manage_TerrainFollow_transition_S = &desired_height_control_S;
}

static void desired_height_control_S(void)
{
    if (height_above_ground_cm <  HEIGHT_AGL_TO_START_TERRAIN_FOLLOWING)
    {
        ent_terrain_follow_height_control_S();
    }
    else
    {
        state_flags._.terrain_follow = 0;
    }
}

static void ent_terrain_follow_height_control_S(void)
{
    state_flags._.terrain_follow = 1;
    desiredHeightAGL32.WW = height_above_ground_meters.WW;
    terrain_height = IMUlocationz.WW  - height_above_ground_meters.WW;
    terrain_height_change = 0;
    previous_terrain_height = terrain_height;
    manage_TerrainFollow_transition_S = &terrain_follow_height_control_S;  
}

static void terrain_follow_height_control_S(void)
{
    if (height_above_ground_cm >  HEIGHT_AGL_TO_STOP_TERRAIN_FOLLOWING)
    {
        ent_desired_height_control_S();
    }
    else
    {
        state_flags._.terrain_follow = 1;
        terrain_height = IMUlocationz.WW  - height_above_ground_meters.WW;
        terrain_height_change = terrain_height - previous_terrain_height;
        previous_terrain_height = terrain_height;  
    }
}

#endif //(USE_RANGER_INPUT != 0)

void calculate_desiredHeight(int32_t desiredHeight_increment,int16_t throttleInOffset)
{
        static boolean previous_height_increment_was_zero = true;
        
        if (desiredHeight_increment == 0)
                {
                    if (previous_height_increment_was_zero == false)
                    // Pilot just put the elevator control to neutral
                    {
                        desiredHeight32.WW = IMUlocationz.WW;
#if (USE_RANGER_INPUT != 0)
                        desiredHeightAGL32.WW = height_above_ground_meters.WW +
                                terrain_height_change;
#endif
                        previous_height_increment_was_zero = true;
                    }   
                }
                else // Pilot requests change in height
                {
                    desiredHeight32.WW = IMUlocationz.WW + desiredHeight_increment;
#if (USE_RANGER_INPUT != 0)
                    desiredHeightAGL32.WW = height_above_ground_meters.WW + \
                            + terrain_height_change + desiredHeight_increment;
#endif
                    previous_height_increment_was_zero = false;
                }   
}


static void normalAltitudeCntrl(void)
{
	union longww throttleAccum;
	union longww pitchAccum;
    union longww heightError = { 0 };
	int16_t throttleIn;
	int16_t throttleInOffset;
    int32_t desiredHeight_increment;

	int32_t speed_height;
    int16_t elevInOffset;
    
    elevator_override = 0;
	speed_height = excess_energy_height(); // equivalent height of the airspeed
	if (udb_flags._.radio_on == 1)
	{
		throttleIn = udb_pwIn[THROTTLE_INPUT_CHANNEL];
		// keep the In and Trim throttle values within 2000-4000 to account for
		// Spektrum receivers using failsafe values below 2000.
		throttleInOffset = udb_servo_pulsesat(udb_pwIn[THROTTLE_INPUT_CHANNEL]) - udb_servo_pulsesat(udb_pwTrim[THROTTLE_INPUT_CHANNEL]);
	}
	else
	{
		throttleIn = udb_pwTrim[THROTTLE_INPUT_CHANNEL];
		throttleInOffset = 0;
	}
    if (THROTTLE_CHANNEL_REVERSED)
	{
		throttleInOffset = -throttleInOffset;
	}
    
	if (state_flags._.altitude_hold_throttle || state_flags._.altitude_hold_pitch)
	{
		if (state_flags._.GPS_steering)
		{
			navigate_desired_height();
		}
		else
		{
			if (settings._.AltitudeholdStabilized == AH_PITCH_ONLY)
			{
				// In stabilized mode using pitch-only altitude hold, use desiredHeight as
				// set from the state machine upon entering stabilized mode in ent_stabilizedS().
			}
			else if (settings._.AltitudeholdStabilized == AH_FULL)
			{
				// In stabilized mode using full altitude hold, use the throttle stick value to determine desiredHeight,
				desiredHeight32._.W1 = ((__builtin_mulss((int16_t)(HEIGHTTHROTTLEGAIN), throttleInOffset - ((int16_t)(THROTTLE_DEADBAND)))) >> 11)
				                + (int16_t)(altit.HeightTargetMin);
            }
            else if (settings._.AltitudeholdStabilized == AH_FULL_ELEV)
			{
#if (USE_RANGER_INPUT > 0)
                if (settings._.AllowTerrainFollow == 1)
                {
                    (*manage_TerrainFollow_transition_S)();
                }
#endif
                // Use elevator stick to control desired height.
                elevInOffset = get_elevInOffset();
                desiredHeight_increment = incremental_height_from_elevator_control(elevInOffset);
                elevator_override = calculate_elevator_override(elevInOffset);        
                calculate_desiredHeight(desiredHeight_increment, throttleInOffset);
            }
            if (desiredHeight32._.W1 < (int16_t)(altit.HeightTargetMin))
                    desiredHeight32._.W1 = (int16_t)(altit.HeightTargetMin);
            if (desiredHeight32._.W1 > (int16_t)(altit.HeightTargetMax)) 
                    desiredHeight32._.W1 = (int16_t)(altit.HeightTargetMax);
            if (desiredHeightAGL32.WW < MINIMUM_TERRAIN_FOLLOWING_HEIGHT)  
                    desiredHeightAGL32.WW = MINIMUM_TERRAIN_FOLLOWING_HEIGHT ; // Fractional, ._.W1 is in meters.
		}
                    
        // Section that calculates Throttle Adjustment
        throttleAccum = calculate_throttle(throttleInOffset, speed_height);
        
        // Section that Calculates pitchAltitudeAdjust
#if (USE_RANGER_INPUT != 0)
        if ( state_flags._.terrain_follow == 0 )
#endif
        {
            heightError.WW = - desiredHeight32.WW ;
            heightError.WW = (heightError.WW + IMUlocationz.WW) >> 13;
        }
#if (USE_RANGER_INPUT != 0)
        else
        {
            heightError.WW = - desiredHeightAGL32.WW ;
            heightError.WW = (heightError.WW + height_above_ground_meters.WW) >> 13;
        }
#endif
        if (heightError._.W0 < (- (int16_t)(altit.HeightMargin*8.0)))
        {
            pitchAltitudeAdjust = (int16_t)(PITCHATMAX);
        }
        else if (heightError._.W0 > (int16_t)(altit.HeightMargin*8.0))
        {
            pitchAltitudeAdjust = (int16_t)(PITCHATZERO);
        }
        else
        {
            pitchAccum.WW = __builtin_mulss((int16_t)(PITCHHEIGHTGAIN), - heightError._.W0 - (int16_t)(altit.HeightMargin*8.0)) >> 3;
            pitchAltitudeAdjust = (int16_t)(PITCHATMAX) + pitchAccum._.W0;
        }     

        if (!state_flags._.altitude_hold_throttle)
		{
			manualThrottle(throttleIn);
		}
		else if (state_flags._.GPS_steering && desired_behavior._.land)
		{
			// place a ceiling, in other words, go down, but not up.
			if (pitchAltitudeAdjust > 0)
			{
				pitchAltitudeAdjust = 0;
			}
			
			throttleFiltered.WW += (((int32_t)(udb_pwTrim[THROTTLE_INPUT_CHANNEL] - throttleFiltered._.W1)) << THROTTLEFILTSHIFT);
			set_throttle_control(throttleFiltered._.W1 - throttleIn);
			filterManual = true;
		}
		else
		{
			// Servo reversing is handled in servoMix.c
			int16_t throttleOut = udb_servo_pulsesat(udb_pwTrim[THROTTLE_INPUT_CHANNEL] + throttleAccum.WW);
			throttleFiltered.WW += (((int32_t)(throttleOut - throttleFiltered._.W1)) << THROTTLEFILTSHIFT);
			set_throttle_control(throttleFiltered._.W1 - throttleIn);
			filterManual = true;
		}
        if (!state_flags._.altitude_hold_pitch)
		{
			pitchAltitudeAdjust = 0;
		}
	}
	else
	{
		pitchAltitudeAdjust = 0;
		manualThrottle(throttleIn);
	}
}

static void manualThrottle(int16_t throttleIn)
{
	int16_t throttle_control_pre;

	throttleFiltered.WW += (((int32_t)(throttleIn - throttleFiltered._.W1)) << THROTTLEFILTSHIFT);
	if (filterManual)
	{
		// Continue to filter the throttle control value in manual mode to avoid large, instant
		// changes to throttle value, which can burn out a brushed motor.  But after fading over
		// to the new throttle value, stop applying the filter to the throttle out to allow
		// faster control.
		throttle_control_pre = throttleFiltered._.W1 - throttleIn;
		if (throttle_control_pre < 10) filterManual = false;
	}
	else
	{
		throttle_control_pre = 0;
	}
	set_throttle_control(throttle_control_pre);
}

// For now, hovering does not attempt to control the throttle, and instead
// gives manual throttle control back to the pilot.
static void hoverAltitudeCntrl(void)
{
	int16_t throttle_control_pre;
	int16_t throttleIn = (udb_flags._.radio_on == 1) ? udb_pwIn[THROTTLE_INPUT_CHANNEL] : udb_pwTrim[THROTTLE_INPUT_CHANNEL];

	throttleFiltered.WW += (((int32_t)(throttleIn - throttleFiltered._.W1)) << THROTTLEFILTSHIFT);
	if (filterManual)
	{
		// Continue to filter the throttle control value in manual mode to avoid large, instant
		// changes to throttle value, which can burn out a brushed motor.  But after fading over
		// to the new throttle value, stop applying the filter to the throttle out to allow
		// faster control.
		throttle_control_pre = throttleFiltered._.W1 - throttleIn;
		if (throttle_control_pre < 10) filterManual = false;
	}
	else
	{
		throttle_control_pre = 0;
	}
	set_throttle_control(throttle_control_pre);
}

#else

void init_altitudeCntrl(void)
{
}

#endif //(ALTITUDE_GAINS_VARIABLE != 1)


