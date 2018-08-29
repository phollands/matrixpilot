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


#include "libDCM.h"
#include "gpsData.h"
#include "gpsParseCommon.h"
#include "../libUDB/heartbeat.h"
#include "../libUDB/magnetometer.h"
#include "../libUDB/barometer.h"
#include "../libUDB/ADchannel.h"
#include "estAltitude.h"
#include "mathlibNAV.h"
#include "rmat.h"
#include "mag_drift.h"


union dcm_fbts_word dcm_flags;
int16_t angleOfAttack;
uint16_t gravity_axis_at_startup;

void send_HILSIM_outputs(void);

void SetAofA(int16_t AofA)
{
	angleOfAttack = AofA;
}

int16_t GetAofA(void)
{
	return angleOfAttack;
}

void dcm_init(void)
{
	dcm_flags.W = 0;
	dcm_flags._.first_mag_reading = 1;
	dcm_init_rmat();
}

extern inline void read_accel(void) ;

void dcm_align_tilt(void)
{
	uint16_t minMag;
    uint16_t maxMag;
    uint16_t most_level_axis;
	int16_t temporary[3] ;
	read_accel() ;
	vector3_normalize( &rmat[6] , gplane ) ;
    
    // Find the axis with the strongest gravity and the sign of the gravity on the axis
    // Can be used to signal a flight plane when powering up.
    maxMag = abs(rmat[6]);
    if (rmat[6] > 0)
        gravity_axis_at_startup = GRAVITY_X_POSITIVE;
    else
        gravity_axis_at_startup = GRAVITY_X_NEGATIVE;

    if (abs(rmat[7]) > maxMag)
    {
        maxMag = abs(rmat[7]);
        if (rmat[7] > 0)
            gravity_axis_at_startup = GRAVITY_Y_POSITIVE;
        else
            gravity_axis_at_startup = GRAVITY_Y_NEGATIVE;
    }
    
	if (abs(rmat[8]) > maxMag)
    {
        maxMag = abs(rmat[8]);
        if (rmat[8] > 0)
            gravity_axis_at_startup = GRAVITY_Z_POSITIVE;
        else
            gravity_axis_at_startup = GRAVITY_Z_NEGATIVE;
    }

    // Work out which IMU axis is the most level with the earth axis
    // Used for the initialisation of the rmat matrix
	temporary[0] = gplane[0] ;
	temporary[1] = gplane[1] ;
	temporary[2] = gplane[2] ;
    
	minMag = abs( rmat[6] ) ;
	if ( abs( rmat[7] ) < minMag )
	{
		minMag = abs( rmat[7] ) ;
		most_level_axis = IMU_AXIS_Y ;
	}
	if ( abs( rmat[8] ) < minMag )
	{
		minMag = abs( rmat[8] ) ;
		most_level_axis = IMU_AXIS_Z;
	}
    else 
    {
        most_level_axis = IMU_AXIS_X;
    }

	if ( most_level_axis == IMU_AXIS_X )
	{
		temporary[0] = temporary[1] ;
		temporary[1] = - temporary[2] ;
		temporary[2] = temporary[0] ;
		temporary[0] = 0 ;
	}
	else if ( most_level_axis == IMU_AXIS_Y)
	{
		temporary[1] = temporary[2] ;
		temporary[2] = - temporary[0] ;
		temporary[0] = temporary[1] ;
		temporary[1] = 0 ;
	}
	else
	{
		temporary[2] = temporary[0] ;
		temporary[0] = - temporary[1] ;
		temporary[1] = temporary[2] ;
		temporary[2] = 0 ;
	}

	vector3_normalize( temporary , temporary ) ;
	rmat[3] = temporary[0] ;
	rmat[4] = temporary[1] ;
	rmat[5] = temporary[2] ;

	VectorCross( &rmat[0] , &rmat[3] , &rmat[6] ) ;
    
}


#if (DCM_CALIB_COUNT > DCM_GPS_COUNT)
#error here
#endif

void dcm_run_calib_step(uint16_t count)
{
	if (count == DCM_CALIB_COUNT)
	{
		DPRINT("calib_finished\r\n");
		dcm_flags._.calib_finished = 1;
		dcm_calibrate();    // Finish calibration
        dcm_align_tilt();
	}
}

static boolean gps_run_init_step(uint16_t count)
{
	if (count <= DCM_GPS_COUNT)
	{
		gps_startup_sequence(DCM_GPS_COUNT - count); // Counts down from GPS_COUNT to 0
	}
	else
	{
		DPRINT("init_finished\r\n");
		return true;    // indicate that we are done
	}
	return false;
}


// We want to be returning reading from both the magnetometer and the barometer at 4Hz.
// However each device may need to be called many times before a reading is returned.
// The code below ensures that a given sensor is called repeatedly until it reaches
// a state where the other device can be called safely over the same I2C2 bus, even if 
// the first device has not yet returned a complete set of sensor data.
// For example, at startup the barometer could need 9 calls before returning pressure.
// But in normal operation it will need 5 calls.
// By contrast in normal operations the magnetometer only needs one call, once it is calibrated.

void get_data_from_I2C_sensors(void) // Expected to be called at 40Hz
{
	static uint8_t barometer_needs_servicing = BAROMETER_SERVICE_CAN_PAUSE;
	static uint8_t magnetometer_needs_servicing = MAGNETOMETER_SERVICE_CAN_PAUSE;
	static uint8_t counter_40Hz = 0;
	
	// Split a 1 second interval (40 calls) into 8 interleaved segments using the following logic
	if ((counter_40Hz == 0) || (counter_40Hz == 10) || (counter_40Hz == 20) || ( counter_40Hz == 30))
	{
#if (USE_BAROMETER_ALTITUDE == 1 && HILSIM != 1)
		barometer_needs_servicing = rxBarometer(udb_barometer_callback);
#endif // (USE_BAROMETER_ALTITUDE == 1 && HILSIM != 1)
	}
	else if // Allow a total of 6 consecutive calls to the barometer to return pressure
		(((counter_40Hz >= 1) && (counter_40Hz <= 5))
		|| ((counter_40Hz >= 11) && (counter_40Hz <= 15))
		|| ((counter_40Hz >= 21) && (counter_40Hz <= 25))
		|| ((counter_40Hz >= 31) && (counter_40Hz <= 35)))
	{
		if (barometer_needs_servicing)
		{
#if (USE_BAROMETER_ALTITUDE == 1 && HILSIM != 1)
			barometer_needs_servicing = rxBarometer(udb_barometer_callback);
#endif // (USE_BAROMETER_ALTITUDE == 1 && HILSIM != 1)
		}
	}
	else if ((counter_40Hz == 6) || (counter_40Hz == 16) || (counter_40Hz == 26) || ( counter_40Hz == 36))
	{
#if (MAG_YAW_DRIFT == 1)
		magnetometer_needs_servicing = rxMagnetometer(mag_drift_callback);
#endif // (MAG_YAW_DRIFT == 1)
	}
	else // Should be my_modulo at 7,8,9  17,18,19   27,28,29  37,38,39 : a total of 4 consecutively calls to mag.
	{
		if (magnetometer_needs_servicing)
		{
#if (MAG_YAW_DRIFT == 1 && HILSIM != 1)
			magnetometer_needs_servicing = rxMagnetometer(mag_drift_callback);
#endif  // (MAG_YAW_DRIFT == 1 && HILSIM != 1)
		}	   
	}
	counter_40Hz++;
	if (counter_40Hz >= 40) counter_40Hz = 0;
}

// Called at HEARTBEAT_HZ
void udb_heartbeat_callback(void)
{
	if (udb_pulse_counter % (HEARTBEAT_HZ / 40) == 0)
	{
		get_data_from_I2C_sensors(); // TODO: this should always be be called at 40Hz
	}
//  when we move the IMU step to the MPU call back, to run at 200 Hz, remove this
	if (dcm_flags._.calib_finished)
	{
		dcm_run_imu_step(angleOfAttack);
	}

	dcm_heartbeat_callback();    // this was called dcm_servo_callback_prepare_outputs();

	if (udb_pulse_counter % (HEARTBEAT_HZ / 40) == 0)
	{
		if (!dcm_flags._.calib_finished)
		{
			dcm_run_calib_step(udb_pulse_counter / (HEARTBEAT_HZ / 40));
		}
		if (!dcm_flags._.init_finished)
		{
			dcm_flags._.init_finished = gps_run_init_step(udb_pulse_counter / (HEARTBEAT_HZ / 40));
		}
	}

#if (HILSIM == 1)
	send_HILSIM_outputs();
#endif
}

// dcm_calibrate is called twice during the startup sequence.
// Firstly 10 seconds after startup, then immediately before the first waggle, which is 10 seconds after getting radio link.  
// This makes sure we get initialized when there's no radio, or when bench testing, 
// and 2nd time is at a time the user knows to keep the plane steady before a flight.
void dcm_calibrate(void)
{
	// Don't allow re/calibrating before the initial calibration period has finished
	if (dcm_flags._.calib_finished)
	{
		udb_a2d_record_offsets();
	}
}

void dcm_set_origin_location(int32_t o_lon, int32_t o_lat, int32_t o_alt)
{
	union longbbbb accum_nav;
	unsigned char lat_cir;

	lat_origin.WW = o_lat;
	lon_origin.WW = o_lon;
	alt_origin.WW = o_alt;

	// scale the low 16 bits of latitude from GPS units to gentleNAV units
	accum_nav.WW = __builtin_mulss(LONGDEG_2_BYTECIR, lat_origin._.W1);

	lat_cir = accum_nav.__.B2;  // effectively divides by 256
	// estimate the cosine of the latitude, which is used later computing desired course
	cos_lat = cosine(lat_cir);
}

struct relative3D dcm_absolute_to_relative(struct waypoint3D absolute)
{
	struct relative3D rel;

	rel.z = absolute.z;
	rel.y = (absolute.y - lat_origin.WW) / 90; // in meters
	rel.x = long_scale((absolute.x - lon_origin.WW) / 90, cos_lat);
	return rel;
}

#ifdef USE_EXTENDED_NAV
struct relative3D_32 dcm_absolute_to_relative_32(struct waypoint3D absolute)
{
	struct relative3D_32 rel;

	rel.z = absolute.z;
	rel.y = (absolute.y - lat_origin.WW) / 90; // in meters
	rel.x = long_scale((absolute.x - lon_origin.WW) / 90, cos_lat);
	return rel;
}
#endif // USE_EXTENDED_NAV

vect3_32t dcm_rel2abs(vect3_32t rel)
{
	vect3_32t abs;

	abs.z = rel.z;
	abs.y = (rel.y * 90) + lat_origin.WW;
	abs.x = (rel.x * 90) + lon_origin.WW;
//	abs.x = long_scale((rel.x * 90), cos_lat) + lon_origin.WW;

	return abs;
}
