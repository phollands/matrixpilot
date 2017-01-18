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
#include "gpsParseCommon.h"
#include "deadReckoning.h"
#include "mathlibNAV.h"
#include "estWind.h"
#include "gpsData.h"
#include "rmat.h"
#include "../libUDB/heartbeat.h"




int16_t dead_reckon_clock = DR_PERIOD;

// velocity, as estimated by the IMU: high word is cm/sec
union longww IMUvelocityx = { 0 };
union longww IMUvelocityy = { 0 };
union longww IMUvelocityz = { 0 };

int16_t forward_ground_speed = 0 ;

// location, as estimated by the IMU
// high word is meters, low word is fractional meters
union longww IMUlocation1x = { 0 };
union longww IMUlocation1y = { 0 };
union longww IMUlocation1z = { 0 };

union longww IMUlocationx = { 0 };
union longww IMUlocationy = { 0 };
union longww IMUlocationz = { 0 };

// integral of acceleration
union longww IMUintegralAcceleration1x = { 0 };
union longww IMUintegralAcceleration1y = { 0 };
union longww IMUintegralAcceleration1z = { 0 };

union longww IMUintegralAccelerationx = { 0 };
union longww IMUintegralAccelerationy = { 0 };
union longww IMUintegralAccelerationz = { 0 };

uint16_t air_speed_3DIMU = 0;
int16_t total_energy = 0;

// GPSlocation - IMUlocation: meters
fractional locationErrorEarth[] = { 0, 0, 0 };
// GPSvelocity - IMUvelocity
fractional velocityErrorEarth[] = { 0, 0, 0 };

void dead_reckon(void)
{
	int16_t air_speed_x, air_speed_y, air_speed_z;
	union longww accum;
	union longww energy;

	if (dcm_flags._.dead_reckon_enable == 1)  // wait for startup of GPS
	{
		// integrate the accelerometers to update IMU velocity
		IMUintegralAccelerationx.WW += __builtin_mulss(((int16_t)(ACCEL2DELTAV)), accelEarth[0]);
		IMUintegralAccelerationy.WW += __builtin_mulss(((int16_t)(ACCEL2DELTAV)), accelEarth[1]);
		IMUintegralAccelerationz.WW += __builtin_mulss(((int16_t)(ACCEL2DELTAV)), accelEarth[2]);
		
		IMUintegralAcceleration1x.WW = IMUintegralAccelerationx.WW;
		IMUintegralAcceleration1y.WW = IMUintegralAccelerationy.WW;
		IMUintegralAcceleration1z.WW = IMUintegralAccelerationz.WW;

		// integrate IMU velocity to update the IMU location	
		IMUlocationx.WW += (__builtin_mulss(((int16_t)(VELOCITY2LOCATION)), IMUintegralAccelerationx._.W1)>>4);
		IMUlocationy.WW += (__builtin_mulss(((int16_t)(VELOCITY2LOCATION)), IMUintegralAccelerationy._.W1)>>4);
		IMUlocationz.WW += (__builtin_mulss(((int16_t)(VELOCITY2LOCATION)), IMUintegralAccelerationz._.W1)>>4);
		
		IMUlocation1x.WW = IMUlocationx.WW;
		IMUlocation1y.WW = IMUlocationx.WW;
		IMUlocation1z.WW = IMUlocationx.WW;

		if (dead_reckon_clock > 0)
		// apply drift adjustments only while valid GPS data is in force.
		// This is done with a countdown clock that gets reset each time new data comes in.
		{
			dead_reckon_clock --;

			IMUintegralAccelerationx.WW += __builtin_mulss(DR_FILTER_GAIN1, velocityErrorEarth[0]);
			IMUintegralAccelerationy.WW += __builtin_mulss(DR_FILTER_GAIN1, velocityErrorEarth[1]);
			IMUintegralAccelerationz.WW += __builtin_mulss(DR_FILTER_GAIN1, velocityErrorEarth[2]);

			IMUlocationx.WW += __builtin_mulss(DR_FILTER_GAIN2, locationErrorEarth[0]);
			IMUlocationy.WW += __builtin_mulss(DR_FILTER_GAIN2, locationErrorEarth[1]);
			IMUlocationz.WW += __builtin_mulss(DR_FILTER_GAIN2, locationErrorEarth[2]);

			IMUvelocityx.WW = IMUintegralAccelerationx.WW +
			                  __builtin_mulus(ONE_OVER_TAU, 100*locationErrorEarth[0]);
			IMUvelocityy.WW = IMUintegralAccelerationy.WW +
			                  __builtin_mulus(ONE_OVER_TAU, 100*locationErrorEarth[1]);
			IMUvelocityz.WW = IMUintegralAccelerationz.WW +
			                  __builtin_mulus(ONE_OVER_TAU, 100*locationErrorEarth[2]);

		}
		else  // GPS has gotten disconnected
		{
			yaw_drift_reset();
			dcm_flags._.gps_history_valid = 0; // restart GPS history variables
			IMUvelocityx.WW = IMUintegralAccelerationx.WW;
			IMUvelocityy.WW = IMUintegralAccelerationy.WW;
			IMUvelocityz.WW = IMUintegralAccelerationz.WW;
		}

		if (gps_nav_valid() && (dcm_flags._.reckon_req == 1))
		{
			// compute error indications and restart the dead reckoning clock to apply them
			dcm_flags._.reckon_req = 0;
			dead_reckon_clock = DR_PERIOD;

			locationErrorEarth[0] = GPSlocation.x - IMUlocationx._.W1;
			locationErrorEarth[1] = GPSlocation.y - IMUlocationy._.W1;
			locationErrorEarth[2] = GPSlocation.z - IMUlocationz._.W1;

			velocityErrorEarth[0] = GPSvelocity.x - IMUintegralAccelerationx._.W1;
			velocityErrorEarth[1] = GPSvelocity.y - IMUintegralAccelerationy._.W1;
			velocityErrorEarth[2] = GPSvelocity.z - IMUintegralAccelerationz._.W1;
		}
	}
	else
	{
		IMUintegralAccelerationx.WW = 0;
		IMUintegralAccelerationy.WW = 0;
		IMUintegralAccelerationz.WW = 0;

		IMUvelocityx.WW = 0;
		IMUvelocityy.WW = 0;
		IMUvelocityz.WW = 0;

		IMUlocationx.WW = 0;
		IMUlocationy.WW = 0;
		IMUlocationz.WW = 0;
	}
	air_speed_x = IMUvelocityx._.W1 - estimatedWind[0];
	air_speed_y = IMUvelocityy._.W1 - estimatedWind[1];
	air_speed_z = IMUvelocityz._.W1 - estimatedWind[2];

	accum.WW = ((__builtin_mulss(-IMUintegralAccelerationx._.W1, rmat[1])
	                          + __builtin_mulss( IMUintegralAccelerationy._.W1, rmat[4])) << 2);
	forward_ground_speed = accum._.W1 ;

	air_speed_3DIMU = vector3_mag(air_speed_x, air_speed_y, air_speed_z);

	accum.WW   = __builtin_mulsu(air_speed_x, 37877);
	energy.WW  = __builtin_mulss(accum._.W1, accum._.W1);

	accum.WW   = __builtin_mulsu(air_speed_y, 37877);
	energy.WW += __builtin_mulss(accum._.W1, accum._.W1);

	accum.WW   = __builtin_mulsu(air_speed_z, 37877);
	energy.WW += __builtin_mulss(accum._.W1, accum._.W1);

	energy.WW += IMUlocationz.WW;
	total_energy = energy._.W1;
}
