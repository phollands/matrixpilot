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


#include "libDCM_internal.h"
#include "../libUDB/heartbeat.h"


// heartbeats
#define DR_PERIOD (int16_t)((HEARTBEAT_HZ/GPS_RATE)+4)

// seconds
#define DR_TIMESTEP (1.0/HEARTBEAT_HZ)

// 1.0 in 0.16 format
#define MAX16 (4.0*RMAX)

// seconds
#define DR_TAU 2.5

// seconds * (cm/sec^2 / count) ??? is G always represented as cm/sec^2 ?
// GRAVITYM is 980 cm/sec^2, GRAVITY is 2000 counts
// dx/dt^2 * ACCEL2DELTAV = cm/sec
#define ACCEL2DELTAV ((DR_TIMESTEP*GRAVITYM*MAX16)/GRAVITY)

// seconds; the .01 must convert from cm/sec^2 to m/sec^2
// cm/sec * VELOCITY2LOCATION = meters
#define VELOCITY2LOCATION (DR_TIMESTEP*.01*MAX16*16.0)
//	The factor of 16 is so that the gain is more precise.
//	There is a subsequent right shift by 4 to cancel the multiply by 16.

// dimensionless
#define DR_FILTER_GAIN (int16_t) (DR_TIMESTEP*MAX16/DR_TAU)

// 1/seconds
#define ONE_OVER_TAU (uint16_t) (MAX16/DR_TAU)
//#define ENABLE_VEL_OFFSET
#undef ENABLE_VEL_OFFSET

int16_t dead_reckon_clock = DR_PERIOD;

//      velocity, as estimated by the IMU: high word is cm/sec
union longww IMUvelocityx =  { 0 };
union longww IMUvelocityy =  { 0 };
union longww IMUvelocityz =  { 0 };

// floating point GPS coordinates
extern struct relative3D_f GPSloc_f;

// 32 bit fractional GPS coordinates
extern struct relative3D_32 GPSlocation_32;

//      location, as estimated by the IMU
// high word is meters, low word is fractional meters
union longww IMUlocationx =  { 0 };
union longww IMUlocationy =  { 0 };
union longww IMUlocationz =  { 0 };

//	integral of acceleration
union longww IMUintegralAccelerationx = { 0 };
union longww IMUintegralAccelerationy = { 0 };
union longww IMUintegralAccelerationz = { 0 };

uint16_t air_speed_3DIMU = 0;
int16_t total_energy = 0;

//	GPSlocation - IMUlocation: meters
//fractional locationErrorEarth[] = { 0 , 0 , 0 };
int32_t locationErrorEarth[] = { 0 , 0 , 0 };
//	GPSvelocity - IMUvelocity
fractional velocityErrorEarth[] = { 0 , 0 , 0 };

extern int16_t errorYawground[];

void dead_reckon(void)
{
    int i = 0;
	if (dcm_flags._.dead_reckon_enable == 1)  // wait for startup of GPS
	{
		//	integrate the accelerometers to update IMU velocity
		IMUintegralAccelerationx.WW += __builtin_mulss(((int16_t)(ACCEL2DELTAV)) ,  accelEarth[0]);
		IMUintegralAccelerationy.WW += __builtin_mulss(((int16_t)(ACCEL2DELTAV)) ,  accelEarth[1]);
		IMUintegralAccelerationz.WW += __builtin_mulss(((int16_t)(ACCEL2DELTAV)) ,  accelEarth[2]);

		//	integrate IMU velocity to update the IMU location
		IMUlocationx.WW += (__builtin_mulss(((int16_t)(VELOCITY2LOCATION)) ,  IMUintegralAccelerationx._.W1)>>4);
		IMUlocationy.WW += (__builtin_mulss(((int16_t)(VELOCITY2LOCATION)) ,  IMUintegralAccelerationy._.W1)>>4);
		IMUlocationz.WW += (__builtin_mulss(((int16_t)(VELOCITY2LOCATION)) ,  IMUintegralAccelerationz._.W1)>>4);

		if (dead_reckon_clock > 0)
		//	apply drift adjustments only while valid GPS data is in force.
		//  This is done with a countdown clock that gets reset each time new data comes in.
		{
			dead_reckon_clock --;

			IMUintegralAccelerationx.WW += __builtin_mulss(DR_FILTER_GAIN ,  velocityErrorEarth[0]);
			IMUintegralAccelerationy.WW += __builtin_mulss(DR_FILTER_GAIN ,  velocityErrorEarth[1]);
			IMUintegralAccelerationz.WW += __builtin_mulss(DR_FILTER_GAIN ,  velocityErrorEarth[2]);

                        // when locationErrorEarth becomes 32 bits, this will need to operate on a 16 bit
                        // subset, perhaps saturating at a magnitude of 64 meters
                        // shift the 32 bit location error to the right 7 bits to place the sign bit at bit 15 of the low word
                        // this is equivalent to multiplication by 2^9 since the decimal point is now between bits 9 and 10
			IMUlocationx.WW += __builtin_mulss(DR_FILTER_GAIN ,  locationErrorEarth[0]>>7) >> 9;
			IMUlocationy.WW += __builtin_mulss(DR_FILTER_GAIN ,  locationErrorEarth[1]>>7) >> 9;
			IMUlocationz.WW += __builtin_mulss(DR_FILTER_GAIN ,  locationErrorEarth[2]>>7) >> 9;

			IMUvelocityx.WW = IMUintegralAccelerationx.WW;
			IMUvelocityy.WW = IMUintegralAccelerationy.WW;
			IMUvelocityz.WW = IMUintegralAccelerationz.WW;

#ifdef ENABLE_VEL_OFFSET
			IMUvelocityx.WW += __builtin_mulus(ONE_OVER_TAU , 100*locationErrorEarth[0]);
			IMUvelocityy.WW += __builtin_mulus(ONE_OVER_TAU , 100*locationErrorEarth[1]);
			IMUvelocityz.WW += __builtin_mulus(ONE_OVER_TAU , 100*locationErrorEarth[2]);
#endif
		}
		else  // GPS has gotten disconnected
		{
			errorYawground[0] = errorYawground[1] = errorYawground[2] = 0; // turn off yaw drift
			dcm_flags._.gps_history_valid = 0; // restart GPS history variables
			IMUvelocityx.WW = IMUintegralAccelerationx.WW;
			IMUvelocityy.WW = IMUintegralAccelerationy.WW;
			IMUvelocityz.WW = IMUintegralAccelerationz.WW;
		}

		if (gps_nav_valid() && (dcm_flags._.reckon_req == 1))
		{
			//	compute error indications and restart the dead reckoning clock to apply them
			dcm_flags._.reckon_req = 0;
			dead_reckon_clock = DR_PERIOD;

                        // modify this to utilize 32 bit precision GPSlocation_32
                        // change locationErrorEarth to 32 bits
			locationErrorEarth[0] = GPSlocation_32.x - IMUlocationx.WW;
			locationErrorEarth[1] = GPSlocation_32.y - IMUlocationy.WW;
			locationErrorEarth[2] = GPSlocation_32.z - IMUlocationz.WW;
                        // clamp to +/-64 meters
                        for (i=0; i<3; i++) {
                            magClamp32(&locationErrorEarth[i], 64*65536);
                        }

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

	int16_t air_speed_x , air_speed_y , air_speed_z;

	air_speed_x = IMUvelocityx._.W1 - estimatedWind[0];
	air_speed_y = IMUvelocityy._.W1 - estimatedWind[1];
	air_speed_z = IMUvelocityz._.W1 - estimatedWind[2];

#if (HILSIM == 1)
	air_speed_3DIMU = as_sim.BB; // use Xplane as a pitot
#else
	air_speed_3DIMU = vector3_mag (air_speed_x , air_speed_y , air_speed_z);
#endif

	union longww accum;
	union longww energy;

	accum.WW = __builtin_mulsu (air_speed_x , 37877);
	energy.WW = __builtin_mulss (accum._.W1 , accum._.W1);

	accum.WW = __builtin_mulsu (air_speed_y , 37877);
	energy.WW += __builtin_mulss (accum._.W1 , accum._.W1);

	accum.WW = __builtin_mulsu (air_speed_z , 37877);
	energy.WW += __builtin_mulss (accum._.W1 , accum._.W1);

	energy.WW += IMUlocationz.WW;
	total_energy = energy._.W1;
}
