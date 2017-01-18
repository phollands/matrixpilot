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


#ifndef DEADRECKONING_H
#define DEADRECKONING_H

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
// The factor of 16 is so that the gain is more precise.
// There is a subsequent right shift by 4 to cancel the multiply by 16.

// dimensionless
#define DR_FILTER_GAIN1 (int16_t)(10*DR_TIMESTEP*MAX16/DR_TAU)
#define DR_FILTER_GAIN2 (int16_t)(DR_TIMESTEP*MAX16/DR_TAU)

// 1/seconds
#define ONE_OVER_TAU (uint16_t)(MAX16/DR_TAU)

extern int16_t dead_reckon_clock;

extern uint16_t air_speed_3DIMU;
extern int16_t total_energy;
extern fractional locationErrorEarth[3];
extern fractional velocityErrorEarth[3];

extern union longww IMUlocationx, IMUlocationy, IMUlocationz;
extern union longww IMUlocation1x, IMUlocation1y, IMUlocation1z;
extern union longww IMUvelocityx, IMUvelocityy, IMUvelocityz;
extern union longww IMUintegralAccelerationx;
extern union longww IMUintegralAccelerationy;
extern union longww IMUintegralAccelerationz;
extern union longww IMUintegralAcceleration1x;
extern union longww IMUintegralAcceleration1y;
extern union longww IMUintegralAcceleration1z;

extern int16_t forward_ground_speed;

#define IMUheight IMUlocationz._.W1


void dead_reckon(void);


#endif // DEADRECKONING_H
