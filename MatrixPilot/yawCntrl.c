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
#include "heartbeat.h"
#include "libDCM_internal.h"
extern float xgain;

#define HOVERYOFFSET ((int32_t)(HOVER_YAW_OFFSET*(RMAX/57.3)))

#if (USE_CONFIGFILE == 1)
#include "config.h"
#include "redef.h"

	uint16_t yawkdrud;
	uint16_t rollkprud;
	uint16_t rollkdrud;
	uint16_t hoveryawkp;
	uint16_t hoveryawkd;
#elif ((SERIAL_OUTPUT_FORMAT == SERIAL_MAVLINK) || (GAINS_VARIABLE == 1))
	uint16_t yawkdrud			= (uint16_t)(YAWKD_RUDDER*SCALEGYRO*RMAX);
	uint16_t rollkprud			= (uint16_t)(ROLLKP_RUDDER*RMAX);
	uint16_t rollkdrud			= (uint16_t)(ROLLKD_RUDDER*SCALEGYRO*RMAX);
	uint16_t hoveryawkp			= (uint16_t)(HOVER_YAWKP*RMAX);
	uint16_t hoveryawkd			= (uint16_t)(HOVER_YAWKD*SCALEGYRO*RMAX);
#else
	const uint16_t yawkdrud		= (uint16_t)(YAWKD_RUDDER*SCALEGYRO*RMAX);
	const uint16_t rollkprud	= (uint16_t)(ROLLKP_RUDDER*RMAX);
	const uint16_t rollkdrud	= (uint16_t)(ROLLKD_RUDDER*SCALEGYRO*RMAX);
	const uint16_t hoveryawkp	= (uint16_t)(HOVER_YAWKP*RMAX);
	const uint16_t hoveryawkd	= (uint16_t)(HOVER_YAWKD*SCALEGYRO*RMAX);
#endif

#include "filters.h"
union int32_w2 xacc_filt;
extern fractional accelerometer[];
void normalYawCntrl(void);
void hoverYawCntrl(void);

#if (USE_CONFIGFILE == 1)
void init_yawCntrl(void)
{
	yawkdrud 	= (uint16_t)(YAWKD_RUDDER*SCALEGYRO*RMAX);
	rollkprud 	= (uint16_t)(ROLLKP_RUDDER*RMAX);
	rollkdrud 	= (uint16_t)(ROLLKD_RUDDER*SCALEGYRO*RMAX);
	hoveryawkp 	= (uint16_t)(HOVER_YAWKP*RMAX);
	hoveryawkd 	= (uint16_t)(HOVER_YAWKD*SCALEGYRO*RMAX);
}
#endif

void yawCntrl(void)
{
	if (canStabilizeHover() && current_orientation == F_HOVER)
	{
		hoverYawCntrl();
	}
	else
	{
		normalYawCntrl();
	}
}

void normalYawCntrl(void)
{
	int16_t yawNavDeflection;
	union longww gyroYawFeedback;
        int16_t xacc;

        // lowpass filter the x accelerometer samples
        // The MPU6000 applies a 42Hz digital lowpass filter, but we probably
        // want just a few Hz of bandwidth for the accelerometer readings.
        // Note that this is executed at HEARTBEAT_HZ = 200, so the 3dB point
        // for lp2 with LPCB_45_HZ will be 4.5Hz
#if (HILSIM == 1)
                // acceleration data comes from the Xplane plugin
                int16_t x_acc = (float)g_a_x_sim.BB;
#else
                // acceleration data comes from the onboard sensor
                int16_t x_acc = accelerometer[0];
#endif
        // scale value up such that 1g of lateral acceleration has magnitude 16K
        xacc = lp2(x_acc, &xacc_filt, LPCB_45_HZ);
        magClamp(&xacc, 16384/(ACCEL_RANGE)); // saturate at 16K
        xacc *= (ACCEL_RANGE);

        // yaw gyro reports rate as a signed 16 bit integer with range [-500, 500) deg/sec
        // for a setpoint range of +/-60 deg/sec = +/-4000, multiply PWM input by 4
        int16_t yaw_manual =  REVERSE_IF_NEEDED(RUDDER_CHANNEL_REVERSED,
            (udb_pwIn[RUDDER_INPUT_CHANNEL] - udb_pwTrim[RUDDER_INPUT_CHANNEL]));
        yaw_rate = yaw_manual << 2;

#ifdef TestGains
	flags._.GPS_steering = 0; // turn off navigation
	flags._.pitch_feedback = 1; // turn on stabilization
#endif 
	if (RUDDER_NAVIGATION && flags._.GPS_steering)
	{
                // this method output is clamped to +/-16000 => +/-240 deg/sec
		yawNavDeflection = determine_navigation_deflection('y');
		
		if (canStabilizeInverted() && current_orientation == F_INVERTED)
		{
			yawNavDeflection = -yawNavDeflection;
		}
                yaw_rate += yawNavDeflection;
                // limit combined manual and nav roll setpoint to less than 240 deg/sec
                magClamp(&roll_setpoint, 16000);
	}

        gyroYawFeedback.WW = 0;
	if (flags._.pitch_feedback)
	{
		gyroYawFeedback.WW = __builtin_mulus(yawkdrud, yaw_rate - omegaAccum[2]);
#if (HILSIM == 1)
                // acceleration data comes from the Xplane plugin
                int16_t xacc_scaled = -(1.0/16) * xacc;
                if ((udb_heartbeat_counter % (HEARTBEAT_HZ/10)) == 0) {
                    printf("xacc: %i, xacc feedback: %i, total feedback: %i\n", g_a_x_sim.BB, xacc_scaled, gyroYawFeedback._.W1);
                }
#else
                // acceleration data comes from the onboard sensor
                int16_t xacc_scaled = -xgain * xacc;
#endif
		gyroYawFeedback._.W1 += xacc_scaled;
        } else {
            // no stabilization; pass manual input through
            gyroYawFeedback._.W1 = yaw_manual;
        }
      
	yaw_control = gyroYawFeedback._.W1;
        // Servo reversing is handled in servoMix.c
}

void hoverYawCntrl(void)
{
	union longww yawAccum;
	union longww gyroYawFeedback;

	if (flags._.pitch_feedback)
	{
		gyroYawFeedback.WW = __builtin_mulus(hoveryawkd , omegaAccum[2]);
		
		int16_t yawInput = (udb_flags._.radio_on == 1) ? REVERSE_IF_NEEDED(RUDDER_CHANNEL_REVERSED, udb_pwIn[RUDDER_INPUT_CHANNEL] - udb_pwTrim[RUDDER_INPUT_CHANNEL]) : 0;
		int16_t manualYawOffset = yawInput * (int16_t)(RMAX/2000);
		
		yawAccum.WW = __builtin_mulsu(rmat[6] + HOVERYOFFSET + manualYawOffset , hoveryawkp);
	}
	else
	{
		gyroYawFeedback.WW = 0;
		yawAccum.WW = 0;
	}

	yaw_control = (int32_t)yawAccum._.W1 - (int32_t)gyroYawFeedback._.W1;
}
