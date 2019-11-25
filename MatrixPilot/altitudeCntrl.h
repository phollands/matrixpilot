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


#ifndef ALTITUDECNTRL_H
#define ALTITUDECNTRL_H

extern boolean filterManual;
extern union longww desiredHeight32 ;
extern union longww desiredHeightAGL32;
// Variables required for mavlink.  Used in AltitudeCntrlVariable and airspeedCntrl
extern int16_t height_target_min;
extern int16_t height_target_max;
extern int16_t height_margin;
extern fractional alt_hold_throttle_min;
extern fractional alt_hold_throttle_max;
extern int16_t alt_hold_pitch_min;
extern int16_t alt_hold_pitch_max;
extern int16_t alt_hold_pitch_high;
extern int16_t rtl_pitch_down;


extern union longww throttleFiltered;
extern int16_t pitchAltitudeAdjust;
extern int16_t elevator_override;
extern int16_t desiredSpeed; // Stored in 10ths of meters per second

void setTargetAltitude(int16_t targetAlt);

#if (AIRFRAME_TYPE == AIRFRAME_GLIDER)
inline int16_t get_autopilotBrake(void);
#endif

#define HEIGHT_AGL_TO_START_TERRAIN_FOLLOWING 1000 
#define HEIGHT_AGL_TO_STOP_TERRAIN_FOLLOWING  1500
#define MINIMUM_TERRAIN_FOLLOWING_HEIGHT      ((int32_t) 3 * 65536) // Fractional Meters

#endif // ALTITUDECNTRL_H
