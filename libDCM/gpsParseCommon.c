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
#include "gpsParseCommon.h"
#include "estAltitude.h"
#include "heartbeat.h"
#include <string.h>

#if (SILSIM == 1)
#include <stdlib.h>
#endif

struct relative3D GPSlocation = { 0, 0, 0 };
struct relative3D_f GPSloc_f = { 0.0f, 0.0f, 0.0f };
struct relative3D GPSvelocity = { 0, 0, 0 };

union longbbbb lat_gps, long_gps, alt_sl_gps, tow;  // latitude, longitude, altitude
union intbb sog_gps, climb_gps, week_no;   // speed over ground, climb
union uintbb cog_gps;   // course over ground, units: degrees * 100, range [0-35999]
union intbb as_sim;
uint8_t hdop;                                       // horizontal dilution of precision
union longbbbb lat_origin, long_origin, alt_origin;
//union longbbbb xpg, ypg, zpg;                     // gps x, y, z position
//union intbb    xvg, yvg, zvg;                     // gps x, y, z velocity
//uint8_t mode1, mode2;                             // gps mode1, mode2
uint8_t svs;                                        // number of satellites
uint8_t lat_cir;
int16_t cos_lat = 0;
// floating point version of location and cos_lat
float loc_f[3];
float   cos_lat_f;
int16_t gps_data_age;
uint8_t *gps_out_buffer = 0;
int16_t gps_out_buffer_length = 0;
int16_t gps_out_index = 0;

extern void (*msg_parse)(uint8_t inchar);
extern union longww IMUlocationx;
extern union longww IMUlocationy;
extern union longww IMUlocationz;
extern union longww IMUintegralAccelerationx;
extern union longww IMUintegralAccelerationy;
extern union longww IMUintegralAccelerationz;

void gpsoutbin(int16_t length, const uint8_t msg[]) // output a binary message to the GPS
{
	gps_out_buffer = 0; // clear the buffer pointer first, for safety, in case we're interrupted
	gps_out_index = 0;
	gps_out_buffer_length = length;
	gps_out_buffer = (uint8_t*)msg;

	udb_gps_start_sending_data();
}

void gpsoutline(const char *message) // output one NMEA line to the GPS
{
	gpsoutbin(strlen(message), (uint8_t*)message);
}

int16_t udb_gps_callback_get_byte_to_send(void)
{
	if (gps_out_buffer != 0 && gps_out_index < gps_out_buffer_length)
	{
		// We have a byte to send
		return (uint8_t)(gps_out_buffer[gps_out_index++]);
	}
	else
	{
		// No byte to send, so clear the link to the buffer
		gps_out_buffer = 0;
	}
	return -1;
}

// Got a character from the GPS
void udb_gps_callback_received_byte(uint8_t rxchar)
{
	//bin_out(rxchar);      // binary out to the debugging USART
	(*msg_parse)(rxchar);   // parse the input byte
}

int8_t actual_dir;
float actual_dir_f;
uint16_t ground_velocity_magnitudeXY = 0;
int16_t forward_acceleration = 0;
uint16_t air_speed_magnitudeXY = 0;
uint16_t air_speed_3DGPS = 0;
int8_t calculated_heading;

static int8_t cog_previous = 64;    // North
static float cog_prev_f = 90;       // North
static int16_t sog_previous = 0;
static int16_t climb_rate_previous = 0;
static int16_t location_previous[] = { 0, 0, 0 };
static float location_prev_f[] = { 0, 0, 0 };
static uint16_t velocity_previous = 0;

// Received a full set of GPS messages
void udb_background_callback_triggered(void)
{
	union longbbbb accum_nav;
	union longbbbb accum;
	union longww accum_velocity;
	int8_t cog_circular;
        float cog_circ_f;        // simulated circular
	int8_t cog_delta;
	float cog_delta_f;
	int16_t sog_delta;
	int16_t climb_rate_delta;
	int16_t location[3];
	int16_t location_deltaZ;
	float location_deltaZ_f;
	struct relative2D location_deltaXY;
	struct relative2D_f location_deltaXY_f;
	struct relative2D velocity_thru_air;
	int16_t velocity_thru_airz;

	dirovergndHRmat[0] = rmat[1];
	dirovergndHRmat[1] = rmat[4];
	dirovergndHRmat[2] = 0;

	if (gps_nav_valid())
	{
		LED_BLUE = LED_ON;
		commit_gps_data();

		gps_data_age = 0;

		dcm_callback_gps_location_updated();

		accum_nav.WW = ((lat_gps.WW - lat_origin.WW)/90); // in meters, range is about 20 miles
		location[1] = accum_nav._.W0;

		accum_nav.WW = long_scale((long_gps.WW - long_origin.WW)/90, cos_lat);
		location[0] = accum_nav._.W0;

#ifdef USE_PRESSURE_ALT
#warning "using pressure altitude instead of GPS altitude"
                // division by 100 implies alt_origin is in centimeters; not documented elsewhere
                // longword result = (longword/10 - longword)/100 : range
                accum_nav.WW = ((get_barometer_altitude()/10) - alt_origin.WW)/100; // height in meters
#else
		accum_nav.WW = (alt_sl_gps.WW - alt_origin.WW)/100; // height in meters
#endif // USE_PRESSURE_ALT
		location[2] = accum_nav._.W0;

		// convert GPS course of 360 degrees to a binary model with 256
		accum.WW = __builtin_muluu (COURSEDEG_2_BYTECIR, cog_gps.BB) + 0x00008000;
		// re-orientate from compass (clockwise) to maths (anti-clockwise) with 0 degrees in East
		cog_circular = -accum.__.B2 + 64;

                // compensate for GPS reporting latency.
		// The dynamic model of the EM406 and uBlox is not well known.
		// However, it seems likely much of it is simply reporting latency.
		// This section of the code compensates for reporting latency.

		if (dcm_flags._.gps_history_valid)
		{
			cog_delta = cog_circular - cog_previous;
			sog_delta = sog_gps.BB - sog_previous;
			climb_rate_delta = climb_gps.BB - climb_rate_previous;

			location_deltaXY.x = location[0] - location_previous[0];
			location_deltaXY.y = location[1] - location_previous[1];
			location_deltaZ = location[2] - location_previous[2];
		}
		else
		{
			cog_delta = 0;
                        sog_delta = 0;
                        climb_rate_delta = 0;
			location_deltaXY.x = 0;
                        location_deltaXY.y = 0;
                        location_deltaZ = 0;
		}
		dcm_flags._.gps_history_valid = 1;
		actual_dir = cog_circular + cog_delta;
		cog_previous = cog_circular;

		// Note that all these velocities are in centimeters / second

		ground_velocity_magnitudeXY = sog_gps.BB + sog_delta;
		sog_previous = sog_gps.BB;

		GPSvelocity.z = climb_gps.BB + climb_rate_delta;
		climb_rate_previous = climb_gps.BB;
		
		accum_velocity.WW = (__builtin_mulss(cosine(actual_dir), ground_velocity_magnitudeXY) << 2) + 0x00008000;
		GPSvelocity.x = accum_velocity._.W1;

		accum_velocity.WW = (__builtin_mulss(sine(actual_dir), ground_velocity_magnitudeXY) << 2) + 0x00008000;
		GPSvelocity.y = accum_velocity._.W1;

		rotate(&location_deltaXY, cog_delta); // this is a key step to account for rotation effects!!

                //  rotate xy by angle, measured in a counter clockwise sense.
		GPSlocation.x = location[0] + location_deltaXY.x;
		GPSlocation.y = location[1] + location_deltaXY.y;
		GPSlocation.z = location[2] + location_deltaZ;

		location_previous[0] = location[0];
		location_previous[1] = location[1];
		location_previous[2] = location[2];

		velocity_thru_air.y = GPSvelocity.y - estimatedWind[1];
		velocity_thru_air.x = GPSvelocity.x - estimatedWind[0];
		velocity_thru_airz  = GPSvelocity.z - estimatedWind[2];

#if (SILSIM != 1 && BOARD_TYPE == AUAV3_BOARD)
                // assert digital out 2
                DIG2 = 1;
#endif
                // repeat the location[] calculation in single precision floating point
                // estimated additional 11000 cycles per call
                //________________________________________________________________________
                //                const float eR = 6371.0e3f;

                // difference is 32 bit integer degrees*10^7; precision about 1/90 meter, about 10^-2
                // Since 1 meter = 90 counts, converting the difference to 32 bit floating point
                // with 24 bit mantissa provides precision of 2^-24 = 6*10^-8 meters * 2^exponent. 
                // For displacements up to 131 Kilometers (exponent 17)
                // this exceeds the integer precision.
                loc_f[1] = (float) (lat_gps.WW - lat_origin.WW) / 89.983f; // 122 + 361 cycles
                loc_f[0] = ((float) (long_gps.WW - long_origin.WW) / 89.983f) * cos_lat_f; // 122 + 109
                loc_f[2] = (float) (alt_sl_gps.WW - alt_origin.WW) / 100.0f; // 122 + 361

                // convert from unit16_t deg*100 to FP degrees and Cartesian angle [-180, 180) degrees
                cog_circ_f = circ360_f(90.0f - (cog_gps.BB / 100.0f)); // 2*122 + 361

                if (dcm_flags._.gps_history_valid) {
                    cog_delta_f = circ360_f(cog_circ_f - cog_prev_f); // 2*122 cycles

                    location_deltaXY_f.x = loc_f[0] - location_prev_f[0]; // 122
                    location_deltaXY_f.y = loc_f[1] - location_prev_f[1]; // 122
                    location_deltaZ_f = loc_f[2] - location_prev_f[2]; // 122
                } else {
                    cog_delta_f = 0;
                    location_deltaXY_f.x = 0.0f;
                    location_deltaXY_f.y = 0.0f;
                    location_deltaZ_f = 0.0f;
                }
		actual_dir_f = circ360_f(cog_circ_f + cog_delta_f);   // 2*122
                cog_prev_f = cog_circ_f;

                rotate_f(&location_deltaXY_f, cog_delta_f); // 6167 cycles

                GPSloc_f.x = loc_f[0] + location_deltaXY_f.x;   // 3 * 122
		GPSloc_f.y = loc_f[1] + location_deltaXY_f.y;
		GPSloc_f.z = loc_f[2] + location_deltaZ_f;

		location_prev_f[0] = loc_f[0];
		location_prev_f[1] = loc_f[1];
		location_prev_f[2] = loc_f[2];

                //________________________________________________________________________

                // from dead_reckon()
                IMUlocationx._.W1 = (int) GPSloc_f.x;
                IMUlocationy._.W1 = (int) GPSloc_f.y;
                IMUlocationz._.W1 = (int) GPSloc_f.z;

                IMUlocationx._.W0 = abs((int) (65536 * (GPSloc_f.x - IMUlocationx._.W1) - 0.5)); // 3 * (2 * 122 + 109) = 1059
                IMUlocationy._.W0 = abs((int) (65536 * (GPSloc_f.y - IMUlocationy._.W1) - 0.5));
                IMUlocationz._.W0 = abs((int) (65536 * (GPSloc_f.z - IMUlocationz._.W1) - 0.5));

                IMUintegralAccelerationx._.W0 = 0;
                IMUintegralAccelerationy._.W0 = 0;
                IMUintegralAccelerationz._.W0 = 0;

                IMUintegralAccelerationx._.W1 = GPSvelocity.x;
                IMUintegralAccelerationy._.W1 = GPSvelocity.y;
                IMUintegralAccelerationz._.W1 = GPSvelocity.z;

#if (SILSIM != 1 && BOARD_TYPE == AUAV3_BOARD)
                // deassert digital out 2
                DIG2 = 0;
                // elapsed time ~100 usec = 7000 cycles at 70 MIPS
                // at 4 Hz, this is 28000/70e6 = 0.04% cpu load
#endif
#if (HILSIM == 1)
		air_speed_3DGPS = as_sim.BB; // use Xplane as a pitot
#else
		air_speed_3DGPS = vector3_mag(velocity_thru_air.x, velocity_thru_air.y, velocity_thru_airz);
#endif

		calculated_heading  = rect_to_polar(&velocity_thru_air);
		// veclocity_thru_air.x becomes XY air speed as a by product of CORDIC routine in rect_to_polar()
		air_speed_magnitudeXY = velocity_thru_air.x; // in cm / sec

#if (GPS_RATE == 1)
                // EM406 standard GPS reports once per second
                forward_acceleration = (air_speed_3DGPS - velocity_previous);
#else
                // multiply by GPS_RATE
                forward_acceleration = (air_speed_3DGPS - velocity_previous) * GPS_RATE;
#endif

		velocity_previous = air_speed_3DGPS;

		estimateWind();
		estAltitude();
		estYawDrift();
		dcm_flags._.yaw_req = 1;       // request yaw drift correction
		dcm_flags._.reckon_req = 1;    // request dead reckoning correction
		dcm_flags._.rollpitch_req = 1;
#if (DEADRECKONING == 0)
		process_flightplan();
#endif
		LED_BLUE = LED_OFF;
	}
	else
	{
		gps_data_age = HEARTBEAT_HZ * (GPS_DATA_MAX_AGE + 1);
		dirovergndHGPS[0] = dirovergndHRmat[0];
		dirovergndHGPS[1] = dirovergndHRmat[1];
		dirovergndHGPS[2] = 0;
		dcm_flags._.yaw_req = 1;           // request yaw drift correction
		dcm_flags._.gps_history_valid = 0; // gps history has to be restarted
	}
}

#define MS_PER_DAY 86400000 // = (24 * 60 * 60 * 1000)
const uint8_t days_in_month[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
static uint8_t day_of_week;

int16_t calculate_week_num(int32_t date)
{
//	printf("date %li\r\n", date);

	// Convert date from DDMMYY to week_num and day_of_week
	uint8_t year = date % 100;
	date /= 100;
	uint8_t month = date % 100;
	date /= 100;
	int16_t day = date % 100;

	// Wait until we have real date data
	if (day == 0 || month == 0) return 0;

	// Begin counting at May 1, 2011 since this 1st was a Sunday
	uint8_t m = 5;                          // May
	uint8_t y = 11;                         // 2011
	int16_t c = 0;                          // loop counter

	while (m < month || y < year)
	{
		day += days_in_month[m-1];          // (m == 1) means Jan, so use days_in_month[0]
		if ((m == 2) && (y % 4 == 0) && (y % 100 != 0))
		{
			day += 1;                       // Add leap day
		}
		m++;
		if (m == 13)
		{
			m = 1;
			y++;
		}
		if (++c > 1200) break;              // Emergency escape from this loop.  Works correctly until May 2111.
	}
	day_of_week = (day % 7) - 1;
	return (1634 + (day / 7));              // We started at week number 1634
}

int32_t calculate_time_of_week(int32_t time)
{
//	printf("time %li\r\n", time);

	// Convert time from HHMMSSmil to time_of_week in ms
	int16_t ms = time % 1000;
	time /= 1000;
	uint8_t s = time % 100;
	time /= 100;
	uint8_t m = time % 100;
	time /= 100;
	uint8_t h = time % 100;
	time = (((((int32_t)(h)) * 60) + m) * 60 + s) * 1000 + ms;
	return (time + (((int32_t)day_of_week) * MS_PER_DAY));
}
