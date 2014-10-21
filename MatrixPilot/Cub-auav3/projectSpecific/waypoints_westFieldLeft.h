// AAM West Field runway center  39°50'31.97"N  105°13'10.17"W (105.2194917, 39.842213889)
#define USE_FIXED_ORIGIN		1
#define FIXED_ORIGIN_LOCATION	{ -1052194917, 398422138, 1817.0 }

#pragma message "West Field lefthand pattern"

////////////////////////////////////////////////////////////////////////////////
// This is a lefthand pattern for takeoff to the east

#define USE_FIXED_ORIGIN		1

const struct waypointDef waypoints[] = {
	{ { 84, 3, 20 } , F_NORMAL + F_TAKEOFF, CAM_VIEW_LAUNCH } , //Waypoint 1
	{ { 83, 49, 30 } , F_NORMAL , CAM_VIEW_LAUNCH } , //Waypoint 2
	{ { -103, 53, 30 } , F_NORMAL , CAM_VIEW_LAUNCH } , //Waypoint 3
	{ { -103, 7, 25 } , F_NORMAL , CAM_VIEW_LAUNCH } , //Waypoint 4
	{ { -59, 6, 10 } , F_NORMAL , CAM_VIEW_LAUNCH } , //Waypoint 4
	{ { -14, 4,10 } , F_NORMAL , CAM_VIEW_LAUNCH } , //Waypoint 5
	{ { 15, 3, 20 } , F_NORMAL + F_TRIGGER , CAM_VIEW_LAUNCH } , //Waypoint 6
};
//const struct waypointDef waypoints[] = {
//	{ { 51, 3, 20 } , F_TAKEOFF , CAM_VIEW_LAUNCH } , //Waypoint 1
//	{ { 98, 2, 35 } , F_CROSS_TRACK , CAM_VIEW_LAUNCH } , //Waypoint 2
//	{ { 109, 4, 35 } , F_CROSS_TRACK , CAM_VIEW_LAUNCH } , //Waypoint 3
//	{ { 111, 18, 35 } , F_CROSS_TRACK , CAM_VIEW_LAUNCH } , //Waypoint 4
//	{ { 110, 57, 35 } , F_CROSS_TRACK , CAM_VIEW_LAUNCH } , //Waypoint 5
//	{ { 104, 62, 35 } , F_CROSS_TRACK , CAM_VIEW_LAUNCH } , //Waypoint 6
//	{ { 94, 63, 35 } , F_CROSS_TRACK , CAM_VIEW_LAUNCH } , //Waypoint 7
//	{ { -100, 68, 35 } , F_CROSS_TRACK , CAM_VIEW_LAUNCH } , //Waypoint 8
//	{ { -109, 63, 35 } , F_CROSS_TRACK , CAM_VIEW_LAUNCH } , //Waypoint 9
//	{ { -115, 50, 35 } , F_CROSS_TRACK , CAM_VIEW_LAUNCH } , //Waypoint 10
//	{ { -114, 21, 20 } , F_CROSS_TRACK , CAM_VIEW_LAUNCH } , //Waypoint 11
//	{ { -109, 11, 20 } , F_CROSS_TRACK , CAM_VIEW_LAUNCH } , //Waypoint 12
//	{ { -99, 5, 20 } , F_CROSS_TRACK , CAM_VIEW_LAUNCH } , //Waypoint 13
//	{ { -84, 5, 10 } , F_CROSS_TRACK , CAM_VIEW_LAUNCH } , //Waypoint 14
//	{ { -67, 5, 10 } , F_CROSS_TRACK , CAM_VIEW_LAUNCH } , //Waypoint 15
//	{ { -51, 4, 10 } , F_CROSS_TRACK , CAM_VIEW_LAUNCH } , //Waypoint 16
//	{ { -28, 4, 10 } , F_CROSS_TRACK , CAM_VIEW_LAUNCH } , //Waypoint 17
//	{ { -10, 5, 10 } , F_CROSS_TRACK , CAM_VIEW_LAUNCH } , //Waypoint 18
//	{ { 9, 4, 10 } , F_CROSS_TRACK + F_TRIGGER , CAM_VIEW_LAUNCH } , //Waypoint 19
//};

const struct waypointDef rtlWaypoints[] = {
		{ { 0, 30,  30 } , F_LOITER, CAM_VIEW_LAUNCH } ,
} ;




