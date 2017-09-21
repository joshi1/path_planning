#ifndef PTG_CONSTANTS_H
#define PTG_CONSTANTS_H
#include <vector>

using namespace std;

#define AGGRESSIVE_DRIVING   false

/** Vehicle constraints **/
#define VEHICLE_LENGTH       5
#define VEHICLE_CLOSEST_DIST (VEHICLE_LENGTH)

/** Road/Environment Constants **/
#define EGO_UPDATE_TIME      0.02 //Ego updated every .02 second
#define MAX_PATH_POINTS      75

#define MIN_LOOKAHEAD_DIST   50
#define MAX_LOOKAHEAD_DIST   220
#define MIN_NEXT_XY_TO_ADD   3
#define MIN_NEXT_XY_VALS     30
#define MAX_NEXT_XY_VALS     85

#define NUM_LANES            3
#define LANE_WIDTH           4
#define SPEED_LIMIT          22.352 //50mph in m/s
#define SPEED_LIMIT_MAX      (0.996*SPEED_LIMIT) 
#define PREFERRED_BUFFER     (2*VEHICLE_LENGTH) //meters
#define FRONT_SCAN_RADIUS    100 //meters
#define FRONT_SCAN_FURTHEST_RADIUS 200 //meters
#define MIN_SPEED_FOR_LC     10 //close to 25mph


//TAKE into account VEHICLE_LENGTH. 
//#define BACK_SCAN_RADIUS     PREFERRED_BUFFER + VEHICLE_LENGTH + 2 //meters. 
//#define BACK_SCAN_RADIUS     2*VEHICLE_LENGTH //meters. 
#define BACK_SCAN_RADIUS     20 //meters. 

/******* BELOW UNUSED *******/
/** PTG Constants **/
#define N_SAMPLES   10

#define SIGMA_S     {10.0, 4.0, 2.0} // sigma for s, s_dot, s_double_dot
#define SIGMA_D     {1.0, 1.0, 1.0}
#define SIGMA_T     2.0

// m/s/s/s
#define MAX_ACCL   (SPEED_LIMIT * EGO_UPDATE_TIME)
//#define MAX_ACCL   (SPEED_LIMIT_MAX * EGO_UPDATE_TIME)
//#define MAX_ACCL   0.224

// m/s/s
#define EXPECTED_JERK_IN_ONE_SEC   2
//m/sz
#define EXPECTED_ACC_IN_ONE_SEC    1 

#define VELOCITY_STOP_COST  0.8

#define DERIVATIVE_UPTO     3
#endif
