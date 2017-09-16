#ifndef PTG_H
#define PTG_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "map.h"
#include "vehicle.h"

using namespace std;


extern void
ptg_get_trajectory(double target_d,
		   int total_points,
		   double ref_s,
		   double ref_v,
		   double car_x,
		   double car_y,
		   double car_yaw,//in radians
		   int lane,
		   vector <double> *prev_path_x,
		   vector <double> *prev_path_y,
		   vector <double> *next_x_vals,
		     vector <double> *next_y_vals,
		   Waypoints *waypoints);

extern double
ptg_get_lateral_accel(vector< double> pt1,
		      vector <double> pt2,
		      vector<double> pt3,
		      double veloctiy);


#endif /* PTG_H */
