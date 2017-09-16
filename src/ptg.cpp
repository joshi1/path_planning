#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include "constants.h"
#include "ptg.h"
#include "debug.h"
#include "helpers.h"
#include "spline.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

void
ptg_get_trajectory(double target_d,
		   int total_points,
		   double ref_s,
		   double ref_v,
		   double car_x,
		   double car_y,
		   double car_yaw,//in radians
		   int lane,
		   vector <double> *previous_path_x_ptr,
		   vector <double> *previous_path_y_ptr,
		   vector <double> *next_x_vals_ptr,
		   vector <double> *next_y_vals_ptr,
		   Waypoints *waypoints_ptr)
{
  vector <double> &previous_path_x = *previous_path_x_ptr;
  vector <double> &previous_path_y = *previous_path_y_ptr;
  vector <double> &next_x_vals     = *next_x_vals_ptr;
  vector <double> &next_y_vals     = *next_y_vals_ptr;
  
  Waypoints &waypoints = *waypoints_ptr;
  
  GET_CUR_TIME_START();    
  int prev_size = previous_path_x.size();
  vector <double> ptsx;
  vector <double> ptsy;
		
  double ref_x = car_x;
  double ref_y = car_y;
  double ref_yaw = deg2rad(car_yaw);

  DEBUG("Get path trajectory...");
  DEBUG("  previous path leftover " << prev_size);
	       
  //Find the target s, v, 
  if(prev_size < 2)
    {
      double prev_car_x = car_x - cos(car_yaw);
      double prev_car_y = car_y - sin(car_yaw);

      ptsx.push_back(prev_car_x);
      ptsy.push_back(prev_car_y);
		    
      ptsx.push_back(car_x);
      ptsy.push_back(car_y);

    }
  else
    {
      ref_x = previous_path_x[prev_size - 1];
      ref_y = previous_path_y[prev_size - 1];


      double ref_x_prev = previous_path_x[prev_size - 2];
      double ref_y_prev = previous_path_y[prev_size - 2];

      ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

      ptsx.push_back(ref_x_prev);
      ptsy.push_back(ref_y_prev);

      ptsx.push_back(ref_x);
      ptsy.push_back(ref_y);

    }

  DEBUG(" reference s: " << ref_s);
  DEBUG(" to lane: " << (lane-1));
  
  vector <double> next_wp0 = getXY(ref_s+30,
				   2+4*(lane - 1),
				   waypoints.s,
				   waypoints.x,
				   waypoints.y);
		
  vector <double> next_wp1 = getXY(ref_s+60,
				   2+4*(lane - 1),
				   waypoints.s,
				   waypoints.x,
				   waypoints.y);;
		
  vector <double> next_wp2 = getXY(ref_s+90,
				   2+4*(lane - 1),
				   waypoints.s,
				   waypoints.x,
				   waypoints.y);;
		
  ptsx.push_back(next_wp0[0]);
  ptsy.push_back(next_wp0[1]);
		
  ptsx.push_back(next_wp1[0]);
  ptsy.push_back(next_wp1[1]);

  ptsx.push_back(next_wp2[0]);
  ptsy.push_back(next_wp2[1]);

  DEBUG(" pushed next 30, 60, 90 wps. total points " << ptsx.size());

  DEBUG(" Transform to vehicle coordinate system");



  std::tie(ptsx, ptsy) =
    tx_to_vehicle_coordinates(ptsx, ptsy, ref_x, ref_y, ref_yaw);


  tk::spline s;

  s.set_points(ptsx, ptsy);



  //vector<double> next_x_vals;
  //vector<double> next_y_vals;
		
  for(int i = 0; i < previous_path_x.size(); i++)
    {
      next_x_vals.push_back(previous_path_x[i]);
      next_y_vals.push_back(previous_path_y[i]);
    }

  double target_x = target_d;
  double target_y = s(target_x);
  double target_dist = sqrt((target_x*target_x) + (target_y*target_y));

  DEBUG(" target distance  " << target_dist);

  double N = (target_dist/(EGO_UPDATE_TIME*ref_v)); //(target_dist/(0.02*ref_v*0.44704)));
  DEBUG(" Divide target dist into " << N << " points"); 
  

  DEBUG(" total points " << total_points);
  DEBUG(" previous points " << previous_path_x.size());

  int points_added = total_points - previous_path_x.size();
  DEBUG(" added path points " << points_added);

  if(points_added <= 0){
    DEBUG(" **** Points failure " << points_added);
    exit(EXIT_FAILURE);
  }
  double x_add_on = 0;
  
  for(int i = 1; i <= points_added; i++){
				  
    double x_point = x_add_on + (target_x)/N;
    double y_point = s(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    x_point = (x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw));
    y_point = (x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw));

    x_point += ref_x;
    y_point += ref_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);

  }
  GET_CUR_TIME_END();
  GET_DURATION("******* new_trajectory:");

  DEBUG("Returning trajectory");

}

double
ptg_get_radius_of_curvature(vector< double> pt1,
			     vector <double> pt2,
			     vector<double> pt3)
{
  double x1 = pt1[0];
  double y1 = pt1[1];

  double x2 = pt2[0];
  double y2 = pt2[1];
  
  double x3 = pt3[0];
  double y3 = pt3[1];

  double m1 = (y2 - y1)/(x2-x1);
  double m2 = (y3 - y2)/(x3-x2);

  double xc = ((m1*m2)*(y1 - y3) + m2*(x1 + x2) - m1*(x2 + x3))/2*(m2-m1);

  double yc = -(1/m1)*(xc - (x1+x2)/2) + (y1+y2)/2;

  double curvature = sqrt((x1-xc)*(x1-xc) + (y1-yc)*(y1-yc));

  return curvature;
  
}

double
ptg_get_lateral_accel(vector< double> pt1,
		      vector <double> pt2,
		      vector<double> pt3,
		      double v)
{
  //radius of curvature
  double r = ptg_get_radius_of_curvature(pt1, pt2, pt3);
  
  return (v*v)/r;
}
