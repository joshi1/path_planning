#ifndef WAYPOINTS_SPLINE_H
#define WAYPOINTS_SPLINE_H

#include <vector>
#include <map>

#include "Eigen-3.3/Eigen/Core"

using namespace std;

struct coord { 
    double x, y; 

    bool operator=(const coord &o) {
        return x == o.x && y == o.y;
    }

    bool operator<(const coord &o) const {
        return x < o.x || (x == o.x && y < o.y);
    }
};


typedef struct {
  //Input
  vector<double> x;
  vector<double> y;
  vector<double> s;
  vector<double> dx;
  vector<double> dy;
  int num_spline_points;
  
  //Required to spline the d points
  vector<double> x_plusd;
  vector<double> y_plusd;
  
  //Output
  //Points added to the map points
  vector<double> x_spline;
  vector<double> y_spline;
  vector<double> s_spline;
  
  //Points added to the d points
  vector<double> x_plusd_spline;
  vector<double> y_plusd_spline;

  //This is the distance between the above
  //IMPORTANT NOTE: NEED to verify how accurate this might actually be!!
  //  Since the x,y and dx,dy are splined independently
  vector<double> spline_d;

  map <coord, int> m;
} Waypoints;

extern bool waypoints_spline(Waypoints *waypoints);

extern double deg2rad(double x);
extern double rad2deg(double x);

extern vector<double>
getXY(double s,
      double d,
      vector<double> maps_s,
      vector<double> maps_x,
      vector<double> maps_y);

extern vector <double>
getFrenet(double x,
	  double y,
	  double theta,
	  vector<double> maps_x,
	  vector<double> maps_y);

extern int
NextWaypoint(double x,
	     double y,
	     double theta,
	     vector<double> maps_x,
	     vector<double> maps_y);

extern int
ClosestWaypoint(double x,
		double y,
		vector<double> maps_x,
		vector<double> maps_y);

extern int
ClosestWaypoint_hash(double x,
		     double y,
		     map <coord, int> *m);

extern double
distance(double x1, double y1, double x2, double y2);



#endif //WAYPOINTS_SPLINE_H
