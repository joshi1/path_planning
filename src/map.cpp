#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
//#include "matplotlibcpp.h"
#include "spline.h"
#include "map.h"
#include "debug.h"
#include "helpers.h"
//namespace plt = matplotlibcpp;

using namespace std;


// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }


std::tuple<vector<double>, vector<double>, vector<double> >
spline_waypoints_xys(vector<double> X, 
		     vector<double> Y,
		     vector<double> S,
		     int num){

  std::vector<double> newX, newY, newS;

  tk::spline s_x;
  s_x.set_points(S, X);

  tk::spline s_y;
  s_y.set_points(S, Y);
  
  
  int num_points = S.size();
  
  for(int i=0; i< X.size()-1; i++)
    {
      double range = (S[i+1] - S[i])/num;

      for(int j=1; j<=num; j++){
	double s = S[i] + j*range;
	double x = s_x(s);
	double y = s_y(s);
	newX.push_back(x);
	newY.push_back(y);
	newS.push_back(s);
      }
    }
  
  return std::make_tuple(newX, newY, newS);
  
}




std::tuple<vector<double>, vector<double> >
spline_waypoints_xy(vector<double> X, 
		    vector<double> Y,
		    int num){
  
  std::vector<double> newX, newY;

  bool x_ascending = true;
  bool y_ascending = true;

  for(int i = 0 ; i < X.size()-1; i++){
    if(X[i] > X[i+1]){
      x_ascending = false;
    }
  }
  
  for(int i = 0 ; i < Y.size()-1; i++){
    if(Y[i] > Y[i+1]){
      y_ascending = false;
    }
  }

  if((x_ascending == false)
     && (y_ascending == false)){
    printf("*************************************** \n");
    printf("********* CANNOT USE SPLINE!! ********* \n");
    printf("*************************************** \n");
  }

  if(x_ascending)
    {
      tk::spline spline;
      spline.set_points(X,Y);
      
      int num_points = X.size();
      
      for(int i=0; i< X.size()-1; i++)
	{
	  
	  double x_range = (X[i+1] - X[i])/num;
	  
	  for(int j=1; j<=num; j++){
	    double x = X[i] + j*x_range;
	    double y = spline(x);
	    newX.push_back(x);
	    newY.push_back(y);
	  }
	}
    }
  else
    {
      tk::spline spline;
      spline.set_points(Y,X);
      
      int num_points = Y.size();
      
      for(int i=0; i< Y.size()-1; i++)
	{
	  
	  double y_range = (Y[i+1] - Y[i])/num;
	  
	  for(int j=1; j<=num; j++){
	    double y = Y[i] + j*y_range;
	    double x = spline(y);
	    newX.push_back(x);
	    newY.push_back(y);
	  }
	}
    }
  return std::make_tuple(newX, newY);
  
}

std::vector<double>
spline_waypoints_d(vector<double> x1, 
		   vector<double> y1,
		   vector<double> x2, 
		   vector<double> y2){

  std::vector<double> d;

  //Note that x2.size is deliberate. Because x2 is smaller here because
  //there is no way to find out the xd,yd for spline points of the last segment.
  //Setting them to 1. See the use of the function.
  for(int i=0; i< x2.size(); i++)
    {
      double d_val = distance(x1[i], y1[i], x2[i], y2[i]);
      d.push_back(d_val);
    }

  return d;
}

double 
spline_length(vector<double> X, 
	      vector<double> Y)
{
  double dist = 0;
  
  for(int i=1; i< X.size(); i++)
    {

      dist += distance(X[i-1], Y[i-1], X[i], Y[i]); 
    }
  
  return dist;
  
}


void
waypoints_deduplicate(Waypoints *waypoints)
{
  vector<double> map_waypoints_x  = waypoints->x;
  vector<double> map_waypoints_y  = waypoints->y;
  vector<double> map_waypoints_s  = waypoints->s;
  vector<double> map_waypoints_dx = waypoints->dx;
  vector<double> map_waypoints_dy = waypoints->dy;
  int num_spline_points           = waypoints->num_spline_points;

  vector<double> new_x;
  vector<double> new_y;
  vector<double> new_s;
  vector<double> new_dx;
  vector<double> new_dy;
  
  bool created = false;
  int num_created = 0;
  for(int i = 0 ; i < map_waypoints_x.size() - 1; i++)
    {
      if((created == false)
	 && (map_waypoints_s[i+1] > map_waypoints_s[i]))
	{
	  new_x.push_back(map_waypoints_x[i]);
	  new_y.push_back(map_waypoints_y[i]);
	  new_s.push_back(map_waypoints_s[i]);
	  new_dx.push_back(map_waypoints_dx[i]);
	  new_dy.push_back(map_waypoints_dy[i]);
	  num_created ++;
	}
      else
	{
	  created = true;
	  break;
	  int ix = i%num_created;
	  int ix2 = i+1;
	  if((!doubleEquals(new_x[ix], map_waypoints_x[ix2]))
	     || !doubleEquals(new_y[ix], map_waypoints_y[ix2])
	     || !doubleEquals(new_s[ix], map_waypoints_s[ix2])
	     || !doubleEquals(new_dx[ix], map_waypoints_dx[ix2])
	     || !doubleEquals(new_dy[ix], map_waypoints_dy[ix2])){
	    DEBUG(" Match Failed! " << i << "," << ix);
	    DEBUG(" (" << new_x[ix]
		  << "," << new_y[ix]
		  << "," << new_s[ix]
		  << "," << new_dx[ix]
		  << "," << new_dy[ix]
		  << ")");
	    DEBUG(" (" << map_waypoints_x[ix2]
		  << "," << map_waypoints_y[ix2]
		  << "," << map_waypoints_s[ix2]
		  << "," << map_waypoints_dx[ix2]
		  << "," << map_waypoints_dx[ix2]
		  << ")");
	    
	  } else {
	    DEBUG(" Matched!");
	  }
	}
      
    }
  waypoints->x = new_x;
  waypoints->y = new_y;
  waypoints->s = new_s;
  waypoints->dx = new_dx;
  waypoints->dy = new_dy;
  
  
  //exit(EXIT_SUCCESS);
}

void
waypoints_hash(Waypoints *waypoints)
{
  for(int i = 0; i < waypoints->x_spline.size(); i++)
    {
      pair<coord, int> p((coord{waypoints->x_spline[i], waypoints->y_spline[i]}),
			 i);
      waypoints->m.insert(p);
    }
  
}

int
ClosestWaypoint_hash(double x,
		     double y,
		     map <coord, int> *m)
{

  GET_CUR_TIME_START();
  int ix = 0;
  coord pt = {x, y};
  
  map <coord, int>::iterator it;

  it = m->find(pt);
  
  if(it != m->end())
    {
      ix = it->second;
    }
  else
    {
      DEBUG(" *** Problem finxing index!! ");
    }
  GET_CUR_TIME_END();
  GET_DURATION("closest waypoint hash lookup: ");
  
  return ix;
}

bool
waypoints_spline(Waypoints *waypoints){

  waypoints_deduplicate(waypoints);
  
  vector<double> map_waypoints_x  = waypoints->x;
  vector<double> map_waypoints_y  = waypoints->y;
  vector<double> map_waypoints_s  = waypoints->s;
  vector<double> map_waypoints_dx = waypoints->dx;
  vector<double> map_waypoints_dy = waypoints->dy;
  int num_spline_points           = waypoints->num_spline_points;

  int num_map_waypoints = map_waypoints_x.size();
  DEBUG("Waypoints " << num_map_waypoints);

  vector<double> map_waypoints_xd;
  vector<double> map_waypoints_yd;


 
  for (int i = 0; i < map_waypoints_x.size() ; i++) {

    DEBUG(i << "."
	  "x:"<< map_waypoints_x[i] << ","
	  "y:"<< map_waypoints_y[i] << ","
	  "s:"<< map_waypoints_s[i] << ","
	  "dx:"<< map_waypoints_dx[i] << ","
	  "dy:"<< map_waypoints_dy[i]);
    
    map_waypoints_xd.push_back(map_waypoints_x[i] + map_waypoints_dx[i]);
    map_waypoints_yd.push_back(map_waypoints_y[i] + map_waypoints_dy[i]);
      
  }
  
  DEBUG("Call waypoints_xys...");
  //For the centre of the road
  vector<double> waypoints_x_spline;
  vector<double> waypoints_y_spline;
  vector<double> waypoints_s_spline;
  
  std::tie(waypoints_x_spline, waypoints_y_spline, waypoints_s_spline) =
    spline_waypoints_xys(map_waypoints_x, map_waypoints_y, map_waypoints_s,
			 num_spline_points);

  
  //Circular route. So spline for the last 2 points
  vector<double> waypoints_x_spline_last;
  vector<double> waypoints_y_spline_last;
  vector<double> waypoints_s_spline_last;

  DEBUG(" X " << "(" 
	<< map_waypoints_x[num_map_waypoints-1] << ","
	<< map_waypoints_x[0] << ","
	<< map_waypoints_x[1] << ")");

  DEBUG(" Y " << "("
	<< map_waypoints_y[num_map_waypoints-1] << ","
	<< map_waypoints_y[0] << ","
	<< map_waypoints_y[1] << ")");

  DEBUG(" S " << "("
	<< map_waypoints_s[num_map_waypoints-1] << ","
	<< map_waypoints_s[0] << ","
	<< map_waypoints_s[1] << ")");

  DEBUG("Call waypoints_xy last...");
  std::tie(waypoints_x_spline_last, waypoints_y_spline_last) =
    spline_waypoints_xy(
			{map_waypoints_x[num_map_waypoints-1],
			    map_waypoints_x[0],
			    map_waypoints_x[1]},
			{map_waypoints_y[num_map_waypoints-1],
			    map_waypoints_y[0],
			    map_waypoints_y[1]},
			num_spline_points);

  //Only keep the first num_spline_points
  vector <double> tmp_x;
  vector <double> tmp_y;
  for(int i = 0; i < num_spline_points; i++){
    tmp_x.push_back(waypoints_x_spline_last[i]);
    tmp_y.push_back(waypoints_y_spline_last[i]);
  }  
  waypoints_x_spline_last = tmp_x;
  waypoints_y_spline_last = tmp_y;
  
  double spline_len = spline_length(waypoints_x_spline_last,
				    waypoints_y_spline_last);

  DEBUG(" Last spline len " << spline_len);
  double spline_len_inc = spline_len/num_spline_points;
  double spline_s_last = map_waypoints_s[num_map_waypoints-1];
  
  for(int i = 0; i < num_spline_points; i++){
    spline_s_last += spline_len_inc;
    waypoints_s_spline_last.push_back(spline_s_last);
  }
  
  for(int i = 0; i < waypoints_x_spline_last.size() ; i++){
    DEBUG(i+1 << "."
	  << " X, Y, S " << "("
	  << waypoints_x_spline_last[i] << ", "
	  << waypoints_y_spline_last[i] << ", "
	  << waypoints_s_spline_last[i]
	  << ")");
      
  }
  
  //Concatenate the vectors to get a final vectors
  waypoints_x_spline.insert(waypoints_x_spline.end(),
			    waypoints_x_spline_last.begin(),
			    waypoints_x_spline_last.end());

  waypoints_y_spline.insert(waypoints_y_spline.end(),
			    waypoints_y_spline_last.begin(),
			    waypoints_y_spline_last.end());

  waypoints_s_spline.insert(waypoints_s_spline.end(),
			    waypoints_s_spline_last.begin(),
			    waypoints_s_spline_last.end());

  

  //For d, the first lane from the centre of the road
  vector<double> waypoints_xd_spline;
  vector<double> waypoints_yd_spline;
  vector<double> unused1;
  
  std::tie(waypoints_xd_spline, waypoints_yd_spline, unused1) =
    spline_waypoints_xys(map_waypoints_xd, map_waypoints_yd, map_waypoints_s,
			 num_spline_points);
  /***
  //Circular route. So spline for the last 2 points
  vector<double> waypoints_xd_spline_last;
  vector<double> waypoints_yd_spline_last;
  
  std::tie(waypoints_xd_spline_last, waypoints_yd_spline_last) =
    spline_waypoints_xy(
			{map_waypoints_xd[num_map_waypoints-1],
			    map_waypoints_xd[0],
			    map_waypoints_xd[1]},
			{map_waypoints_yd[num_map_waypoints-1],
			    map_waypoints_yd[0],
			    map_waypoints_yd[1]},
			num_spline_points);

  
  //Only keep the first num_spline_points
  vector <double> tmp_xd;
  vector <double> tmp_yd;
  for(int i = 0; i < num_spline_points; i++){
    tmp_x.push_back(waypoints_xd_spline_last[i]);
    tmp_y.push_back(waypoints_yd_spline_last[i]);
  }  
  waypoints_xd_spline_last = tmp_x;
  waypoints_yd_spline_last = tmp_y;
  
  
  for (int i = 0; i < num_spline_points; i++) {
    waypoints_xd_spline.push_back(1);
    waypoints_yd_spline.push_back(1);
    }
  **/
  vector<double> waypoints_spline_d;
  waypoints_spline_d = spline_waypoints_d(waypoints_x_spline,
					  waypoints_y_spline,
					  waypoints_xd_spline,
					  waypoints_yd_spline);

  DEBUG(" spline_waypoints_d size = " << waypoints_spline_d.size());
  for (int i = 0; i < num_spline_points; i++) {
    waypoints_spline_d.push_back(1.0);
  }
  
  //Copy back to variable
  waypoints->x_spline = waypoints_x_spline;
  waypoints->y_spline = waypoints_y_spline;
  waypoints->s_spline = waypoints_s_spline;
  
  waypoints->x_plusd_spline = waypoints_xd_spline;
  waypoints->y_plusd_spline = waypoints_yd_spline;

  waypoints->spline_d = waypoints_spline_d;
  
  //plt::plot(waypoints->x_spline, waypoints->y_spline, "r^");
  //plt::plot(waypoints->x_plusd_spline, waypoints->y_plusd_spline, "g^");
  //plt::plot(waypoints_xd_spline, waypoints_yd_spline);
  //plt::show();

  DEBUG("X Spline size " << waypoints_x_spline.size());
  DEBUG("Y Spline size " << waypoints_y_spline.size());
  DEBUG("S Spline size " << waypoints_s_spline.size());
  DEBUG("X+d Spline size " << waypoints_xd_spline.size());  
  DEBUG("Y+d Spline size " << waypoints_yd_spline.size());  
  DEBUG("d Spline size " << waypoints_spline_d.size());  

  
  for (int i = 0; i < waypoints_x_spline.size() ; i++) {
    DEBUG(i << "."
	  << "x:" << waypoints_x_spline[i]
	  << "y:" << waypoints_y_spline[i]
	  << "s:" << waypoints_s_spline[i]
	  << "d:" << waypoints_spline_d[i]);

  }
  return true;
  
}

double
distance(double x1, double y1, double x2, double y2)
{
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}


int
ClosestWaypoint(double x,
		double y,
		vector<double> maps_x,
		vector<double> maps_y)
{

  
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for(int i = 0; i < maps_x.size(); i++)
    {
      double map_x = maps_x[i];
      double map_y = maps_y[i];
      double dist = distance(x,y,map_x,map_y);
      if(dist < closestLen)
	{
	  closestLen = dist;
	  closestWaypoint = i;
	}

    }


  return closestWaypoint;

}

int
NextWaypoint(double x,
	     double y,
	     double theta,
	     vector<double> maps_x,
	     vector<double> maps_y)
{

  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2( (map_y-y),(map_x-x) );

  double angle = abs(theta-heading);

  if(angle > pi()/4)
    {
      closestWaypoint++;
    }

  return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double>
getFrenet(double x,
	  double y,
	  double theta,
	  vector<double> maps_x,
	  vector<double> maps_y)
{
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if(next_wp == 0)
    {
      prev_wp  = maps_x.size()-1;
    }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if(centerToPos <= centerToRef)
    {
      frenet_d *= -1;
    }

  // calculate s value
  double frenet_s = 0;
  for(int i = 0; i < prev_wp; i++)
    {
      frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
    }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double>
getXY(double s,
      double d,
      vector<double> maps_s,
      vector<double> maps_x,
      vector<double> maps_y)
{
  int prev_wp = -1;

  while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
    {
      prev_wp++;
    }
  //DEBUG("          prev_wp = " << prev_wp);
  
  int wp2 = (prev_wp+1)%maps_x.size();
  //DEBUG("          wp2     = " << wp2);
  
  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));

  //DEBUG("          heading= " << heading);
  
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);
  //DEBUG("          distance= " << seg_s);
  
  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);
  //DEBUG("             seg_x= " << seg_x);
  //DEBUG("             seg_y= " << seg_y);
  
  double perp_heading = heading-pi()/2;
  //DEBUG("             perp_heading= " << perp_heading);
  
  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  //DEBUG("             final_x= " << x);
  //DEBUG("             final_y= " << y);
  
  return {x,y};

}


/*******************************
 * Unused/Trial Functions below
 *******************************/ 
void spline_example()
{
  std::vector<double> X(5), Y(5);
  X[0]=0.1;
  X[1]=0.4;
  X[2]=1.2;
  X[3]=1.8;
  X[4]=2.0;
  Y[0]=0.1;
  Y[1]=0.7;
  Y[2]=0.6;
  Y[3]=1.1;
  Y[4]=0.9;

  tk::spline s;
  s.set_points(X,Y);
  
  for(size_t i=0; i<X.size(); i++){
    printf("%f %f\n", X[i], Y[i]);
  }
  printf("\n");

  std::vector<double> newX, newY;
  
  for(int i=-50; i<250; i++){
    double x=0.01*i;
    double y = s(x);
    printf("%f %f\n", x, s(x));
    newX.push_back(x);
    newY.push_back(y);
  }

  //plt::plot(X, Y);
  //plt::plot(newX, newY, "ro");
  //plt::show();
  
}
