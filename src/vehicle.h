#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iterator>
#include "map.h"


using namespace std;

#define SAVED_PREV_N        50 //Store the last 1 second info
#define EGO_V_ID            360007
#define HIGH_COST           999
#define INVALID_COST        9999

//
extern double previous_cycle_time;

class Vehicle {
 public:

  /**
   * Constructor
   */
  //Vehicle(); //default constructor
  Vehicle(int v_id, double x, double y, double vx, double vy, double s, double d);
  
  /**
   * Destructor
   */
  virtual ~Vehicle();

  int v_id;
  
  Waypoints *waypoints;

  typedef struct {
    double time_stamp; //time at which the measurement received
    
    double x; //map x. given.
    
    double y; //map y. given.
    
    double vx; //given
    
    double vy; //given
    
    double s; //given
    
    double yaw; //given. only for ego vehicle

    double v;

    double a;

    double jerk;

    int lane;

    double d; //given

    double dv;
    
    double da;
    
  } vehicle_info;
  
  vehicle_info prev[SAVED_PREV_N];
  vehicle_info cur;

  double ref_vel = 0;

  typedef enum {
    KL   = 0,
    PLCL = 1,
    PLCR = 2,
    LCL  = 3,
    LCR  = 4
  }states_t;

  string states_str [5] = {
    "KL",
    "PLCL",
    "PLCR",
    "LCL",
    "LCR"
  };
  
  states_t state;
  
  vector <int> go_directions = {-2, -1, 0, 1, 2};
  
  typedef enum {
    EXTREME_LEFT,
    LEFT,
    STRAIGHT,
    RIGHT,
    EXTREME_RIGHT
  }go_dir_t;
  
  string go_dir_str [5] = {
    "EXTREME LEFT",
    "LEFT",
    "STRAIGHT",
    "RIGHT",
    "EXTREME RIGHT"
  };
  

  /** NEW **/
  
  vector<vector<double> > generate_predictions(int horizon);
  
  void update(double time_stamp, double x, double y, double vx, double vy, double s, double d);

  void update_ego(double time_stamp, double x, double y, double s, double d, double v, double yaw);


  double get_cur_accl();

  double get_cur_jerk();

  double get_cur_d_vel();
  
  double get_cur_d_accl();
  
  void update_previous();

  void configure(Waypoints *waypoints);

  int cur_lane_number();

  int lane_number(double s, double d);

  typedef struct {
    bool collision;
    int v_id;
    double in_time;
  }sx_collision_t;

  typedef struct {
    Vehicle *v;
    double segment;
    double separation;
    bool truly_open_road;
  }nbr_t;
  
  typedef struct sx_trajectory {
    double ref_s;
    double new_lane;
    double ref_vel;
    double new_accl;
    double target_dist;
    nbr_t in_front;
    nbr_t in_back;
    vector <double> next_x_vals;
    vector <double> next_y_vals;
    sx_collision_t coll;
    bool open_road;
    double lateral_accel;
  }sx_trajectory_t;
  
  typedef struct {
    int prev_lane;
    int new_lane;
  }lcx_obj_t;

  lcx_obj_t *lcx_obj;
  
  //structure to store state transition results...
  typedef struct {
    states_t state;
    double cost;
    double ref_vel;
    vector <double> next_x_vals;
    vector <double> next_y_vals;
    sx_trajectory_t *cur_t;
    sx_trajectory_t *new_t;
  }sx_result_t;

  //State transition functions
  //  typedef std::tuple < double, double, vector <double > , vector <double> >
  typedef sx_result_t
    (Vehicle::*state_x_func_t) (states_t new_state,
				sx_trajectory_t *ex_left,
				sx_trajectory_t *left,
				sx_trajectory_t *straight,
				sx_trajectory_t *right,
				sx_trajectory_t *ex_right,
				map<int, Vehicle> *vehicles);
  
  //Finite state machine
  vector<vector <state_x_func_t> > fsm;

  
  //Class functions
  sx_result_t
    sx_NOP(states_t new_state,
	   sx_trajectory_t *ex_left,
	   sx_trajectory_t *left,
	   sx_trajectory_t *straight,
	   sx_trajectory_t *right,
	   sx_trajectory_t *ex_right,
	   map<int, Vehicle> *vehicles);
  
  sx_result_t
    sx_func(states_t new_state,
	    sx_trajectory_t *ex_left,
	    sx_trajectory_t *left,
	    sx_trajectory_t *straight,
	    sx_trajectory_t *right,
	    sx_trajectory_t *ex_right,
	    map<int, Vehicle> *vehicles);

  std::tuple <vector <double>, vector <double> > 
    update_state(map<int, Vehicle> *vehicles,
		 vector <double> *prev_path_x,
		 vector <double> *prev_path_y,
		 Waypoints *waypoints,
		 double ref_s);

  std::tuple <sx_trajectory_t *, sx_trajectory_t *, sx_trajectory_t *>
    sx_get_cur_new_trajectories(states_t old_state,
				states_t new_state,
				sx_trajectory_t *ex_left,
				sx_trajectory_t *left,
				sx_trajectory_t *straight,
				sx_trajectory_t *right,
				sx_trajectory_t *ex_right);

  
  vector<double> state_at(double t);

  sx_result_t
    get_next_best_state(states_t cur_state,
			sx_trajectory_t *left,
			sx_trajectory_t *straight,
			sx_trajectory_t *right,
			vector <sx_result_t> sx_rets);

  double
    collision_cost(sx_trajectory_t *trajectory, string dbg_str);

  double
    time_diff_cost(double lookahead_dist, sx_trajectory_t *trajectory,
		   string dbg_str);

  double
    time_diff_ext_cost(double lookahead_dist, sx_trajectory_t *trajectory,
		       string dbg_str);

  double
    open_road_cost(sx_trajectory_t *trajectory, string dbg_str);
  
  double
    truly_open_road_cost(sx_trajectory_t *trajectory, string dbg_str);
  
  double
    lateral_accl_cost(sx_trajectory_t *trajectory, string dbg_str);

  double
    lc_min_speed_cost();

  double
    lc_complete_cost();
    

  tuple < bool, int, double>
    check_for_collision(vector <double> *x,
			vector <double> *y,
			nbr_t in_front,
			nbr_t in_back,
			double new_ref_vel);
  
  std::tuple<double, double, double, bool>
    get_new_ref_vel_for_lane(bool prep_lc,
			     nbr_t in_front,
			     nbr_t in_back,
			     int lane,
			     double segment);
  
  std::tuple <nbr_t, nbr_t>
    find_nearest_front_back_in_lane(map<int, Vehicle> *vehicles,
				    int lane, double s);
  bool
    sx_valid_lane_change(states_t new_state);

  std::tuple < double, double, bool>
    get_max_accel_for_lane(nbr_t in_front,
			   int lane, double s);

  std::tuple < double, double, bool>
    get_accel_for_plcx(nbr_t in_front,
		       nbr_t in_back,
		       int lane, double segment);
};

#endif
