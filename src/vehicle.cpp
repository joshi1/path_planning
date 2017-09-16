#include <iostream>
#include <iostream>
#include <math.h>
#include <map>
#include <string>
#include <iterator>
#include "debug.h"

#include "vehicle.h"
#include "constants.h"
#include "helpers.h"
#include "ptg.h"
#include "road.h"

using namespace std;


/**
 * Initializes Vehicle
 */


Vehicle::Vehicle(int v_id, double x, double y,
		 double vx, double vy, double s, double d)
{
  
  ::memset( &this->cur, 0, sizeof( this->cur ) );
  ::memset( &this->prev, 0, sizeof( this->prev ) );

  this->state  = KL;
  this->v_id   = v_id;
  this->cur.x  = x;
  this->cur.y  = y;
  this->cur.vx = vx;
  this->cur.vy = vy;
  this->cur.s  = s;
  this->cur.d  = d;

  this->cur.v = sqrt(vx*vx + vy*vy);
  
  this->cur.yaw = 0;

  this->cur.time_stamp = -1;

  for(int i = 0; i < SAVED_PREV_N; i++){
    this->prev[i].time_stamp = -1; //Indicates invalid entry
  }

  
#ifdef DEBUGGING_KL_ONLY
  this->fsm = {
    /* KL   */{&Vehicle::sx_func}       //KL
  };
  
#else//DEBUGGING_KL_ONLY
  
  this->fsm = {
    /* KL   */{&Vehicle::sx_func,      //KL
	       &Vehicle::sx_func,      //PLCL
	       &Vehicle::sx_func,      //PLCR
	       &Vehicle::sx_NOP,       //LCL 
	       &Vehicle::sx_NOP},      //LCR
    /* PLCL */{&Vehicle::sx_func,          //KL
	       &Vehicle::sx_func,          //PLCL
	       &Vehicle::sx_NOP,           //PLCR
	       &Vehicle::sx_func,          //LCL
	       &Vehicle::sx_NOP},          //LCR
    /* PLCR */{&Vehicle::sx_func,      //KL
	       &Vehicle::sx_NOP,       //PLCL
	       &Vehicle::sx_func,      //PLCR
	       &Vehicle::sx_NOP,       //LCL
	       &Vehicle::sx_func},     //LCR
    /*  LCL */{&Vehicle::sx_func,         //KL
	       &Vehicle::sx_NOP,          //PLCL
	       &Vehicle::sx_NOP,          //PLCR
	       &Vehicle::sx_func,         //LCL
	       &Vehicle::sx_NOP},         //LCR
    /* LCR */ {&Vehicle::sx_func,     //KL
	       &Vehicle::sx_NOP,      //PLCL
	       &Vehicle::sx_NOP,      //PLCR
	       &Vehicle::sx_NOP,      //LCL
	       &Vehicle::sx_func}     //LCR
  };
#endif
  
}

void Vehicle::update_previous(){

  for(int i = SAVED_PREV_N-1; i > 0; i--) {
    this->prev[i].time_stamp  = this->prev[i-1].time_stamp;
    this->prev[i].x  = this->prev[i-1].x;
    this->prev[i].y  = this->prev[i-1].y;
    this->prev[i].vx = this->prev[i-1].vx;
    this->prev[i].vy = this->prev[i-1].vy;
    this->prev[i].s  = this->prev[i-1].s;
    this->prev[i].d  = this->prev[i-1].d;
    this->prev[i].v  = this->prev[i-1].v;
    this->prev[i].a  = this->prev[i-1].a;
    this->prev[i].jerk  = this->prev[i-1].jerk;
    this->prev[i].lane  = this->prev[i-1].lane;
  }

  this->prev[0].time_stamp  = this->cur.time_stamp;
  this->prev[0].x  = this->cur.x;
  this->prev[0].y  = this->cur.y;
  this->prev[0].vx = this->cur.vx;
  this->prev[0].vy = this->cur.vy;
  this->prev[0].s  = this->cur.s;
  this->prev[0].d  = this->cur.d;
  this->prev[0].v  = this->cur.v;
  this->prev[0].a  = this->cur.a;
  this->prev[0].jerk  = this->cur.jerk;
  this->prev[0].lane  = this->cur.lane;
  
  
}

/*
 * Calculated by comparing rate of change of average speed over 0.2 seconds
 * SAVED_PREV_N = 50. And since we get called every 0.02 seconds, we have
 * 1 second worth of history. So look at the last 0.2 seconds i.e. i = 9
 */

double
Vehicle::get_cur_accl()
{
  
  double accl = 0;

  if(0)
    {
      for (int i = SAVED_PREV_N-1; i >= 0; i--){
	if(this->prev[i].time_stamp != -1){
	  double vi = this->prev[i].v;
	  double vf = this->cur.v;
	  double ti = this->prev[i].time_stamp;
	  double tf = this->cur.time_stamp;
	  DEBUG("Calculate Accl: vi=" << vi
	      << "vf= " << vf
	      << "ti= " << ti
	      << "tf= " << tf);
	  accl = (vf - vi)/(tf - ti);
	  break;
	}
      }
    } // if 0
  else
    {
      if(this->prev[0].time_stamp != -1){

	double vi = this->prev[0].v;
	double vf = this->cur.v;
	double ti = this->prev[0].time_stamp;
	double tf = this->cur.time_stamp;

	accl = (vf - vi)/(tf - ti);
      }
    }
  
  return accl;
  
}

/*
 * Jerk is the average acceleration over 1 second interval
 */
double
Vehicle::get_cur_jerk()
{
  double accl = 0;
  int count = 0;
  double jerk = 0;

  for (int i = SAVED_PREV_N-1; i >= 0; i--){
    
    if(this->prev[i].time_stamp != -1){
      accl += this->prev[i].a;
      count++;
    }
  }

  accl += this->cur.a;
  count++;

  jerk = accl/count;
	
  return jerk;
  
}

double
Vehicle::get_cur_d_vel()
{
  
  double vel = 0;

  if(this->prev[0].time_stamp != -1){
    
    double di = this->prev[0].d;
    double df = this->cur.d;
    double ti = this->prev[0].time_stamp;
    double tf = this->cur.time_stamp;
    
    vel = (df - di)/(tf - ti);
  }
  
  return vel*0.44704;
  
}

double
Vehicle::get_cur_d_accl()
{
  
  double accl = 0;

  if(this->prev[0].time_stamp != -1){
    
    double vi = this->prev[0].dv;
    double vf = this->cur.dv;
    double ti = this->prev[0].time_stamp;
    double tf = this->cur.time_stamp;
    
    accl = (vf - vi)/(tf - ti);
  }
  
  return accl;
  
}


void Vehicle::update(double cur_time, double x, double y,
		     double vx, double vy, double s, double d)
{

  //DEBUG("In Update vehicle...");

  this->cur.time_stamp = cur_time;
 
  this->cur.x  = x;
  this->cur.y  = y;
  this->cur.vx = vx;
  this->cur.vy = vy;
  this->cur.v = sqrt(vx*vx + vy*vy);// * 0.44704; //In m/sec
  
  this->cur.s  = s;
  
  //NOTE: Below for other vehicles may not give the right acceleration?
  this->cur.a = this->get_cur_accl(); 

  this->cur.jerk = this->get_cur_jerk();
  
  this->cur.d   = d;
  this->cur.dv  = this->get_cur_d_vel();
  this->cur.da  = this->get_cur_d_accl();

  this->cur.lane = this->cur_lane_number();

  
  this->update_previous();

}

void Vehicle::update_ego(double cur_time, double x, double y,
			 double s, double d, double v, double yaw)
{

  this->cur.time_stamp = cur_time;
    
  this->cur.x   = x;
  this->cur.y   = y;
  this->cur.v   = v* 0.44704; //In m/sec

  this->cur.s  = s;

  this->cur.yaw = yaw;
  
  this->cur.a = this->get_cur_accl();

  this->cur.jerk = this->get_cur_jerk();

  this->cur.d   = d;
  this->cur.dv  = this->get_cur_d_vel();
  this->cur.da  = this->get_cur_d_accl();

  this->cur.lane = this->cur_lane_number();

  this->update_previous();

}

Vehicle::~Vehicle() {}


//std::tuple < double, double, vector <double > , vector <double> >
Vehicle::sx_result_t
Vehicle::sx_NOP(states_t new_state,
		sx_trajectory_t *ex_left,
		sx_trajectory_t *left_ptr,
		sx_trajectory_t *straight_ptr,
		sx_trajectory_t *right_ptr,
		sx_trajectory_t *ex_right,
		map<int, Vehicle> *vehicles)
{
  
  sx_result_t res;
  res.state = new_state;
  
  DEBUG("*State X: INVALID: " << states_str[state] << "->" << states_str[new_state]);
  //Return high cost
  //return std::make_tuple(INVALID_COST, 0, prev_path_x, prev_path_y);
  res.cost = INVALID_COST;

  return res;

}

Vehicle::sx_result_t
Vehicle::sx_func(states_t new_state,
		 sx_trajectory_t *ex_left,
		 sx_trajectory_t *left,
		 sx_trajectory_t *straight,
		 sx_trajectory_t *right,
		 sx_trajectory_t *ex_right,
		 map<int, Vehicle> *vehicles)
{

  GET_CUR_TIME_START();
  DEBUG("");
  DEBUG("*State X: " << states_str[state] << "->" << states_str[new_state]);
  
  sx_result_t res;
  res.state = new_state;

  if(sx_valid_lane_change(new_state) == false){
    res.cost = INVALID_COST;
    return res; 
  }
  
  sx_trajectory_t *cur_t;
  sx_trajectory_t *new_t;
  sx_trajectory_t *ext_t;
  
  states_t cur_state = this->state;

  std::tie (cur_t, new_t, ext_t) =
    sx_get_cur_new_trajectories(cur_state, new_state,
				ex_left, left, straight, right, ex_right);

  res.cur_t = cur_t;
  res.new_t = new_t;
  
  double tot_cost = 0;
  double inc_cost = 0;

  
  //What would be the time cost for new lane and new_ref_vel.
  // Assume some distance, say 100mts

  DEBUG(" Computing costs...");

  //The below prevents multiple lane changes at the same time
  
  if(this->lcx_obj
     && (new_state == LCR || new_state == LCL))
    {
      DEBUG(" ----- lcx obj found ----");
      DEBUG(" ----- prev_lane "  << this->lcx_obj->prev_lane);
      DEBUG(" ----- new_lane "  << this->lcx_obj->new_lane);
      DEBUG(" ----- cur_lane "  << this->cur.lane);
      //inc_cost = logistic(0);
      double new_d = abs(this->lcx_obj->new_lane - this->cur.lane);
      double prev_d = abs(this->lcx_obj->prev_lane - this->cur.lane);

      inc_cost = logistic(new_d);
      DEBUG(" ----- 1. new_d "  << new_d << " logistic " << inc_cost);
      
      tot_cost += inc_cost;
      
      inc_cost = logistic(prev_d);
      DEBUG(" ----- 2. prev_d "  << prev_d << " logistic " << inc_cost);
	
      tot_cost += inc_cost;
      
    }
  else
    {
      inc_cost = logistic(1);
      
      DEBUG(" ----- NO lcx_obj -----");
    }
  DEBUG(" ------ LCX cost " << inc_cost);
  tot_cost += inc_cost;
    
  //Logic to prefer open road i.e. vehicle not within FRONT_SCAN_RADIUS mts
  if(cur_t->open_road == false){
    tot_cost += 1;
    DEBUG(" !open_road cost(cur)");
  }

  if(new_t->open_road == false){
    tot_cost += 1;
    DEBUG(" !open_road cost(new)");
  }

  if(new_t->in_front.truly_open_road == false) {
    tot_cost += 1;
    DEBUG(" !truly_open_road cost(new)");
  }

  if(cur_t->in_front.truly_open_road == false) {
    tot_cost += 1;
    DEBUG(" !truly_open_road cost(cur)");
  }

  
  if(ext_t)
    {
      if (ext_t->open_road == false){
	tot_cost += 1;
	DEBUG(" !open_road cost(ext_t)");
      }
    }
  else
    {
      tot_cost += 1;
      DEBUG(" !open_road cost(ext_t == NULL)");
    }
  
  //Keep the lookahead dist the same for all possible next states.
  // The above causes too many lane changes. 
  double lookahead_dist_new = 100;
  double lookahead_dist_cur = lookahead_dist_new;

  
  //double lookahead_dist_cur = (cur_t->ref_s + cur_t->add_on) - this->cur.s;
  //cost time diff for cur
  double time = lookahead_dist_cur/new_t->ref_vel;
  inc_cost = logistic(time);
  
  DEBUG(" Time diff cost(new): " << inc_cost
	<< " ref vel:" << new_t->ref_vel
	<< " lookahead dist " << lookahead_dist_cur);
  
  tot_cost += inc_cost;

  inc_cost = collision_cost(cur_t);
  
  DEBUG(" Collision cost (cur):" << inc_cost);
  tot_cost += inc_cost;

  if(cur_t->lateral_accel > MAX_ACCL){
    DEBUG(" Lateral accel (cur)" << cur_t->lateral_accel);
    tot_cost += 1;
  }
  
  sx_trajectory_t *check_t;
  if((new_state == PLCR) || (new_state == PLCL)){
    check_t = cur_t;
  } else {
    check_t = new_t;
  }

  inc_cost = collision_cost(check_t);
  
  DEBUG(" Collision cost (new):" << inc_cost);
  tot_cost += inc_cost;


  //cost time diff for new
  time = lookahead_dist_new/check_t->ref_vel;
  inc_cost = logistic(time);
  
  DEBUG(" Time diff cost (again): " << inc_cost
	<< " ref vel:" << check_t->ref_vel
	<< " lookahead dist " << lookahead_dist_new);
  
  tot_cost += inc_cost;


  if(check_t->lateral_accel > MAX_ACCL){
    DEBUG(" Lateral accel (new)" << check_t->lateral_accel);
    tot_cost += 1;
  }
  
  res.cost = tot_cost;
  res.ref_vel = check_t->ref_vel;
  res.next_x_vals = check_t->next_x_vals;
  res.next_y_vals = check_t->next_y_vals;
  
  DEBUG("Total Cost: " << tot_cost);
  DEBUG("");

  GET_CUR_TIME_END();
  GET_DURATION(" ****** sx_func:");
  return res;
  
}

std::tuple < double, double, bool>
Vehicle::get_max_accel_for_lane(nbr_t in_front, int lane, double s)
{
  GET_CUR_TIME_START();

  bool open_road = false;
  
  double delta_v_til_target = SPEED_LIMIT_MAX - this->cur.v;
  double max_acc = min(MAX_ACCL, delta_v_til_target);
  double target_dist = MIN_LOOKAHEAD_DIST;

  if(in_front.v != NULL)
    {
      //DEBUG("");
      DEBUG("  Ego vehicle s " << s);
      DEBUG("  Target Vehicle " << in_front.v->v_id << " at " << in_front.v->cur.s);
      
      vector <double> next_state = in_front.v->state_at(1);
      double next_pos = next_state[0];
      DEBUG("  Target next_pos " << next_pos << "with vel:" << in_front.v->cur.v);
      
      double my_next = s + this->cur.v;
      DEBUG("  my_next_pos = " << my_next << " with vel:" << this->cur.v);

      double separation_next = next_pos - my_next;
      DEBUG("  separation = " << separation_next);
      
      target_dist = separation_next - PREFERRED_BUFFER;
      
      DEBUG("  target_dist = " << target_dist);
      double available_room = (target_dist*EGO_UPDATE_TIME);

      DEBUG("  available room = " << available_room);
      max_acc = min(max_acc, available_room);

      /**
      if(target_dist < 0){
      	max_acc = -MAX_ACCL;
      }
      **/
      
      double rel_vel = in_front.v->cur.v - this->cur.v;
      DEBUG("  relative velocity " << rel_vel);
      
      //Maintain at least 2 seconds worth of distance
      double d = fabs(rel_vel*2);
      DEBUG("  relative velocity d " << d);
      
      if(target_dist > d){
	max_acc = MAX_ACCL;
      } else {
	double cur_v = this->cur.v;
	if(cur_v == 0) {
	  cur_v = 1;
	}
	
	max_acc = (cur_v/SPEED_LIMIT)*MAX_ACCL;
	if(rel_vel < 0){
	  max_acc *= -1;
	} else {
	  max_acc *= 1;
	}
      }
      
      
    }
  else
    {
      DEBUG("  Open road...");
      DEBUG("     target_dist " << target_dist);
      open_road = true;
      //To give open road preference
      //max_acc += .001;
    }
  
  DEBUG("  max_acc = " << max_acc);
  DEBUG(" max_accel_for_lane:Out");

  GET_CUR_TIME_END();
  GET_DURATION(" ****** max_accl_in_lane:");

  return std::make_tuple(max_acc, target_dist, open_road);

}


//TODO: CLEANUP Input variables!
std::tuple <double, double, double, bool>
Vehicle::get_new_ref_vel_for_lane(bool prep_lc,
				  nbr_t in_front,
				  nbr_t in_back,
				  int lane,
				  double segment)
{
  GET_CUR_TIME_START();
  double accl;
  double target_dist;
  bool open_road = false;
  
  if(prep_lc == true)
    {
      std::tie(accl, target_dist, open_road) = get_accel_for_plcx(in_back,
								  lane,
								  segment);
    }
  else
    {
      std::tie(accl, target_dist, open_road) = get_max_accel_for_lane(in_front,
								      lane,
								      segment);
    }
  DEBUG(" Max acceleration " << accl);

  double new_ref_vel = this->ref_vel + accl;

  if(doubleGreater(new_ref_vel, SPEED_LIMIT_MAX, 0.0001, true))
    {
      DEBUG(" Speed exceeding speed limit. Set ref vel to " << this->ref_vel);
      
      new_ref_vel = this->ref_vel;
    }
  else if(new_ref_vel < 0)
    {
      new_ref_vel = 0;
      DEBUG("New ref vel set to 0!!");

    }

  DEBUG("Final ref vel " << new_ref_vel << ". new accl " << accl);
  
  GET_CUR_TIME_END();
  GET_DURATION("********* new_ref_val:");

  return std::make_tuple(new_ref_vel, accl, target_dist, open_road);
  
}


double
Vehicle::collision_cost(sx_trajectory_t *trajectory)
{
  
  double cost = 0;

  if(trajectory->coll.collision == true){
    cost = HIGH_COST;
  }

  return cost;
  
}


tuple < bool, int, double>
Vehicle::check_for_collision(vector <double> *x_ptr,
			     vector <double> *y_ptr,
			     nbr_t in_front,
			     nbr_t in_back,
			     double new_ref_vel)
{
  bool collision = false;
  int coll_v_id = 0;
  double coll_time = 0;
  
  if(in_front.v == NULL && in_back.v == NULL) {
    DEBUG("check for collision: no vehicle front and back");
    return make_tuple(collision, coll_v_id, coll_time);
  }

  
  vector <double> & x = *x_ptr;
  vector <double> & y = *y_ptr;
  
  DEBUG("Collision check... " );
  DEBUG("   number of pts " << x.size());
  int closest_vid = 0;

  vector < vector <double> > xyts;

  GET_CUR_TIME_START();
  //Create x, y and time array for the trajectory
  for(int i = 0; i < x.size(); i += 2 )
    {
      vector <double> xyt;

      //int wp_trial = ClosestWaypoint_hash(x[i], y[i], &this->waypoints.m);

      // Get the segment from x and y
      int wp = ClosestWaypoint(x[i],
			       y[i],
			       this->waypoints->x_spline,
			       this->waypoints->y_spline);
      double segment = this->waypoints->s_spline[wp];
      
      //Now, given the new velocity, when does ego vehicle reach this segment
      double dist = segment - this->cur.s;

      if(dist < 0) continue;
      
      double t = dist/new_ref_vel;

      xyt.push_back(x[i]);
      xyt.push_back(y[i]);
      xyt.push_back(t);
      
      xyts.push_back(xyt);
      
    }
  GET_CUR_TIME_END();
  GET_DURATION("closest waypoints lookup: ");

  for(int i = 0; i < xyts.size(); i += 2)
    {
      
      double t = xyts[i][2];

      if(in_front.v != NULL)
	{
	  vector <double> in_front_state = in_front.v->state_at(t);
	  
	  vector <double> xy = 
	    getXY(in_front_state[0], in_front_state[3],
		  this->waypoints->s_spline,
		  this->waypoints->x_spline,
		  this->waypoints->y_spline);

	  double d = distance(xy[0], xy[1], x[i], y[i]);

	  if(0){
	    DEBUG(i << ". Forward");
	    DEBUG("    trajec  pt "
		  << "(" << xyts[i][0] << "," << xyts[i][1] << ")");
	    DEBUG("    other v pt "
		  << "(" << xy[0] << "," << xy[1] << ")");
	    DEBUG("    d=" << d); 
	  }

	  
	  if(d < VEHICLE_CLOSEST_DIST){
	    DEBUG("   Collision predicted with Vehicle (in front) " << in_front.v->v_id
		  << " seg: " << in_front.v->cur.s
		  << " speed: " << in_front.v->cur.v);
	    DEBUG("      in time " << t);
	    DEBUG("      come close as (mts)" << d);
	    if(this->cur.v > in_front.v->cur.v){
	      DEBUG("    !! shouldnt be an accident. relative vel "
		    << (this->cur.v - in_front.v->cur.v));
	    }
	      
	    collision = true;
	    coll_v_id = in_front.v->v_id;
	    coll_time = t;
	    break;
	  }

	}

      if(in_back.v != NULL)
	{
	  vector <double> in_back_state = in_back.v->state_at(t);
	  
	  vector <double> xy = 
	    getXY(in_back_state[0], in_back_state[3],
		  this->waypoints->s_spline,
		  this->waypoints->x_spline,
		  this->waypoints->y_spline);

	  double d = distance(xy[0], xy[1], x[i], y[i]);

	  if(0){
	    DEBUG(i << ". Backward/Side");
	    DEBUG("    trajec  pt "
		  << "(" << xyts[i][0] << "," << xyts[i][1] << ")");
	    DEBUG("    other v pt "
		  << "(" << xy[0] << "," << xy[1] << ")");
	    DEBUG("    d=" << d); 
	  }
	  
      
	  if(d < VEHICLE_CLOSEST_DIST){
	    DEBUG("   Collision predicted with Vehicle (in back) " << in_back.v->v_id
		  << " seg: " << in_back.v->cur.s
		  << " speed: " << in_back.v->cur.v);
	    DEBUG("      in time " << t);
	    DEBUG("      come close as (mts) " << d);

	    if(this->cur.v > in_back.v->cur.v){
	      DEBUG("    !! shouldnt be an accident. relative vel "
		    << (this->cur.v - in_back.v->cur.v));
	    }

	    collision = true;
	    coll_v_id = in_back.v->v_id;
	    coll_time = t;
	    break;
	  }

	    
	}
    }
  
  return make_tuple(collision, coll_v_id, coll_time);
    
}

bool
Vehicle::sx_valid_lane_change(states_t new_state)
{
  int go_direction = ((new_state == PLCL || new_state == LCL) ? -1 :
		      ((new_state == PLCR || new_state == LCR) ? +1 : 0));
  
  int new_lane = this->cur.lane + go_direction;
  if (new_lane > NUM_LANES || new_lane < 1)
    {
      DEBUG(" ** Invalid Lane transition to " << new_lane <<
	    ". From "<< state << " To "	<< new_state);
      return false;
    }

  return true;
}


Vehicle::sx_result_t
Vehicle::get_next_best_state(states_t cur_state, vector <sx_result_t> sx_res)
{
  GET_CUR_TIME_START();
  DEBUG(" get_next_best_state: In");
  
  double min_cost = INVALID_COST;

  sx_result_t best_res;

  if(0){
    DEBUG("Forcing best state to KL");
    best_res = sx_res[0];
    return best_res;
  }
  
  for(int i = 0; i < sx_res.size(); i++)
    {
      if(doubleGreater(min_cost, sx_res[i].cost)){
	min_cost = sx_res[i].cost;
      }	
    }

  DEBUG("  min cost "  << min_cost);
  //Check if there are multiple states with min cost
  vector <sx_result_t>  possible_best_states;

  for(int i = 0; i < sx_res.size(); i++)
    {
      sx_result_t res = sx_res[i];
      if(doubleEquals(min_cost, res.cost)){
	possible_best_states.push_back(sx_res[i]);
      }
    }
  
  if(possible_best_states.size() > 1)
    {
      bool found = false;
      sx_result_t ret;

      DEBUG(" Multiple states with min cost " << min_cost);

      //Multiple states with min cost

      //1. check if PLCX -> LCX possible
      for(int i = 0; i < possible_best_states.size(); i++)
	{
	  sx_result_t res = possible_best_states[i];
	  
	  if((cur_state == PLCL && res.state == LCL)
	     || (cur_state == PLCR && res.state == LCR))
	    {
	      found = true;
	      best_res = possible_best_states[i];
	      DEBUG("  Prefer LCX");
	    }
	  
	}

      //2. if above failed to find, then preserve current state if possible
      if(found == false)
	{
	  for(int i = 0; i < possible_best_states.size(); i++){
	    sx_result_t res = possible_best_states[i];
	    if(cur_state == res.state)
	      {
		found = true;
		best_res = possible_best_states[i];
		DEBUG("  Preserve cur state");
		break;
	      }
	  }
	}
      
      //3. Finally, if the above also, fails find min state
      int min_id = 99;
      if(found == false){
	//Return one with the smallest id
	for(int i = 0; i < possible_best_states.size(); i++){
	  sx_result_t res = possible_best_states[i];
	  if(min_id > res.state){
	    min_id = possible_best_states[i].state;
	    best_res = possible_best_states[i];
	    DEBUG("   Cur state not with the same cost. Return one with lowest id");
	  }
	}
      }
    }
  else
    {
      best_res = possible_best_states[0];
    }

  DEBUG(" get_next_best_state: Out");
  GET_CUR_TIME_END();
  GET_DURATION(" ****** get_best_state:");

  return best_res;
  
}

std::tuple <Vehicle::sx_trajectory_t *, Vehicle::sx_trajectory_t *, Vehicle::sx_trajectory_t *>
Vehicle::sx_get_cur_new_trajectories(states_t cur_state,
				     states_t new_state,
				     sx_trajectory_t *ex_left,
				     sx_trajectory_t *left,
				     sx_trajectory_t *straight,
				     sx_trajectory_t *right,
				     sx_trajectory_t *ex_right)
{

  //sx_trajectory_t &left     = *left_ptr;
  //sx_trajectory_t &straight = *straight_ptr;
  //sx_trajectory_t &right    = *right_ptr;
  
  sx_trajectory_t *cur_t;
  sx_trajectory_t *new_t;
  sx_trajectory_t *ext_t;
  
  switch (cur_state)
    {
    case KL:
      {
	switch (new_state)
	  {
	  case KL:
	    {
	      cur_t = straight;
	      new_t = straight;
	      ext_t = straight;
	      break;
	    }
	  case PLCL:
	    {
	      cur_t = straight;
	      new_t = left;
	      ext_t = ex_left;
	      break;
	    }
	  case PLCR:
	    {
	      cur_t = straight;
	      new_t = right;
	      ext_t = ex_left;
	      break;
	    }
	  case LCL:
	    {
	      DEBUG("cur_new_trajectory: Invalid sx 1!!");
	      break;
	    }
	  case LCR:
	    {
	      DEBUG("cur_new_trajectory: Invalid sx 2!!");
	      break;
	    }
	  }//switch new_state
	break;
      }
    case PLCL:
      {
	switch (new_state)
	  {
	  case KL:
	    {
	      cur_t = straight;
	      new_t = straight;
	      ext_t = straight;
	      break;
	    }
	  case PLCL:
	    {
	      cur_t = straight;
	      new_t = left;
	      ext_t = ex_left;
	      break;
	    }
	  case PLCR:
	    {
	      cur_t = straight;
	      new_t = right;
	      ext_t = ex_right;
	      break;
	    }
	  case LCL:
	    {
	      cur_t = straight;
	      new_t = left;
	      ext_t = ex_left;
	      break;
	    }
	  case LCR:
	    {
	      DEBUG("cur_new_trajectory: Invalid sx 4!!");
	      break;
	    }
	  }//switch new_state
	break;
      }
    case PLCR:
      {
	switch (new_state)
	  {
	  case KL:
	    {
	      cur_t = straight;
	      new_t = straight;
	      ext_t = straight;
	      break;
	    }
	  case PLCL:
	    {
	      cur_t = straight;
	      new_t = left;
	      ext_t = ex_left;
	      break;
	    }
	  case PLCR:
	    {
	      cur_t = straight;
	      new_t = right;
	      ext_t = ex_right;
	      break;
	    }
	  case LCL:
	    {
	      DEBUG("cur_new_trajectory: Invalid sx 6!!");
	      break;
	    }
	  case LCR:
	    {
	      cur_t = straight;
	      new_t = right;
	      ext_t = ex_right;
	      break;
	    }
	  }//switch new_state
	break;
      }
    case LCL:
      {
	switch (new_state)
	  {
	  case KL:
	    {
	      cur_t = left;
	      new_t = straight;
	      ext_t = straight;
	      break;
	    }
	  case PLCL:
	    {
	      DEBUG("cur_new_trajectory: Invalid sx 7!!");
	      break;
	    }
	  case PLCR:
	    {
	      DEBUG("cur_new_trajectory: Invalid sx 8!!");
	      break;
	    }
	  case LCL:
	    {
	      cur_t = left;
	      new_t = left;
	      ext_t = ex_left;
	      break;
	    }
	  case LCR:
	    {
	      DEBUG("cur_new_trajectory: Invalid sx 9!!");
	      break;
	    }
	  }//switch new_state
	break;
      }
    case LCR:
      {
	switch (new_state)
	  {
	  case KL:
	    {
	      cur_t = right;
	      new_t = straight;
	      ext_t = straight;
	      break;
	    }
	  case PLCL:
	    {
	      DEBUG("cur_new_trajectory: Invalid sx 10!!");
	      break;
	    }
	  case PLCR:
	    {
	      DEBUG("cur_new_trajectory: Invalid sx 11!!");
	      break;
	    }
	  case LCL:
	    {
	      DEBUG("cur_new_trajectory: Invalid sx 12!!");
	      break;
	    }
	  case LCR:
	    {
	      cur_t = right;
	      new_t = right;
	      ext_t = ex_right;
	      break;
	    }
	  }//switch new_state
	break;
      }
    }//switch cur_state

  return make_tuple(cur_t, new_t, ext_t);
}


std::tuple <vector <double>, vector <double> > 
Vehicle::update_state(map<int, Vehicle> *vehicles,
		      vector <double> *prev_path_x,
		      vector <double> *prev_path_y,
		      Waypoints *waypoints,
		      double ref_s)
{


  GET_CUR_TIME_START();
  //state = "KL"; // this is an example of how you change state.
  
  //Look the current state and for the possible next states, predict the paths

  states_t cur_state = state;
  
  DEBUG("Current state:" << state);

  vector <state_x_func_t> possible_next_states = this->fsm[cur_state];

  //compute all trajectories
  // left, straight, extreme_left & extreme_right
  vector < sx_trajectory_t > trajectories;
    
  for(int i = 0; i < go_directions.size(); i++)
    {
      //Allocate trajectory structure
      
      sx_trajectory_t trajectory;
      
      int go_direction = go_directions[i];
      
      int new_lane = this->cur.lane + go_direction;

      if (new_lane > NUM_LANES || new_lane < 1)
	{
	  trajectories.push_back(trajectory);
	  continue;
	}

      DEBUG("");
      DEBUG("Trajectory for " << go_dir_str[i]);


      nbr_t in_front, in_back;

      std::tie(in_front, in_back) =
	find_nearest_front_back_in_lane(vehicles, new_lane, this->cur.s);
      
      
      double new_ref_vel;
      double new_accl;
      double target_dist;
      bool open_road;
      std::tie(new_ref_vel, new_accl, target_dist, open_road) =
	get_new_ref_vel_for_lane(false, in_front, in_back, new_lane, this->cur.s);
      
      
      //Dependendent on the velocity. The higher the velocty you can project
      // a little further ahead
      //int num_points = 30 + (75 - 30)*new_ref_vel/SPEED_LIMIT_M;


      double lookahead_dist = target_dist;
      if(lookahead_dist < MIN_LOOKAHEAD_DIST) lookahead_dist = MIN_LOOKAHEAD_DIST;

      if(lookahead_dist > MAX_LOOKAHEAD_DIST) lookahead_dist = MAX_LOOKAHEAD_DIST;

      
      int next_xy_vals = (previous_cycle_time/(EGO_UPDATE_TIME*1000) + //number of points consumed
		      MIN_NEXT_XY_VALS +
		      (MAX_LOOKAHEAD_DIST - MIN_LOOKAHEAD_DIST)/lookahead_dist);
      
      //Add MIN_NEXT_XY_TO_ADD points
      int num_new_xy = next_xy_vals - prev_path_x->size();
      
      if(num_new_xy < 0)
	{
	  DEBUG(" next_xy fewer than prev");
	  next_xy_vals = prev_path_x->size() + MIN_NEXT_XY_TO_ADD;
	}
      else if( num_new_xy < MIN_NEXT_XY_TO_ADD)
	{
	  //Make it minimum of MIN_NEXT_XY_TO_ADD number of points
	  DEBUG(" not enough points for MIN_NEXT_XY_TO_ADD");
	  next_xy_vals += (MIN_NEXT_XY_TO_ADD - num_new_xy);
	}

      DEBUG("target dist " << target_dist);
      DEBUG("lookahead distance " << lookahead_dist);
      DEBUG("num_new_xy  " << num_new_xy);
      DEBUG("next_xy points " << next_xy_vals);
      
      ptg_get_trajectory(lookahead_dist,
			 next_xy_vals,
			 ref_s,
			 new_ref_vel,
			 this->cur.x,
			 this->cur.y, 
			 this->cur.yaw,
			 new_lane,
			 prev_path_x,
			 prev_path_y,
			 &trajectory.next_x_vals,
			 &trajectory.next_y_vals,
			 waypoints);

      sx_collision_t coll;
      std::tie(coll.collision, coll.v_id, coll.in_time) =
	check_for_collision(&trajectory.next_x_vals,
			    &trajectory.next_y_vals,
			    in_front,
			    in_back,
			    new_ref_vel);

      vector <double> pt1 = {trajectory.next_x_vals[0], trajectory.next_y_vals[0]};
      vector <double> pt2 = {trajectory.next_x_vals[1], trajectory.next_y_vals[1]};
      vector <double> pt3 = {trajectory.next_x_vals[2], trajectory.next_y_vals[2]};
      double lateral_accel = ptg_get_lateral_accel(pt1, pt2, pt3, new_ref_vel);

      if(lateral_accel > MAX_ACCL){
	DEBUG("*** Lateral accl exceeded!");
      }
      
      trajectory.coll            = coll;
      trajectory.ref_s           = ref_s;
      trajectory.new_lane        = new_lane;
      trajectory.ref_vel         = new_ref_vel;
      trajectory.new_accl        = new_accl;
      trajectory.target_dist     = target_dist;
      trajectory.in_front        = in_front;
      trajectory.in_back         = in_back;
      trajectory.open_road       = open_road;
      trajectory.lateral_accel   = fabs(lateral_accel);
      
      trajectories.push_back(trajectory);
      DEBUG("");
    }
  
  //DEBUG
  debug_print_all_trajectories(&trajectories, go_dir_str);
  
    
  //Now walk through the possible next steps and select the action
  // with the lowest cost.
  vector <sx_result_t> ress; //Collect the results in this vector
  for(int i = 0; i < possible_next_states.size(); i++)
    {
      states_t new_state = static_cast <states_t> (i);

      sx_result_t res;
      
      res = (this->*fsm[cur_state][i])(new_state,
				       &trajectories[EXTREME_LEFT],
				       &trajectories[LEFT],
				       &trajectories[STRAIGHT],
				       &trajectories[RIGHT],
				       &trajectories[EXTREME_RIGHT],
				       vehicles);
      
      ress.push_back(res);
    }

  //DEBUG
  debug_print_all_sx_func_results(states_str[state], &ress, states_str);

  sx_result_t best_res = get_next_best_state(cur_state, ress);

  DEBUG(" ** State Transition " << states_str[state]
	<< " to " << states_str[best_res.state]);
  DEBUG("       New ref vel " << best_res.ref_vel);

  if(state != best_res.state)
    {
      if(best_res.state == LCL || best_res.state == LCR)
	{
	  lcx_obj_t *lcx_obj = new lcx_obj_t;
	  lcx_obj->new_lane  = best_res.new_t->new_lane;
	  lcx_obj->prev_lane = this->cur.lane;
	  this->lcx_obj = lcx_obj;
	}
      else 
	{
	  //if(this->lcx_obj) delete this->lcx_obj;
	  this->lcx_obj = NULL;
	}
    }

  
  if(best_res.state == PLCL || best_res.state == PLCR)
    {
      DEBUG(" *** New best state PLCX: Get new ref velocity");
      
      double new_ref_vel, new_accl, target_dist;
      bool open_road;
      
      //Get best acceleration for lane change
      std::tie(new_ref_vel, new_accl, target_dist, open_road)
	= get_new_ref_vel_for_lane(true,
				   best_res.new_t->in_front,
				   best_res.new_t->in_back,
				   best_res.new_t->new_lane,
				   this->cur.s);
      
	      
      //Regnerate trajectory
      
      
    }
  
  this->state = best_res.state;
  this->ref_vel = best_res.ref_vel;
  
  GET_CUR_TIME_END();
  GET_DURATION("****** update_state:");

  vector <double> final_next_x_vals = best_res.next_x_vals;
  vector <double> final_next_y_vals = best_res.next_y_vals;

  return std::make_tuple(final_next_x_vals, final_next_y_vals);
  
}

void Vehicle::configure(Waypoints *waypoints) {

  this->waypoints = waypoints;
  
}

vector<double> Vehicle::state_at(double t)
{
  /*
    Predicts state of vehicle in t seconds (assuming constant acceleration)
  */
  //t = t/0.02;
  
  double s_new      = this->cur.s + (this->cur.v * t) + (this->cur.a * t * t) / 2;
  double s_dot_new  = this->cur.v + (this->cur.a * t);
  double s_dot2_new = this->cur.a; //Can do better given the available history?

  double d_new      = this->cur.d + (this->cur.dv * t) + (this->cur.da * t * t) / 2;
  double d_dot_new  = this->cur.dv + (this->cur.da * t); //Assume 0?
  double d_dot2_new = this->cur.da; //Assume 0?
  
  return {s_new, s_dot_new, s_dot2_new, d_new, d_dot_new, d_dot2_new};
}

std::tuple <Vehicle::nbr_t, Vehicle::nbr_t>
Vehicle::find_nearest_front_back_in_lane(map<int, Vehicle> *vehicles,
					 int lane, double s){
  GET_CUR_TIME_START();
  
  nbr_t found_front;
  nbr_t found_back;
  double min_front_dist = 9999;
  double min_back_dist = 9999;

  found_front.v = NULL;
  found_front.truly_open_road = false;
  found_back.v = NULL;
  
  map<int, Vehicle>::iterator it = vehicles->begin();
  while(it != vehicles->end())
    {
      int v_id = it->first;
      Vehicle vehicle = it->second;
      double dist = abs(vehicle.cur.s - s);
      //Check if we have received this from the sensor. If not dont
      //worry about it

      if (vehicle.cur.lane == lane)
	{
	  if((vehicle.cur.s > s)
	     && (doubleLess(dist, min_front_dist) == true)
	     && (doubleLess(dist, FRONT_SCAN_RADIUS) == true))
	    {
	      DEBUG("  nearest fwd candidate " << v_id
		    << " at " << vehicle.cur.s
		    << " with separation " << dist << " from ego."
		    << " in lane " << lane);
	      min_front_dist = dist;
	      found_front.v = &it->second;
	      found_front.segment = vehicle.cur.s;
	      found_front.separation = dist;
	    }
	  //Additional check for backward. Dont return car behind us.
	  else if((vehicle.cur.s < s)
		  && (doubleLess(dist, min_back_dist, 0.0001, true) == true)
		  && (doubleLess(dist, BACK_SCAN_RADIUS, 0.0001, true) == true)
		  && (vehicle.cur.lane != this->cur.lane))
	    {
	      DEBUG("  nearest bwd candidate " << v_id
		    << " at " << vehicle.cur.s
		    << " with separation " << dist << " from ego."
		    << " in lane " << lane);
	      min_back_dist = dist;
	      found_back.v = &it->second;
	      found_back.segment = vehicle.cur.s;
	      found_back.separation = dist;
	    }
	}
      //they are in different lanes. but check to see that the difference in d
      // is greater than 3. if not, the car in front maybe chanding lanes
      else if(vehicle.cur.lane != this->cur.lane)
	{
	  if ((vehicle.cur.s > s)
	      && (doubleLess(dist, min_front_dist) == true)
	      && (doubleLess(dist, FRONT_SCAN_RADIUS) == true)
	      && fabs(vehicle.cur.d - this->cur.d) < (LANE_WIDTH - 1))
	    {
	      //Likely lane change in front of us. Put it on the list
	      DEBUG("  nearest fwd candidate (Lane change in other lane!!) "
		    << v_id
		    << " at " << vehicle.cur.s
		    << " with separation " << dist << " from ego."
		    << " in lane " << vehicle.cur.lane);
	      min_front_dist = dist;
	      found_front.v = &it->second;
	      found_front.segment = vehicle.cur.s;
	      found_front.separation = dist;
	    }
	  
	}
      it++;
    }

  
  if(found_front.v == NULL)
    {
      found_front.truly_open_road = true;
      //If there is no vehicle in lane then set truly_open_road to true
      it = vehicles->begin();
      while(it != vehicles->end())
	{
	  Vehicle vehicle = it->second;
	  if(vehicle.cur.lane == lane
	     && (vehicle.cur.s > s + VEHICLE_LENGTH/2)
	     && (vehicle.cur.s < s + FRONT_SCAN_FURTHEST_RADIUS)){
	    found_front.truly_open_road = false;
	  }
	  it++;
	}
    }
  
  GET_CUR_TIME_END();
  GET_DURATION(" ****** find_nearest_front_back_in_lane:");
  
  return std::make_tuple(found_front, found_back);
  
}


std::tuple < double, double, bool>
Vehicle::get_accel_for_plcx(nbr_t in_back,
			    int lane,
			    double segment)
{
  double accl;
  double target_dist = MIN_LOOKAHEAD_DIST;
  bool open_road = false;
  
  DEBUG("get_accel_for_plcx: In ");
  
  if(in_back.v != NULL)
    {
      vector <double> next_state0 = in_back.v->state_at(0);
      vector <double> next_state1 = in_back.v->state_at(1);
      double target_vel = next_state1[0] - next_state0[0];
      double delta_v = this->cur.v - target_vel;
      double delta_s = this->cur.s - next_state0[0];
      if(delta_v != 0)
    	{

	  double time = -2 * delta_s/delta_v;

	  if (time == 0)
	    {
	      accl = this->cur.a;
	    }
	  else
	    {
	      accl = delta_v/time;
	    }
	  if(accl > MAX_ACCL)
	    {
	      accl = MAX_ACCL;
	    }
	  if(accl < -MAX_ACCL)
	    {
	      accl = -MAX_ACCL;
	    }
	  DEBUG_F("  Car in lane behind. new accl " << accl);
	}
      else
    	{
	  double my_min_acc = max(-MAX_ACCL,-delta_s);
	  accl = my_min_acc;
	  DEBUG_F("  Car in lane behind & delta_v is zero. new accl " << accl);
    	}
      
    }
  DEBUG("  accl " << accl);
  return std::make_tuple(accl, target_dist, open_road);
}

double
Vehicle::lane_d(double s, double d, int lane)
{
    vector <double> pos = getXY(s,
			     d,
			     this->waypoints->s_spline,
			     this->waypoints->x_spline,
			     this->waypoints->y_spline);
  
    
  int wp = ClosestWaypoint(pos[0],
			   pos[1],
			   this->waypoints->x_spline,
			   this->waypoints->y_spline);
  
  //DEBUG("  Closest waypoint index to this car " << wp);

  double waypoint_d = this->waypoints->spline_d[wp];
  
  //DEBUG("  Waypoint d " << waypoint_d);

  //return (waypoint_d + (lane - 1)*LANE_WIDTH + LANE_WIDTH/2);
  return ((lane - 1)*LANE_WIDTH + LANE_WIDTH/2);
}


/*
 * Computes if the vehicle is in one of the lanes. And if so, returns 
 * the lane number. If not returns 0
 */

int
Vehicle::cur_lane_number() {
  
  int wp = ClosestWaypoint(this->cur.x,
			   this->cur.y,
			   this->waypoints->x_spline,
			   this->waypoints->y_spline);
  
  //DEBUG("  Closest waypoint index to this car " << wp);

  double waypoint_d = this->waypoints->spline_d[wp];
  
  //DEBUG("  Vechile D vector " << this->cur.d);

  for (int i = 1; i <= NUM_LANES; i++)
    {
      if(abs(this->cur.d) < waypoint_d + i*LANE_WIDTH) {
	return i;
      }
    }

  return 0;
}

int
Vehicle::lane_number(double s, double d) {

  
  vector <double> pos = getXY(s,
			     d,
			     this->waypoints->s_spline,
			     this->waypoints->x_spline,
			     this->waypoints->y_spline);
  
  
  int wp = ClosestWaypoint(pos[0],
			   pos[1],
			   this->waypoints->x_spline,
			   this->waypoints->y_spline);
  
  //DEBUG("  Closest waypoint index to this car " << wp);

  double waypoint_d = this->waypoints->spline_d[wp];
  
  //DEBUG("  Waypoint d " << waypoint_d);

  for (int i = 1; i <= NUM_LANES; i++)
    {
      //Some times d is negative! use abs
      if(abs(d) < waypoint_d + i*LANE_WIDTH) {
	return i;
      }
    }

  return 0;
}



vector<vector<double> >
Vehicle::generate_predictions(int horizon = 10) {

  vector<vector<double> > predictions;
  for( int i = 0; i < horizon; i++)
    {
      vector<double> check1 = state_at(i);
      int lane = lane_number(check1[0], check1[3]); 
      vector<double> lane_s = {static_cast<double> (lane), check1[0]};
      predictions.push_back(lane_s);
    }
  //this->predictions.push_back(predictions);
  return predictions;
  
}
