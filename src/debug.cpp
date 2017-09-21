#include <iostream>
#include <iostream>
#include <math.h>
#include <map>
#include <string>
#include <iterator>
#include "debug.h"

#include "vehicle.h"
#include "constants.h"

#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

using namespace std;


bool debugging_enabled   = false;
bool debugging_enabled_f = false;
bool debugging_enabled_t = false;

void
debug_print_vehicle_info(string s, Vehicle v){
  DEBUG("  Vehicle Id:" << v.v_id);
  DEBUG("   Current: at time " << v.cur.time_stamp);
  DEBUG("       x: " << v.cur.x);
  DEBUG("       y: " << v.cur.y);
  DEBUG("     yaw: " << v.cur.yaw);
  DEBUG("     seg: " << v.cur.s);
  DEBUG("     vel: " << v.cur.v);
  DEBUG("    accl: " << v.cur.a);
  DEBUG("    jerk: " << v.cur.jerk);
  DEBUG("    lane: " << v.cur.lane);
  DEBUG("       d: " << v.cur.d);
  DEBUG("      dv: " << v.cur.dv);
  DEBUG("      da: " << v.cur.da);

}

void
debug_print_all_goals(string s,
		      vector < vector <double> >all_goals)
{
  
  DEBUG(s);

  DEBUG("Total goals " << all_goals.size());
  
  for (int i = 0; i < all_goals.size(); i++)
    {
      debug_print_state_at_t("state:", all_goals[i]);
    }
}

void
debug_print_state_at_t(string s, vector <double> state)
{
  DEBUG(s << endl
	<< " time=" << state[6] << endl
	<< " s=" << state[0] << endl
	<< " s_dot=" << state[1] << endl
	<< " s_dot2=" << state[2] << endl
	<< " d=" << state[3] << endl
	<< " d_dot=" << state[4] << endl
	<< " d_dot2=" << state[5]);

}

void
debug_print_all_trajectories(vector < Vehicle::sx_trajectory_t > *trajectories,
			     string go_dir_str[])
{
  DEBUG("Printing all trajectory info:");
  for (int i = 0; i < trajectories->size(); i++){
    Vehicle::sx_trajectory trajectory = (*trajectories)[i];
    DEBUG(" Trajectory " << go_dir_str[i]);
    DEBUG("  ref_vel " << trajectory.ref_vel);
    DEBUG("  in_front " << trajectory.in_front.v);
    DEBUG("     truly open road " << trajectory.in_front.truly_open_road);
    DEBUG("  in_back " << trajectory.in_back.v);
    DEBUG("     truly open road " << trajectory.in_back.truly_open_road);
    DEBUG("  num next_x_vals " << trajectory.next_x_vals.size());
    DEBUG("  num next_y_vals " << trajectory.next_y_vals.size());
  }

}

void
debug_print_all_sx_func_results(string cur_state_str,
				vector <Vehicle::sx_result_t> *ress,
				string states_str[])
{
  DEBUG("Printing all sx results info:");
  DEBUG("Curent state : " << cur_state_str);
  for(int i = 0; i < ress->size(); i++){
    Vehicle::sx_result_t res = (*ress)[i];
    DEBUG(" New state: " << states_str[res.state] << ".Cost: " << res.cost);
    DEBUG("    ref_Vel: " << res.ref_vel);
  }

}
void
debug_plt_scatterplot(vector <double> x, vector <double> y)
{
  
  //plt::plot(waypoints->x_spline, waypoints->y_spline, "r^");
  //plt::plot(waypoints->x_plusd_spline, waypoints->y_plusd_spline, "g^");
  plt::plot(x, y);
  plt::show();

}
