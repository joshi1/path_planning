#ifndef DEBUG_H_
#define DEBUG_H_

#include <vector>
#include "map.h"
#include "vehicle.h"
#include <iostream>
#include <chrono>

using namespace std;
using namespace std::chrono;

//#define DEBUGGING_KL_ONLY

extern bool debugging_enabled;

extern bool debugging_enabled_f;

extern bool debugging_enabled_t;

#define DEBUGGING_PATH_PLANNING

#ifdef DEBUGGING_PATH_PLANNING

#define DEBUG_F(x) do {					  \
    if (debugging_enabled_f) { std::cerr << __FUNCTION__ << "(" << __LINE__ << ")" << ":" << x << std::endl; } \
} while (0)

#define DEBUG(x) do {					  \
    if (debugging_enabled) { std::cerr << x << std::endl; } \
} while (0)

#define GET_CUR_TIME_START() \
    high_resolution_clock::time_point t1 = high_resolution_clock::now();

#define GET_CUR_TIME_END()						\
  high_resolution_clock::time_point t2 = high_resolution_clock::now();

#define GET_DURATION(s) do {\
    auto duration = duration_cast<milliseconds>( t2 - t1 ).count();\
    if(debugging_enabled_t) {cout << s << " " << duration << " milliseconds" << endl;}	\
} while (0)

#else

#define DEBUG_F(x)

#define DEBUG(x)

#define GET_CUR_TIME_START()

#define GET_CUR_TIME_END()						\

#define GET_DURATION(s)


#endif

extern void
debug_print_vehicle_info(string s, Vehicle vehicle);

extern void
debug_print_all_goals(string s,
		      vector < vector <double> >all_goals);

extern void
debug_print_state_at_t(string s, vector <double> state);

extern void
debug_print_all_trajectories(vector < Vehicle::sx_trajectory_t > *trajectories,
			     string go_dir_str[]);

void
debug_print_all_sx_func_results(string cur_state_str,
				vector <Vehicle::sx_result_t> *ress,
				string states_strp[]);


extern void
debug_plt_scatterplot(vector <double> x, vector <double> y);

    

#endif //DEBUG_H
