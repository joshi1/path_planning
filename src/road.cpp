#include <iostream>
#include "road.h"
#include "vehicle.h"
#include <iostream>
#include <math.h>
#include <map>
#include <string>
#include <iterator>
#include <cstdlib>
#include "debug.h"
#include "constants.h"
/**
 * Initializes Road
 */
Road::Road(Waypoints waypoints)
{
  this->waypoints   = waypoints;
}

Road::~Road() {}

/*** UNUSED 
map<int ,vector<vector<double> > >
Road::generate_predictions(double cur_time) {

  //First, generate predictions for other vehicles on the road
  map<int ,vector<vector<double> > > predictions;

  map<int, Vehicle>::iterator it = this->vehicles.begin();
  while(it != this->vehicles.end())
    {
      int v_id = it->first;

      //Check to see if the vehicle is on one of the lanes on the right side of
      // the road. If not, ignore.
      //NOTE: Ideally we would want the state_at() function decide. but in its
      // current simplistic form. it does not matter. So decided not to modify it.

      if(it->second.cur.lane == 0)
	{
	  DEBUG(" Vehicle NOT on road. Ignoring. " << v_id);
	}
      else 
	{
	  
	  DEBUG(" Generating predictions v_id:" << v_id << " ...");
	  
	  //debug_print_vehicle_info("", it->second);
	  vector<vector<double> > preds = it->second.generate_predictions(HORIZON_T);
	  predictions.insert(std::pair<int, vector<vector<double> > >
			     (v_id, preds));
	}
      
      it++;
    }
  
  return predictions;
  
}
***/
