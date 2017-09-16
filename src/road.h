#ifndef ROAD_H
#define ROAD_H

#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <set>
#include <map>
#include <string>
#include <iterator>
#include "vehicle.h"
#include "map.h"
#include "debug.h"

using namespace std;

extern double previous_cycle_time;

class Road {
 public:

  Vehicle ego_vehicle {EGO_V_ID, 0, 0, 0, 0, 0, 0};
  
  map<int, Vehicle> vehicles;

  Waypoints waypoints;

  /**
   * Constructor
   */
  Road(Waypoints waypoints);
  
  /**
   * Destructor
   */
  virtual ~Road();

};

#endif
