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

#include <map>
#include "spline.h"
#include "ptg.h"
#include "map.h"
#include "debug.h"
#include "road.h"
#include "constants.h"
#include "helpers.h"
#include "vehicle.h"
using namespace std;


// for convenience
using json = nlohmann::json;


// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}


int main(int argc, char** argv) {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from the provided file specific to the track
  string map_file_;
  string right_usage = "./path_planning [bosch | bosch_challenge]";
  if (argc == 1){
    map_file_ = "../data/highway_map.csv";
  } else if (argc > 2){
    cout << right_usage << endl;
    exit(EXIT_FAILURE);
  } else {
    if (strcmp(argv[1], "bosch") == 0){
      map_file_ = "../data/highway_map_bosch1.csv";  
    } else if (strcmp(argv[1], "bosch_challenge") == 0){
      map_file_ = "../data/highway_map_bosch1_final.csv";
    } else {
      cout << right_usage << endl;
      exit(EXIT_FAILURE);
    }
  }
    
  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  // In this approach we try to spline the map x,y points to add 50 
  //  point in between consecutive points for better accuracy
  
  Waypoints waypoints;
  waypoints.x  = map_waypoints_x;
  waypoints.y  = map_waypoints_y;
  waypoints.s  = map_waypoints_s;
  waypoints.dx = map_waypoints_dx;
  waypoints.dy = map_waypoints_dy;
  waypoints.num_spline_points = 50;

  printf(" ********************* \n");
  printf(" Waypoints being splined to add more points for accuracy...\n");

  DEBUG("current points " << waypoints.x.size());
  waypoints_spline(&waypoints);

  
  // Create an instance of Road class.
  // It holds
  //    - ego_vehicle
  //    - other vehicles on the road as per sensor fusion data
  //    - waypoints data for the road
  auto road = Road(waypoints);

  printf(" Waypoints spline successful! \n");
  printf(" Old number of points %lu \n", map_waypoints_x.size());
  printf(" NEW number of points %lu \n", waypoints.x_spline.size());
  printf(" ********************* \n");

  cout << endl;
  if(argc == 1){
    cout << " udacity track!" << endl;
  } else {
    cout << argv[1] << " track" << endl;
  }
  cout << endl;
  
  Vehicle *ego_vehicle = &road.ego_vehicle;

  // Send the waypoints info to the ego vehicle
  ego_vehicle->configure(&road.waypoints);

  // Get the current time and store in start_time
  high_resolution_clock::time_point start_time = high_resolution_clock::now();
  
  // cur_time_msec will be used to get the start of each cycle in
  // relation to the start_time
  double cur_time_msec = 0;
  
  // For debugging
  int debug_loop = 1;
  int debug_loop_end = 100000;

  h.onMessage([&road, &start_time, &cur_time_msec, &debug_loop, &debug_loop_end]
	      (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
	       uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    auto sdata = string(data).substr(0, length);
    cout << sdata << endl;

    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      
      auto s = hasData(data);
      
      if (s != "") {
	auto j = json::parse(s);
        
	string event = j[0].get<string>();
        
	if (event == "telemetry") {

	  cur_time_msec = get_current_time(start_time, cur_time_msec);
	  
	  /**
	  if(debug_loop > debug_loop_end){
	    DEBUG("DEBUG LOOP END " << debug_loop << "," << debug_loop_end);
	    exit(EXIT_SUCCESS);
	  }
	  **/
	  
	  DEBUG(" ****** ITER %ul " << debug_loop++ << " ******");
	  DEBUG(" ****** Previous cycle time = " << previous_cycle_time << " msecs");

	  // Main car's localization Data
	  double car_x     = j[1]["x"];
	  double car_y     = j[1]["y"];
	  double car_s     = j[1]["s"];
	  double car_d     = j[1]["d"];
	  double car_yaw   = j[1]["yaw"];
	  double car_speed = j[1]["speed"];

	  // Previous path data given to the Planner
	  auto previous_path_x = j[1]["previous_path_x"];
	  auto previous_path_y = j[1]["previous_path_y"];
	  
	  // Previous path's end s and d values 
	  double end_path_s = j[1]["end_path_s"];
	  double end_path_d = j[1]["end_path_d"];

	  // Sensor Fusion Data, a list of all other cars on the same side of the road.
	  auto sensor_fusion = j[1]["sensor_fusion"];

	  GET_CUR_TIME_START();

	  int prev_size = previous_path_x.size();
	  
	  Vehicle *ego_vehicle = &road.ego_vehicle;

	  // Update ego vehicle with the latest info
	  ego_vehicle->update_ego(cur_time_msec,
				  car_x,
				  car_y,
				  car_s,
				  car_d,
				  car_speed,
				  car_yaw);

	  debug_print_vehicle_info("Ego Vehicle", *ego_vehicle);

	  // This is an important step for how the algorithm works.
	  // The current segment of the car is set to the end_path_s
	  // This means we create the trajectory only after the last
	  // point we had provided in the previous cycle
	  if(prev_size > 0){
	    car_s = end_path_s;
	  }
	  
	  // Update other vehicle locations from sensor fusion data
	  DEBUG("Sensor fusion size " << sensor_fusion.size());
	  
	  for (int i = 0; i < sensor_fusion.size() ; i++)
	    {

	      double vehicle_d = sensor_fusion[i][6];
	      
	      map<int, Vehicle>::iterator it = road.vehicles.begin();

	      it = road.vehicles.find(sensor_fusion[i][0]);

	      if (it != road.vehicles.end())
		{

		  // If the vehicle has been seen before update the dictionary
		  Vehicle *vehicle = &it->second;
		  vehicle->update(cur_time_msec,
				  sensor_fusion[i][1], 
				  sensor_fusion[i][2],
				  sensor_fusion[i][3],
				  sensor_fusion[i][4],
				  sensor_fusion[i][5],
				  sensor_fusion[i][6]);

		  debug_print_vehicle_info("", *vehicle);
		  
		}
	      else
		{
		  // If the vehicle is seen for the first time then create
		  // the vehicles and put it in the dictionary	    
		  Vehicle vehicle = Vehicle(sensor_fusion[i][0],
					    sensor_fusion[i][1], 
					    sensor_fusion[i][2],
					    sensor_fusion[i][3],
					    sensor_fusion[i][4],
					    sensor_fusion[i][5],
					    sensor_fusion[i][6]);

		  vehicle.configure(&road.waypoints);

		  road.vehicles.insert(std::pair<int, Vehicle>
				       (sensor_fusion[i][0], vehicle));
		  
		}
	      
	    }
	  
	  // Convertion previous path to vector of double
	  vector <double> prev_path_x;
	  vector <double> prev_path_y;
	  for(int i = 0; i < previous_path_x.size(); i++)
	    {
	      prev_path_x.push_back(previous_path_x[i]);
	      prev_path_y.push_back(previous_path_y[i]);
	    }
	  
	  vector<double> next_x_vals;
	  vector<double> next_y_vals;

	  // update_state function in vehicle.cpp will return the final points
	  // This includes the previous points + additional points.
	  std::tie(next_x_vals, next_y_vals) = 
	    ego_vehicle->update_state(&road.vehicles,
				      &prev_path_x,
				      &prev_path_y,
				      &road.waypoints,
				      car_s);
	  	  
	  json msgJson;

	  msgJson["next_x"] = next_x_vals;
	  msgJson["next_y"] = next_y_vals;

	  auto msg = "42[\"control\","+ msgJson.dump()+"]";

	  GET_CUR_TIME_END();
	  GET_DURATION("******* Total duration:");

	  //this_thread::sleep_for(chrono::milliseconds(1000));
	  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
	}
      } else {
	// Manual driving
	std::string msg = "42[\"manual\",{}]";
	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
      
    }
    
	      });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
		    const std::string s = "<h1>Hello world!</h1>";
		    if (req.getUrl().valueLength == 1) {
		      res->end(s.data(), s.length());
		    } else {
		      // i guess this should be done more gracefully?
		      res->end(nullptr, 0);
		    }
		  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
      std::cout << "Connected!!!" << std::endl;
    });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
		      ws.close();
		      std::cout << "Disconnected" << std::endl;
		    });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
