#include <chrono>
#include <fstream>
#include <iostream>
#include <math.h>
#include <thread>
#include <uWS/uWS.h>
#include <vector>

#include "json.hpp"
#include "sdc_constants.h"
#include "sdc_utils.h"

#include "behavior_planner.h"
#include "trajectory_generator.h"

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

int main() {
  uWS::Hub h;

  // Load map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0exi
  double max_s = 6945.554;

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

  TrajectoryGenerator tg(map_waypoints_x, map_waypoints_y, map_waypoints_s,
                         map_waypoints_dx, map_waypoints_dy);

  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
               &map_waypoints_dx, &map_waypoints_dy, &tg](
                  uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                  uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization data.
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner.
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values.
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of
          // the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          // Define a path made up of (x,y) points that the car will visit
          // sequentially every .02 seconds.
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // Copy over unused previous path points.
          const int num_points_to_keep = previous_path_x.size();
          for (int i = 0; i < num_points_to_keep; ++i) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Compute car state at the end of the previous path.
          double curr_x = car_x;
	  double curr_y = car_y;
	  double curr_s = car_s;
	  double curr_d = car_d;
	  double curr_theta = deg2rad(car_yaw);
	  double curr_v = car_speed;
          // If there were remaining values in previous_path, let the values at
	  // the end of previous_path be the current state.
          if (num_points_to_keep > 0) {
  	    double prev_x;
            double prev_y;
	    if (num_points_to_keep == 1) {
              prev_x = car_x;
	      curr_x = next_x_vals.back();
              prev_y = car_y;
	      curr_y = next_y_vals.back();
	    } else if (num_points_to_keep >= 2) {
	      prev_x = next_x_vals[next_x_vals.size() - 2];
              curr_x = next_x_vals.back();
	      prev_y = next_y_vals[next_y_vals.size() - 2];
              curr_y = next_y_vals.back();
	    }
            curr_theta = atan2(curr_y - prev_y, curr_x - prev_x);
            const vector<double> sd = GetFrenet(
                curr_x, curr_y, curr_theta, map_waypoints_x, map_waypoints_y);
            curr_s = sd[0];
            curr_d = sd[1];
            curr_v = distance(prev_x, prev_y, curr_x, curr_y) / TIMESTEP;
	  }

          // Determine what maneuver to execute (i.e. which lane to aim for).
          const vector<double> plan = PlanBehavior(
              car_s, curr_s, curr_d, curr_v, sensor_fusion, num_points_to_keep);

          // Generate a smooth trajectory for the target lane.
          const int target_lane = (int) plan[0];
          const double target_speed = plan[1];
          tg.GenerateTrajectory(
              sensor_fusion, num_points_to_keep, &next_x_vals, &next_y_vals,
              deg2rad(car_yaw), curr_x, curr_y, curr_s, curr_d, curr_theta,
	      curr_v, target_lane, target_speed);

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

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
