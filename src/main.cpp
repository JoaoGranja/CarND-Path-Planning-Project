#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"


// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;
  
  //velocity reference
  double ref_vel = 0.0;
  
  //lane index
  int curr_lane = 1;
  
  //state
  int curr_state = KL;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &ref_vel, &curr_lane, &curr_state]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          //std::cout << "car_x " << car_x  << std::endl;
          //std::cout << "car_y " << car_y  << std::endl;
          
          

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          vector<double> ptsx;
          vector<double> ptsy;
          double next_state, next_lane;
          
          // previous path size regarding number of points 
          int prev_size = previous_path_x.size();
          
          if (prev_size > 0)
          {
            car_s = end_path_s; //last path point
          }
          
          bool so_close = false;
          
          //find ref_vel to use
          for(int i = 0; i < sensor_fusion.size(); i++)
          {
            //car is in my line??
            float d = sensor_fusion[i][6];
            if( (d < (2+4*curr_lane+2)) && (d > (2+4*curr_lane-2)))
            {
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt((vx*vx)+(vy*vy));
              double check_car_s = sensor_fusion[i][5];
              
              check_car_s+=((double)prev_size*0.02*check_speed);
              
              if ((check_car_s > car_s) && ((check_car_s - car_s) < 30))
              {
                so_close = true;
              }
              
            }
          }
          std::cout << "curr_state " << curr_state  << " curr_lane " << curr_lane << std::endl;
          if (so_close)
          {
            next_state = choose_next_state(sensor_fusion, car_s, curr_state, curr_lane, prev_size, ref_vel);
            next_lane = successor_lane(curr_lane, next_state);
            ref_vel = successor_velocity(sensor_fusion, car_s, next_lane, prev_size, ref_vel, curr_state, next_state);
            curr_state = next_state;
            curr_lane = next_lane;
            std::cout << "next_state " << next_state  << " curr_lane " << curr_lane << std::endl;
          }
          else if( ref_vel <= 49.5)
          {
            ref_vel += 0.224;
          }
          
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          
          // 1 -  add all previous_path points
          for (int i=0; i<prev_size; i++){
            
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
            
          
          if (prev_size < 2)
          {
            //std::cout << "car_x " << car_x  << std::endl;
            //std::cout << "car_y " << car_y  << std::endl;
            //std::cout << "car_yaw " << car_yaw  << std::endl;
            
            double prev_car_x = ref_x - cos(ref_yaw);
            double prev_car_y = ref_y - sin(ref_yaw);
            
            ptsx.push_back(prev_car_x);
            ptsx.push_back(ref_x);
            
            ptsy.push_back(prev_car_y);
            ptsy.push_back(ref_y);
          }
          else
          {
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];
            
            double prev_ref_x = previous_path_x[prev_size-2];
            double prev_ref_y = previous_path_y[prev_size-2];
            
            ref_yaw = atan2(ref_y-prev_ref_y,ref_x-prev_ref_x);
            
            ptsx.push_back(prev_ref_x);
            ptsx.push_back(ref_x);
            
            ptsy.push_back(prev_ref_y);
            ptsy.push_back(ref_y);
          }
          
          vector<double> next_wp0 = getXY(car_s + 30, (2 + 4*curr_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);  
          vector<double> next_wp1 = getXY(car_s + 60, (2 + 4*curr_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y); 
          vector<double> next_wp2 = getXY(car_s + 90, (2 + 4*curr_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y); 
          
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
            
          //shift rotation and translation
          for (int i = 0; i < ptsx.size(); i++)
          {
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;
            
            ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
            ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
            
            //std::cout << "spline points " << i  << std::endl;
            //std::cout << "y points " << ptsy[i]  << std::endl;
            //std::cout << "x points " << ptsx[i]  << std::endl;
          }
          
          // create a spline
          tk::spline s;
          s.set_points(ptsx, ptsy);
                
          // calculate how to break up spline points so that we travel at our desired reference velocity
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x*target_x) + (target_y*target_y));
          
          double x_add_on = 0;
          
          // 2 -  add remaining points which belongs to the spline
          double N = (target_dist/(0.02 * ref_vel/2.24));
          for (int i = 0; i < 50-prev_size; i++){
            
            x_add_on += (target_x/N);
            
            double x_ref = x_add_on;
            double y_ref = s(x_add_on);
            
            //std::cout << "x_ref y_ref " << x_ref << "   " << y_ref << std::endl;
            
            //rotate back to normal coordinate system 
            double x_point;
            double y_point;
            x_point = ref_x + (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
            y_point = ref_y + (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));           
            
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }
          //END
          
          //std::cout << "Finish iteration " << std::endl;
          //for (int i = 0; i < next_x_vals.size(); i++){
          //  std::cout << "x next_x_vals " << next_x_vals[i]  << std::endl;
          //  std::cout << "y next_y_vals " << next_y_vals[i]  << std::endl;
          //}
          
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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