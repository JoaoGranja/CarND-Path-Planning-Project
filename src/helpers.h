#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>
#include "spline.h"

#define KL   1
#define LCL  2
#define LCR  3
#define MAX_SPEED 49.5

// for convenience
using std::string;
using std::vector;


// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
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

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
                    const vector<double> &maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, 
                 const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, 
                         const vector<double> &maps_x, 
                         const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
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

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                         (maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}


// Provides the possible next states given the current state
vector<int> successor_states(int curr_lane, int curr_state) {
  vector<int> next_states;
  next_states.push_back(KL);
  if(curr_state == KL)
  {
  	if (curr_lane != 0)
    {
    	next_states.push_back(LCL);
    }
	if (curr_lane < 2 )
    {
    	next_states.push_back(LCR);
    }
  }
  return next_states;
} 

// Provides the possible next lane given the next state
int successor_lane(int curr_lane, int next_state) {
  
  if(next_state == LCL)
  {
    return curr_lane - 1;
  }
  else if(next_state == LCR)
  {
    return curr_lane + 1;
  }
  
  return curr_lane;
} 
 

// Provides the possible next velocity reference given the next state
double successor_velocity(vector<vector<double>> sensor_fusion, double car_s, int next_lane, double prev_size) {
  double next_vel;
  next_vel = MAX_SPEED;
 
  for(int i = 0; i < sensor_fusion.size(); i++)
  {
    //is there any car on my intended lane?
    float d = sensor_fusion[i][6];
    if( (d < (4+4*next_lane)) && (d > (4*next_lane)))
    {
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed = sqrt((vx*vx)+(vy*vy));
      double check_car_s = sensor_fusion[i][5];

      check_car_s+=((double)prev_size*0.02*check_speed);

      if ((check_car_s > car_s) && ((check_car_s - car_s) < 30))//|| (((car_s - check_car_s) < 15) && (check_speed < curr_vel) ))
      {
        if (check_speed < next_vel)
        {
          next_vel = check_speed;
          //std::cout << "i " << i << " check_speed " << check_speed  << std::endl;
        }
      }
    }
  }

  return next_vel;
}

// calcualte the velocity cost for each possible next state
float vel_cost(vector<vector<double>> sensor_fusion, double car_s, int curr_lane, double prev_size, double curr_vel, double next_state){
  double lane_speed, nearest_car_s;
  double reference_speed = MAX_SPEED;

  
  // if no car is on the lane
  lane_speed = 2*MAX_SPEED;
  nearest_car_s = 50;
  
  for(int i = 0; i < sensor_fusion.size(); i++)
  {
    //car is in my line??
    float d = sensor_fusion[i][6];
    if( (d < (4+4*curr_lane)) && (d > (4*curr_lane)))
    {
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed = sqrt((vx*vx)+(vy*vy));
      double check_car_s = sensor_fusion[i][5];

      check_car_s+=((double)prev_size*0.02*check_speed); 
      std::cout << " next_state " << next_state << " check_car_s " << check_car_s << " car_s " << car_s << std::endl;

      
      // if a car is very close to our car, return a high cost value otherwise determine the "lane_speed"
      if(next_state == KL){
        if ( ((check_car_s - car_s) > 0) && ((check_car_s - car_s) < 20) )
        {
          return 2;
        }  
        else
        {
          if ( ((check_car_s - car_s) < nearest_car_s) && ((check_car_s - car_s) > 0) )
          {
            nearest_car_s = (check_car_s - car_s);
            lane_speed = check_speed;
            std::cout << "nearest_car_s " << nearest_car_s << " curr_lane " << curr_lane << " lane_speed " << lane_speed << std::endl;
          }
        }
      }
      else
      {
        if ( ((check_car_s - car_s) > -10) && ((check_car_s - car_s) < 20) )
        {
          return 2;
        }  
        else if ( ((check_car_s - car_s) > - 20) && ((check_car_s - car_s) < -10) && (check_speed > curr_vel))
        {
          return 1;
        } 
        else
        {
          if (( (check_car_s - car_s) > - nearest_car_s) && (((check_car_s - car_s) < nearest_car_s) ))
          {
            nearest_car_s = (check_car_s - car_s);
            if(nearest_car_s < 0)
              nearest_car_s = nearest_car_s * -1;
            lane_speed = check_speed;
            std::cout << "nearest_car_s " << nearest_car_s << " curr_lane" << curr_lane << " lane_speed " << lane_speed << std::endl;
          }
        }
      }
    }
  }
  
  return (reference_speed - lane_speed)/reference_speed;
}

// find the best state to follow
int choose_next_state(vector<vector<double>> sensor_fusion, double car_s, double car_d, int curr_state, int curr_lane, double prev_size, double curr_vel){
  int next_state;
  vector <int> next_states;
  float min_cost, cost;
  bool First_time = true;
  
  // if the car is not around the center of the lane keep the state
  if( (car_d > (2.5+4*curr_lane)) && (car_d < (4*curr_lane + 1.5)))
  {
     return curr_state;
  }
  
  next_states = successor_states(curr_lane, curr_state);
  next_state = next_states[0];
  
  // find the best next state using a cost function for speed
  for(vector<int>::iterator it = next_states.begin(); it < next_states.end(); ++it)
  {
    double next_lane = successor_lane(curr_lane, *it);
    cost = vel_cost(sensor_fusion, car_s, next_lane, prev_size, curr_vel, *it);
    
    //penalize cost for lane different than center one. Because from lane 1 we have more chance to change for a faster lane
    if (next_lane != 1 && cost < 1)
    	cost += 0.1;
    
    //penalize cost for chaging lane instead of keep lane. This is to avoid changing lane for a small velocity diference
    if (*it != KL )
    	cost += 0.05;
    
    std::cout << "state " << *it << " lane " << next_lane <<  " cost " << cost << std::endl;
    
    if (min_cost > cost || First_time)
    {
      min_cost = cost;
      next_state = *it;
      First_time = false;
    }
  }
  return next_state;
}

vector<vector<double>> trajectory_generation(double car_x, double car_y, double car_yaw, double car_s, 
                                            vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> previous_path_x, vector<double> previous_path_y, double ref_vel, double curr_lane, double &curr_vel){
  
  int prev_size = previous_path_x.size();
  double ref_x = car_x;
  double ref_y = car_y;
  double ref_yaw = deg2rad(car_yaw);
  vector<vector<double>> next_vals(2);
  vector<double> ptsx;
  vector<double> ptsy;
  
  

  // 1 -  add all previous_path points
  for (int i=0; i<prev_size; i++){

    next_vals[0].push_back(previous_path_x[i]);
    next_vals[1].push_back(previous_path_y[i]);
  }


  if (prev_size < 2)
  {

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

  }

  // create a spline
  tk::spline s;
  s.set_points(ptsx, ptsy);

  // calculate how to break up spline points so that we travel at our desired reference velocity
  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt((target_x*target_x) + (target_y*target_y));

  double x_add_on = 0;
  double vel = curr_vel;
  
  // 2 -  add remaining points which belongs to the spline
  for (int i = 0; i < 50-prev_size; i++){
    //calculate next point following the formula delta_x = v*t+0.5*a*t*t
    if (vel < ref_vel)
    {
      vel += 0.224;
      x_add_on += (vel*0.02/2.24) + (0.5*0.224*0.02*0.02);
    }
    else if (vel > ref_vel)
    {
      vel -= 0.224;
      x_add_on += (vel*0.02/2.24) - (0.5*0.224*0.02*0.02);
    }
    else
    {
      x_add_on += (vel*0.02/2.24);
    }
    double x_ref = x_add_on;
    double y_ref = s(x_add_on);


    //rotate back to normal coordinate system 
    double x_point;
    double y_point;
    x_point = ref_x + (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
    y_point = ref_y + (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));           

    next_vals[0].push_back(x_point);
    next_vals[1].push_back(y_point);
  }
  
  //update current velocity
  curr_vel = vel;
  return next_vals;
}

#endif  // HELPERS_H