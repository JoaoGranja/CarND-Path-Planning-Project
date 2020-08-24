#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>

#define KL   1
#define LCL  4
#define LCR  5
#define MAX_SPEED 50

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
double successor_velocity(vector<vector<double>> sensor_fusion, double car_s, int intended_lane, double prev_size, double curr_vel, int curr_state, int next_state) {
  double next_vel;
  next_vel = curr_vel;

  if (curr_state == next_state)
  {
    next_vel -= 0.224;
  }
  else 
  {
    for(int i = 0; i < sensor_fusion.size(); i++)
  	{
      //is there any car on my intended lane?
      float d = sensor_fusion[i][6];
      if( (d < (4+4*intended_lane)) && (d > (4*intended_lane)))
      {
        double vx = sensor_fusion[i][3];
        double vy = sensor_fusion[i][4];
        double check_speed = sqrt((vx*vx)+(vy*vy));
        double check_car_s = sensor_fusion[i][5];

        check_car_s+=((double)prev_size*0.02*check_speed);

        if (((check_car_s - car_s) < 30) || (((car_s - check_car_s) < 30) && (check_speed < curr_vel) ))
        {
          if (check_speed > curr_vel)
            next_vel += 0.224;
          else
            next_vel -= 0.224;
          //std::cout << "i " << i << " check_speed " << check_speed << std::endl;
        }
      }
    }
  }
  return next_vel;
}

float calc_cost(vector<vector<double>> sensor_fusion, double car_s, int curr_lane, double prev_size, double curr_vel){
  double lane_speed, nearest_car_s;
  double reference_speed = MAX_SPEED;
  
  // if no car is on the lane
  lane_speed = MAX_SPEED;
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
      // if a car is very close to our car, return the maximum cost value otherwise determine the "lane_speed"
	  if (( (check_car_s - car_s) > - 20) && (((check_car_s - car_s) < 20) ))
      {
        std::cout << "check_car_s " << check_car_s << " car_s " << car_s << std::endl;
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
          std::cout << "i " << i << " lane_speed " << lane_speed << std::endl;
        }
      }

    }
  }
  return (reference_speed - lane_speed)/reference_speed;
}

int choose_next_state(vector<vector<double>> sensor_fusion, double car_s, int curr_state, int curr_lane, double prev_size, double curr_vel){
  int next_state;
  vector <int> next_states;
  float min_cost, cost;
  bool First_time = true;
  
  next_states = successor_states(curr_lane, curr_state);
  next_state = next_states[0];
  
  // find the best next state using a cost function for speed
  for(vector<int>::iterator it = next_states.begin(); it < next_states.end(); ++it)
  {
    double next_lane = successor_lane(curr_lane, *it);
    cost = calc_cost(sensor_fusion, car_s, next_lane, prev_size, curr_vel);
    std::cout << "next_state " << *it << " next lane " << next_lane <<  " cost " << cost << std::endl;
    
    if (min_cost > cost || First_time)
    {
      min_cost = cost;
      next_state = *it;
      First_time = false;
    }
  }
  return next_state;
}

#endif  // HELPERS_H