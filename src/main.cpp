#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include <math.h>


// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using namespace std;
//using json = nlohmann::json;

constexpr double pi() { return M_PI; }
double deg_to_rad(double x) { return x * pi() / 180; }
double rad_to_deg(double x) { return x * 180 / pi(); }

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

//Convert Frenet s,d to cartesian x,y
vector<double> get_xy(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1)))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();
	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	double seg_s = (s-maps_s[prev_wp]);
	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);
	double perp_heading = heading-pi()/2;
	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};
}

// json object in str format will be returned, else the empty str
string has_data(string s) {
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

// Convert cartesian x,y to Frenet s,d
vector<double> get_Frenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);
  int prev_wp;

  prev_wp = next_wp-1;
  if(next_wp == 0)
  {
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

  if(centerToPos <= centerToRef)
  {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for(int i = 0; i < prev_wp; i++)
  {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};

}

int get_next_waypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y){

  int closest_waypoint = ClosestWaypoint(x,y,maps_x,maps_y);
  double map_x = maps_x[closest_waypoint];
  double map_y = maps_y[closest_waypoint];
  double heading = atan2( (map_y-y),(map_x-x) );
  double angle = abs(theta-heading);

  if(angle > pi()/4)
  {
    closest_waypoint++;
  }
  return closest_waypoint;
}

int get_closest_waypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y){

  double closest_len = 100000; //large number
  int closest_waypoint = 0;

  for(int i = 0; i < maps_x.size(); i++)
  {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if(dist < closest_len)
    {
      closest_len = dist;
      closest_waypoint = i;
    }
  }
  return closest_waypoint;
}


int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

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

  //start in lane 1:
  int lane = 1;

  //have a ref. velocity to target
  double ref_vel = 0.0;  //mph

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
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

          int prev_size = previous_path_x.size();

          // Preventing collitions.
          if (prev_size > 0) {
          	car_s = end_path_s;
          }
          // Predict neighbor cars positions
          bool car_ahead = false;
          bool car_left = false;
          bool car_righ = false;
          for ( int i = 0; i < sensor_fusion.size(); i++ ){
              float d = sensor_fusion[i][6];
              int car_lane = -1;
              // is it on the same lane we are
              if ( d > 0 && d < 4 ) {
                car_lane = 0;
              } else if ( d > 4 && d < 8 ) {
                car_lane = 1;
              } else if ( d > 8 && d < 12 ) {
                car_lane = 2;
              }
              if (car_lane < 0) {
                continue;
              }
              // Find car speed.
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx + vy*vy);
              double check_car_s = sensor_fusion[i][5];
              // Estimate car s position after executing previous trajectory.
              check_car_s += ((double)prev_size*0.02*check_speed);
              if ( car_lane == lane ) {
                // Car in our lane.
                car_ahead |= check_car_s > car_s && check_car_s - car_s < 30;
              } else if ( car_lane - lane == -1 ) {
                // Car left
                car_left |= car_s - 30 < check_car_s && car_s + 30 > check_car_s;
              } else if ( car_lane - lane == 1 ) {
                // Car right
                car_righ |= car_s - 30 < check_car_s && car_s + 30 > check_car_s;
              }
          }  

          // Check what we need to to (behav)
          double speed_diff = 0;
          const double max_spd = 49.5;
          const double max_accel = .224;
          if ( car_ahead ) { // Car ahead
            if ( !car_left && lane > 0 ) {
              // if there is no car left and there is a left lane.
              lane--; // Change lane left.
            } else if ( !car_righ && lane != 2 ){
             // if there is no car right and there is a right lane.
              lane++; // Change lane right.
            } else {
              speed_diff -= max_accel;
            }
          } else {
            if ( lane != 1 ) { // if we are not on the center lane.
              if ( ( lane == 0 && !car_righ ) || ( lane == 2 && !car_left ) ) {
                lane = 1; // Back to center.
              }
            }
            if ( ref_vel < max_spd ) {
              speed_diff += max_accel;
            }
          }

		  vector<double> ptsx;
          vector<double> ptsy;
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg_to_rad(car_yaw);

          // Do I have have previous points
          if (prev_size < 2){
              // There are not too many...
              double prev_car_x = car_x - cos(car_yaw);
              double prev_car_y = car_y - sin(car_yaw);
              ptsx.push_back(prev_car_x);
              ptsx.push_back(car_x);
              ptsy.push_back(prev_car_y);
              ptsy.push_back(car_y);
          }else{
              // Use the last two points.
              ref_x = previous_path_x[prev_size - 1];
              ref_y = previous_path_y[prev_size - 1];
              double ref_x_prev = previous_path_x[prev_size - 2];
              double ref_y_prev = previous_path_y[prev_size - 2];
              ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
              ptsx.push_back(ref_x_prev);
              ptsx.push_back(ref_x);
              ptsy.push_back(ref_y_prev);
              ptsy.push_back(ref_y);
          }

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          double dist_inc = 0.3;
		  for (int i = 0; i < 50; ++i) {
		  	double next_s = car_s + (i+1)*dist_inc;
		  	double next_d = 6;
		  	vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

  			next_x_vals.push_back([0]);
  			next_y_vals.push_back([1]);
		  }

          //End

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