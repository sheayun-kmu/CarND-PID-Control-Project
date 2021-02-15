#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  PID steer_pid;
  PID throttle_pid;
  /**
   * TODO: Initialize the pid variable.
   */
  double Kp_s = 0.08;
  double Ki_s = 0.1;
  double Kd_s = 0.1;
  steer_pid.Init(Kp_s, Ki_s, Kd_s);

  double max_speed = 100.0;
  double Kp_t = 0.05;
  double Ki_t = 0.0;
  double Kd_t = 0.0;
  throttle_pid.Init(Kp_t, Ki_t, Kd_t);

  int simul_count = 0;

  h.onMessage(
    [&simul_count, &steer_pid, &throttle_pid, &max_speed]
    (
      uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
      uWS::OpCode opCode
    ) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));
      simul_count++;

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          double throttle_value;
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
          steer_pid.UpdateError(cte);
          steer_value = steer_pid.GetControlValue();
          if (steer_value > 1.0) {
            steer_value = 1.0;
          } else if (steer_value < -1.0) {
            steer_value = -1.0;
          }

          // determine target speed taking angle into account
          double acc_factor = 2.0;
          double target_speed = max_speed;
          target_speed -= acc_factor * fabs(angle);

          // calculate throttle value to apply for target speed
          double speed_err = speed - target_speed;
          throttle_pid.UpdateError(speed_err);
          throttle_value = throttle_pid.GetControlValue();
          if (throttle_value > 1.0) {
            throttle_value = 1.0;
          } else if (speed < -1.0) {
            throttle_value = -1.0;
          }

          // DEBUG
          /*
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value
                    << " Throttle Value: " << throttle_value << std::endl;
          std::cout << "\tSpeed: " << speed <<"\tAngle: " << angle << std::endl;
          */
          if (simul_count % 100 == 0) {
            std::cout << "count: " << simul_count << std::endl;
          }
          if (simul_count == 500) {
            std::cout << "Resetting accumulated error." << std::endl;
          } else if (simul_count == 2000) {
            std::cout << "MSE: " << steer_pid.TotalError() << std::endl;
            exit(0);
          }

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage

  h.onConnection([](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([](uWS::WebSocket<uWS::SERVER> ws, int code,
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