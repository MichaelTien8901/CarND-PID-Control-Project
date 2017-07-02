#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

void RestartSimulator(uWS::WebSocket<uWS::SERVER> ws){
  std::string reset_msg = "42[\"reset\",{}]";
  ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
}

class TRAINING {
  private:  
    int steps;
    int count;
    double params_[3], dp_[3];
    int state, index;
    double err, best_err;
    double tolerance;
  public:
    bool enabled;
    TRAINING(): enabled(false) {}

    void Init( PID &pid, double params[], double dp[], int aSteps, double aTolerance)
    {
        steps = aSteps;
        tolerance = aTolerance;
        count = 0;
        state = -2;
        for( int i = 0;i < 3; i ++) {
          this->params_[i] = params[i];
          this->dp_[i] = dp[i];
        }
        pid.Init(params_[0], params_[1], params_[2]);
        enabled = true;
    }
    bool UpdateError( PID &pid, double cte)
    {
        const double REDUCE_FACTOR = 0.5;  // 0.9
        const double EXPAND_FACTOR = 1.5; // 1.1
        if ( state > 2 ) return false;
        if ( ++count < (steps/2)) return false;
        err += cte * cte;
        if ( !(state >= 0 && (err > best_err) ) && ( count < steps)) return false;
        std::cout << "err: " << err << std::endl;
        switch(state) {
        case -1:
        case -2:
           best_err = err;
           std::cout << "best error: " << best_err << std::endl;
           index = 0;  
           state ++;
           if ( state == 0 ) {
             params_[index] += dp_[index];
             pid.Init(params_[0], params_[1], params_[2]);
           }
           break;
        case 0:
           if ( err < best_err) {
               best_err = err;
               dp_[index] *= EXPAND_FACTOR;
               std::cout << "best err:" << best_err << std::endl;
               std::cout << "params:" << params_[0] << "," << params_[1] << "," << params_[2] << std::endl;
               // increase index 
               next_entry:
               state = 0;
               if ( ++index >= 3 ) {                
                   index = 0;
                   // check tolerance
                   std::cout << "current params:" << params_[0] << "," << params_[1] << "," << params_[2] << std::endl;
                   std::cout << "current dp:" << dp_[0] << "," << dp_[1] << "," <<  dp_[2] << std::endl;
                   double sum = dp_[0] + dp_[1] + dp_[2];
                   if ( sum < tolerance) {
                      state = 10;
                      std::cout << "params:" << params_[0] << params_[1] << params_[2] << std::endl;
                      std::cout << "stop training mode:" << std::endl;
                      pid.Init(params_[0], params_[1], params_[2]);
                      return true;
                   } 
                   
               }
               params_[index] += dp_[index];

           } else {
               params_[index] -= 2 * dp_[index];
               state = 1;
           }
           break;
        case 1:
           if ( err < best_err) {
               best_err = err;
               dp_[index] *= EXPAND_FACTOR;
               std::cout << "best err:" << best_err << std::endl;
               std::cout << "params:" << params_[0] << "," << params_[1] << "," << params_[2] << std::endl;
           } else { // recover previous value
               params_[index] += dp_[index];
               dp_[index] *= REDUCE_FACTOR;
           }
           goto next_entry;
        default:
           return false; // do nothing
        }
        count = 0;
        err = 0;
        pid.Init(params_[0], params_[1], params_[2]);
        return true;
    }
};

int main(int argc, char *argv[])
{
  uWS::Hub h;

  PID throttle_pid;
  throttle_pid.Init(0.2, 0.0001, 3.0); // borrow from behavior clone project
  // throttle pid for specified speed
  double set_speed = 30.0; 
  bool enable_twiddle = false;
  PID steering_pid;

  double InitKp = -0.291163;
  double InitKi = -3.8771e-05;
  double InitKd = -4.9;

  int twiddle_step = 6000;
  if ( argc > 3) {
    InitKp = atof(argv[1]);
    InitKi = atof(argv[2]);
    InitKd = atof(argv[3]);
  }

  if ( argc > 4) {
      twiddle_step = atoi(argv[4]);
  }
  steering_pid.Init(InitKp, InitKi, InitKd);

//#define Uses_TRAINING
  double params[3] = { InitKp, InitKi, InitKd };
  double dp[3] = { 0.00351472,5.72311e-07,0.0489643};
  if ( argc > 3 ) {
     dp[0] = fabs(InitKp) / 10;
     dp[1] = fabs(InitKi) / 10;
     dp[2] = fabs(InitKd) / 10;
   }

  TRAINING training;

  if ( argc > 4 || enable_twiddle )
      training.Init(steering_pid, params, dp, twiddle_step, 0.001);

  h.onMessage([&steering_pid, &throttle_pid, &set_speed, &training](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          //double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          steering_pid.UpdateError(cte);
          steer_value = steering_pid.TotalError();
          /* throttle pid error */
          double throttle_error = set_speed - speed - fabs(cte) * 3.5;
          throttle_pid.UpdateError(throttle_error);
          double throttle_value = throttle_pid.TotalError();

          if ( steer_value > 1) {
            steer_value = 1;
            throttle_value = 0; // brake ...
          } else if ( steer_value < -1 ) {
            steer_value = -1;
            throttle_value = 0;
          }
          // DEBUG
          //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
//          msgJson["throttle"] = 0.3;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
//          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#define   Uses_TRAINING
#ifdef  Uses_TRAINING
          if ( training.enabled) {
            if ( training.UpdateError( steering_pid, cte)) {
                RestartSimulator(ws);
                throttle_pid.Reset();
                steering_pid.Reset();
            }
          }

#endif  
        }      
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
