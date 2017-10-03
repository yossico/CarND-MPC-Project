#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

using namespace std;
using namespace Eigen;

// for convenience
using json = nlohmann::json;


// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
const double dt = 0.1; //0.1 second between timestamps
const double Lf = 2.67; //0.1 second between timestamps



// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(VectorXd xvals, VectorXd yvals, int order) 
{
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

/*
void OriginTransform(VectorXd &px, VectorXd& py, VectorXd &Tx, VectorXd& Ty, double x, double y, const double psi)
{
	const int N = px.size();
	for (int i = 0; i < N; i++)
	{	  //adapting car location to 0,0	
		const double shift_x = px[i] - x;
		const double shift_y = py[i] - y;
		//adapting path points to the new location of the car as 0,0
		Tx[i] = (shift_x*cos(-psi) - shift_y*sin(-psi));
		Ty[i] = (shift_x*sin(-psi) + shift_y*cos(-psi));
	}
}*/

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
		  const double px = j[1]["x"];
		  const double py = j[1]["y"];
		  const double psi = j[1]["psi"];
		  const double v_mph = j[1]["speed"];
		  const double v = v_mph * 0.447;// mph to m/s
		  const double steering_angle = j[1]["steering_angle"];
		  const double throttle = j[1]["throttle"];
		            
          /* TODO: Calculate steering angle and throttle using MPC. Both are in between [-1, 1]. */

		  const int N = ptsx.size();  
		  
		  VectorXd ptsxT(N);
		  VectorXd ptsyT(N);
		  //OriginTransform(&px, &py, &ptsxT, &ptsyT, px, py,  psi);
		  for (int i = 0; i < N; i++)
		  {	  //adapting car location to 0,0	
			  const double shift_x = ptsx[i] - px;
			  const double shift_y = ptsy[i] - py;
			  //adapting path points to the new location of the car as 0,0
			  ptsxT[i] = (shift_x*cos(-psi) - shift_y*sin(-psi));
			  ptsyT[i] = (shift_x*sin(-psi) + shift_y*cos(-psi));
		  }

		  //find the coefficients of a 3rd degree polynomial which fit the waypoints
		  auto coeffs = polyfit(ptsxT, ptsyT, 3);
		   //calculate the cross-track-error using polyeval
		  double cte = coeffs[0];// polyeval(coeffs, 0);
		  //calculate the orientation as atan of coeff[1] (the rest of the variables in the calculation are zeros)
		  double epsi = -atan(coeffs[1]);

		  // Kinematic model used to predict vehicle state at current_time +dt 
		  const double px_act = v * dt;
		  const double py_act = 0;
		  const double psi_act = -v * steering_angle * dt / Lf;
		  const double v_act = v + throttle * dt;
		  const double cte_act = cte + v * sin(epsi) * dt;
		  const double epsi_act = epsi + psi_act;
		  Eigen::VectorXd state(6);
		  //set the vechicle state
		  state << px_act, py_act, psi_act, v_act, cte_act, epsi_act;
		  
		  //solve			
		  vector<double> mpc_solution = mpc.Solve(state, coeffs);
		  std::cout << "finished solve" << std::endl;

		  double steer_value = mpc_solution[0] / deg2rad(25); // divide by deg2rad(25), convert to [-1..1] range
		  double throttle_value = mpc_solution[1];
		  //Display the MPC predicted trajectory (the GREEN line we are going to drive on) 
		  vector<double> mpc_x_vals = mpc.mpc_x;
		  vector<double> mpc_y_vals = mpc.mpc_y;

		  //create the trajectory points
		  vector<double> next_x_vals;
		  vector<double> next_y_vals;
		  double poly_inc = 2.5;
		  for (int i = 0; i < ptsx.size(); i++)
		  {
			  //the waypoints/reference line (desired trajectory) calculated using the polyfit with the coeffs
			  next_x_vals.push_back(ptsxT[i]);// poly_inc*i);
			  next_y_vals.push_back(ptsyT[i]);// polyeval(coeffs, poly_inc*i));
		  } 
				  
		  
		  json msgJson;
		  msgJson["steering_angle"] = -steer_value;
		  msgJson["throttle"] = throttle_value;
		  //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system the points in the simulator are connected by a Yellow line
		  msgJson["next_x"] = next_x_vals;
		  msgJson["next_y"] = next_y_vals;
		  msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;
          
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where the car does actuate the commands instantly. 
		  // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.          
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
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
