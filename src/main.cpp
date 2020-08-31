#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include <thread>
#include "json.hpp"
#include <vector>
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "MPC.h"

// for convenience
using nlohmann::json;
using std::string;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

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

    MPC mpc;

    h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char* data, size_t length,
        uWS::OpCode opCode) {
            // "42" at the start of the message means there's a websocket message event.
            // The 4 signifies a websocket message
            // The 2 signifies a websocket event
            if (length && length > 2 && data[0] == '4' && data[1] == '2') {
                auto s = hasData(string(data).substr(0, length));

                if (s != "") {
                    auto j = json::parse(s);

                    string event = j[0].get<string>();

                    if (event == "telemetry") {
                        // j[1] is the data JSON object
			vector<double> ptsx = j[1]["ptsx"];
			vector<double> ptsy = j[1]["ptsy"];
			double px = j[1]["x"];
			double py = j[1]["y"];
			double psi = j[1]["psi"];
			double v = j[1]["speed"];
			double delta = j[1]["steering_angle"];
			double a = j[1]["throttle"];

			// rotation and translation to change reference system to be centered at origin and 0 degrees
			for (size_t i = 0; i < ptsx.size(); ++i){
				double shift_x = ptsx[i] - px; 
				double shift_y = ptsy[i] - py;

				ptsx[i] = shift_x * cos(-psi) - shift_y * sin(-psi);
				ptsy[i] = shift_x * sin(-psi) + shift_y * cos(-psi);
			}
			
			// convert to Eigen::VectorXd
			double *ptrx = &ptsx[0];
			Eigen::Map<Eigen::VectorXd> ptsx_transform(ptrx, 6);
			double *ptry = &ptsy[0];
			Eigen::Map<Eigen::VectorXd> ptsy_transform(ptry, 6);

			// fit polynomial to trajectory points
			auto coeffs = polyfit(ptsx_transform, ptsy_transform, 3);

			double cte = polyeval(coeffs, 0);

			double epsi = psi - atan(coeffs[1] + 2*px*coeffs[2] + 3*px*px*coeffs[3]);

			//double epsi = -atan(coeffs[1]);

			const double dt = 0.1; // latency for predicting time during actuation
			const double Lf = 2.67;

			//predict future states after latency period

			//double pred_px = px + v*cos(psi)*dt;
			double pred_px = 0 + v*dt;// px = 0 in the new reference system
			//double pred_py = py + v*sin(psi)*dt;
			double pred_py = 0; // py = 0 in the new reference system
			//double pred_psi = psi + v*(-delta/Lf)*dt; 
			double pred_psi = 0 + v*(delta/Lf)*dt;	// psi = 0 in the new reference system
			double pred_v = v + a*dt;
			double pred_cte = cte + v*sin(epsi)*dt;
			double pred_epsi = epsi + v*(delta/Lf)*dt;

			Eigen::VectorXd state(6);
			state << pred_px, pred_py, pred_psi, pred_v, pred_cte, pred_epsi;

			auto vars = mpc.Solve(state,coeffs);	

			// create vector to display next waypoints
			vector<double> next_x_vals;
			vector<double> next_y_vals;
			double poly_inc = 2.5;
			int num_points = 25;
			for (int i = 1; i < num_points; ++i){
				double future_x = poly_inc*i;
				double future_y = polyeval(coeffs,future_x);
				next_x_vals.push_back(future_x);
				next_y_vals.push_back(future_y);
			}
					
			const double angle_normalize_factor = deg2rad(25)*Lf;
			double steer_value = -vars[0]/angle_normalize_factor;
			double throttle_value = -vars[1];

			// display MPC predicted trajectory
			vector<double> mpc_x_vals;
			vector<double> mpc_y_vals;

			for (size_t i = 2; i < vars.size(); ++i){	
				if (i % 2 == 0)
					mpc_x_vals.push_back(vars[i]);
				else
					mpc_y_vals.push_back(vars[i]);
			}				
			
			// create message to return to simulator
			json msgJson;
			msgJson["steering_angle"] = steer_value;
			msgJson["throttle"] = throttle_value;
			msgJson["mpc_x"] = mpc_x_vals;
			msgJson["mpc_y"] = mpc_y_vals;
			msgJson["next_x"] = next_x_vals;
			msgJson["next_y"] = next_y_vals;

                        auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                        std::cout << msg << std::endl;
			this_thread::sleep_for(chrono::milliseconds(100));
                        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                    }  // end "telemetry" if
                }
                else {
                    // Manual driving
                    string msg = "42[\"manual\",{}]";
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }
            }  // end websocket message if
        }); // end h.onMessage

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
        });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
        char* message, size_t length) {
            ws.close();
            std::cout << "Disconnected" << std::endl;
        });

    int port = 4567;
    if (h.listen(port)) {
        std::cout << "Listening to port " << port << std::endl;
    }
    else {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }

    h.run();
}
