#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"
#include <math.h>
// for convenience
using nlohmann::json;
using std::string;
#ifndef M_PI
#define M_PI   3.14159265358979323846264338327950288;
#endif
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

int main()
{
    uWS::Hub h;

    PID pid_s;
    PID pid_t;

    /**
     * Initialize the pid variable.
     */

     /* Manually tuned PID coefficients for steering control */
    pid_s.Init(0.05, 0.005, 1.7);

    /* Manually tuned PID coefficients for throttle control */
    pid_t.Init(0.001, 0.0, 1.7);

    h.onMessage([&pid_s, &pid_t](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode)
        {
            // "42" at the start of the message means there's a websocket message event.
            // The 4 signifies a websocket message
            // The 2 signifies a websocket event
            if (length && length > 2 && data[0] == '4' && data[1] == '2')
            {
                auto s = hasData(string(data).substr(0, length));

                if (s != "")
                {
                    auto j = json::parse(s);

                    string event = j[0].get<string>();

                    if (event == "telemetry")
                    {
                        // j[1] is the data JSON object
                        double cte = std::stod(j[1]["cte"].get<string>());
                        double speed = std::stod(j[1]["speed"].get<string>());
                        double angle = std::stod(j[1]["steering_angle"].get<string>());

                        /* Initial steer and throttle value */
                        double steer_value = 0.0;
                        double throttle_value = 0.3;

                        /* PART 1: Error computation for sterring control */
                        pid_s.UpdateError(cte);
#if flag_Twiddle
                        pid.Twiddle();
#endif // flag_Twiddle
                        steer_value = pid_s.TotalError();

                        /* PART 2: Error computation for throttle control */
                        pid_t.UpdateError(cte);
                        throttle_value = throttle_value - pid_t.TotalError();


                        json msgJson;
                        msgJson["steering_angle"] = steer_value;
                        msgJson["throttle"] = throttle_value;
                        auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                        //std::cout << msg << std::endl;
                        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

                    }  // end "telemetry" if
                }
                else
                {
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
        char *message, size_t length) {
            ws.close();
            std::cout << "Disconnected" << std::endl;
        });

    int port = 4567;
    if (h.listen("127.0.0.1", port)) {
        std::cout << "Listening to port " << port << std::endl;
    }
    else {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }

    h.run();
}