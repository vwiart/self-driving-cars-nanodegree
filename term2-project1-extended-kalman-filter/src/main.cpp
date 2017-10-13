#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include <math.h>
#include "measure.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_first_of("]");
    if (found_null != std::string::npos) {
        return "";
    } else if (b1 != std::string::npos && b2 != std::string::npos) {
        return s.substr(b1, b2 - b1 + 1);
    }
    return "";
}

VectorXd computeRMSE(const vector<VectorXd> estimations, const vector<VectorXd> actuals) {
    VectorXd rmse(4);
    rmse << 0, 0, 0, 0;

    if (estimations.size() == 0 || estimations.size() != actuals.size()) {
        cout << "Cannot compute RMSE " << endl;
        return rmse;
    }

    for (int i = 0; i < estimations.size(); i++) {
        VectorXd residuals = estimations[i] - actuals[i];

        residuals = residuals.array() * residuals.array();
        rmse += residuals;
    }

    rmse = rmse / estimations.size();
    rmse = rmse.array().sqrt();
    return rmse;
}

int main() {
    uWS::Hub h;

    Measure measure = Measure();

    // used to compute the RMSE later
    vector<VectorXd> estimations;
    vector<VectorXd> ground_truth;

    h.onMessage([&measure,&estimations,&ground_truth](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event

        if (length && length > 2 && data[0] == '4' && data[1] == '2') {
            auto s = hasData(std::string(data));
            if (s != "") {
                auto j = json::parse(s);
                std::string event = j[0].get<std::string>();
            
                if (event == "telemetry") {
                    // j[1] is the data JSON object

                    string sensor_measurment = j[1]["sensor_measurement"];
                    istringstream iss(sensor_measurment);

                    // reads first element from the current line
                    string sensor_type;
                    iss >> sensor_type;

                    MeasureData md;
                    if (sensor_type.compare("L") == 0) {
                        md.type = LIDAR;
                        iss >> md.x;
                        iss >> md.y;
                        iss >> md.timestamp;
                    } else if (sensor_type.compare("R") == 0) {
                        md.type = RADAR;
                        iss >> md.rho;
                        iss >> md.phi;
                        iss >> md.rhodot;
                        iss >> md.timestamp;
                        
                    }
                    float x_gt, y_gt, vx_gt, vy_gt;
                    iss >> x_gt;
                    iss >> y_gt;
                    iss >> vx_gt;
                    iss >> vy_gt;
                    VectorXd gt_values(4);
                    gt_values(0) = x_gt;
                    gt_values(1) = y_gt; 
                    gt_values(2) = vx_gt;
                    gt_values(3) = vy_gt;
                    ground_truth.push_back(gt_values);

                    measure.process(md);

                    #ifdef __DEBUG__
                        cout << "Ground truth:" << endl << gt_values << endl;
                    #endif

                    VectorXd estimate = measure.getStateVector();
                    estimations.push_back(estimate);
                    VectorXd rmse = computeRMSE(estimations, ground_truth);

                    json msgJson;
                    msgJson["estimate_x"] = estimate(0);
                    msgJson["estimate_y"] = estimate(1);
                    msgJson["rmse_x"] =  rmse(0);
                    msgJson["rmse_y"] =  rmse(1);
                    msgJson["rmse_vx"] = rmse(2);
                    msgJson["rmse_vy"] = rmse(3);

                    auto msg = "42[\"estimate_marker\"," + msgJson.dump() + "]";
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }
            } else {
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
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
    if (h.listen(port)) {
        std::cout << "Listening to port " << port << std::endl;
    } else {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    h.run();
}
