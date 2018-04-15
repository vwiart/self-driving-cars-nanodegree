#ifndef __CAR_POSITION_H__
#define __CAR_POSITION_H__

#include <vector>
#include <math.h>
#include "../json.hpp"

using json = nlohmann::json;
using namespace std;

class Vehicle {
public:
    Vehicle();
    void update_state(json telemetry);
    void set_speed(double speed);
    void set_yaw(double yaw);
    void set_lane(int line);
    const double x();
    const double y();
    const double s();
    const double d();
    const double yaw();
    const int lane();
    const double speed();
    const vector<double> prev_path_x();
    const vector<double> prev_path_y();
    const int prev_path_size();
private:
    int lane_;
    double x_, y_, s_, d_, yaw_, speed_;
    vector<double> previous_path_x_, previous_path_y_;
};

#endif
