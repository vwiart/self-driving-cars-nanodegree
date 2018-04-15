#include "vehicle.h"

Vehicle::Vehicle() {
    this->lane_ = 1;
}

// update_state updates the internal state.
void Vehicle::update_state(json telemetry) {
    this->x_ = telemetry[1]["x"];
    this->y_ = telemetry[1]["y"];
    this->s_ = telemetry[1]["s"];
    this->d_ = telemetry[1]["d"];
    this->yaw_ = ((double)telemetry[1]["yaw"]) * M_PI / 180.0; //use rad only

    vector<double> previous_path_x = telemetry[1]["previous_path_x"];
    this->previous_path_x_ = previous_path_x;
    
    vector<double> previous_path_y = telemetry[1]["previous_path_y"];
    this->previous_path_y_ = previous_path_y;

    double end_path_s = telemetry[1]["end_path_s"];
    if (previous_path_x_.size() > 0) {
        this->s_ = end_path_s;
    }
}

// set_speed sets the speed of the vehicle.
void Vehicle::set_speed(double speed) {
    this->speed_ = speed;
}

// set_yaw sets the angle (in rad) of the vehicle.
void Vehicle::set_yaw(double yaw) {
    this->yaw_ = yaw;
}

// set_lane sets the lane of the vehicle.
void Vehicle::set_lane(int lane) {
    this->lane_ = lane;
}

const double Vehicle::x() { return this->x_;}
const double Vehicle::y() { return this->y_;}
const double Vehicle::s() { return this->s_;}
const double Vehicle::d() { return this->d_;}
const double Vehicle::yaw() { return this->yaw_;}
const vector<double> Vehicle::prev_path_x() { return this->previous_path_x_;}
const vector<double> Vehicle::prev_path_y() { return this->previous_path_y_;}
const int Vehicle::prev_path_size() { return this->previous_path_x_.size();}
const int Vehicle::lane() { return this->lane_;}

const double Vehicle::speed() {  return this->speed_; }
