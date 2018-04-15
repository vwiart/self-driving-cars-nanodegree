#ifndef __PATH_PLANNER_H__
#define __PATH_PLANNER_H__

#include <vector>
#include <math.h>

#include "../json.hpp"
#include "spline.h"

#include "vehicle.h"
#include "sensor_fusion.h"
#include "../frenet/frenet.h"

using json = nlohmann::json;

using namespace std;

enum Behaviour {
    Accelerating,
    Steady,
    ChangingLane
};

struct Path {
    vector<double> xs;
    vector<double> ys;
};

class PathPlanner {
public:
    PathPlanner(Vehicle v, SensorFusion sf);
    void update_state(json telemetry,
                      const vector<double> &maps_s,
                      const vector<double> &maps_x,
                      const vector<double> &maps_y);
    Path drive();
private:
    Path generate_path();
    void steady(NearbyVehicles nearby);
    void accelerating(NearbyVehicles nearby);
    void change_lane(NearbyVehicles nearby);
    void switch_behaviour(Behaviour target_behaviour);
    Vehicle vehicle;
    SensorFusion sensor_fusion;
    Behaviour behaviour;
    vector<double> maps_s, maps_x, maps_y;
    double dist_inc, end_path_s, end_path_d, max_speed, acceleration;
};

#endif
