#ifndef __SENSOR_FUSION_H__
#define __SENSOR_FUSION_H__

#include <vector>
#include "vehicle.h"
#include "../json.hpp"

using json = nlohmann::json;
using namespace std;

struct NearbyVehicles {
    int same_lane, left_lane, right_lane;
};

class SensorFusion {
public:
    SensorFusion();
    void update_state(json telemetry);
    NearbyVehicles get_nearby_vehicles(Vehicle vehicle);
private:
    bool is_in_lane(int car_id, int lane);
    vector<vector<double>> sensor_fusion_;
    int safe_dist_;
};

#endif
