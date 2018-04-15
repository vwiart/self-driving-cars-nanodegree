#include "sensor_fusion.h"

SensorFusion::SensorFusion(){
    this->safe_dist_ = 30;
}

// update_state updates the internal state.
void SensorFusion::update_state(json telemetry) {
    vector<vector<double>> sensor_fusion = telemetry[1]["sensor_fusion"];
    this->sensor_fusion_ = sensor_fusion;
}

// is_in_lane returns true if the given car is in the given_lane.
bool SensorFusion::is_in_lane(int car_id, int lane) {
    float d = sensor_fusion_[car_id][6];
    return d < 4+4*lane && d > 4 * lane;
}

// get_nearby_vehicles shuffles the nearby vehicles in their corresponding lanes.
NearbyVehicles SensorFusion::get_nearby_vehicles(Vehicle vehicle) {
    const int lane = vehicle.lane();
    NearbyVehicles nearby;
    nearby.left_lane = 0;
    nearby.right_lane = 0;
    nearby.same_lane = 0;
    for (int i = 0; i < sensor_fusion_.size(); i++) {
        double vx = sensor_fusion_[i][3];
        double vy = sensor_fusion_[i][4];
        double speed = sqrt(vx * vx + vy * vy);
        double s = sensor_fusion_[i][5] + ((double)vehicle.prev_path_size()) * 0.02 * speed;
        double d = sensor_fusion_[i][6];

        if (s < vehicle.s()) {
            continue;
        }

        if (s - vehicle.s() < safe_dist_) {
            switch (lane) {
            case 0:
                if (is_in_lane(i, 0)) {
                    nearby.same_lane++;
                } else  if (is_in_lane(i, 1)) {
                    nearby.right_lane++;
                }
                break;
            case 1:
                if (is_in_lane(i, 0)) {
                    nearby.left_lane++;
                } else  if (is_in_lane(i, 2)) {
                    nearby.right_lane++;
                } else {
                    nearby.same_lane++;
                }
                break;
            case 2:
                if (is_in_lane(i, 2)) {
                    nearby.same_lane++;
                } else  if (is_in_lane(i, 1)) {
                    nearby.left_lane++;
                }
                break;
            }
        }
    }
    return nearby;
}
