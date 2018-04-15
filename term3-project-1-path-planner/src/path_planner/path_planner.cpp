#include "path_planner.h"

PathPlanner::PathPlanner(Vehicle v, SensorFusion sf) {
    this->dist_inc = 0.5;
    this->vehicle = v;
    this->sensor_fusion = sf;
    this->behaviour = Steady;
    this->max_speed = 22; //kph
    this->acceleration = 0.1; //kph
}

// update_state uses telemetry and maps to update the internal state.
void PathPlanner::update_state(json telemetry,
    const vector<double> &maps_s,
    const vector<double> &maps_x,
    const vector<double> &maps_y) {

    vehicle.update_state(telemetry);
    sensor_fusion.update_state(telemetry);

    this->end_path_s = telemetry[1]["end_path_s"];
    this->end_path_d = telemetry[1]["end_path_d"];

    this->maps_s = maps_s;
    this->maps_x = maps_x;
    this->maps_y = maps_y;
}

// generate_path uses a spline to generate the trajectory of the vehicle.
Path PathPlanner::generate_path() {
    vector<double> ptsx, ptsy;
    
    double ref_x = vehicle.x();
    double ref_y = vehicle.y();
    double ref_s = vehicle.s();
    double ref_yaw = vehicle.yaw();

    double prev_car_x, prev_car_y;

    const int prev_size = vehicle.prev_path_size();
    if (prev_size < 2) {
        prev_car_x = ref_x - cos(vehicle.yaw());
        prev_car_y = ref_y - sin(vehicle.yaw());        
    } else {
        ref_x = vehicle.prev_path_x()[prev_size - 1];
        ref_y = vehicle.prev_path_y()[prev_size - 1];
        
        prev_car_x = vehicle.prev_path_x()[prev_size - 2];
        prev_car_y = vehicle.prev_path_y()[prev_size - 2];

        ref_yaw = atan2(ref_y - prev_car_y, ref_x - prev_car_x);
    }

    ptsx.push_back(prev_car_x);
    ptsx.push_back(ref_x);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(ref_y);

    Frenet frenet(maps_s, maps_x, maps_y);
    int interval[3] = {30, 45, 60};
    for (int i = 0; i < 3; i++) {
        double s_ = ref_s + interval[i];
        double d_ = 2 + 4 * vehicle.lane();
        frenet.to_cartesian(s_, d_);

        ptsx.push_back(frenet.x());
        ptsy.push_back(frenet.y());
    }

    for (int i = 0; i < ptsx.size(); i++) {
        double shift_x = ptsx[i]  - ref_x;
        double shift_y = ptsy[i]  - ref_y;

        ptsx[i] = shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw);
        ptsy[i] = shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw);
    }

    tk::spline s;
    s.set_points(ptsx, ptsy);

    Path path;
    for (int i = 0; i < prev_size; i++) {
        path.xs.push_back(vehicle.prev_path_x()[i]);
        path.ys.push_back(vehicle.prev_path_y()[i]);
    }

    const double target_x = 30.0;
    const double target_y = s(target_x);
    const double target_dist = sqrt(pow(target_x, 2) + pow(target_y, 2));
    double x_add_on = 0;

    for (int i = 0; i < 50 - prev_size; i++) {
        const double n = target_dist / (0.02 * vehicle.speed());
        double x_point = x_add_on + target_x / n;
        double y_point = s(x_point);

        x_add_on = x_point;
        double x_ref = x_point;
        double y_ref = y_point;

        x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
        y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

        x_point += ref_x;
        y_point += ref_y;

        path.xs.push_back(x_point);
        path.ys.push_back(y_point);
    }

    return path;
}

// steady handles the `Steady` state.
void PathPlanner::steady(NearbyVehicles nearby) {
    double speed = vehicle.speed();

    if (speed == 0) {
        switch_behaviour(Accelerating);
        vehicle.set_speed(acceleration);
        return;
    }

    //there's someone in front of us, we might want to avoid an accident !
    if (nearby.same_lane) {
        switch_behaviour(ChangingLane);
        return;
    }
}

// accelerating handles the `Accelerating` state.
void PathPlanner::accelerating(NearbyVehicles nearby) {
    //there's someone in front of us, we might want to avoid an accident !
    if (nearby.same_lane) {
        switch_behaviour(ChangingLane);
        return;
    }

    double speed = vehicle.speed();
    speed += acceleration;

    //If we reach the max speed, stop accelerating.
    if (speed > max_speed) {
        switch_behaviour(Steady);
        vehicle.set_speed(max_speed);
        return;
    }
    vehicle.set_speed(speed);
}

// change_lane handles the `ChangingLane` state.
void PathPlanner::change_lane(NearbyVehicles nearby) {
    const int lane = vehicle.lane();

    // nobody to our left ? take over !
    if (lane > 0 && nearby.left_lane == 0) {
        vehicle.set_lane(lane - 1);
        switch_behaviour(Accelerating);
        return;
    }

    // nobody to our right ? take over !
    if (lane < 2 && nearby.right_lane == 0) {
        vehicle.set_lane(lane + 1);
        switch_behaviour(Accelerating);
        return;
    }

    // nobody in front of us, let's go go go !
    if (nearby.same_lane == 0) {
        double speed = vehicle.speed();
        speed += acceleration;
        if (speed <= max_speed) {
            vehicle.set_speed(speed);
            return;
        }
        return;
    }

    double speed = vehicle.speed();
    speed -= acceleration;
    vehicle.set_speed(speed);
}

// switch_behaviour is a helper function to log behaviour state change.
void PathPlanner::switch_behaviour(Behaviour target_behaviour) {
    behaviour = target_behaviour;
    switch(target_behaviour) {
        case Steady:
            cout << "Behaviour : Steady" << endl;
            break;
        case ChangingLane:
            cout << "Behaviour : ChangingLane" << endl;
            break;
        case Accelerating:
            cout << "Behaviour : Accelerating" << endl;
            break;
    }
}

// drive drives the car.
Path PathPlanner::drive() {
    NearbyVehicles nearby = sensor_fusion.get_nearby_vehicles(vehicle);
    switch(behaviour) {
        case Steady:
            steady(nearby);
            break;
        case Accelerating:
            accelerating(nearby);
            break;
        case ChangingLane:
            change_lane(nearby);
            break;
    }

    return generate_path();
}
