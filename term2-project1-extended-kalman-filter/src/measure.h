#ifndef __MEASURE_H__
#define __MEASURE_H__

#include <iostream>
#include <cstdio>
#include "Eigen/Dense"

using namespace std;
using namespace Eigen;

// SensorType represents the type of a sensor : LIDAR or RADAR
enum SensorType {
    LIDAR, RADAR
};

// MeasureData represents the data measured by the sensors.
struct MeasureData {
    float rho, phi, rhodot; // data coming from the radar
    float x, y, vx, vy; // data coming from the lidar
    long long timestamp; // timestamp of the measurement
    SensorType type; // type of sensor
};

// State stores the inner state of the Kalman Filter
struct State {
    bool initialized;
    long long timestamp;
    VectorXd x;
    MatrixXd P;
};

// Measure handles measurement
class Measure {
public:
    Measure();
    ~Measure();
    void process(const MeasureData);
    VectorXd getStateVector();
private:
    void init(const MeasureData);
    void predict(const MeasureData);
    void update(const VectorXd, const MatrixXd, const MatrixXd,
        VectorXd (*y)(const VectorXd, const VectorXd, const MatrixXd));
    void updateRadar(const VectorXd z);
    void updateLidar(const VectorXd z);
    State state;
};

// jacobian computes the Jacobian matrix of a vector
MatrixXd jacobian(const VectorXd x);

#endif