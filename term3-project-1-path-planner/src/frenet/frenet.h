#ifndef __FRENET_H__
#define __FRENET_H__

#include <vector>
#include <math.h>
#include <iostream>

using namespace std;

class Frenet {
public:
    Frenet(const vector<double> &maps_s,
	    const vector<double> &maps_x,
        const vector<double> &maps_y);
    void from_cartesian(double x, double y, double theta);
    void to_cartesian(double s, double d);
    const double s() {return s_;}
    const double d() {return d_;}
    const double x() {return x_;}
    const double y() {return y_;}
private:    
    int next_waypoint(double x, double y, double theta);
    int closest_waypoint(double x, double y);
    double distance(double x1, double y1, double x2, double y2);
    vector<double> map_s, map_x, map_y;
    double s_, d_, x_, y_;
};

#endif