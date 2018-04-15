#ifndef __MAIN_H__
#define __MAIN_H__

#include <stdlib.h>
#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

#include "path_planner/path_planner.h"
#include "path_planner/vehicle.h"
#include "path_planner/sensor_fusion.h"

using namespace std;
using json = nlohmann::json;

#endif
