#ifndef __PARTICLE_FILTER_H__
#define __PARTICLE_FILTER_H__

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "helper_functions.h"

using namespace std;

struct Particle {
	int id;
	double x;
	double y;
	double theta;
	double weight;
	std::vector<int> associations;
	std::vector<double> sense_x;
	std::vector<double> sense_y;
};

struct Measurement {
	double x, y, theta, prev_velocity, prev_yawrate, dt;
	double* sigma_pos;
};

class ParticleFilter {
public:
	ParticleFilter(Map map, double sensor_range, double sigma_landmark[]);
	std::vector<Particle> particles;
	void process(Measurement m, vector<LandmarkObs> noisy_observations);
	Particle SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y);
	std::string getAssociations(Particle best);
	std::string getSenseX(Particle best);
	std::string getSenseY(Particle best);
private:
	void init(double x, double y, double theta, double std[]);
	void prediction(double delta_t, double std_pos[], double velocity, double yaw_rate);
	void dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations);
	void updateWeights(const std::vector<LandmarkObs> &observations);
	void resample();
	Map map;
	int num_particles; 
	bool initialized;
	std::vector<double> weights;
	double initial_weight, sensor_range;
	double* std_landmark;
};



#endif
