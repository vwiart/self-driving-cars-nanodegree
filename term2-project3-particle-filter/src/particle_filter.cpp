#include "particle_filter.h"

using namespace std;

ParticleFilter::ParticleFilter(Map _map, double sensor_range_, double sigma_landmark[]) {
	initialized = false;
	num_particles = 150;
	initial_weight = 1.0;
	map = _map;
	std_landmark = sigma_landmark;
	sensor_range = sensor_range_;
}

void ParticleFilter::process(Measurement m, vector<LandmarkObs> observations) {
	if (!initialized) {
		init(m.x, m.y, m.theta, m.sigma_pos);
		return;
	}
	
	prediction(m.dt, m.sigma_pos, m.prev_velocity, m.prev_yawrate);
	updateWeights(observations);
	resample();
}

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	default_random_engine gen;
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);

	for (int i = 0; i < num_particles; i++) {
		Particle particle;
		particle.id = i;
		particle.x = dist_x(gen);
		particle.y = dist_y(gen);
		particle.theta = dist_theta(gen);
		particle.weight = initial_weight;

		weights.push_back(initial_weight);
		particles.push_back(particle);
	}
	initialized = true;
}

void ParticleFilter::prediction(double dt, double std_pos[], double velocity, double yaw_rate) {
	default_random_engine gen;
	normal_distribution<double> dist_x(0.0, std_pos[0]);
	normal_distribution<double> dist_y(0.0, std_pos[1]);
	normal_distribution<double> dist_theta(0.0, std_pos[2]);

	for (int i = 0; i < num_particles; i++) {
		double theta = particles[i].theta;
		double noise_x = dist_x(gen);
		double noise_y = dist_y(gen);
		double noise_theta = dist_theta(gen);

		if (abs(yaw_rate) == 0) {
			particles[i].x += velocity * dt * cos(theta) + noise_x;
			particles[i].y += velocity * dt * sin(theta) + noise_y;
			particles[i].theta += noise_theta;
		} else {
			double phi_theta = theta + dt * yaw_rate;
			particles[i].x += velocity / yaw_rate * (sin(phi_theta) - sin(theta)) + noise_x;
			particles[i].y += velocity / yaw_rate * (cos(theta) - cos(phi_theta)) + noise_y;
			particles[i].theta = phi_theta + noise_theta;
		}
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted,
									 std::vector<LandmarkObs>& observations) {
	auto _dist = [](LandmarkObs pred, LandmarkObs obs) -> double {
		return pow(pred.x - obs.x, 2) + pow(pred.y - obs.y, 2);
	};
	for (int i = 0; i < observations.size(); i++) {
		int closest = 0;
		double min_distance = numeric_limits<double>::max();

		for (int j = 0; j < predicted.size(); j++) {
			double distance = _dist(predicted[j], observations[i]);
	
			if (distance < min_distance) {
      			closest = j;
      			min_distance = distance;
			}
    	}
    	observations[i].id = closest;
	}
}

void ParticleFilter::updateWeights(const std::vector<LandmarkObs> &observations) {
	for (int i = 0; i < num_particles; i++) {
		double px = particles[i].x;
		double py = particles[i].y;
		double p_theta = particles[i].theta;

		vector<LandmarkObs> landmarks;
		vector<LandmarkObs> transformed_landmarks;

		for (int j = 0; j < observations.size(); j++) {
			int obs_id = observations[j].id;
			double obs_x = observations[j].x;
			double obs_y = observations[j].y;

			LandmarkObs tl;
			tl.id = obs_id;
			tl.x = px + obs_x * cos(p_theta) - obs_y * sin(p_theta);
			tl.y = py + obs_y * cos(p_theta) + obs_x * sin(p_theta);

			transformed_landmarks.push_back(tl);
		}
    
		for (int j = 0; j < map.landmark_list.size(); j++) {
			int landmark_id = map.landmark_list[j].id_i;
			double landmark_x = map.landmark_list[j].x_f;
			double landmark_y = map.landmark_list[j].y_f;

			double distance = sqrt(pow(landmark_x - px, 2) + pow(landmark_y - py, 2));

			if (distance < sensor_range) {
				LandmarkObs lm;
				lm.id = landmark_id;
				lm.x = landmark_x;
				lm.y = landmark_y;

				landmarks.push_back(lm);
			}
		}

		dataAssociation(landmarks, transformed_landmarks);

		double x = std_landmark[0];
		double y = std_landmark[1];

    	double weight = initial_weight;
		for (int j = 0; j < transformed_landmarks.size(); j++) {
			int obs_id = transformed_landmarks[j].id;

			double a = pow(transformed_landmarks[j].x - landmarks[obs_id].x, 2) / (2 * pow(x, 2));
			double b = pow(transformed_landmarks[j].y - landmarks[obs_id].y, 2) / (2 * pow(y, 2));

			weight *= exp(-(a + b)) / sqrt(2.0 * M_PI * x * y);
		}

		double norm_weight = min(weight, 0.001);
		particles[i].weight = norm_weight;
		weights[i] = norm_weight;
  	}
}

void ParticleFilter::resample() {
	vector<Particle> sampled_particles;
	default_random_engine gen;
	discrete_distribution<int> index(weights.begin(), weights.end());

	for (int i = 0; i < num_particles; i++) {
		const int idx = index(gen);
		Particle particle;
		particle.id = idx;
		particle.x = particles[idx].x;
		particle.y = particles[idx].y;
		particle.theta = particles[idx].theta;
		particle.weight = initial_weight;

		sampled_particles.push_back(particle);
	}
	particles = sampled_particles;
}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y) {
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
}

string ParticleFilter::getAssociations(Particle best) {
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);
    return s;
}

string ParticleFilter::getSenseX(Particle best) {
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);
    return s;
}

string ParticleFilter::getSenseY(Particle best) {
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);
    return s;
}
