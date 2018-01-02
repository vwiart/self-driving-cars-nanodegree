#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include <math.h>
#include "particle_filter.h"

using namespace std;

using json = nlohmann::json;

std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

std::string getMapFile() {
   char* val = getenv("map_file");
   if (!val) {
     return "../data/map_data.txt";
   }
   return val;
}

int main() {
  uWS::Hub h;

  double delta_t = 0.1; // Time elapsed between measurements [sec]
  double sensor_range = 50; // Sensor range [m]

  double sigma_pos [3] = {0.3, 0.3, 0.01}; // GPS measurement uncertainty [x [m], y [m], theta [rad]]
  double sigma_landmark [2] = {0.3, 0.3}; // Landmark measurement uncertainty [x [m], y [m]]

  // Read map data
  Map map;

  auto map_file = getMapFile();
  if (!read_map_data(map_file, map)) {
	  cout << "Error: Could not open map file" << endl;
	  return -1;
  }

  // Create particle filter
  ParticleFilter pf(map, sensor_range, sigma_landmark);

  h.onMessage([&pf,&map,&delta_t,&sensor_range,&sigma_pos,&sigma_landmark](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(std::string(data));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();

        if (event == "telemetry") {
          // receive noisy observation data from the simulator
          // sense_observations in JSON format [{obs_x,obs_y},{obs_x,obs_y},...{obs_x,obs_y}]
          vector<LandmarkObs> noisy_observations;
          string sense_observations_x = j[1]["sense_observations_x"];
          string sense_observations_y = j[1]["sense_observations_y"];

          std::vector<float> x_sense;
          std::istringstream iss_x(sense_observations_x);

          std::copy(std::istream_iterator<float>(iss_x),
        	          std::istream_iterator<float>(),
        	          std::back_inserter(x_sense));

          std::vector<float> y_sense;
          std::istringstream iss_y(sense_observations_y);

          std::copy(std::istream_iterator<float>(iss_y),
                    std::istream_iterator<float>(),
                    std::back_inserter(y_sense));

        	for(int i = 0; i < x_sense.size(); i++) {
            LandmarkObs obs;
        		obs.x = x_sense[i];
				    obs.y = y_sense[i];
            noisy_observations.push_back(obs);
        	}

          Measurement m;
          m.x = std::stod(j[1]["sense_x"].get<std::string>());
          m.y = std::stod(j[1]["sense_y"].get<std::string>());
          m.theta = std::stod(j[1]["sense_theta"].get<std::string>());
          m.prev_velocity = std::stod(j[1]["previous_velocity"].get<std::string>());
          m.prev_yawrate = std::stod(j[1]["previous_yawrate"].get<std::string>());
          m.dt = delta_t;
          m.sigma_pos = sigma_pos;

          pf.process(m, noisy_observations);

		      // Calculate and output the average weighted error of the particle filter over all time steps so far.
		      vector<Particle> particles = pf.particles;
		      int num_particles = particles.size();
		      double highest_weight = -1.0;
		      Particle best_particle;
		      double weight_sum = 0.0;
		      for (int i = 0; i < num_particles; ++i) {
			      if (particles[i].weight > highest_weight) {
			      	highest_weight = particles[i].weight;
			      	best_particle = particles[i];
			      }
			      weight_sum += particles[i].weight;
		      }
		    
          json msgJson;
          msgJson["best_particle_x"] = best_particle.x;
          msgJson["best_particle_y"] = best_particle.y;
          msgJson["best_particle_theta"] = best_particle.theta;

          //Optional message data used for debugging particle's sensing and associations
          msgJson["best_particle_associations"] = pf.getAssociations(best_particle);
          msgJson["best_particle_sense_x"] = pf.getSenseX(best_particle);
          msgJson["best_particle_sense_y"] = pf.getSenseY(best_particle);

          auto msg = "42[\"best_particle\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
