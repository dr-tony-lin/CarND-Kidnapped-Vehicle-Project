#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <tuple>
#include "json.hpp"
#include "filter/ParticleFilter.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("]");
  if (found_null != std::string::npos) {
    return "";
  } else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main(int argc, char* argv[]) {
  uWS::Hub h;
  Partition2D<Map::single_landmark_s> partition;

  // Set up parameters here
  double delta_t = 0.1;      // Time elapsed between measurements [sec]
  double sensor_range = 50;  // Sensor range [m]

  double sigma_pos[3] = {
      0.3, 0.3,
      0.01};  // GPS measurement uncertainty [x [m], y [m], theta [rad]]
  
  double sigma_landmark[2] = {
      0.3, 0.3};  // Landmark measurement uncertainty [x [m], y [m]]

  int nParticles = 1000;
  
  // Process command line options
  for (int i = 1; i < argc; i++) {
    if (std::string((argv[i])) == "-parts") { // Set the number of particles
      if (sscanf(argv[++i], "%d", &nParticles) != 1) {
        std::cerr << "Invalid number of particles: " << argv[i] << std::endl;
        exit(-1);
      }
      if (nParticles <= 0) {
        std::cerr << "Invalid number of particles: " << argv[i] << std::endl;
        exit(-1);
      }
    } else if (std::string((argv[i])) == "-stdgps") { // set std GPS deviation
      if (sscanf(argv[++i], "%lf", &sigma_pos[0]) != 1) {
        std::cerr << "Invalid GPS standard deviation x: " << argv[i] << std::endl;
        exit(-1);
      }
      if (sscanf(argv[++i], "%lf", &sigma_pos[1]) != 1) {
        std::cerr << "Invalid GPS standard deviation y: " << argv[i] << std::endl;
        exit(-1);
      }
      if (sscanf(argv[++i], "%lf", &sigma_pos[2]) != 1) {
        std::cerr << "Invalid GPS standard deviation yaw: " << argv[i] << std::endl;
        exit(-1);
      }
      // Make sure std yaw deviation is between 0 and 2PI
      if (sigma_pos[2] > M_PI * 2) {
        sigma_pos[2] = M_PI * 2;
      } else if (sigma_pos[2] < 0) {
        sigma_pos[2] = 0;
      }
    } else if (std::string((argv[i])) == "-stdland") { // set standard landmark measurement deviation
      if (sscanf(argv[++i], "%lf", &sigma_landmark[0]) != 1) {
        std::cerr << "Invalid landmark standard deviation x: " << argv[i] << std::endl;
        exit(-1);
      }
      if (sscanf(argv[++i], "%lf", &sigma_landmark[1]) != 1) {
        std::cerr << "Invalid landmark standard deviation y: " << argv[i] << std::endl;
        exit(-1);
      }
    } else {
      std::cerr << "Unknown option: " << argv[i] << std::endl;
      exit(-1);
    }
  }

  // Read map data
  Map map;
  if (!read_map_data("../data/map_data.txt", map)) {
    cout << "Error: Could not open map file" << endl;
    return -1;
  }

  // Bounding rectangle of the world
  float x0 = 1E20, y0 = 1E20, x1 = -1E20, y1 = -1E20;
  for (auto it = map.landmark_list.begin(); it != map.landmark_list.end(); it++) {
    x0 = min(x0, it->x());
    x1 = max(x1, it->x());
    y0 = min(y0, it->y());
    y1 = max(y1, it->y());
  }

  cout << "World: " << x0 << ", " << y0 << ", " << x1 << ", " << y1 << endl;
  cout << "Landmarks: " << map.landmark_list.size() << endl;

  // Initialize the space partition
  partition.Initialize(x0-1, y0-1, x1+1, y1+1, 5, 50);
  // Partition the map
  partition.AddPointObjects(map.landmark_list);

#ifdef TEST_PARTITION
  // Test the 2D space partition algorithm
  for (auto it = map.landmark_list.begin(); it != map.landmark_list.end(); it++) {
    Map::single_landmark_s* nearest;
    double dist;
    int searched;

    // Find each landmarks from the landmarks location
    std::tie(nearest, dist, searched) = partition.FindNearest(it->x(), it->y());
    if (nearest) {
      if (nearest->id() != it->id()) {
        cout << "Error finding landmark, is " << it->id() << "(" << it->x() << "," << it->y() << ") but got: "
             << nearest->id() << "(" << nearest->x() << "," << nearest->y() << ")" << endl;
      }
      else {
        cout << "Found with searches: " << searched << endl;
      }
    }
    else {
      cout << "Error! landmark not found: " << it->id() << "(" << it->x() << "," << it->y() << ")" << endl;
    }

    // Find each landmarks from the landmarks's nearby location.
    std::tie(nearest, dist, searched) = partition.FindNearest(it->x() + 20, it->y() + 10);
    if (nearest) {
      if (nearest->id() != it->id()) {
        cout << "Finding landmark at " << "(" << (it->x() + 20) << "," << (it->y() + 10) << ") got: "
              << nearest->id() << "(" << nearest->x() << "," << nearest->y() << "), searches: " << searched << endl;
      }
    }
    else {
      cout << "Found no landmark!" << endl;
    }
  }
#endif

  // Create particle filter
  ParticleFilter pf(nParticles);

  h.onMessage([&pf, &partition, &delta_t, &sensor_range, &sigma_pos, &sigma_landmark](
      uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
      uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(std::string(data));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          if (!pf.initialized()) {
            // Sense noisy position data from the simulator
            double sense_x = std::stod(j[1]["sense_x"].get<std::string>());
            double sense_y = std::stod(j[1]["sense_y"].get<std::string>());
            double sense_theta =
                std::stod(j[1]["sense_theta"].get<std::string>());

            pf.init(sense_x, sense_y, sense_theta, sigma_pos);
          } else {
            // Predict the vehicle's next state from previous (noiseless
            // control) data.
            double previous_velocity =
                std::stod(j[1]["previous_velocity"].get<std::string>());
            double previous_yawrate =
                std::stod(j[1]["previous_yawrate"].get<std::string>());

            pf.prediction(delta_t, previous_velocity, previous_yawrate);
          }

          // receive noisy observation data from the simulator
          // sense_observations in JSON format
          // [{obs_x,obs_y},{obs_x,obs_y},...{obs_x,obs_y}]
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

          for (int i = 0; i < x_sense.size(); i++) {
            LandmarkObs obs;
            obs.x = x_sense[i];
            obs.y = y_sense[i];
            noisy_observations.push_back(obs);
          }

          // Update the weights and resample
          pf.updateWeights(sensor_range, sigma_landmark, noisy_observations,
                           partition);
          pf.resample();

          // Calculate and output the average weighted error of the particle
          // filter over all time steps so far.
          vector<Particle> particles = pf.particles;
          int num_particles = particles.size();
          double highest_weight = -1.0;
          Particle* best_particle;
          double weight_sum = 0.0;
          for (int i = 0; i < num_particles; ++i) {
            if (particles[i].weight > highest_weight) {
              highest_weight = particles[i].weight;
              best_particle = &particles[i];
            }
            weight_sum += particles[i].weight;
          }
          cout << "highest w " << highest_weight << endl;
          cout << "average w " << weight_sum / num_particles << endl;
          cout << "average object searched " << pf.averageSearch() << endl;

          json msgJson;
          msgJson["best_particle_x"] = best_particle->x;
          msgJson["best_particle_y"] = best_particle->y;
          msgJson["best_particle_theta"] = best_particle->theta;

          // Optional message data used for debugging particle's sensing and
          // associations
          msgJson["best_particle_associations"] =
              pf.getAssociations(*best_particle);
          msgJson["best_particle_sense_x"] = pf.getSenseX(*best_particle);
          msgJson["best_particle_sense_y"] = pf.getSenseY(*best_particle);

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

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    //ws.close(); // Will crash on Windows if try to close
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
