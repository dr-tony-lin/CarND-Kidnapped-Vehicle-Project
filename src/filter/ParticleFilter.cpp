/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <sstream>
#include <string>
#include <tuple>
#include "ParticleFilter.h"

using namespace std;

std::default_random_engine ParticleFilter::generator;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  // Initialize all particles to first position (based on estimates of
  //   x, y, theta and their uncertainties from GPS) and all weights to 1.
  // Add random Gaussian noise to each particle.
  // Initialize the normal distribution for x, y, and theta
  distribution_x = normal_distribution<double>(0, std[0]);
  distribution_y = normal_distribution<double>(0, std[1]);
  distribution_theta = normal_distribution<double>(0, std[2]);
  for (int i = 0; i < num_particles; i++) {
		Particle p(
								i,
								x + distribution_x(generator), 
								y + distribution_y(generator),
								theta + normalizeAngle(distribution_theta(generator)));
    particles.push_back(p);
  }
  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double velocity, double yaw_rate) {
  // Add prediction to each particle and add random Gaussian noise.
  for (int i = 0; i < num_particles; i++) {
    Particle& particle = particles[i];
    double new_yaw = particle.theta + yaw_rate * delta_t;
    // We need to habdle the situation when yaw rate is 0
    if (fabs(yaw_rate) > EPSILON) { // yaw rate is not 0
      particle.x += velocity / yaw_rate * (sin(new_yaw) - sin(particle.theta)) +
                    distribution_x(generator);
      particle.y += velocity / yaw_rate * (cos(particle.theta) - cos(new_yaw)) +
                    distribution_y(generator);
    }
    else { // yaw rate is 0
      particle.x += velocity * cos(particle.theta) + distribution_x(generator);
      particle.y += velocity * sin(particle.theta) + distribution_y(generator);
    }
    
    particle.theta = new_yaw + normalizeAngle(distribution_theta(generator));
  }
}

void ParticleFilter::updateWeights(
    double sensor_range, double std_landmark[],
    const std::vector<LandmarkObs>& observations,
    const Partition2D<Map::single_landmark_s>& partition) {
  // Update the weights of each particle using a mult-variate Gaussian
  weights.clear();
  for (int i = 0; i < num_particles; i++) {
		Particle& particle = particles[i];
		particle.associations.clear();
		particle.sense_x.clear();
		particle.sense_y.clear();
    double sin_theta = sin(particle.theta);
    double cos_theta = cos(particle.theta);
    double weight = 1.;
    for (auto it = observations.begin();
         it != observations.end(); it++) {
      const LandmarkObs& obs = *it;
#ifdef VERBOSE_OUT
      std::cout << "Search:" << particle.id << " (" << particle.x << "," << particle.y << "," << particle.theta << ")" 
                << "(" << obs.x << "," << obs.y << ")"<< std::endl;
#endif

      // Transform observation coordinate to map coordinate
      double x = particle.x + obs.x * cos_theta - obs.y * sin_theta;
      double y = particle.y + obs.x * sin_theta + obs.y * cos_theta;
      Map::single_landmark_s* nearest;
      double dist;
      int searched;
      searches++;
      std::tie(nearest, dist, searched) = partition.FindNearest(x, y);
      if (nearest) {  // we have found one
        this->searched += searched;
#ifdef VERBOSE_OUT
        std::cout << "Found " << nearest->id() << "(" << nearest->x() << "," << nearest->y() << "), distance: " 
                  << dist << ", searched: " << searched << std::endl;
#endif
        double dx = x - nearest->x();
        double dy = y - nearest->y();
        // Compute the probability according to the distance deviation between the nearest landmark and the
        // particle's "observation". However, when there is a bigger deviation, the probability may become
        // very low, and results in 0 weights for all particles. When this happen, the filter will not
        // be able to produce useful result. To avoid this problem, we flatten the distribution be an order
        // of magnitude - by dividing the exponent by 10. This is fine since weights are relative.
        double p = 0.5/(M_PI*std_landmark[0]*std_landmark[1] * 
									 exp((square(dx/std_landmark[0]) + square(dy/std_landmark[1]))/20));
				weight *= p;
				particle.associations.push_back(nearest->id());
				particle.sense_x.push_back(x);
				particle.sense_y.push_back(y);
      } else {
        weight *= 1E-10;
      }
    }
    particle.weight = weight;
    weights.push_back(weight);
  }
}

void ParticleFilter::resample() {
  // Resample particles with replacement with probability proportional to
  // their weight.
	std::discrete_distribution<> distribution(weights.begin(), weights.end());
	std::vector<Particle> samples;
	for (int i = 0; i < num_particles; i++) {
		int rand = distribution(generator);
		samples.push_back(particles[rand]);
	}
	particles.clear();
	particles = samples;
}

Particle ParticleFilter::SetAssociations(Particle particle,
                                         std::vector<int> associations,
                                         std::vector<double> sense_x,
                                         std::vector<double> sense_y) {
  // particle: the particle to assign each listed association, and association's
  // (x,y) world coordinates mapping to
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates

  // Clear the previous associations
  particle.associations.clear();
  particle.sense_x.clear();
  particle.sense_y.clear();

  particle.associations = associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;

  return particle;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  stringstream ss;
  copy(v.begin(), v.end(), ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseX(Particle best) {
  vector<double> v = best.sense_x;
  stringstream ss;
  copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseY(Particle best) {
  vector<double> v = best.sense_y;
  stringstream ss;
  copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1);  // get rid of the trailing space
  return s;
}
