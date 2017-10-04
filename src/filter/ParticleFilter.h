/*
 * particle_filter.h
 *
 * 2D particle filter class.
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_

//#define VERBOSE_OUT

#include <random>
#include "../utils/helper_functions.h"
#include "../map/Map.h"
#include "../map/Partition2D.h"

struct Particle {
	int id;
	double x;
	double y;
	double theta;
	double weight;

	std::vector<int> associations;
	std::vector<double> sense_x;
	std::vector<double> sense_y;
	
	/**
	 * Construct a new particle
	 * @param x the x coordinate
	 * @param y the y coordinate
	 * @param theta the yaw
	 * @param weight the weight
	 */  
	Particle(int id, double x, double y, double theta, double weight = 1) {
		this->id = id;
		this->x = x;
		this->y = y;
		this->theta = theta;
		this->weight = weight;
	}
};

class ParticleFilter {
	// Random number generator
	static std::default_random_engine generator;

	// Number of particles to draw
	int num_particles; 
	
	// Flag, if filter is initialized
	bool is_initialized;
	
	// Vector of weights of all particles
	std::vector<double> weights;

	// Random distributions
  std::normal_distribution<double> distribution_x;
  std::normal_distribution<double> distribution_y;
  std::normal_distribution<double> distribution_theta;
	
public:
	
	// Set of current particles
	std::vector<Particle> particles;

	// Constructor
	// @param nParticles Number of particles
	ParticleFilter(int nParticles) : num_particles(nParticles), is_initialized(false) {}

	// Destructor
	~ParticleFilter() {}

	/**
	 * init Initializes particle filter by initializing particles to Gaussian
	 *   distribution around first position and all the weights to 1.
	 * @param x Initial x position [m] (simulated estimate from GPS)
	 * @param y Initial y position [m]
	 * @param theta Initial orientation [rad]
	 * @param std[] GPS measurement uncertainty, it is an array of dimension 3 [standard deviation of x [m],
	 *   standard deviation of y [m], and standard deviation of yaw [rad]]
	 */
	void init(double x, double y, double theta, double std[]);

	/**
	 * prediction Predicts the state for the next time step
	 *   using the process model.
	 * @param delta_t Time between time step t and t+1 in measurements [s]
	 * @param velocity Velocity of car from t to t+1 [m/s]
	 * @param yaw_rate Yaw rate of car from t to t+1 [rad/s]
	 */
	void prediction(double delta_t, double velocity, double yaw_rate);
	
	/**
	 * updateWeights Updates the weights for each particle based on the likelihood of the 
	 *   observed measurements. 
	 * @param sensor_range Range [m] of sensor
	 * @param std_landmark[] Array of dimension 2 [Landmark measurement uncertainty [x [m], y [m]]]
	 * @param observations Vector of landmark observations
	 * @param partition Partition2D class containing a space partition for the landmarks
	 */
	void updateWeights(double sensor_range, double std_landmark[], const std::vector<LandmarkObs> &observations,
			const Partition2D<Map::single_landmark_s> &partition);
	
	/**
	 * resample Resamples from the updated set of particles to form
	 *   the new set of particles.
	 */
	void resample();

	/*
	 * Set a particles list of associations, along with the associations calculated world x,y coordinates
	 * This can be a very useful debugging tool to make sure transformations are correct and assocations correctly connected
	 */
	Particle SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y);
	
	std::string getAssociations(Particle best);
	std::string getSenseX(Particle best);
	std::string getSenseY(Particle best);

	/**
	 * initialized Returns whether particle filter is initialized yet or not.
	 */
	const bool initialized() const {
		return is_initialized;
	}
};



#endif /* PARTICLE_FILTER_H_ */
