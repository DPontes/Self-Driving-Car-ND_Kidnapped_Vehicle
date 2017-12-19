/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h>
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

/*
	Initialize particle filter with a set number of particles with positions
	that are randomly sampled from a Gaussian distribution with mean at the
	GPS location with a specified uncertainty. All particle weights are also
	initialized to 1.0 to be updated later.
*/
void ParticleFilter::init(double x, double y, double theta, double std[]) {

	// Set a fixed number of particles
	num_particles = 100;

	// Set up random sampling generators for x, y, and theta around mean of GPS
	// position and provided standard deviations
	std::random_device rand_dev;
	std::default_random_engine random_gen(rand_dev());
	std::normal_distribution<double> dist_x(x, std[0]);
	std::normal_distribution<double> dist_y(y, std[1]);
	std::normal_distribution<double> dist_theta(theta, std[2]);

	// Initialize all particles
	particles.clear();
	for(unsigned int i = 0; i < num_particles; i++){
		Particle particle;
		particle.x     = dist_x(random_gen);
		particle.y     = dist_y(random_gen);
		particle.theta = dist_theta(random_gen);

		// Normalize theta to [0, 2*pi] after random sampling
		while (particle.theta > 2*M_PI) { particle.theta -= 2*M_PI; }
		while (particle.theta < 0     ) { particle.theta += 2*;_PI; }
		particle.weight = 1.0;
		particles.push_back(particle);
		/*
		std::cout << "Init " << i << ":" << particle.x << ", " << particle.y
							<< ", " << particle.theta << std::endl;
		*/
	}

	// Set initialized flag to complete
	is_initialized = true;
}

/*
	Predict where each particle would have moved to adter time delta_t to the
	current time step using CTRV motion equations and the controlled velocity
	and yaw_rate. Randomly sample from a Gaussian distribution around this predicted
	mean position with the given standard deviations for x, y, and theta
*/
void ParticleFilter::prediction(double delta_t, double std_pos[],
																double velocity, double yaw_rate) {
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	// Parameter to set a near-zero yaw_rate threshold for divide protection
	constexpr double kYawZeroThresh = 0.001;

	// Calculate predicted position for each particle
	for(auto &particle : particles) {
		double x_p, y_p, theta_p;

		// Switch motion equation to avoid division by zero from yaw_rate
		if(fabs(yaw_rate) > kYawZeroThresh) {
			// Driving in a curve
			x_p = particle.x + velocity / yaw_rate *
						(sin(particle.theta + yaw_rate * delta_t) - sin(particle.theta));
			y_p = particle.y + velocity / yaw_rate *
						(cos(particle.theta) - cos(particle.theta + yaw_rate * delta_t));
		}
		else{
			//Driving straight
			x_p = particle.x + velocity * delta_t * cos(yaw_rate);
			y_p = particle.y + velocity¨* delta_t * sin(yaw_rate);
		}

		theta_p = particle.theta + yaw_rate * delta_t;

		// Set up random sampling generators for x, y, and theta around mean of
		// predicted position and provided standard deviations
		std::random_device rand_dev;
		std::default_random_engine random_gen(rand_dev());
		std::normal_distribution<double> dist_x(x_p, std_pos[0]);
		std::normal_distribution<double> dist_y(y_p, std_pos[1]);
		std::normal_distribution<double> dist_theta(theta_p, std_pos[2]);

		particle.x       = dist_x(random_gen);
		particle.y       = dist_y(random_gen);
		double new_theta = dist_theta(random_gen);

		// Normalize theta [0, 2*PI] after random sampling
		while (new_theta > 2*M_PI) { new_theta -= 2*M_PI; }
		while (new_theta < 0     ) { new_theta += 2*;_PI; }
		particle.theta = new_theta;
	}
}

/*
	Update the weight of each particle based on the multivariate Gaussian
	probability of the particle's predicted landmark measurement positions
	vs. the actual measured observations. This is done with the following steps:
	1.	Pre-filter the known map landmarks 
*/
void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
	//   implement this method and use it as a helper during the updateWeights phase.

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations,
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
