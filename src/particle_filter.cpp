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

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

	// Set num_particles
	num_particles = 1000;

	// Initialize weights to 1
	weights.resize(num_particles);
	for (double& w: weights)
		w = 1.0;

	// standard deviations of x [m], y [m], and yaw [rad]]
	double std_x = std[0];
	double std_y = std[1];
	double std_theta = std[2];
	// Create normal distributions around x, y, and theta
	normal_distribution<double> dist_x(x, std_x);
	normal_distribution<double> dist_y(y, std_y);
	normal_distribution<double> dist_theta(theta, std_theta);

	// Initialize partilces
	default_random_engine gen;
	particles.resize(num_particles);
	for (Particle& p: particles) {
		// add random noise to x, y, and theta by sampling their respective normal distrubtions
		p.x = dist_x(gen);
	    p.y = dist_y(gen);
	    p.theta = dist_theta(gen);
	   	// while (p.theta < -M_PI) p.theta += 2*M_PI;
	    // while (p.theta > M_PI) p.theta -= 2*M_PI;
	    // Initialize particle weights to 1.0
	    p.weight = 1.0;
	}
	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	// standard deviations of x [m], y [m], and yaw [rad]]
	double std_x = std_pos[0];
	double std_y = std_pos[1];
	double std_theta = std_pos[2];
	default_random_engine gen;

	for (Particle& p: particles) {
		// calculate predicted x, y, and theta based on contol inputs
		double x, y, theta;
		if (yaw_rate == 0) {
			x = p.x + velocity * delta_t * cos(p.theta);
			y = p.y + velocity * delta_t * sin(p.theta);
		}
		else { //yaw_rate != 0
			x = p.x + (velocity / yaw_rate) * (sin(p.theta + yaw_rate * delta_t) - sin(p.theta));
			y = p.y + (velocity / yaw_rate) * (-cos(p.theta + yaw_rate * delta_t) + cos(p.theta));
		}
		theta = p.theta + yaw_rate * delta_t;

		// create normal distributions around x, y, and theta
		normal_distribution<double> dist_x(x, std_x);
		normal_distribution<double> dist_y(y, std_y);
		normal_distribution<double> dist_theta(theta, std_theta);
		// add random noise to p.x, p.y, and p.theta by sampling their respective normal distrubtions
		p.x = dist_x(gen);
	    p.y = dist_y(gen);
	    p.theta = dist_theta(gen);
	    // while (p.theta < -M_PI) p.theta += 2*M_PI;
	    // while (p.theta > M_PI) p.theta -= 2*M_PI;
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

	//find the closest landamrk (nearest neighbor) to each observation
	for (int i=0; i<observations.size(); i++) {
		double min_range = numeric_limits<double>::max();
		int lm_id = 0;
		for (int j=0; j<predicted.size(); j++) { //find closest landmark
			double range = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);
			if (range < min_range) {
				min_range = range;
				lm_id = predicted[j].id;
			}
		}
		//set observation id to id of closest landmark
		observations[i].id = lm_id;
	}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
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

	//x & y uncertainties (std. dev.)
	double sig_x = std_landmark[0];
	double sig_y = std_landmark[1];
	//sum of weights
	double sum_w = 0;

	for (int i=0; i<particles.size(); i++) {
		Particle &p = particles[i];

		//copy map_landmarks within snesor_range into an an array of LandmarkObs
		std::vector<LandmarkObs> p_landmarks;
		for (auto map_lm: map_landmarks.landmark_list) {
			double range = dist(p.x, p.y, map_lm.x_f, map_lm.y_f);
			//skip if beyond the sensor range
			if (range > sensor_range)
				continue;
			LandmarkObs landmark;
			landmark.id = map_lm.id_i;
			landmark.x = map_lm.x_f;
			landmark.y = map_lm.y_f;
			p_landmarks.push_back(landmark);
		}

		//convert observations from local (partilce) coordinates to global (map) coordinates		
		std::vector<LandmarkObs> global_observations;
		global_observations.reserve(observations.size());
		for (LandmarkObs obs: observations) {
			LandmarkObs conv_obs;
			conv_obs.x = p.x + cos(p.theta) * obs.x - sin(p.theta) * obs.y;
			conv_obs.y = p.y + sin(p.theta) * obs.x + cos(p.theta) * obs.y;
			global_observations.push_back(conv_obs);
		}

		//associate each observation with its closest landmark (nearest neighbor)
		dataAssociation(p_landmarks, global_observations);

		//calculate weights (probabilties)
		double wt = 1;
		for (LandmarkObs obs: global_observations) {
			//landmarks array index is id-1
			auto lm = map_landmarks.landmark_list[obs.id-1];
			//mean is the closest landmark position
			double mu_x = lm.x_f;
			double mu_y = lm.y_f;
			//x & y are the converted measurment position
			double x = obs.x;
			double y = obs.y;
			//multivariate (x,y) gaussian prob. distrib.
			double prob = (1 / (2*M_PI*sig_x*sig_y)) * 
					exp(-(x - mu_x)*(x - mu_x) / (2*sig_x*sig_x) - (y - mu_y)*(y - mu_y) / (2*sig_y*sig_y));
			wt *= prob;
		}
		weights[i] = wt;
		sum_w += wt;
	}
	//normalize weights
	for (int i=0; i<particles.size(); i++) {
		weights[i] /= sum_w;
		particles[i].weight = weights[i];
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	default_random_engine gen;
	discrete_distribution<> distr(weights.begin(), weights.end());
	vector<Particle> resampled_p(particles.size());
	for (int i=0; i<particles.size(); i++) {
		int sample_id = distr(gen);
		resampled_p[i] = particles[sample_id];
	}
	particles = resampled_p;
}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
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
