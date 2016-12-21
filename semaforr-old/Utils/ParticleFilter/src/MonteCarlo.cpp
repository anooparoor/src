/*
 * MonteCarlo.cpp
 *
 *  Created on: Dec 21, 2008
 *  Author: George Rabanca
 */
#include <stdlib.h>
#include <algorithm>
#include <iostream>
#include "MonteCarlo.h"
#include "MCLPositionEstimator.h"

using namespace std;

//const float STARTING_PARTICLE_PROBABILITY = .2;

MonteCarlo::MonteCarlo(Map * m) 
  : map(m), PARTICLE_NUM(500), particlesToReseed(10), STARTING_PARTICLE_PROBABILITY(.2) {
  for (int i = 0; i < PARTICLE_NUM; i++) {
    particles.push_back(seedParticle());
  }
	
  setRandomCoefficients(.1, 10, M_PI / 16, 20); 
  setTrackingCoefficients(.1, 0, .2); 
  
  debugger = NULL;
	
  // TODO delete
  particleSelected = false;
}

MonteCarlo::MonteCarlo(Map * m, int pt_num, int pt_reseed, double st_pt_prob)
  : map(m), PARTICLE_NUM(pt_num), particlesToReseed(pt_reseed), STARTING_PARTICLE_PROBABILITY(st_pt_prob) {
  for (int i = 0; i < PARTICLE_NUM; i++) {
    particles.push_back(seedParticle());
  }
	
  setRandomCoefficients(.1, 10, M_PI / 16, 20); 
  setTrackingCoefficients(.1, 0, .2); 

  debugger = NULL;

  // Later, if error model is available for a robot (read from robot.conf file) these will be set to true
  transErrAvailable = false; 
  driftErrAvailable = false; 
  rotLeftErrAvailable = false; 
  rotRightErrAvailable = false; 
	
  // TODO delete
  particleSelected = false;
}

void MonteCarlo::setRandomCoefficients(double thetaConfident,
		double xyConfident, double theta, double xy) {

	DELTA_RANDOM_THETA_CONFIDENT = thetaConfident;
	DELTA_RANDOM_XY_CONFIDENT = xyConfident;
	DELTA_RANDOM_THETA = theta;
	DELTA_RANDOM_XY = xy;
}

void MonteCarlo::setTrackingCoefficients(double trackingRandX, double trackingRandY, double trackingRandTheta){
	TRACKING_RANDOM_X = trackingRandX;
	TRACKING_RANDOM_Y = trackingRandY;
	TRACKING_RANDOM_THETA = trackingRandTheta;
}

void MonteCarlo::debug(vector<Observation*>& obs) {
	if (debugger != NULL) {
		debugger->setPosition(getPosition());
		debugger->setConfidence(confidence);
		debugger->setParticles(particles);
		debugger->setObservations(obs);
	}
}

Position MonteCarlo::getPosition() {
  MCLPositionEstimator estimator = MCLPositionEstimator(map->getLength(), map->getHeight());
  position = estimator.getPosition(particles, confidence);
  return position;
}

//debug method for a bug where the position is outside the map or the theta is outside of
// (-PI PI] interval
void MonteCarlo::testAllParticlesInsideMap(char * message) {
	for (unsigned int i = 0; i < particles.size(); i++) {
		//if particle is outside of the field reseed it;
		Position pos = particles[i].getPosition();
		if (!isInsideMap(particles[i]) || pos.getTheta() > PI || pos.getTheta()
				<= -PI) {
			std::cout<< "Particle outside bounds: " << pos.getX() << "/"
					<< pos.getY() << "/" << pos.getTheta() << " " << message
					<< std::endl;

			particles[i] = seedParticle();
		}
	}
}

void MonteCarlo::updateFilter(Move delta, vector<Observation*>& obs) {

  applyMoveToParticles(delta);

  // only update particle probabilities when not moving. This works best since the moving camera feed is
  // crap but also means we are relying on odometry (applyMoveToParticles) when moving and recieving 
  // good images. In practice the way this code talks to player, there is significant lag between what is 
  // sent and what is recieved ex: we may recieve 'not moving' from Player when the robot is clearly moving,
  // therefore it is not appearent to me what to do other than this. 
  if ( obs.size() != 0 && ( delta.getX() == 0 && delta.getY() == 0 && delta.getTheta() == 0) ){
    vector<Observation*>::iterator iter; 
    //for( iter = obs.begin(); iter != obs.end(); iter++ ) {
    //(*iter)->print();  
    //}
    updateParticleProbabilities(obs);
  }

  resample();
  
  //TODO: do not do it twice
  //we are doing it in resample but before introducing new particles.
  normalizeParticleProbabilities();

  debug(obs);
}

//this method updates particle positions according to the given move and some induced randomness.
//particles that have a bad probability move more randomly than particles with high probability.
//particles that move outside of the map are reseeded.
void MonteCarlo::applyMoveToParticles(Move delta) {
  if ( !(transErrAvailable && driftErrAvailable && rotRightErrAvailable && rotLeftErrAvailable )) {
    unsigned int i;
    double deltaXY;
    double deltaTheta;
    
    if (confidence > .3) {
      deltaXY = DELTA_RANDOM_XY_CONFIDENT;
      deltaTheta = DELTA_RANDOM_THETA_CONFIDENT;
    } else {
      deltaXY = DELTA_RANDOM_XY;
      deltaTheta = DELTA_RANDOM_THETA;
    }
    
    for (i = 0; i < particles.size(); i++) {
      double probability = particles[i].probability;
      double x = induceRandomness(delta.getX(), TRACKING_RANDOM_X, deltaXY, probability);
      double y = induceRandomness(delta.getY(), TRACKING_RANDOM_Y, deltaXY, probability);
      double theta = induceRandomness(delta.getTheta(), TRACKING_RANDOM_THETA, deltaTheta, probability);
      
      particles[i].updatePosition(Move(x, y, theta));
      
      //if particle is outside of the field reseed it;
      if (!isInsideMap(particles[i])) {
	particles[i] = seedParticle();
      }
    }
  }
  // if the error model is available sample from the distribution 
  else {
    double x = 0.0, y = 0.0, theta = 0.0, dTheta = delta.getTheta(); 
    for (unsigned int i = 0; i < particles.size(); i++) {
      x = delta.getX() * Utils::box_muller(TRANSLATION_MEAN, TRANSLATION_VAR); 
      theta = Utils::box_muller(DRIFT_MEAN, DRIFT_VAR); 
      if ( dTheta > 0 ) 
	theta += dTheta * Utils::box_muller(ROT_LEFT_MEAN, ROT_LEFT_VAR); 
      else
	theta += dTheta * Utils::box_muller(ROT_RIGHT_MEAN, ROT_RIGHT_VAR); 
      
      particles[i].updatePosition(Move(x, y, theta));
      
      //if particle is outside of the field reseed it;
      if (!isInsideMap(particles[i])) {
	particles[i] = seedParticle();
      }
    }
  }
}

//this method updates particles probabilities according to the current observations;
void MonteCarlo::updateParticleProbabilities(const vector<Observation*>& obs) {
  //TODO make sure it is implemented ok...  test it !
  for (unsigned int i = 0; i < particles.size(); i++) {
    if ( MC_DEBUG )
      if ( particles[i].probability >= 0.2 && !particleSelected ){
	cout << "new particle selected" << endl;
	particleSelected = true;
	particles[i].selected = true;
      }
    particles[i].updateProbability(obs);
  }
}

//this method updates the set of particles;  it duplicates successful particles and gets rid of
//particles that have bad probability;
//the expected number of a particle with probability p is TotalParticleNum * p / sum(all prob);
void MonteCarlo::resample() {
	sort(particles.begin(), particles.end(), &Particle::isMoreProbable);
	vector<double> normalizedP = normalizeParticleProbabilities();

	int particleNum = particles.size();
	int i = 0;
	vector<Particle> resampled;

	//we are trying a different approach to reseeding...
	while (particles[i].probability > .1 && static_cast<int>(resampled.size()) < PARTICLE_NUM
	       - particlesToReseed) {
	  int copies = (int) max(normalizedP[i] * static_cast<int>(particles.size()), 1.0);
	  resampled.insert(resampled.end(), copies, particles[i]);
	  particleNum -= copies;
	  i++;
	}

	//reseed about 'particlesToReseed' particles every iteration
	//TODO make this reseeding smarter: account for landmarks and history of landmarks
	while (particleNum > 0) {
		resampled.push_back(seedParticle());
		particleNum--;
	}

	particles = resampled;
}

//TODO particles have now normalizedProbabilities so we should use that
//field instead of returning a different vector.
vector<double> MonteCarlo::normalizeParticleProbabilities() {
	double probabilitySum = addParticleProbabilities();
	return divideProbabilitiesBy(probabilitySum);
}

double MonteCarlo::addParticleProbabilities() {
	double probabilitySum = 0;
	for (unsigned int i = 0; i < particles.size(); i++) {
		probabilitySum += particles[i].probability;
	}
	return probabilitySum;
}

vector<double> MonteCarlo::divideProbabilitiesBy(double probabilitySum) {
	vector<double> normalizedProbabilities;
	for (unsigned int i = 0; i < particles.size(); i++) {
		double normalized = particles[i].probability / probabilitySum;
		particles[i].normalizedProbability = normalized;
		normalizedProbabilities.push_back(normalized);
	}
	return normalizedProbabilities;
}

double MonteCarlo::induceRandomness(double x, double percentRandom, double maxRandom, double probability) {
	return (x + x * percentRandom * Utils::getRandom(-1, 1) + maxRandom * Utils::getRandom(-1, 1) * (1 - probability));
}

Particle MonteCarlo::seedParticle() {
	Particle newParticle = Particle();

	double x = Utils::getRandom(0, map->getLength());
	double y = Utils::getRandom(0, map->getHeight());
	double theta = Utils::getRandom(-PI, PI);

	newParticle.setPosition(Position(x, y, theta));
	newParticle.probability = STARTING_PARTICLE_PROBABILITY;
	newParticle.normalizedProbability = 0;

	return newParticle;
}

bool MonteCarlo::isInsideMap(Particle particle) {
	double x = particle.getPosition().getX();
	double y = particle.getPosition().getY();
	if (x <= 0 || y <= 0 || x >= map->getLength() || y >= map->getHeight()) {
		return false;
	}
	return true;
}
