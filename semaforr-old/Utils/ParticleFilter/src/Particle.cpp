/*
 * Particle.cpp
 *
 *  Created on: Dec 21, 2008
 *  Author: George Rabanca
 */
#include <math.h>
#include <time.h>		//for randomize seed
#include <iostream>

#include "Particle.h"
#include "Utils.h"

void Particle::updatePosition(Move move)
{
  position.moveRelative(move);
}

// each observation calculates the probability for the given position of the particle
// then the probability is not changed with more than MAX_PROBABILITY_INCREASE or MAX_PROBABILITY_DECREASE
void Particle::updateProbability(const vector<Observation*> &obs)
{
  double newProbability = 0;
  //double observationValue = 1 ; // TODO: find a good approach to calculate this
  
  // for debug. displays a particle's prob update at every step
  if ( selected ) {
    for ( unsigned int i = 0 ; i < obs.size() ; i++ ) 
      obs[i]->printInfo = true;  
      cout << "updating probability for particle at (" << position.getX() 
	 << "," << position.getY() << "," << position.getTheta() << ")" << endl;
  }
  
  unsigned int i;
  for (i=0; i<obs.size(); i++) {
    newProbability += obs[i]->calculateLikelihoodForPosition(position) * obs[i]->getValue();
  }
  
  //probability = ( learningRate * probability ) + 
  //  ( 1 - learningRate ) * ( ( 1 - observationValue ) * probability + observationValue * newProbability );
  newProbability /= obs.size(); 
  if ( selected )
    cout << "probability = (" << learningRate << " * " << probability << ") + ( 1 - " 
	 << learningRate << " ) * " << newProbability << endl;
  probability = ( learningRate * probability ) + ( 1 - learningRate ) * newProbability ;
}

void Particle::updatePostion(double rotation, double distance)
{
  position.rotate(rotation);
  position.walk(distance);
}

bool Particle::isMoreProbable(Particle p1, Particle p2){
  if (p1.probability > p2.probability) return true;
  return false;
}

/*
void Particle::changeProbability(double newProbability)
{
  if(probability - 0.05 > newProbability) {
    probability = probability - 0.05;
    return;
  }
  
  if(probability + 0.03 < newProbability){
    probability = probability + 0.03;
    return;
  }
  
  probability = newProbability;
  }
*/
