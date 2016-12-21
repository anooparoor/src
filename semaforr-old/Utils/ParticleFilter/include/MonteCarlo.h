/*
 * MonteCarlo.h
 *
 *  This is the main class for the MonteCarlo localization.
 *  It should be initialized with a map containing markers and their positions.
 *  Then should be continuously updated with the robot moves and marker observations.
 *
 *  The getPosition() and getConfidence() methods are giving the approximate position
 *  of the robot.
 */

#ifndef MONTECARLO_H_
#define MONTECARLO_H_

#include "vector"
#include "Utils.h"
#include "Map.h"
#include "Particle.h"
#include "Move.h"
#include "MonteCarloDebugger.h"

using namespace std;

//#define PARTICLE_NUM 500
#define MC_DEBUG false

class MonteCarlo {
public:
  explicit MonteCarlo(Map*);
  MonteCarlo(Map*, int, int, double);

  void updateFilter(Move delta, vector<Observation*>& obs);
  
  double getConfidence() const { return confidence; }
  vector<Particle> getParticles() { return particles; }
  void setDebugger(MonteCarloDebugger * debugger) { this->debugger = debugger; }
  MonteCarloDebugger* getDebugger() { return debugger; }
  Position getPosition();
  
  void setRandomCoefficients(double thetaConfident, double xyConfident, double theta, double xy);
  void setTrackingCoefficients(double trackingRandX, double trackingRandY, double trackingRandTheta);
  
  // set motion error model pdf parameters 
  void setTranslationError(double t_mean, double t_var) {
    TRANSLATION_MEAN = t_mean;
    TRANSLATION_VAR = t_var;
    transErrAvailable = true;
  }
  void setDriftError(double d_mean, double d_var){
    DRIFT_MEAN = d_mean;
    DRIFT_VAR = d_var;
    driftErrAvailable = true;
  }
  void setRotationLeftError(double r_l_mean, double r_l_var){
    ROT_LEFT_MEAN = r_l_mean;
    ROT_LEFT_VAR = r_l_var;
    rotLeftErrAvailable = true;
  } 
  void setRotationRightError(double r_r_mean, double r_r_var){
    ROT_RIGHT_MEAN = r_r_mean;
    ROT_RIGHT_VAR = r_r_var;
    rotRightErrAvailable = true;
  }

  // print DEBUG info TODO delete
  bool particleSelected;
private:
  vector<Particle> particles;
  Map *map;
  int PARTICLE_NUM;
  int particlesToReseed;
  double STARTING_PARTICLE_PROBABILITY;

  double confidence;
  Position position;
  MonteCarloDebugger * debugger;
  
  double DELTA_RANDOM_THETA_CONFIDENT;
  double DELTA_RANDOM_XY_CONFIDENT;
  double DELTA_RANDOM_THETA;
  double DELTA_RANDOM_XY;
  
  double TRACKING_RANDOM_X;
  double TRACKING_RANDOM_Y;
  double TRACKING_RANDOM_THETA;
  
  // Motion error distributions
  bool transErrAvailable; 
  bool driftErrAvailable; 
  bool rotLeftErrAvailable; 
  bool rotRightErrAvailable; 

  double TRANSLATION_MEAN; 
  double TRANSLATION_VAR;
  double DRIFT_MEAN; 
  double DRIFT_VAR; 
  double ROT_LEFT_MEAN; 
  double ROT_LEFT_VAR; 
  double ROT_RIGHT_MEAN; 
  double ROT_RIGHT_VAR; 
  
  void updateParticleProbabilities(const vector<Observation*>& obs);
  void applyMoveToParticles(Move delta);
  void resample();
  Particle seedParticle();
  
  double induceRandomness(double x, double percentRandom, double maxRandom, double probability);
  bool isInsideMap(Particle particle);
  vector<double> normalizeParticleProbabilities();
  double addParticleProbabilities();
  vector<double> divideProbabilitiesBy(double probabilitySum);
  
  void testAllParticlesInsideMap(char * message);
  
  void debug(vector<Observation*>& obs);
  
  friend class MonteCarloTest;
};

#endif /* MONTECARLO_H_ */
