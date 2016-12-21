/*
 * CameraObservation.h
 *
 * Author: A. Tuna Ozgelen
 */

#ifndef CAMERA_OBSERVATION_H_
#define CAMERA_OBSERVATION_H_

#include "Observation.h"
#include "Utils.h"

class CameraObservation : public Observation {
  Position pose; 
  double distance_variance;      // in cm
  double angle_variance;         // in degrees
 public:
  CameraObservation(double d = 50, double a = 120) : Observation(), distance_variance(d), angle_variance(a) {
    Position p(0,0,0); 
    pose = p; 
  } 

  double calculateLikelihoodForPosition(Position p) const { 
    double expectation = 0; 
    
    double d_exp = calculateLikelihoodForXY(p.getX(), p.getY()); 
    double a_exp = calculateLikelihoodForTheta(p.getTheta()); 

    expectation = d_exp * a_exp;
    
    return  expectation;
  }

  double calculateLikelihoodForXY(double x, double y) const {
    double d_exp = 0; 
    
    double distance = Utils::get_euclidian_distance(pose.getX(), pose.getY(), x, y); 
    double d_var = distance_variance; 
    d_exp = Utils::gaussian(distance, 0, d_var) / Utils::gaussian(0,0,d_var); 

    return d_exp; 
  }

  double calculateLikelihoodForTheta(double theta) const {
    double a_exp = 0; 
    
    double delta_angle = abs(pose.getTheta() - theta);
    double a_var = Utils::toRadians(angle_variance) ; 
    a_exp = Utils::gaussian(delta_angle, 0, a_var) / Utils::gaussian(0,0,a_var);
    
    return a_exp; 
  }

  void print() const {
    cout << "Camera observation at pose: (" << pose.getX() << "," 
	 << pose.getY() << "," << pose.getTheta() << ")" << endl;
  }
  
  void setPose(Position p) { pose = p ; }
  Position getPose() { return pose; }
};

#endif
