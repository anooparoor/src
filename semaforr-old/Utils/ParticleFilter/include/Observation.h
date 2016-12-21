/*
 * Observation.h
 *
 *      Author: A. Tuna Ozgelen
 */

#ifndef OBSERVATION_H_
#define OBSERVATION_H_

#include <iostream>
#include "Position.h"
using namespace std; 

class Observation {
 private:
  double value;
  double variance;
 public:
  Observation(double v=1, double var=0) : value(v), variance(var) {}
  virtual double calculateLikelihoodForPosition(Position) const {
    return 0;
  }

  /* FOR DEBUGGING ONLY. REMOVE ASAP */
  virtual double calculateLikelihoodForXY(double x, double y) const {
    return 0; 
  }
  virtual double calculateLikelihoodForTheta(double theta) const {
    return 0; 
  }
  /* END DEBUG */

  virtual void calculateValue() { setValue(1); }
  virtual void print() const {
    cout << "Observation value: " << value  << endl; 
  }  

  double getValue() const { return value; }
  void setValue(double value) { this->value = value; }

  mutable bool printInfo;
};

#endif /* OBSERVATION_H_ */
