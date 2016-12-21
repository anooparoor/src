/*
 * MarkerObservation.h
 *
 *      Author: richardmarcley
 */

#ifndef MARKER_OBSERVATION_H_
#define MARKER_OBSERVATION_H_

#include "Observation.h"
#include "MapMarker.h"
#include "MapWall.h"
#include "Position.h"
#include "Map.h"

class MarkerObservation : public Observation {
public:
 MarkerObservation(string markerId, Map* map, double variance, double bearingLeft = InvalidBearing, double bearingRight = InvalidBearing) :
  bearing(0) {
    // for DEBUG info TODO delete
    printInfo = false;
    this->map = map;
    this->markerId = markerId;
    //this->bearing = bearing;
    this->variance = variance;
    this->bearingLeft = bearingLeft; 
    this->bearingRight = bearingRight;
    calculateValue();
    //print();
  }

  // TODO : test this with the first version of blob processing where used blobs were erased on the fly and causing free() invalid pointer error.
  //~MarkerObservation(){}
  
  double calculateLikelihoodForPosition(Position) const;
  
  double getBearing() const { return bearing; }
  double getBearingLeft() const { return bearingLeft; } 
  double getBearingRight() const { return bearingRight; }
  
  string getMarkerId() const{
    return markerId;
  }
  
  void setMap(Map * map) {
    this->map = map;
  }

  double getValue() const { return value; }
  void setValue(double value) {
    this->value = value; 
  }

  void print() const {
    cout << "MarkerObservation " << markerId 
	 << " bearing for left: " << bearingLeft 
      //<< "\tcenter: " << bearing 
	 << "\tright: " << bearingRight 
	 << "\tvalue: " << value << endl;  
  } 
  
  const static double InvalidBearing = -M_PI * 3;

  // for printing DEBUG info for a single particle TODO delete
  mutable bool printInfo;
  
private:  
  double calculateLikelihoodForMarkerAndPosition(MapMarker marker, Position position) const;
  void calculateValue();
  
  bool isWallBlocking(MapMarker marker, Position position) const; 
  
  Map * map;
  string markerId;
  double value;
  double bearing, bearingLeft, bearingRight;
  double variance;
};

#endif /* OBSERVATION_H_ */
