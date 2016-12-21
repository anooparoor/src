/*
 * Markerobservation.cpp
 *
 *  Created on: Jan 13, 2009
 *      Author: Geotty
 */

#include <math.h>
#include "Utils.h"
#include "MarkerObservation.h"

#include <iostream>

/* Calculates and returns the probability of seeing this observation from a given position.
 * If the marker in the observation is not unique, then it repeats this process for each 
 * marker and assigns the probability of the most likely one. 
 */ 
double MarkerObservation::calculateLikelihoodForPosition(Position position) const {
  vector<MapMarker> markers = map->getMarkerById(markerId);
  
  double probability = 0;
  for (unsigned int i = 0; i < markers.size(); i++) {
    if ( printInfo )
      cout << "\tCalculating likelihood for a " << markerId 
	   << " marker at (" << markers[i].getX() << "," << markers[i].getY() 
	   << ")-left(" << markers[i].getLeftX() << "," << markers[i].getLeftY() 
	   << ")-right(" << markers[i].getRightX() << "," << markers[i].getRightY() << ") -- from position (" 
	   << (int) position.getX() << "," << (int) position.getY() << "," << (int) Utils::toDegrees(position.getTheta()) << ")" << endl; 
    double likelihood = calculateLikelihoodForMarkerAndPosition(markers[i],
								position);
    if ( printInfo )
      cout << "\tlikelihood :" << likelihood << endl;
    if (likelihood > probability)
      probability = likelihood;
  }
  
  if ( printInfo ) cout << "\tprobability : " << probability << endl; 
  printInfo = false; 
  return probability;
}

/* Given a marker and a position, this function checks the difference between the angles where the marker is observed
 * and where it should be observed. In addition distance is used as the variance for expectation computation. 
 * If there is not a wall between the position and marker and the distance is not too close, then the expectation 
 * of observing this marker from this position is returned.
 */
double MarkerObservation::calculateLikelihoodForMarkerAndPosition(MapMarker marker, Position position) const {
  double distanceToMarker = position.getDistance(Position(marker.getX(), marker.getY(), 0));
  
  if (distanceToMarker < 5) {
    if ( printInfo ) cout << "\t\tmarker too close" << endl;
    return 0;
  }
  
  /* check if there are any walls in the line of sight */
  if (isWallBlocking(marker, position)) {
    if ( printInfo ) cout << "\t\twall obstructing line of sight" << endl;
    return 0;
  }
  
  /*double variance = 100 / distanceToMarker * Utils::toRadians(this->variance);
  if (printInfo) 
    cout << "\t\tdistanceToMarker: " << distanceToMarker << " - variance: " << variance << endl;
  */
  int n = 0; 
  double expectation = 0;
  /*
  if ( printInfo ) cout << "\t\tbearing " << (int) Utils::toDegrees(bearing) << endl;
  double expectedBearing = marker.getBearing(position);
  if ( printInfo ) cout << "\t\texpectedBearing " << (int) Utils::toDegrees(expectedBearing) << endl;
  double deltaAngle = fabs((double) (bearing - expectedBearing));
  if ( printInfo ) cout << "\t\tdeltaAngle " << (int) Utils::toDegrees(deltaAngle) << endl;
  double expectation = Utils::gaussian(deltaAngle, 0, variance)
    / Utils::gaussian(0, 0, variance);
  if ( printInfo ) cout << "\t\texpectation " << expectation << endl;
  */
  if ( bearingLeft != InvalidBearing && marker.getLeftX() != -1 && marker.getLeftY() != -1) {
    double distanceToLeft = position.getDistance(Position(marker.getLeftX(), marker.getLeftY(), 0));
    double varianceLeft = 100 / distanceToLeft * Utils::toRadians(this->variance);
    if (printInfo) 
      cout << "\t\tdistanceToLeft: " << distanceToLeft << " - varianceLeft: " << varianceLeft << endl;
  
    if ( printInfo ) cout << "\t\tbearingLeft " << (int) Utils::toDegrees(bearingLeft) << endl;
    double expectedBearingLeft = marker.getLeftBearing(position);
    if ( printInfo ) cout << "\t\texpectedBearingLeft " << (int) Utils::toDegrees(expectedBearingLeft) << endl;
    double deltaAngleLeft = fabs((double) (bearingLeft - expectedBearingLeft));
    if ( printInfo ) cout << "\t\tdeltaAngleLeft " << (int) Utils::toDegrees(deltaAngleLeft) << endl;
    double expectationLeft = Utils::gaussian(deltaAngleLeft, 0, varianceLeft)
      / Utils::gaussian(0, 0, varianceLeft);
    if ( printInfo ) cout << "\t\texpectationLeft " << expectationLeft << endl;
    expectation += expectationLeft;
    n++;
  }

  if ( bearingRight != InvalidBearing && marker.getRightX() != -1 && marker.getRightY() != -1) {
    double distanceToRight = position.getDistance(Position(marker.getRightX(), marker.getRightY(), 0));
    double varianceRight = 100 / distanceToRight * Utils::toRadians(this->variance);
    if (printInfo) 
      cout << "\t\tdistanceToRight: " << distanceToRight << " - varianceRight: " << varianceRight << endl;

    if ( printInfo ) cout << "\t\tbearingRight " << (int) Utils::toDegrees(bearingRight) << endl;
    double expectedBearingRight = marker.getRightBearing(position);
    if ( printInfo ) cout << "\t\texpectedBearingRight " << (int) Utils::toDegrees(expectedBearingRight) << endl;
    double deltaAngleRight = fabs((double) (bearingRight - expectedBearingRight));
    if ( printInfo ) cout << "\t\tdeltaAngleRight " << (int) Utils::toDegrees(deltaAngleRight) << endl;
    double expectationRight = Utils::gaussian(deltaAngleRight, 0, varianceRight)
      / Utils::gaussian(0, 0, varianceRight);
    if ( printInfo ) cout << "\t\texpectationRight " << expectationRight << endl;
    expectation += expectationRight;
    n++;
  }
  
  ( n!= 0 ) ? expectation /= n : expectation = 0 ;
 
  return expectation;
}

/* This assigns a preset value for representing the information gain of a marker
 * more blobs in a marker means there is less chance of misinterpretation and
 * if both sides of the marker is in view, it is more valuable as an observation.
 */
void MarkerObservation::calculateValue() {
  int numberOfBlobs = 1;
  string mark = markerId;
  while ( mark.find("/") != string::npos ){
    mark.replace(0, mark.find("/")+1, ""); 
    numberOfBlobs++; 
  }
  switch( numberOfBlobs ) {
  case 1: 
    if ( bearingRight == InvalidBearing || bearingLeft == InvalidBearing ) 
      value = 0.45; 
    else
      value = 0.65; 
    break; 
  case 2: 
    if ( bearingRight == InvalidBearing || bearingLeft == InvalidBearing ) 
      value = 0.55; 
    else
      value = 0.75; 
    break; 
  case 3: 
    if ( bearingRight == InvalidBearing || bearingLeft == InvalidBearing ) 
      value = 0.85; 
    else
      value = 1; 
    break;
  }
}

bool MarkerObservation::isWallBlocking(MapMarker marker, Position position) const{
  double mx = marker.getX(); 
  double my = marker.getY(); 
  double px = position.getX();
  double py = position.getY(); 
  double ix, iy; 

  vector<MapWall> walls = map->getWalls();
  vector<MapWall>::iterator iter; 
  for( iter = walls.begin(); iter !=walls.end(); iter++ ){
    if ( Utils::get_line_intersection(iter->getX0(),iter->getY0(),iter->getX1(),iter->getY1(),mx,my,px,py,&ix,&iy) ){
      return true;
    }
  }
  return false;
}
