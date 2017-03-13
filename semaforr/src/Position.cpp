/*
 * Position.cpp
 *
 *  Created on: Dec 26, 2008
 *      Author: richardmarcley
 */

#include "Position.h"
#include <stdio.h>
#include "FORRAction.h"
#include <math.h>


Position Position::getExpectedPositionAfterActions(Position initialPosition, vector<FORRAction> actionList){
  Position currentPosition = initialPosition;
  Position nextPosition;
  for(int i = 0 ; i < actionList.size(); i++){
    FORRAction action = actionList[i];
    nextPosition = getExpectedPositionAfterAction(currentPosition, action);
    // cout << "AgentState :: getExpectedPositionAfterActions >> nextposition " << nextPosition.getX() << " " << nextPosition.getY() << endl;
    currentPosition = nextPosition;
  }
  return nextPosition;
}

Position Position::getExpectedPositionAfterAction(Position initialPosition, FORRAction action){
  
  Position result; 
  int intensity = action.parameter;
  FORRActionType type = action.type;
  
  //Estimate of the moves made by slavisa after running the robots on stage multiple times could be different for physical robots
  // <type:parameter> : value (either angular rotation or forward or backward moves)
  //    <NOOP,*> : 0
  //    <FORWARD,1;2;3;4;5> : 3,7,20,25,105
  //    <BACKWARD,1;2;3;4;5> : -3,-7,-20,-25,-105
  //    <RIGHT_TURN,1;2;3;4;5> :
  // Slavisa this part has to be moved to reading from the config file
  double tmp_rotations[] = {.1548, .3048, .65, 1.3, 3.39};  
  int tmp_movements[] = {3, 7, 20, 25, 105};
  
  switch(type){
  case NOOP:
    result = initialPosition;
    break;
  case FORWARD:
    result = afterLinearMove(initialPosition, tmp_movements[intensity-1]);
    break;
  case BACKWARD:
    result = afterLinearMove(initialPosition, -tmp_movements[intensity-1]);
    break;
  case RIGHT_TURN:
    result = afterAngularMove(initialPosition, -tmp_rotations[intensity-1]);
    break;
  case LEFT_TURN:
    result = afterAngularMove(initialPosition, +tmp_rotations[intensity-1]);
    break;
    //currently wide turn is not used in the system
  case WIDE_RIGHT_TURN:
    result = initialPosition;
    break;
    //currently wide turn is not used in the system
  case WIDE_LEFT_TURN:
    result = initialPosition;
    break;
  case PAUSE:
    result = initialPosition;
  case HALT:
    result = initialPosition;
    break;
  }
  return result;
}
  

Position Position::afterLinearMove(Position initialPosition, double distance){
  
  double new_x = initialPosition.getX() + (distance * cos(initialPosition.getTheta()));
  double new_y = initialPosition.getY() + (distance * sin(initialPosition.getTheta()));
  
  return Position(new_x, new_y, initialPosition.getTheta());
}


Position Position::afterAngularMove(Position initialPosition, double angle){

  double new_angle = (angle + initialPosition.getTheta());
  return Position(initialPosition.getX(), initialPosition.getY(), new_angle);
}



Position::Position(double x, double y, double theta) {
	this->x = x;
	this->y = y;
	this->theta = theta;
}

double Position::getDistance(Position other) {
	double dx = x - other.getX();
	double dy = y - other.getY();
	double distance = sqrt(dx * dx + dy * dy);
	return distance;
}

double Position::getDistance(double x1, double y1) {
	double dx = x - x1;
	double dy = y - y1;
	double distance = sqrt(dx * dx + dy * dy);
	return distance;
}

double Position::getX() const {
	return x;
}

void Position::setX(double x) {
	this->x = x;
}

double Position::getY() const {
	return y;
}

void Position::setY(double y) {
	this->y = y;
}

double Position::getTheta() const {
	return theta;
}

void Position::setTheta(double theta) {
	this->theta = theta;
}

bool Position::operator==(Position p){
	return (x == p.getX() && y == p.getY() && theta == p.getTheta());
}
