/**!
  * Task.h
  * 
  * /author: Anoop Aroor
  *
  *          Defines tasks as simple target and stores its status and statistics of completion
  */

#ifndef TASK_H
#define TASK_H

#include "FORRAction.h"
#include "FORRGeometry.h"
#include "Position.h"
#include <vector>
#include <map>
#include <algorithm>
#include <iostream>
#include <fstream>

class Task {
  
 public:
  
  Task(int x_in, int y_in)  
    {
      x = x_in;
      y = y_in; 
      decision_count = 0;
      decisionSequence = new std::vector<FORRAction>;
      pos_hist = new vector<Position>();
      laser_hist = new vector< vector<CartesianPoint> >();
    }

  int getX() { return x; }
  
  int getY() { return y; }

  int getDecisionCount(){return decision_count;} 
 
  int incrementDecisionCount() {decision_count += 1;}

  std::vector<FORRAction> getPreviousDecisions(){
	return *decisionSequence;
  }

  FORRAction saveDecision(FORRAction decision){
	decisionSequence->push_back(decision);
  }

  vector<Position> *getPositionHistory(){return pos_hist;}

  void clearPositionHistory(){pos_hist->clear();}

  void saveSensor(Position currentPosition, vector<CartesianPoint> laserEndpoints){
	if(pos_hist->size() < 1){
		pos_hist->push_back(currentPosition);
		laser_hist->push_back(laserEndpoints);
	}
	else{	
     		Position pos = pos_hist->back();
     		if(!(pos == currentPosition)) {
			pos_hist->push_back(currentPosition);
			laser_hist->push_back(laserEndpoints);
		}
	}
  }
/*
  std::pair < std::vector<CartesianPoint>, std::vector< vector<CartesianPoint> > > getCleanedTrailMarkers(){
	std::pair < std::vector<CartesianPoint>, std::vector<vector<CartesianPoint> > > cleanedMarker;
	// call code to clean the trails
	vector <CartesianPoint> pos_history;
	for(std::vector<Position>::iterator it = pos_hist->begin() ; it != pos_hist->end(); ++it){
		CartesianPoint pos(it->getX(),it->getY());
		pos_history.push_back(pos);
	}
	
	std::vector<CartesianPoint> trailPositions;
	std::vector< vector<CartesianPoint> > trailLaserEndpoints;
	trailPositions.push_back(pos_history[0]);
	trailLaserEndpoints.push_back((*laser_hist)[0]);

	for(int i = 0; i < pos_history.size(); i++){
		cout << "First point: " << pos_history[i].get_x() << " " << pos_history[i].get_y() << endl;
		for(int j = pos_history.size()-1; j > i; j--){
			cout << pos_history[j].get_x() << " " << pos_history[j].get_y() << endl;
			if(canSeePoint((*laser_hist)[i], pos_history[i], pos_history[j])) {
			//if(pos_history[i].get_distance(pos_history[j]) < 5) {
				cout << "CanSeePoint is true" << endl;
				cout << "Next point: " << pos_history[j].get_x() << " " << pos_history[j].get_y() << endl;
				trailPositions.push_back(pos_history[j]);
				trailLaserEndpoints.push_back((*laser_hist)[j]);
				i = j-1;
			}
		}
	}
	cout << pos_history[pos_history.size()-1].get_x() << " " << pos_history[pos_history.size()-1].get_y() << endl;
	cout << trailPositions[trailPositions.size()-1].get_x() << " " << trailPositions[trailPositions.size()-1].get_y() << endl;
	cleanedMarker.first = pos_history;
	cleanedMarker.second = *laser_hist;
	return cleanedMarker;
  }
*/
  vector< vector <CartesianPoint> > *getLaserHistory(){return laser_hist;}

 private:
  
  //<! expected task execution time in seconds 
  float time_taken;

  // distance covered by the robot to get to the target
  float distance_travelled; 

  // Sequence of decisions made while pursuing the target
  std::vector<FORRAction> *decisionSequence;

  // Position History as is: Set of all unique positions the robot has been in , while pursuing the target
  std::vector<Position> *pos_hist; 

  // Laser scan history as is:
  vector< vector<CartesianPoint> > *laser_hist; 

  // Cleaned Position History, along with its corresponding laser scan data : Set of cleaned positions
  std::pair < std::vector<CartesianPoint>, std::vector<vector<CartesianPoint> > > *cleaned_trail;

  // decision count
  int decision_count;

  // t1, t2, t3 counters
  int tier1_decisions, tier2_decisions, tier3_decisions;

  //<! The point in the map, that the robot needs to go in order to execute this task 
  float x,y;
 
  
};

#endif
