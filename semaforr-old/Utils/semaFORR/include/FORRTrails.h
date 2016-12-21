/*******************
FORRTrails.h

Implements the FORRTrails class, which provides SemaFORR with a Trails advisor.  This advisor aims to
utilize the saved, corrected paths from previous task points to guide the robot to reach a target
point that is near where the old path lead.  The advisor votes for moves that would bring the robot along the path


Written by Matthew Evanusa, February 2015
******************/

#ifndef FORRTRAILS_H
#define FORRTRAILS_H


#include <iostream>
#include <FORRGeometry.h>
#include <vector>
#include <fstream>
#include <utility>


using namespace std;

//each trail (cleaned path) gets loaded in at the same time as the wall distance vectors
//for that point to minimize confusion
struct TrailMarker{
  CartesianPoint coordinates;
  vector<CartesianPoint> wallVectorEndpoints;
TrailMarker(CartesianPoint c, vector<CartesianPoint> &v): coordinates(c), wallVectorEndpoints(v){}

};

//determines whether you're following the current trail in reverse or forwards
enum DIRECTION {POSITIVE, NEGATIVE};


class FORRTrails{
 public:
  
  void readTrails(string filename); //populate the trails vector with new paths


  //returns true if there is a visible path to a point given the distance vector distance and the point
  bool canRobotSeeTrailSegment(vector<double> &distanceVectors, CartesianPoint trailpoint1, CartesianPoint trailpoint2,
			      CartesianPoint current_point);


  //returns true if a point along the trail (indexed by trail_index) has wall vectors that come
  //close to target_point
  int doesTrailHaveVisiblePointToTarget(CartesianPoint target_point, int trail_index);


  //returns true if there is a visible path to a point given the endpoints of distance vectors and the point
  bool canTrailMarkerSeeTarget(vector<CartesianPoint> &distanceVectorEndpoints, CartesianPoint target_point, 
			     CartesianPoint current_point, double epsilon);


  //assumes the trail file is trail.conf and the vectors file is corrected_wallvectors.conf
  void updateTrails();
  
  CartesianPoint getFurthestVisiblePointOnChosenTrail(CartesianPoint current, vector<double> &currentDistanceVectors);


  //to be called by the Tier3 Advisor, which has access to the belief endDistances
  //returns an integer index to the top-level vector of trails
  void findNearbyTrail(CartesianPoint target, CartesianPoint start, vector<double> &currentDistanceVectors);

  void printTrails();
  
  
 FORRTrails(): chosen_trail(-1) {}
  
  
  void setDirection(DIRECTION d){dir = d;}
  
  vector<TrailMarker> getTrail(int i){ return trails[i]; }
  
  int getSize(){return trails.size();}
 
  
  void setChosenTrail(int n){ chosen_trail = n;}
  
  int getChosenTrail(){ return chosen_trail; }
  
  
  bool canSeeTrail() {return can_see_trail;}

  void resetChosenTrail(){ chosen_trail = -1;}
  CartesianPoint intersection_p;

 private:
  
  DIRECTION dir;

  vector< vector< TrailMarker> > trails;
    
  //stores the integer index to the trails vector where you have found a usable trail
  int chosen_trail;
  bool can_see_trail;


};

























#endif 
