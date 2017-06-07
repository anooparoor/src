/************************************************
FORRAbstractMap.h 
This file contains the class which contains information about the map that FORR learns and uses

Written by Anoop Aroor, 2014
**********************************************/

#ifndef FORRABSTRACTMAP_H
#define FORRABSTRACTMAP_H

#include <iostream>
#include <fstream>
#include "FORRGeometry.h"
#include "FORRCircle.h"
#include "FORRExit.h"

class FORRAbstractMap{
 public:
  FORRAbstractMap(){};
  vector<FORRCircle> getCircles(){ return circles; }
  FORRCircle getCircle(int index){ return circles[index]; }
  void setCircles(vector<FORRCircle> circles_para) { circles = circles_para; }
  //void addCircle(vector<FORRCircle> circle) { circles.push_back(circle);}

  bool isExitToLeaf(FORRExit exit){
    return isLeaf(circles[exit.getExitCircle()]);
  }
    
  bool isLeaf(FORRCircle circle){
    //cout << "In isleaf" << endl;
    vector<FORRExit> exits = circle.getExits();
    bool isLeaf = false;
    if(exits.size() <= 1){
      isLeaf = true;
      //circle.setIsLeaf(isLeaf);
      return isLeaf;
    }
    double min_angle = 10;
    double max_angle = -10;
    for(int i = 0 ; i < exits.size(); i++){
      double angle = atan2(exits[i].getExitPoint().get_y() - circle.getCenter().get_y(), exits[i].getExitPoint().get_x() - circle.getCenter().get_x());
      if(angle < 0){
	angle += 2*3.14;
      }
      //cout << "Exit no : " << i << " Angle : " << angle << endl; 
      if(angle < min_angle) min_angle = angle;
      if(angle >= max_angle) max_angle = angle;
    }
    //cout << "min_angle :" << min_angle << endl;
    //cout << "max_angle :" << max_angle << endl;
    if(max_angle - min_angle < (3.14)/2){
      isLeaf = true;
      //circle.setIsLeaf(isLeaf);
      //cout << "In isLeaf setting is leaf to true " << endl; 
    }
    return isLeaf;
  } 

  
			   
  void learnRegions(vector<Position> *pos_hist, vector< vector<CartesianPoint> > *laser_hist){
    vector<Position> positionHis = *pos_hist;
    vector < vector <CartesianPoint> > laserHis = *laser_hist;
    cout << "In learning regions" << endl;
    CartesianPoint current_point;
    FORRCircle current_circle;
    for(int k = 0 ; k < laserHis.size(); k++){
      vector <CartesianPoint> laserEndpoints = laserHis[k];
      Position current_position = positionHis[k];

      double radius = 10000;
      int direction = -1;
      for(int i = 0; i< laserEndpoints.size(); i++){
	//cout << "wall distance " << wallDistanceVector[i] << endl;
	double range = laserEndpoints[i].get_distance(CartesianPoint(current_position.getX(),current_position.getY()));
	if (range < radius){
	  radius = range;
	  direction = i;
	}
      }
     
      current_point = CartesianPoint(current_position.getX(), current_position.getY());
      current_circle = FORRCircle(current_point, radius);
      for(int j = 0 ; j < laserHis.size(); j++){
	vector <CartesianPoint> nextLaserEndpoints = laserHis[j];
	Position next_position = positionHis[j];
	// if next position in still inside the current_circle update current_circle radius
	if(current_circle.inCircle(next_position.getX(), next_position.getY()) && j != k){
	  double next_radius = 10000;
	  int next_direction = -1;
	  for(int i = 0; i < nextLaserEndpoints.size(); i++){
	    //cout << "wall distance " << wallDistanceVector[i] << endl;
	    double next_range = nextLaserEndpoints[i].get_distance(CartesianPoint(next_position.getX(),next_position.getY()));
	    if (next_range < next_radius){
	      next_radius = next_range;
	      next_direction = i;
	    }
	  }
	  double x = nextLaserEndpoints[next_direction].get_x();
	  double y = nextLaserEndpoints[next_direction].get_y();
	  double dist = current_circle.getCenter().get_distance(CartesianPoint(x , y));
	  if(dist < current_circle.getRadius())
	    current_circle.setRadius(dist);	       
	} 
      }
      
      // check if the robot is in a previously created region
      int robotRegion = -1;
      for(int i = 0; i < circles.size(); i++){
	if(circles[i].inCircle(current_point.get_x(), current_point.get_y()))
	  robotRegion = i;
      }
      // correct previously create region 
      if(robotRegion != -1){
	double x = laserEndpoints[direction].get_x();
	double y = laserEndpoints[direction].get_y();
	double dist = circles[robotRegion].getCenter().get_distance(CartesianPoint(x,y));
	if(dist < circles[robotRegion].getRadius())
	  circles[robotRegion].setRadius(dist);
      }
      
      bool new_circle = true;
      // if there is atleast one intersecting circle which is bigger than the current circle , dont add the current circle
      for(int i = 0; i < circles.size(); i++){
	if(current_circle.doIntersect(circles[i]) == true)
	  if(current_circle.getRadius() < circles[i].getRadius()){
	    new_circle = false;
	  }
      }
      
      //cout << "Current circle " << current_circle.getCenter().get_x() << "," << current_circle.getCenter().get_y() << ":" << current_circle.getRadius() << endl;
      //cout << "New circle = " << new_circle << endl; 
      
      if(new_circle == true){
	// save the circle
	//cout << "current_circle is a new circle , adding it into the list" << endl;
	
	// delete all intersecting circles
	for(int i = 0; i < circles.size() ; i++){
	  //cout << "for circle " << i << endl;
	  if(current_circle.doIntersect(circles[i])){
	    //cout << "Deleting circle:"  << circles[i].getCenter().get_x() << " " << circles[i].getCenter().get_y() << " " << circles[i].getRadius() << endl;
	    circles.erase(circles.begin() + i);
	    i--;
	  }
	}	
	circles.push_back(current_circle);
      }
    }
    cout << "Exit learning regions"<<endl;
  }

  
  
  
  void learnExits(vector< vector<CartesianPoint> > run_trace){
    // learning gates between different circles
    clearAllExits();
    // for every position in the position history vector .. check if a move is from one circle to another and save it as gate
    //cout << "In learning exits: size of trace is " << run_trace.size() << endl;
    for(int k = 0; k < run_trace.size() ; k++){
      vector<CartesianPoint> history = run_trace[k]; 
      //cout << "Learning exits between circles" << endl;
      int circle_id=-1, previous_position_circle_id=-1,  begin_circle_id, end_circle_id, begin_position , end_position;
      bool beginFound = false;
      for(int j = 0; j < history.size(); j++){  
	//cout << history[j].get_x() << " " << history[j].get_y() << endl;
	previous_position_circle_id = circle_id;
	circle_id = pointInCircles(history[j].get_x(), history[j].get_y());
	
	//cout << "previous circle id : " << previous_position_circle_id << endl;
	//cout << "current circle id : " << circle_id << endl;
	if(circle_id == -1){
	  //cout << "skipping position" << endl;
	  continue;
	}
	// either we have not found a starting circle or 
	if(circle_id != -1 && (previous_position_circle_id == circle_id || beginFound == false)){
	  beginFound = true;
	  begin_circle_id = circle_id;
	  begin_position = j;
	  //cout << "Starting position : " << begin_position << endl;
	  continue;
	}
	if(circle_id != -1 && circle_id != begin_circle_id && beginFound == true){
	  beginFound = false;
	  end_circle_id = circle_id;
	  end_position = j;
	  //cout << "Ending Position : " << end_position << endl;
	  saveExit(history[begin_position], history[begin_position+1], begin_circle_id, history[end_position-1] , history[end_position] , end_circle_id);
	}
      }
    }
    for(int i = 0; i< circles.size(); i++){
      circles[i].print();
    }
  }

  


  void clearAllExits(){
    for(int i = 0; i < circles.size() ; i++){
      circles[i].clearExits();
    }
  }

  void saveExit(CartesianPoint begin1, CartesianPoint begin2, int begin_circle, CartesianPoint end1, CartesianPoint end2, int end_circle){
    //CartesianPoint exit_begin_p = getPointOnCircle(begin2, begin_circle);
    //CartesianPoint exit_end_p = getPointOnCircle(end1, end_circle);
    
    CartesianPoint exit_begin_p = getCrossingPointOnCircle(begin1, begin2, begin_circle);
    CartesianPoint exit_end_p = getCrossingPointOnCircle(end1, end2, end_circle);
    //cout << begin1.get_x() << " " << begin1.get_y() << " " << begin2.get_x() << " " << begin2.get_y() << " " << circles[begin_circle].getCenter().get_x() << " " << circles[begin_circle].getCenter().get_y() << " " << circles[begin_circle].getRadius() << " " << exit_begin_p.get_x() << " " << exit_begin_p.get_y() << " " << exit_begin_pc.get_x() << " " << exit_begin_pc.get_y() << endl;
    //cout << end1.get_x() << " " << end1.get_y() << " " << end2.get_x() << " " << end2.get_y() << " " << circles[end_circle].getCenter().get_x() << " " << circles[end_circle].getCenter().get_y() << " " << circles[end_circle].getRadius() << " " << exit_end_p.get_x() << " " << exit_end_p.get_y() << " " << exit_end_pc.get_x() << " " << exit_end_pc.get_y() << endl;
    //cout << "End exit begin/end" << endl;

    //cout << "In save exit  " << "Start circle : " << begin_circle << "End circle : " << end_circle << endl;
    FORRExit exit_begin ( exit_begin_p , end_circle );
    FORRExit exit_end ( exit_end_p , begin_circle );
    if(!circles[begin_circle].isExitAlreadyPresent(exit_begin))
       circles[begin_circle].addExit(exit_begin);
    if(!circles[end_circle].isExitAlreadyPresent(exit_end))
      circles[end_circle].addExit(exit_end);
    //cout << exit_begin_p.get_x() << " " << exit_begin_p.get_y() << " " ;
    //cout << exit_end_p.get_x() << " " << exit_end_p.get_y() << endl;
  }

  CartesianPoint getPointOnCircle(CartesianPoint pos, int circle_id){
    CartesianPoint center = circles[circle_id].getCenter();
    double radius = circles[circle_id].getRadius();
    double angle = atan2(pos.get_y() - center.get_y(), pos.get_x() - center.get_x()); 
    return CartesianPoint (center.get_x() + (radius * cos(angle)), center.get_y() + (radius * sin(angle))); 
  }

  CartesianPoint getCrossingPointOnCircle(CartesianPoint pos1, CartesianPoint pos2, int circle_id){
    Circle region = Circle(circles[circle_id].getCenter(), circles[circle_id].getRadius());
    return intersection_point(region, LineSegment(pos1, pos2));
  }

  // returns the position of the circle in which the point is located
  int pointInCircles(double x, double y){
    for(int i = 0; i < circles.size(); i++){
      if(circles[i].inCircle(x, y))
	return i;
    }
    return -1;
  }

  
  
 private:
  vector<FORRCircle> circles;
};


#endif
