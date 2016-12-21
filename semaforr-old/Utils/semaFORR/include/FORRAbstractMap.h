/************************************************
FORRAbstractMap.h 
This file contains the class which contains information about the map that FORR learns and uses

Written by Anoop Aroor, 2014
**********************************************/

#ifndef FORRABSTRACTMAP_H
#define FORRABSTRACTMAP_H

#include <iostream>
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

  void readCirclesFromFile(char *filename){
    string line;
    ifstream circlestream (filename);
    if(circlestream.is_open()){
      while(getline(circlestream, line)){
	FORRCircle circle;
	string buf;
	stringstream ss(line);
	vector<double> line_data;
	while(ss >> buf){
	  line_data.push_back(atof(buf.c_str()));       
	}
	circle.setCenter(CartesianPoint(line_data[0],line_data[1]));
	circle.setRadius(line_data[2]);
	vector<FORRExit> exits;
	for(int i = 3; i< line_data.size() ; i+=3){
	  //cout << "FORRAbstractMap::readCirclesFromFile >> " << line_data[i] << " " << line_data[i+1] << " " << line_data[i+2] <<  endl;
	  exits.push_back(  FORRExit(  CartesianPoint(line_data[i],line_data[i+1]) ,(int) (line_data[i+2])  )  );
	}
	circle.setExits(exits);
	circles.push_back(circle);
      }
    }
  }
  

  void saveCirclesIntoFile(string filename){
    ofstream circlestream;
    circlestream.open(filename.c_str());
    for(int i = 0; i < circles.size(); i++){
      circlestream << circles[i].getCenter().get_x() <<" " << circles[i].getCenter().get_y() << " " << circles[i].getRadius() << " " << circles[i].getIsLeaf();
      vector<FORRExit> exits = circles[i].getExits();
      for(int j = 0; j< exits.size(); j++){
	circlestream << " " << exits[j].getExitPoint().get_x() << " " << exits[j].getExitPoint().get_y() <<" " << exits[j].getExitCircle(); 
      }
      circlestream << endl;
    }
    circlestream.close();
  }


  void save(vector<double> wallDistanceVector, Position currentPosition){
    wallDistanceHis.push_back(wallDistanceVector);
    positionHis.push_back(currentPosition);
  }

			   
  void learnRegions(){
    cout << "In learning regions" << endl;
    CartesianPoint current_point;
    FORRCircle current_circle;
    for(int k = 0 ; k < wallDistanceHis.size(); k++){
      vector <double> wallDistanceVector = wallDistanceHis[k];
      Position current_position = positionHis[k];

      double radius = 10000;
      int direction = -1;
      for(int i = 0; i< wallDistanceVector.size(); i++){
	//cout << "wall distance " << wallDistanceVector[i] << endl;
	if (wallDistanceVector[i] < radius){
	  radius = wallDistanceVector[i];
	  direction = i;
	}
      }
     
      current_point = CartesianPoint(current_position.getX(), current_position.getY());
      current_circle = FORRCircle(current_point, radius);
      for(int j = 0 ; j < wallDistanceHis.size(); j++){
	vector <double> nextWallDistanceVector = wallDistanceHis[j];
	Position next_position = positionHis[j];
	// if next position in still inside the current_circle update current_circle radius
	if(current_circle.inCircle(next_position.getX(), next_position.getY()) && j != k){
	  double next_radius = 10000;
	  int next_direction = -1;
	  for(int i = 0; i< nextWallDistanceVector.size(); i++){
	    //cout << "wall distance " << wallDistanceVector[i] << endl;
	    if (nextWallDistanceVector[i] < next_radius){
	      next_radius = nextWallDistanceVector[i];
	      next_direction = i;
	    }
	  }
	  double rotations[] = {0, .1548, -.1548, .3048, -.3048, .65, -.65, 1.3, -1.3, 3.39, -3.39};
	  double short_angle = next_position.getTheta() + rotations[next_direction];
	  double x = next_position.getX() + next_radius * cos(short_angle);
	  double y = next_position.getY() + next_radius * sin(short_angle);
	  double dist = Utils::get_euclidian_distance(current_circle.getCenter().get_x() , current_circle.getCenter().get_y() , x , y);
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
	double rotations[] = {0, .1548, -.1548, .3048, -.3048, .65, -.65, 1.3, -1.3, 3.39, -3.39};
	double short_angle = current_position.getTheta() + rotations[direction];
	double x = current_position.getX() + radius * cos(short_angle);
	double y = current_position.getY() + radius * sin(short_angle);
	double dist = Utils::get_euclidian_distance(circles[robotRegion].getCenter().get_x(),circles[robotRegion].getCenter().get_y() ,x, y);
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
    wallDistanceHis.clear();
    positionHis.clear();
    cout << "Exit learning regions"<<endl;
  }

  void learnExits_new(vector< vector<CartesianPoint> > run_trace){
    
    // learning gates between different circles
    clearAllExits();
    // for every position in the position history vector .. check if a move is from one circle to another and save it as gate
    //cout << "In learning exits: size of trace is " << run_trace.size() << endl;
    for(int k = 0; k < run_trace.size() ; k++){
      vector<CartesianPoint> history = run_trace[k]; 
      //cout << "Learning exits between circles" << endl;
      
      for(int j = 0; j < history.size()-1; j++){  
	int circle_one = -1, circle_two = -1;
	cout << history[j].get_x() << " " << history[j].get_y() << endl;
	circle_one = pointInCircles(history[j].get_x(), history[j].get_y());
	circle_two = pointInCircles(history[j+1].get_x(), history[j+1].get_y());
	//cout << "circle_one : " << circle_one << endl;
	//cout << "circle_two : " << circle_two << endl;
	if(circle_one != -1 && circle_two != -1 && circle_one != circle_two){
	  //cout << "Link found" << endl;
	  saveExit(history[j], history[j+1], circle_one, history[j], history[j+1], circle_two);
	}
      }
    }
    for(int i = 0; i< circles.size(); i++){
      //circles[i].print();
    }

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
    //cout << begin1.get_x() << " " << begin1.get_y() << " " << begin2.get_x() << " " << begin2.get_y() << endl;
    //cout << end1.get_x() << " " << end1.get_y() << " " << end2.get_x() << " " << end2.get_y() << endl;
    CartesianPoint exit_begin_p = getPointOnCircle(begin2, begin_circle);
    CartesianPoint exit_end_p = getPointOnCircle(end1, end_circle);
    
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
  // each value in the list denotes a list of circles and possible exits
  vector< vector <double> > wallDistanceHis;
  vector< Position> positionHis;
};


#endif
