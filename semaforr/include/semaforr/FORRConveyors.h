/*******************
FORRConveyors.h
Implements the FORRconveyor class, which acts as a grid on top of the map
to count the number of cleaned path points that lie within a sufficiently coarse 
overlay.  These act as pseudo-bases, or "conveyors", which aim to guide the 
robot to useful points on the map that promote fast travel

Written by Matthew Evanusa, November 2014 , Edited by Anoop Aroor March 2017
******************/

#ifndef FORRCONVEYORS_H
#define FORRCONVEYORS_H

#include <FORRGeometry.h>
#include "Position.h"

#include <vector>
#include <utility>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>

using namespace std;

class FORRConveyors{

 public:
  
  //Constructor
  FORRConveyors(double width, double height, double granularity_para){
	granularity = granularity_para;  
  	cout << "Initialize conveyors by setting up the grid"<<endl;
  	boxes_width = width/granularity;
  	boxes_height = height/granularity;
  	map_height = height;
  	map_width = width;
  	for(int i = 0; i < boxes_width; i++){
    		vector<int> col;
    		for(int j = 0; j < boxes_height; j++){
      			col.push_back(0);
    		}
    		conveyors.push_back(col);
  	}
  	cout << "Exit setgrid."<<endl;
  	max_grid_value = 0;
  }

  //populate grid from position history
  void populateGridFromPositionHistory(vector<Position> *pos_hist);

  //populate grid from position history
  void populateGridFromTrailTrace(vector<CartesianPoint> trails_points);

  //populate grid from a line segment
  pair<int,int> updateGridFromLine(double x1, double y1, double x2, double y2, pair<int,int> prev);

  //outputs to file
  void outputConveyors(string filename);
  
  //returns the grid overlay postion next to the current position that
  //has the maximum value, given the adjacent non-diagonal positions
  pair<int, int> getNextGridPosition(double curr_x, double curr_y);
  
  //returns the value at the grid [x][y] given map coordinates 
  int getGridValue(double map_x, double map_y);

  int getMaxGridValue();
  
  pair<int,int> convertToGridCoordinates(double x, double y);

  //returns the average grid value of the cell [x][y] and its surrounding cells
  double getAverageGridValue(double map_x, double map_y);

  //clears grid after each population to repopulate
  void clearConveyors();

  int getGranularity(){return granularity;}
  int getMapHeight(){return map_height;}
  int getMapWidth(){return map_width;}
  int getBoxHeight(){return boxes_height;}
  int getBoxWidth(){return boxes_width;}
  vector< vector<int> > getConveyors(){return conveyors;}

  private:
  	//the grid overlay itself that stores the counter of frequented points along cleaned paths
  	vector< vector<int> > conveyors;
 
  	//variable that allows different sizes of granularity for the overlay
  	//for the future, will be set to the minimum diameter of the regions, 
  	//for now set arbitrarily to 20 distance-units
  	int granularity;
	int max_grid_value;

	int boxes_height, boxes_width;
  	int map_height, map_width;

};


#endif
