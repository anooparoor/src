/*******************
FORRWaypoints.h
Implements the FORRwaypoint class, which acts as a grid on top of the map
to count the number of cleaned path points that lie within a sufficiently coarse 
overlay.  These act as pseudo-bases, or "waypoints", which aim to guide the 
robot to useful points on the map that promote fast travel

Written by Matthew Evanusa, November 2014
******************/

#ifndef FORRWAYPOINTS_H
#define FORRWAYPOINTS_H


#include <iostream>
#include <FORRGeometry.h>
#include <vector>
#include <fstream>
#include <utility>

using namespace std;

class FORRWaypoints{

 public:
  //the grid overlay itself that stores the counter of frequented points along cleaned paths
  vector< vector<int> > waypoints;
 
  //variable that allows different sizes of granularity for the overlay
  //for the future, will be set to the minimum diameter of the regions, 
  //for now set arbitrarily to 20 distance-units
  int granularity;
  
  void setGranularity(int value){ granularity = value;}
  
  int boxes_height, boxes_width;
  int map_height, map_width;
  //reads in the file (for now, it's called paths.conf) that contains the cleaned paths
  //and "votes" for each partition
  void populateGrid();
  
  


  //newer version of populateGrid() that takes into account the entire line
  //in incrementing the cells, not just the endpoints
  void populateGridFromFullLine();

  //for the future, if we want to threshold the values of the grid
  int threshold; 


  //outputs to file
  void outputWaypoints(string filename);



  //to be called to partition the grid according to the granularity
  //Width and height are passed in from Controller.cpp, which has access
  //to map dimensions.  Initializes all values to 0
  void setGrid(int width, int height);
  
  
  //returns the grid overlay postion next to the current position that
  //has the maximum value, given the adjacent non-diagonal positions
  pair<int, int> getNextGridPosition(double curr_x, double curr_y);
  

  //returns the value at the grid [x][y] given map coordinates 
  int getGridValue(double map_x, double map_y);

  int getMaxGridValue();
  int max_grid_value;

  //gets grid value directly from x, y of grid, used for drawing in VisualDebugger
  int getGridValueDirect(int grid_x, int grid_y){ return waypoints[grid_x][grid_y];}
  
  pair<int,int> convertToGridCoordinates(double x, double y);

  //default constructor, sets granularity to 20
  FORRWaypoints(){  
    granularity = 20;
  }
  

  //clears grid after each population to repopulate
  void clearWaypoints();

};


#endif
