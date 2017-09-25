/*
 * Map.h
 *
 *  Created on: June 17, 2017
 *      Author: Anoop Aroor
 */

#ifndef MAP_H_
#define MAP_H_

#include <vector>
#include <math.h>
#include <stdio.h>
#include <string>
#include "tinyxml.h"
#include <algorithm>

using namespace std;

class Wall{
	public:
	double x1;
	double y1;
	double x2;
	double y2;
};


class Map {
public:
  Map();
  Map(double, double);
  
  void addWall(double, double, double, double); 
  vector<Wall> getWalls() { return walls; }
  
  double getLength() { return length; }
  double getHeight() { return height; }
  
  bool isWithinBorders( double, double );
  bool isPathObstructed( double, double, double, double );
  bool isAccessible(double x, double y);
  bool isPointInBuffer(double x, double y); 

  vector< vector <bool> > getOccupancyGrid() {return occupancyGrid;}
  int getOccupancySize() { return occupancySize; }

  bool readMapFromXML(string);
  
  static double distance(double x1, double y1, double x2, double y2);
  double distanceFromWall(double x, double y, int wallIndex);
  double distanceFromSegment(double x1, double y1, double x2, double y2, double pointX, double pointY);
  
protected:
  vector<Wall> walls;
  vector< vector <bool> > occupancyGrid;
  int occupancySize;
  
  double length;
  double height;  
  
};

#endif /* MAP_H_ */
