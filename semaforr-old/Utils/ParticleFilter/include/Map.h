/*
 * Map.h
 *
 *  Created on: Dec 21, 2008
 *      Author: richardmarcley
 */

#ifndef MAP_H_
#define MAP_H_

#include <vector>
#include <math.h>
#include <stdio.h>
#include <string>
#include "MapMarker.h"
#include "MapWall.h"
#include "MapVirtualWall.h"
#include "MapWallBuffer.h"
#include "Position.h"

using std::vector;

class Map {
public:
  Map();
  Map(int, int, int);
  
  void addMarker(MapMarker marker);
  vector<MapMarker> getMarkerById(string id);
  MapMarker getMarker(int index);
  vector<MapMarker> getMarkers() { return markers; }
  /* added for walls */
  void addWall(string, int, int, int, int); 
  vector<MapWall> getWallById(string id);
  MapWall getWall(int index); 
  vector<MapWall> getWalls() { return walls; }
  vector<MapWallBuffer> getDisplayWallBuffers(){ return displayBuffers; }
  void updateDisplayWallBuffer(MapWall);
  void updateDisplayWallBuffers();
  /* add for walls ends */ 
  
  /* For camera blind spots. Represented as line segments */
  void addVirtualWall(string, int, int, int, int);

  int getLength() { return length; }
  int getHeight() { return height; }
  int getBufferSize() { return bufferSize; }

  bool isWithinBorders( int, int );
  bool isAccessible(int, int);
  bool isPathObstructed( int, int, int, int );
  bool isPathCrossesBuffer(int, int, int, int);        
  bool isPointInBuffer(int, int);

protected:
  vector<MapMarker> markers;
  vector<MapWall> walls;
  vector<MapVirtualWall> virtualWalls; 
  vector<MapWallBuffer> displayBuffers; 

  int length;
  int height;  
  int bufferSize;
};
#endif /* MAP_H_ */
