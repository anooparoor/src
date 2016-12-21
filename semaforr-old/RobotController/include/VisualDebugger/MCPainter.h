/*
 * MCPainter.h
 *
 *  Created on: Jun 2, 2010
 *      Author: appleapple
 */

#ifndef MCPAINTER_H_
#define MCPAINTER_H_

#include <GL/gl.h>
#include <GL/glut.h>
#include <Map.h>
#include "Controller.h"
#include "MonteCarloDebugger.h"
//#include "Graph.h"

class MCPainter {
  void drawMarkerPatch(MapMarker,char,int,int,int,int);
  void drawWindowBorders(int wm, int hm);
public:
  MCPainter();
  
  void drawGrid(Map * map);
  void drawMarkers(Map * map);
  void drawWalls(Map * map); 
  void drawWallBuffers(Map * map);
  void drawDisplayWallBuffers(Map * map);
  void drawParticles(MonteCarloDebugger * debugger);
  void drawObservations(MonteCarloDebugger * debugger, Localization * itl);
  void drawPosition(Localization * itl, Position realPosition);
  void drawRegions(std::vector<FORRCircle> c);
  void drawGates(std::vector<Gate> g);
  void drawConveyors(FORRWaypoints  &conveyors, int height, int length);
  void drawTrail(vector<TrailMarker> &v);
  void drawAllTrails(FORRTrails &t);


  
  // for path planner
  //void drawNodes(Graph * g);
  //void drawEdges(Graph * g, Map * map);
  //void drawEdges(Graph * g);
  //void drawTempObstacles(Graph * g); 
  //void drawSource(Graph * g, int x, int y); 
  void drawTarget(int x, int y);
  //void drawPath(Localization*, PathPlanner*, Graph*); 

  // for displaying camera image
  void drawCameraImage(Localization * itl);

  // for particle orientations
  void drawParticleTheta(MonteCarloDebugger * debugger);
  void drawPlayerBlobs(Localization * itl); 
  void drawObservationBlobs(Localization * itl);

  // for the fog. just testing for now. 
  void drawFogOfExploration();

  // Slavisa added Sep 14 2013
  void drawFORRTarget(int x, int y);
};

#endif /* MCPAINTER_H_ */
