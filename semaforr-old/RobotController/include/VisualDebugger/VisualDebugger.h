#ifndef VISUAL_DEBUGGER_H
#define VISUAL_DEBUGGER_H

#include "MCPainter.h"
#include "Controller.h"
#include "MonteCarloDebugger.h"

#include "Task.h"

class VisualDebugger {
  Map * myMap;
  Controller * robot; 

  Localization * itl ; 
  //  PathPlanner * planner; 
  //Graph * g; 
  MonteCarlo * mc; 
  MonteCarloDebugger * debugger; 

  bool usingStage;
  int keyboardCtrl; 

  char motionModifier;

 public: 
  //VisualDebugger(Map*, Controller*); 
  VisualDebugger(Map*, Controller*, bool); 
  MonteCarloDebugger* getDebugger() { return debugger; }
  
  void reshape(int, int); 
  void reshapeOrientations(int w, int h);
  void keyboard(unsigned char, int, int); 
  void keyboardSpecial(int, int, int); 
  void mouse(int, int, int, int); 
  void draw(void); 
  void drawFog(void);

  void reshapeCameraWindows(int, int); 
  void drawPlayerBlobs(void); 
  void drawObservationBlobs(void); 
  void drawParticleOrientations(void); 

  // util functions
  int getWinX(int); 
  int getMapX(int); 
  int getWinY(int); 
  int getMapY(int);
};

#endif
