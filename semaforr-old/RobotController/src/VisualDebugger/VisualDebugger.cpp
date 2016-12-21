#include <iomanip>
#include "VisualDebugger.h"
#include <cmath>
#define DRAW_SC true

VisualDebugger::VisualDebugger(Map * m, Controller * c, bool uS): myMap(m), robot(c), usingStage(uS) {
  itl = robot->getLocalization();
  mc = itl->getMonteCarlo();
  debugger = itl->getMCDebugger();
  mc->setDebugger(debugger);
  
  keyboardCtrl = KEY_CTRL_STEP;
  motionModifier = ' ';
}

//called when the window changes position and size
void VisualDebugger::reshape(int w, int h) {
  glViewport(0, 0, (GLsizei) w, (GLsizei) h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluOrtho2D(-10, myMap->getLength() + 10, -10, myMap->getHeight() + 10);
}

//called when the window changes position and size
void VisualDebugger::reshapeOrientations(int w, int h) {
  glViewport(0, 0, (GLsizei) w, (GLsizei) h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluOrtho2D(-10, glutGet(GLUT_WINDOW_WIDTH) + 10, -10, glutGet(GLUT_WINDOW_HEIGHT) + 10);
}

// called to reshape the subwindows that contains camera image
void VisualDebugger::reshapeCameraWindows(int w, int h){

}


int VisualDebugger::getWinX(int x) {
  int wx = ( (double) (glutGet(GLUT_WINDOW_WIDTH) - 40)/ (double) myMap->getLength() ) * x + 20;
  return wx;
}

int VisualDebugger::getWinY(int y) {
  int wy = ( myMap->getHeight() - y ) * ( (double)( glutGet(GLUT_WINDOW_HEIGHT)- 20 ) / (double) myMap->getHeight() ) + 10;
  return wy;
}

int VisualDebugger::getMapX(int x) {
  int mx = ( x - 20 ) * ( (double) myMap->getLength()/ (double) (glutGet(GLUT_WINDOW_WIDTH)-40));
  return mx;
}

int VisualDebugger::getMapY(int y){
  int my = myMap->getHeight() - (( y - 10 ) * ((double) myMap->getHeight() /(double) (glutGet(GLUT_WINDOW_HEIGHT) - 20) ));
  return my;
}


// this bit doesn't work. maybe due to graphix card requirement?
void VisualDebugger::drawFog(void){
    MCPainter painter;
    glClear(GL_COLOR_BUFFER_BIT);
    glClearColor(0.25,0.25,0.25,0.25); // set current color to black

    // call some functioni from MCPainter to draw the fog on top of the map
    painter.drawFogOfExploration();
    glutSwapBuffers();
}

void VisualDebugger::draw(void) {

    //  cerr << "VisualDebugger::draw()" << endl;

  MCPainter painter;
  glClear(GL_COLOR_BUFFER_BIT);
  glClearColor(1,1,1,0); // set current color to white

  painter.drawMarkers(myMap);
  painter.drawObservations(debugger, itl);       // draws the lines from position to markers

  // Slavisa added
  // How should we draw target in the debugger window?
  // let's try brute force
  Beliefs *beliefs = robot->getBeliefs();

  //modified to draw a cross shape instead of a dot
  if(beliefs->getCurrentTask() != NULL){
    painter.drawTarget((beliefs->getCurrentTask())->getX(), (beliefs->getCurrentTask())->getY());
    painter.drawTarget(beliefs->getCurrentTask()->getX()+4, beliefs->getCurrentTask()->getY());
    painter.drawTarget(beliefs->getCurrentTask()->getX()-4, beliefs->getCurrentTask()->getY());
    painter.drawTarget(beliefs->getCurrentTask()->getX(), beliefs->getCurrentTask()->getY()+4);
     painter.drawTarget(beliefs->getCurrentTask()->getX(), beliefs->getCurrentTask()->getY()-4);

  }

  painter.drawWalls(myMap);
  painter.drawDisplayWallBuffers(myMap);
  //painter.drawTempObstacles(g);
  
  if ( !usingStage ){
    painter.drawParticles(debugger);
  }
  painter.drawPosition(itl, Position(0, 0, 0));  // draws the position of the robot
  

  if(DRAW_SC){
    //draw the spatial cognition
    painter.drawRegions(beliefs->abstractMap.getCircles());
    // painter.drawGates(beliefs->gates.getGates());
    FORRWaypoints f = beliefs->Waypoints;
  
    FORRTrails t = beliefs->trail_vectors;
    int map_height = beliefs->getMap()->getHeight();
    int map_width = beliefs->getMap()->getLength();
    painter.drawConveyors(f, map_height, map_width);
    painter.drawAllTrails(t);
    //if(beliefs->trail_vectors.getChosenTrail() != -1){
    //    vector<TrailMarker> single_trail = beliefs->trail_vectors.getTrail(beliefs->trail_vectors.getChosenTrail());
    //  painter.drawTrail(single_trail);
    //}
  }
  glutSwapBuffers();
}


void VisualDebugger::drawPlayerBlobs(void){
    MCPainter painter;
    glClearColor(1,1,1,0); // set current color to black
    glClear(GL_COLOR_BUFFER_BIT);
    //painter.drawCameraImage(itl);
    if (!usingStage)
        painter.drawPlayerBlobs(itl);

    glutSwapBuffers();
}

void VisualDebugger::drawObservationBlobs(void){
    MCPainter painter;
    glClearColor(1,1,1,0); // set current color to black
    glClear(GL_COLOR_BUFFER_BIT);
    //painter.drawCameraImage(itl);
    painter.drawObservationBlobs(itl);
    glutSwapBuffers();
}

void VisualDebugger::drawParticleOrientations(void){
    MCPainter painter;
    glClear(GL_COLOR_BUFFER_BIT);
    glClearColor(1,1,1,0); // set current color to white
    painter.drawParticleTheta(debugger);
    glutSwapBuffers();
}
