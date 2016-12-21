/*
 * MCPainter.cpp
 *
 *  Created on: Jun 2, 2010
 *      Author: appleapple
 */

#include "MCPainter.h"

MCPainter::MCPainter() {
  // TODO Auto-generated constructor stub

}

void MCPainter::drawFogOfExploration(){
  glBegin(GL_POLYGON); 
  {
    glVertex2i(100, 100);
    glVertex2i(100, 300);
    glVertex2i(300, 300);
    glVertex2i(300, 100);
  }
}

void MCPainter::drawGrid(Map * map) {
  glBegin(GL_LINES);
  {
    glColor3f(.75, .75, .75);   
    for (int i = 0; i < 10; i++) {
      glVertex2f(0, i * map->getHeight() / 10.0);
      glVertex2f(map->getLength(), i * map->getHeight() / 10.0);
      glVertex2f(i * map->getLength() / 10.0, 0);
      glVertex2f(i * map->getLength() / 10.0, map->getHeight());
    }
  }
  glEnd();
}

void MCPainter::drawMarkerPatch(MapMarker m, char color, int x1, int x2, int y1, int y2){
  glBegin(GL_POLYGON);
  if ( color == 'p' )
    glColor3f(1,0,1);
  else if ( color == 'b' )
    glColor3f(0,0,1);
  else if ( color == 'o' )
    glColor3f(1,0.5,0);
  else if ( color == 'y' )
    glColor3f(1,1,0);

  glVertex2i(m.getX() + x1, m.getY() + y1);
  glVertex2i(m.getX() + x1, m.getY() + y2);
  glVertex2i(m.getX() + x2, m.getY() + y2);
  glVertex2i(m.getX() + x2, m.getY() + y1);
  
  glEnd();
}

void MCPainter::drawMarkers(Map * map) {
  vector<MapMarker> markers = map->getMarkers();
  for (unsigned int i = 0; i < markers.size(); i++) {
    // corner markers
    if (markers[i].getId() == "p/y"){
      drawMarkerPatch(markers[i], 'p', -2,0,-2,2);
      drawMarkerPatch(markers[i], 'y', 0,2,-2,2);
    }
    else if (markers[i].getId() == "y/p"){
      drawMarkerPatch(markers[i], 'y', -2,0,-2,2);
      drawMarkerPatch(markers[i], 'p', 0,2,-2,2);
    }
    else if (markers[i].getId() == "y/b"){
      drawMarkerPatch(markers[i], 'y', -2,0,-2,2);
      drawMarkerPatch(markers[i], 'b', 0,2,-2,2);
    }
    else if (markers[i].getId() == "b/y"){
      drawMarkerPatch(markers[i], 'b', -2,0,-2,2);
      drawMarkerPatch(markers[i], 'y', 0,2,-2,2);
    }
    
    // room markers
    else if (markers[i].getId() == "b/p/y"){
      drawMarkerPatch(markers[i], 'b', -3,-1,-2,2);
      drawMarkerPatch(markers[i], 'p', -1,1,-2,2);
      drawMarkerPatch(markers[i], 'y', 1,3,-2,2);
    }
    else if (markers[i].getId() == "b/y/p"){
      drawMarkerPatch(markers[i], 'b', -3,-1,-2,2);
      drawMarkerPatch(markers[i], 'y', -1,1,-2,2);
      drawMarkerPatch(markers[i], 'p', 1,3,-2,2);
    }
    else if (markers[i].getId() == "b/o/p"){
      drawMarkerPatch(markers[i], 'b', -3,-1,-2,2);
      drawMarkerPatch(markers[i], 'o', -1,1,-2,2);
      drawMarkerPatch(markers[i], 'p', 1,3,-2,2);      
    }
    else if (markers[i].getId() == "b/p/o"){
      drawMarkerPatch(markers[i], 'b', -3,-1,-2,2);
      drawMarkerPatch(markers[i], 'p', -1,1,-2,2);
      drawMarkerPatch(markers[i], 'o', 1,3,-2,2);
    }
    else if (markers[i].getId() == "b/o/y"){
      drawMarkerPatch(markers[i], 'b', -3,-1,-2,2);
      drawMarkerPatch(markers[i], 'o', -1,1,-2,2);
      drawMarkerPatch(markers[i], 'y', 1,3,-2,2);
    }
    else if (markers[i].getId() == "b/y/o"){
      drawMarkerPatch(markers[i], 'b', -3,-1,-2,2);
      drawMarkerPatch(markers[i], 'y', -1,1,-2,2);
      drawMarkerPatch(markers[i], 'o', 1,3,-2,2);
    }

    // corridor markers
    else if (markers[i].getId() == "p/o/y"){
      drawMarkerPatch(markers[i], 'p', -3,-1,-2,2);
      drawMarkerPatch(markers[i], 'o', -1,1,-2,2);
      drawMarkerPatch(markers[i], 'y', 1,3,-2,2);      
    }
    else if (markers[i].getId() == "y/p/o"){
      drawMarkerPatch(markers[i], 'y', -3,-1,-2,2);
      drawMarkerPatch(markers[i], 'p', -1,1,-2,2);
      drawMarkerPatch(markers[i], 'o', 1,3,-2,2);
    }
    else if (markers[i].getId() == "y/o/p"){
      drawMarkerPatch(markers[i], 'y', -3,-1,-2,2);
      drawMarkerPatch(markers[i], 'o', -1,1,-2,2);
      drawMarkerPatch(markers[i], 'p', 1,3,-2,2);
    }
    else if (markers[i].getId() == "p/y/o"){
      drawMarkerPatch(markers[i], 'p', -3,-1,-2,2);
      drawMarkerPatch(markers[i], 'y', -1,1,-2,2);
      drawMarkerPatch(markers[i], 'o', 1,3,-2,2);
    }

    // entrance markers
    else if (markers[i].getId() == "b"){
      drawMarkerPatch(markers[i], 'b', -1,1,-2,2);
    }
    else if (markers[i].getId() == "o"){
      drawMarkerPatch(markers[i], 'o', -1,1,-2,2);      
    }
  }
}


void MCPainter::drawWalls(Map * map){
  glBegin(GL_LINES);
  {
    vector<MapWall> walls = map->getWalls();
    vector<MapWall>::iterator iter; 
    for ( iter = walls.begin(); iter != walls.end(); iter++ ){
      //glColor3f(1,1,1);   // if background is white
      glColor3f(0,0,0);
      glVertex2f(iter->getX0(), iter->getY0()); 
      glVertex2f(iter->getX1(), iter->getY1());
    }
  }
  glEnd();
}

void MCPainter::drawDisplayWallBuffers(Map * map){
  glBegin(GL_LINES);
  {
    vector<MapWallBuffer> displayBuffers = map->getDisplayWallBuffers();
    vector<MapWallBuffer>::iterator iter; 
    for ( iter = displayBuffers.begin(); iter != displayBuffers.end(); iter++ ){
      glColor3f(0,0,1);
      glVertex2f(iter->getX0(), iter->getY0()); 
      glVertex2f(iter->getX1(), iter->getY1());
    }
  }
  glEnd();
}

/*
void MCPainter::drawNodes(Graph * g){
  if ( g->numNodes() > 0 ) {
    glBegin(GL_LINES);
    {
      vector<Node*> nodes = g->getNodes();
      vector<Node*>::iterator iter; 
      for( iter = nodes.begin(); iter != nodes.end(); iter++ ) {
	if ( !(*iter)->getInBuffer() )
	  glColor3f(0.95,0.95,0.95);       // if bg white
	else
	  glColor3f(1,0.75,0.75);       
	glVertex2f((*iter)->getX()-1, (*iter)->getY()-1); 
	glVertex2f((*iter)->getX()-1, (*iter)->getY()+1); 
	glVertex2f((*iter)->getX()-1, (*iter)->getY()+1); 
	glVertex2f((*iter)->getX()+1, (*iter)->getY()+1); 
	glVertex2f((*iter)->getX()+1, (*iter)->getY()+1); 
	glVertex2f((*iter)->getX()+1, (*iter)->getY()-1); 
	glVertex2f((*iter)->getX()+1, (*iter)->getY()-1); 
	glVertex2f((*iter)->getX()-1, (*iter)->getY()-1); 
      }
    }
    glEnd();
  }
}

void MCPainter::drawEdges(Graph * g){
  glBegin(GL_LINES);
  {
    vector<Node*> nodes = g->getNodes();
    vector<Node*>::iterator it; 
    for( it = nodes.begin(); it != nodes.end(); it++ ) {
      vector<Edge> edges = (*it)->getUsableNodeEdges();
      vector<Edge>::iterator iter; 
      for( iter = edges.begin(); iter != edges.end(); iter++ ) {
	glColor3f(0.1,0.1,0.1);  
	Node n1 = g->getNode(iter->getFrom()); 
	Node n2 = g->getNode(iter->getTo());
	if ( !n1.getInBuffer() && !n2.getInBuffer() ) {
	  glColor3f(0.8,0.8,0.8);  // if bg white
	}
	else if ( n1.getInBuffer() || n2.getInBuffer() ) {
	  glColor3f(1,0.9,0.9);       // if bg white
	}
	else if ( n1.getInBuffer() && n2.getInBuffer() ) {
	  glColor3f(1,0.3,0.3);       // if bg white
	}
	glVertex2f(n1.getX(), n1.getY()); 
	glVertex2f(n2.getX(), n2.getY()); 
      }
    }
    glEnd();
  }
}
*/
/*
void MCPainter::drawEdges(Graph * g, Map * map){
  if ( g->numEdges() > 0 ) {
    glBegin(GL_LINES);
    {
      vector<Edge> edges = g->getEdges();
      vector<Edge>::iterator iter; 
      for( iter = edges.begin(); iter != edges.end(); iter++ ) {
	glColor3f(0.98,0.98,0.98);       // if bg white
	Node n1 = g->getNode(iter->getFrom()); 
	Node n2 = g->getNode(iter->getTo());
	
	if ( !n1.getInBuffer() && !n2.getInBuffer() ) {
	  glColor3f(0.98,0.98,0.98);       // if bg white
	}
	else if ( n1.getInBuffer() || n2.getInBuffer() ) {
	  glColor3f(1,0.8,0.8);       // if bg white
	}
	else if ( n1.getInBuffer() && n2.getInBuffer() ) {
	  glColor3f(1,0.3,0.3);       // if bg white
	}
	//if ( map->isPathCrossesBuffer(n1.getX(), n1.getY(), n2.getX(), n2.getY()) )
	//if ( iter->getCost() > Utils::get_euclidian_distance(n1.getX(), n1.getY(), n2.getX(), n2.getY()) )
	//  glColor3f(1,0.3,0.3);       // if bg white
	//else
	//  glColor3f(0.9,0.9,0.9);       // if bg white
	glVertex2f(n1.getX(), n1.getY()); 
	glVertex2f(n2.getX(), n2.getY()); 
      }
    }
    glEnd();
  }
}
*/

/*
void MCPainter::drawTempObstacles(Graph * g){
  vector<Edge*> edges = g->getEdges();
  vector<Edge*>::iterator iter; 
  for( iter = edges.begin(); iter != edges.end(); iter++ ) {
    if ( (*iter)->getTempCost() > 0 ) {     
      glBegin(GL_LINES);
      {
	Node n1 = g->getNode((*iter)->getFrom()); 
	Node n2 = g->getNode((*iter)->getTo());
	
	double tc = (*iter)->getTempCost() ; 
	double c = (*iter)->getCost() - tc ;
	
	if ( ( tc / c ) >= 1 ) 
	  glColor3f(1,0.8,0.8);       // If white
	if ( ( tc / c ) >= 2 ) 
	  glColor3f(1,0.3,0.3);       // If white
	
	glVertex2f(n1.getX(), n1.getY()); 
	glVertex2f(n2.getX(), n2.getY()); 
      }
      glEnd();
    }
  }   
}

void MCPainter::drawSource(Graph * g, int x, int y){
  glBegin(GL_POLYGON);
  if ( g->getMap()->isWithinBorders(x,y) ) {
    glColor3f(0,1,0);
    glVertex2i(x-2, y-2); 
    glVertex2i(x-2, y+2); 
    glVertex2i(x+2, y+2); 
    glVertex2i(x+2, y-2); 
  }
  glEnd();
}
*/

void MCPainter::drawTarget(int x, int y){
  glBegin(GL_POLYGON);
  glColor3f(1,0,0);
  glVertex2i(x-2, y-2); 
  glVertex2i(x-2, y+2); 
  glVertex2i(x+2, y+2); 
  glVertex2i(x+2, y-2); 
  glEnd();
}

/*

void MCPainter::drawPath(Localization * itl, PathPlanner * planner, Graph * g) {

  list<int> nodes = planner->getPath();

  Position pos = itl->getPosition();

  if ( !nodes.empty() ) {
    
    // draw a line between current position to first node in graph
    glBegin(GL_LINES); 
    glColor3f(0.5,0.5,0);
    glVertex2f(pos.getX(), pos.getY()); 
    glVertex2f(g->getNode(nodes.front()).getX(), g->getNode(nodes.front()).getY()); 
    glEnd();
    
    // draw the lines between nodes
    list<int>::iterator iter;
    for( iter = nodes.begin(); iter != nodes.end(); ) {
      int f = *iter; 
      int x = g->getNode(f).getX();
      int y = g->getNode(f).getY();
      glBegin(GL_POLYGON);
      glColor3f(0.85,0.85,0);
      glVertex2i(x-1, y-1); 
      glVertex2i(x-1, y+1); 
      glVertex2i(x+1, y+1); 
      glVertex2i(x+1, y-1); 
      glEnd();
      
      if( ++iter != nodes.end() ){
	glBegin(GL_LINES);
	glColor3f(0.5,0.5,0);
	glVertex2f(g->getNode(f).getX(), g->getNode(f).getY()); 
	glVertex2f(g->getNode(*iter).getX(), g->getNode(*iter).getY()); 
	glEnd();
      }
    }
    
    // draw a line between the last node to target
    glBegin(GL_LINES); 
    glColor3f(0.5,0.5,0);
    glVertex2f(g->getNode(nodes.back()).getX(), g->getNode(nodes.back()).getY()); 
    glVertex2f(planner->getTarget().getX(), planner->getTarget().getY()); 
    glEnd();
  }
  else {
    // draw a line between the last node to target
    glBegin(GL_LINES); 
    glColor3f(0.5,0.5,0);
    glVertex2f(pos.getX(), pos.getY()); 
    glVertex2f(planner->getTarget().getX(), planner->getTarget().getY()); 
    glEnd();
  }
}
*/
void MCPainter::drawParticles(MonteCarloDebugger * debugger) {
  
  glBegin(GL_POINTS);
  {
    // glVertex2f(0, 0);
    //glVertex2f(150, 150);
    for (unsigned int i = 0; i < debugger->particles.size(); i++) {
      double c = debugger->particles[i].probability;
      glColor3f(.5*(1-c), 1 *c, 0);
      Position p = debugger->particles[i].getPosition();
      glVertex2f(p.getX(), p.getY());
    }
  }
  
  // to plot the particles as lines to see the orientation
  /*
  glBegin(GL_LINES);
  {
    for (int i = 0; i < debugger->particles.size(); i++) {
      double c = debugger->particles[i].probability;
      glColor3f(.5*(1-c), 1 *c, 0);
      Position p = debugger->particles[i].getPosition();
      int len = 5; 
      double xp = p.getX() + len * cos(p.getTheta()) ;
      double yp = p.getY() + len * sin(p.getTheta()) ;
      glVertex2f(p.getX(), p.getY()); 
      glVertex2f(xp,yp);
    } 
  }
  */
  glEnd();
}

void MCPainter::drawParticleTheta(MonteCarloDebugger * debugger){
  // draw x, y coordinates
  glBegin(GL_LINES); 
  {
    glColor3f(1.0,0.0,0.0); 
    glVertex2f( glutGet(GLUT_WINDOW_WIDTH)/2, 0); 
    glVertex2f( glutGet(GLUT_WINDOW_WIDTH)/2,  glutGet(GLUT_WINDOW_HEIGHT));
  }
  glEnd(); 
  
  glBegin(GL_LINES); 
  {
    glColor3f(1.0,0.0,0.0); 
    glVertex2f(0,  glutGet(GLUT_WINDOW_HEIGHT)/2); 
    glVertex2f( glutGet(GLUT_WINDOW_WIDTH),  glutGet(GLUT_WINDOW_HEIGHT)/2); 
  }
  glEnd(); 

  glBegin(GL_POINTS); 
  {
    for (unsigned int i = 0; i < debugger->particles.size(); i++) {
      double c = debugger->particles[i].probability;
      glColor3f(.5*(1-c), 1 *c, 0);
      Position p = debugger->particles[i].getPosition();
      int len =  c * 100; 
      int xp =  glutGet(GLUT_WINDOW_WIDTH)/2 + len * cos(p.getTheta()) ;
      int yp =  glutGet(GLUT_WINDOW_HEIGHT)/2 + len * sin(p.getTheta()) ;
      //cout << "particle " << i << "\tprob: " << c << "\tlen: " << len << "\txp: " << xp << "\typ: " << yp << endl;
      glVertex2f(xp,yp);
    }
  }
  glEnd();
}

void MCPainter::drawPlayerBlobs(Localization * itl){
  BlobfinderProxy * bfp = itl->getBlobfinderProxy();
  int wMult = 2 ;      // width multiplier: the width ratio of display window to camera image 
  int hMult = 2 ;      // height multiplier: the height ratio of display window to camera image 
  drawWindowBorders(wMult, hMult); 
  for (unsigned int i = 0; i < bfp->GetCount(); i++) {
    player_blobfinder_blob p_blob = bfp->GetBlob(i); 
    int color = itl->getBlobColor(p_blob);
    double r, g, b;  
    switch(color){
    case 0: // pink 255 0 255
      r = 1; g = 0; b = 1; 
      break ; 
    case 1: // yellow 255 255 0
      r = 1; g = 1; b = 0; 
      break;
    case 2: // blue 0 0 255
      r = 0; g = 0; b = 1; 
      break;
    case 3: // green 0 255 0
      r = 0; g = 1; b = 0; 
      break; 
    case 4: // orange 255 125 0
      r = 1; g = 0.5; b = 0; 
      break; 
    default: 
      r = 0; g = 0; b = 0; 
    }

    int x = p_blob.x; 
    int y = p_blob.y; 
    int height_half = static_cast<int>((p_blob.bottom - p_blob.top)/2) ; 
    int width_half = static_cast<int>((p_blob.right - p_blob.left)/2) ;
    int win_h = glutGet(GLUT_WINDOW_HEIGHT); 

    // draws 2 rectangles so that the blob appears thick
    glBegin(GL_LINES);
    {
      glColor3f(r, g, b); 
      glVertex2i( (x - width_half) * wMult, win_h - (y - height_half) * hMult ); 
      glVertex2i( (x - width_half) * wMult, win_h - (y + height_half) * hMult ); 

      glVertex2i( (x - width_half) * wMult, win_h - (y + height_half) * hMult ); 
      glVertex2i( (x + width_half) * wMult, win_h - (y + height_half) * hMult ); 

      glVertex2i( (x + width_half) * wMult, win_h - (y + height_half) * hMult ); 
      glVertex2i( (x + width_half) * wMult, win_h - (y - height_half) * hMult ); 

      glVertex2i( (x + width_half) * wMult, win_h - (y - height_half) * hMult ); 
      glVertex2i( (x - width_half) * wMult, win_h - (y - height_half) * hMult ); 

      // inner rectangle
      //glColor3f(.5,.5,.5);
      glVertex2i( (x - width_half) * wMult + 1, win_h - (y - height_half) * hMult + 1 ); 
      glVertex2i( (x - width_half) * wMult + 1, win_h - (y + height_half) * hMult - 1); 

      glVertex2i( (x - width_half) * wMult + 1, win_h - (y + height_half) * hMult - 1); 
      glVertex2i( (x + width_half) * wMult - 1, win_h - (y + height_half) * hMult - 1); 

      glVertex2i( (x + width_half) * wMult - 1, win_h - (y + height_half) * hMult - 1); 
      glVertex2i( (x + width_half) * wMult - 1, win_h - (y - height_half) * hMult + 1); 

      glVertex2i( (x + width_half) * wMult - 1, win_h - (y - height_half) * hMult + 1); 
      glVertex2i( (x - width_half) * wMult + 1, win_h - (y - height_half) * hMult + 1); 
    }
    glEnd();
  }
}

void MCPainter::drawObservationBlobs(Localization * itl){
  /*
  vector<Observation> obs = itl->getObservations(); 
  */
}

void MCPainter::drawCameraImage(Localization * itl){
  /*CameraProxy * cam = itl->getCameraProxy();
  uint cam_width = cam->GetWidth();
  uint cam_height = cam->GetHeight();
  uint cam_depth = cam->GetDepth();

  uint8_t* imgBuffer = new uint8_t[cam_width * cam_height * cam_depth];

  cam->Decompress(); 
  cam->GetImage(imgBuffer);

  
  IplImage * img = cvCreateImage(cvSize(cam_width, cam_height), IPL_DEPTH_8U, 3); 

  for ( uint i = 0 ; i < cam_width ; i++ ){
    for ( uint j = 0 ; j < cam_height ; j++ ){
      img->imageData[cam_width * j*3 + i*3 + 0] = (char)imgBuffer[cam_width * j*3 + i*3 + 2]; 
      img->imageData[cam_width * j*3 + i*3 + 1] = (char)imgBuffer[cam_width * j*3 + i*3 + 1]; 
      img->imageData[cam_width * j*3 + i*3 + 2] = (char)imgBuffer[cam_width * j*3 + i*3 + 0]; 
    }
  }
  
  delete[] imgBuffer; 
  //cvReleaseImage(&img);
  */
}

void MCPainter::drawWindowBorders(int wm, int hm){
  glBegin(GL_LINES);
  {
    glColor3f(0,1,0);
    glVertex2f( 0, 0 ) ; 
    glVertex2f( glutGet(GLUT_WINDOW_WIDTH) * wm, 0 ); 

    glVertex2f( glutGet(GLUT_WINDOW_WIDTH) * wm, 0 ); 
    glVertex2f( glutGet(GLUT_WINDOW_WIDTH) * wm, glutGet(GLUT_WINDOW_HEIGHT) * hm ); 

    glVertex2f( glutGet(GLUT_WINDOW_WIDTH) * wm, glutGet(GLUT_WINDOW_HEIGHT) * hm ); 
    glVertex2f( 0, glutGet(GLUT_WINDOW_HEIGHT) * hm ); 

    glVertex2f( 0, glutGet(GLUT_WINDOW_HEIGHT) * hm ); 
    glVertex2f( 0, 0 ) ; 
  }
  glEnd();
}

void MCPainter::drawObservations(MonteCarloDebugger * debugger, Localization * itl) {
  /*
  vector<Observation*> obs = debugger->getObservations();
  glBegin(GL_LINES);
  {
    for (unsigned int i = 0; i < obs.size(); i++) {
      glColor3f(1, 0, 1);
      
      Position position = itl->getPosition();
      glVertex2f(position.getX(), position.getY());

      int lineLen = 1000;
      int x = position.getX() + lineLen * cos(obs[i]->getBearing() + position.getTheta());
      int y = position.getY() + lineLen * sin(obs[i]->getBearing() + position.getTheta());
      glVertex2f(x, y);
    }
  }
  glEnd();
  */
}

/* current call for this function does not include a realPosition. That
   should be added if there is such an info coming from an overhead camera 
   or something of the sort 
*/
void MCPainter::drawPosition(Localization * itl, Position realPosition) {
  //draw estimated position
  glBegin(GL_POLYGON);
  {
    Position p = itl->getPosition();
    double conf = itl->getConfidence() / 2; // % 0-50
    
    glColor3f(1, 0.75 - 4*conf, 0.75 - 4*conf);  // more confident the robot is about its localization more red it will get
    int lineLen = 15;
    int x1 = p.getX() - lineLen * cos(p.getTheta() + .3);
    int y1 = p.getY() - lineLen * sin(p.getTheta() + .3);
    int x2 = p.getX() - lineLen * cos(p.getTheta() - .3);
    int y2 = p.getY() - lineLen * sin(p.getTheta() - .3);
    glVertex2f(p.getX(), p.getY());
    glVertex2f(x1, y1);
    glVertex2f(x2, y2);
    glVertex2f(p.getX(), p.getY());
  }
  glEnd();

  //draw real position
  glBegin(GL_LINES);
  {
    Position p = realPosition;
    glColor3f(0, 0, 1);
    int lineLen = 100;
    int x = p.getX() + lineLen * cos(p.getTheta());
    int y = p.getY() + lineLen * sin(p.getTheta());
    glVertex2f(p.getX(), p.getY());
    glVertex2f(x, y);
  }
  glEnd();
}

// Slavisa added Sep 14 2013
// This is my function that will draw the FORR target, but this one would
// not need graph as its argument
void MCPainter::drawFORRTarget(int x, int y){
  glBegin(GL_POLYGON);
  
  glColor3f(0,0,1);
  glVertex2i(x-2, y-2); 
  glVertex2i(x-2, y+2); 
  glVertex2i(x+2, y+2); 
  glVertex2i(x+2, y-2); 
  
  glEnd();
}






//code for circle taken from slabode.exofire.net/circle_draw.shtml
void MCPainter::drawRegions(std::vector<FORRCircle> c){
  //need to enable GL_BLEND for alpha values
  //glEnable(GL_BLEND);
  //glColor4f(0.0, 1.0, 0.0, 0.3);
  int NUM_INTERVALS = 20;
  const double pi = 3.1415926535897;
  /*
  if(c.size() > 0){
   
    glBegin(GL_LINE_LOOP);
    //glVertex2f(c[0].getCenter().get_x(), c[0].getCenter().get_y());
     glColor3f(1, 0, 0);
     cout << "Circle center: "<<c[0].getCenter().get_x() << "," << c[0].getCenter().get_y() << endl;
     cout << "Circle radius: "<<c[0].getRadius() << endl;
     for(int j = 0; j < NUM_INTERVALS; j++){
       float angle = 2 * pi * (j / NUM_INTERVALS);
       float x = 100 * cos(angle);
       float y = 100 * sin(angle);
       glVertex2f(x + c[0].getCenter().get_x(), y + c[0].getCenter().get_y());
     
      glEnd();
      
      /*
      glBegin(GL_POLYGON);
      x = c[0].getCenter().get_x();
      y = c[0].getCenter().get_y();
      int r = c[0].getRadius();
      glColor3f(0,0,1);
      glVertex2i(x - r, y - r); 
      glVertex2i(x-r, y+r); 
      glVertex2i(x+r, y+r); 
      glVertex2i(x+r, y-r); 
  
      glEnd();
      
      } 
     
  }
  */
  //std::cout << "IN DRAW REGIONS ---------------------------------------------"<<endl;
  //std::cout << "Circle vector size: "<<c.size();
  //loop through the circles and draw them
  
  for(int i = 0; i < c.size(); i++){
    //cout << "Circle center: "<<c[0].getCenter().get_x() << "," << c[0].getCenter().get_y() << endl;
    //cout << "Circle radius: "<<c[0].getRadius() << endl;
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    //glBegin(GL_LINE_LOOP);
    glBegin(GL_TRIANGLE_FAN);
    glColor4f(1, 0, 0, 0.3);
    //glVertex2f(c[i].getCenter().get_x(), c[0].getCenter().get_y());
    for(int j = 0; j < NUM_INTERVALS; j++){
      float angle = 2.0 * pi * (j * 1.0/ NUM_INTERVALS);
      float x = c[i].getRadius() * cos(angle);
      float y = c[i].getRadius() * sin(angle);
      //cout << "pi: "<< pi << endl;
 
      //cout << "angle: "<<angle<<", x: "<<x<<", y: "<<y<<endl;
      glVertex2f(x + c[i].getCenter().get_x(), y + c[i].getCenter().get_y());
    }
    glEnd();

    //graph the exits
    vector<FORRExit> e = c[i].getExits();
    for(int z = 0; z < e.size(); z++){
      int x = e[z].getExitPoint().get_x();
      int y = e[z].getExitPoint().get_y();
      glBegin(GL_POLYGON);
      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
      glColor4f(1, 0 ,0, 0.7);
      glVertex2i(x-2, y-2); 
      glVertex2i(x-2, y+2); 
      glVertex2i(x+2, y+2); 
      glVertex2i(x+2, y-2); 
      glEnd();

    }
    
  }
  

}


void MCPainter::drawGates(std::vector<Gate> g){
  int x,y;
  int x1, x2, y1, y2;
  //loop through the gates and draw them as small rectangles
  for(int i = 0; i < g.size(); i++){
    glBegin(GL_POLYGON);
    x = g[i].point_from.get_x();
    y = g[i].point_from.get_y();

    glColor3f(0,0,1);
    glVertex2i(x-3, y-3); 
    glVertex2i(x-3, y+3); 
    glVertex2i(x+3, y+3); 
    glVertex2i(x+3, y-3); 
    
    glEnd();
    glBegin(GL_LINES);
    glColor3f(0,0,1);
    x1 = g[i].point_from.get_x();
    y1 = g[i].point_from.get_y();
    x2 = g[i].point_to.get_x();
    y2 = g[i].point_to.get_y();
    glVertex2i(x1,y1);
    glVertex2i(x2,y2);
    glEnd();



  }
  
}




void MCPainter::drawConveyors(FORRWaypoints &conveyors, int height, int length){
  int granularity = conveyors.granularity;

  if(conveyors.max_grid_value != 0){
  for(int x = 0; x < conveyors.boxes_width-1; x++){
    for(int y = 0; y < conveyors.boxes_height-1; y++){
       glBegin(GL_POLYGON);
       glEnable(GL_BLEND);
       glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
       int grid_value = conveyors.getGridValueDirect(x,y);
       int map_x_start = x * granularity;
       int map_y_start = y * granularity;

       glColor4f(0, 1 , 0.3 , (grid_value)/(conveyors.max_grid_value * 1.0) - .2);
       glVertex2i(map_x_start, map_y_start); 
       glVertex2i(map_x_start + granularity, map_y_start); 
       glVertex2i(map_x_start + granularity, map_y_start + granularity); 
       glVertex2i(map_x_start, map_y_start + granularity); 
       glEnd();


    }


  }


  }
}


void MCPainter::drawTrail(vector<TrailMarker> &v){
  for(int i = 0; i < v.size()-1; i++){
    glBegin(GL_LINES);
    glColor3f(0.5,0.5,1);
    glVertex2i(v[i].coordinates.get_x(), v[i].coordinates.get_y());
    glVertex2i(v[i+1].coordinates.get_x(), v[i+1].coordinates.get_y());
    glEnd();
  }
  
}


void MCPainter::drawAllTrails(FORRTrails &t){
  for(int i = 0; i < t.getSize(); i++){
    vector<TrailMarker> v = t.getTrail(i);
    if(v.size() > 1){
    for(int j = 0; j < v.size()-1; j++){
      glBegin(GL_LINES);
      if(i == t.getChosenTrail()){
	glColor3f(.1, .1, 1.0);

      }
      else{
	glColor3f(0.7, 0.7, 0.3);
      }
       glVertex2i(v[j].coordinates.get_x(), v[j].coordinates.get_y());
       glVertex2i(v[j+1].coordinates.get_x(), v[j+1].coordinates.get_y());
       glEnd();
      
    }
    }
    
  }
  
  
}
