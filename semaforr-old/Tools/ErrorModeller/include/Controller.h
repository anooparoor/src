#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "definitions.h"
#include "CommunicationManager.h"
#include "Position.h"
#include "Utils.h"
#include "libplayerc++/playerc++.h"
#include <string>
using namespace PlayerCc;

class Controller {
public:
  Controller(PlayerClient* p, string lab, string ty);
  Controller(string, int, string, int, string, string) ;

  CommunicationManager * getCommunicationManager() { return cMan; }
  void setCommunicationManager(CommunicationManager * c ) { cMan = c; }
  Motion * getMotion() { return mot; }

  void sampleLinearMotion(); 
  void sampleRotationalMotion(); 

  bool isInSafeZone();
  bool isClose(Position);
  Position convertToRobotCoordinates(Position);
  void setCenterXY(){
    Position cPos = mot->getPosition(); 
    center_x = cPos.getX(); 
    center_y = cPos.getY();
    cout << "Center set to (" << center_x << ", " << center_y << ")" << endl;
  } 

  void setFileDir(string dir) { 
    ofname = dir + ofname;
  }
  string getFilename() { return ofname; }

  void moveToInitPosition(Position initpos);
  void move(Position); 
  bool writeMove(Position, ofstream&); 
private:
  PlayerClient * pCli; 
  CommunicationManager * cMan;
  Motion * mot;
  
  string ofname;  

  // ground coordinates of camera view's center point.
  int center_x, center_y; 
  int safe_square_length; 

  // behavior vars
  Position startPos, endPos; 
};

#endif /* CONTROLLER_H */
