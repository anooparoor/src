#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "ConfigManager.h"
#include "definitions.h"
#include "CommunicationManager.h"
#include "Position.h"
#include "Utils.h"
#include "libplayerc++/playerc++.h"
#include <string>
using namespace PlayerCc;

class Controller {
public:
  Controller(string, int, string, int, string, string) ;

  CommunicationManager * getRobot() { return cMan; }

  void sampleLinearMotion(); 
  void sampleRotationalMotion(); 
  
  void calibrateLinearMotion(BlackfinConfig *bc); 
  void calibrateRotationalMotion(BlackfinConfig *bc); 
  

  

  void move(Position); 
private:
  PlayerClient * pCli; 
  CommunicationManager * cMan;
  Motion * mot;
  
  string ofname;  

  // behavior vars
  Position startPos, endPos; 
};

#endif /* CONTROLLER_H */
