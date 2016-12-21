#ifndef MOTION_H
#define MOTION_H

#include "libplayerc++/playerc++.h"
#include "Move.h"
#include "Position.h"

using namespace PlayerCc;

class Motion {
 private: 
  Position currPos;   // current position: updated by the camera (map coordinates)
  Position initPos;   // initial position: position before robot starts moving (map coordinates)
  Position finalPos;  // final position: position after robot finishes move (map coordinates)
  Position2dProxy * p2d;
  boost::mutex robotMutex;

  Position destination;    // intended position relative to the robot before the motion

 public: 
  Position getPosition() { return currPos; }
  Position getInitialPosition() { return initPos; }
  Position getFinalPosition() { return finalPos; }
  void setCurrentPosition(Position p) { currPos = p ; }

  void update(); 

  bool isMoveCompleted(int &count);
  void move(Position relativePosition);
  void reCalibrate();

  void setPosition2dProxy(PlayerClient* pCli){ p2d = new Position2dProxy(pCli, 0); }
  
};

#endif
