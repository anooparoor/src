#ifndef MOTION_H
#define MOTION_H

#include "libplayerc++/playerc++.h"
#include "metrotimer.h"
#include "Move.h"
#include "Position.h"
#include <ctime>

using namespace PlayerCc;

class Motion {
 private: 
  Position currPos;            // current position: updated by the camera (map coordinates)
  unsigned int currPosTime;    // time current position information is received

  Position initPos;            // initial position: position before robot starts moving (map coordinates)
  unsigned int initPosTime;    // time init position information is set
  bool initPosFresh; 

  Position finalPos;           // final position: position after robot finishes move (map coordinates)
  unsigned int finalPosTime;    // time final position information is set
  bool finalPosFresh; 

  Position2dProxy * p2d;
  boost::mutex robotMutex;

  metrobotics::PosixTimer timeoutTimer;
  const static double timeout = 2; 

  Position destination;    // intended position relative to the robot before the motion

  bool waitForCamposeUpdate();

 public: 
  Motion() : destination(Position(0,0,0)) {}
  Position getPosition() { return currPos; }
  unsigned int getPositionTime() { return currPosTime; }

  Position getInitialPosition() { return initPos; }
  unsigned int getInitPositionTime() { return initPosTime; }
  bool isInitPosFresh() { return initPosFresh; }

  Position getFinalPosition() { return finalPos; }
  unsigned int getFinalPositionTime() { return finalPosTime; }
  bool isFinalPosFresh() { return finalPosFresh; }

  void setCurrentPosition(Position p, unsigned int t) { 
    currPos = p ; 
    currPosTime = t; 
  }

  void update(); 

  bool isMoveCompleted();
  bool isDestinationSet(){ return !(destination == Position(0,0,0)) ; }
  void move(Position relativePosition);

  void setPosition2dProxy(PlayerClient* pCli){ p2d = new Position2dProxy(pCli, 0); }
};

#endif
