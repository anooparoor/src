#include "Motion.h"

bool Motion::isMoveCompleted(){
  bool moveCompleted ;

  robotMutex.lock();

  moveCompleted = ( abs(destination.getX() - p2d->GetXPos()) < 0.01 
		    && abs(destination.getY() - p2d->GetYPos()) < 0.01 
		    && abs(destination.getTheta() - p2d->GetYaw()) < 0.03 );

  robotMutex.unlock();

  if ( moveCompleted ) {
    if ( waitForCamposeUpdate() )
      finalPosFresh = true; 
    else 
      finalPosFresh = false; 

    finalPos = currPos; 
    finalPosTime = time(NULL); 
    destination = Position(0,0,0);
  }

  return moveCompleted;
}

void Motion::move(Position relativePosition){
  if ( waitForCamposeUpdate() )
    initPosFresh = true; 
  else
    initPosFresh = false; 

  initPos = currPos; 
  initPosTime = time(NULL); 

  robotMutex.lock();
  
  destination = Position(0, 0, 0);
  p2d->ResetOdometry();
  
  // convert x, y (cm) to (m)
  destination = Position(relativePosition.getX() / 100.0,
			 relativePosition.getY() / 100.0, 
			 relativePosition.getTheta());
  
  p2d->GoTo(destination.getX(), destination.getY(), destination.getTheta());
  
  robotMutex.unlock();
}

bool Motion::waitForCamposeUpdate(){
  unsigned int now = time(NULL);
  
  // wait until an updated campose arrives or timeout
  bool cPosFresh = false; 
  timeoutTimer.start(); 
  while ( timeoutTimer.elapsed() < timeout ) {
    if ( currPosTime > now ) {
      cPosFresh = true; 
      break ; 
    }
  }

  return cPosFresh; 
}
