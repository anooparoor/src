#include "Motion.h"

bool Motion::isMoveCompleted(int &count){
  bool moveCompleted ;

  robotMutex.lock();

  
  
  if(count < 10)
  {
	  if(abs(destination.getX() - p2d->GetXPos()) < 0.01 && abs(destination.getY() - p2d->GetYPos()) < 0.01 
			    && abs(destination.getTheta() - p2d->GetYaw()) < 0.03  && (currPos.getTheta() != initPos.getTheta() || currPos.getX() != initPos.getX() || currPos.getY() != initPos.getY()))
		{
			moveCompleted = true;
		}
		else moveCompleted = false;
		count++;
	}
	else moveCompleted = true;
	
	cout << count << endl;
		
  robotMutex.unlock();

  if ( moveCompleted ) 
  {
  	 usleep(300000);
    finalPos = currPos; 
  }

  return moveCompleted;
}

void Motion::move(Position relativePosition){
  robotMutex.lock();

  initPos = currPos; 
  destination = Position(0, 0, 0);
  p2d->ResetOdometry();

  // convert x, y (cm) to (m)
  destination = Position(relativePosition.getX() / 100.0,
			 relativePosition.getY() / 100.0, 
			 relativePosition.getTheta());

  p2d->GoTo(destination.getX(), destination.getY(), destination.getTheta());
  
  sleep(1);

  robotMutex.unlock();
}

void Motion::reCalibrate()
{
	//cout << "Recalibrating...\n";
	usleep(1000000);
	p2d->ResetCalibration();
	usleep(1000000);
	//cout << "Recalibration Complete\n\n";
}

