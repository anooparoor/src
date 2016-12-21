#include <iostream>
#include <fstream>
#include <libplayerc++/playerc++.h>
#include <unistd.h>
#include <ctime>
#include <cmath>
#include <cstdlib>
#include <sys/time.h>
#include <stdio.h>
#include <time.h>
using namespace std;
using namespace PlayerCc;

PlayerClient    robot;
Position2dProxy pp(&robot, 0);

bool inMotion()
{
  return pp.GetXSpeed() || pp.GetYSpeed() || pp.GetYawSpeed();
}

int main(int argc, char **argv)
{
  // ghetto state machine
  
  pp.ResetOdometry();
  pp.GoTo(0, 0, M_PI_2);
  
  
  
  /*
  enum { STRAIGHT, TURNING } state;
  int stateChanges = 0;
  
  // odometry
  const double distance  = 1.0; // in meters
  const double theta     = M_PI_2; // in radians
  
  pp.ResetOdometry();
  pp.GoTo(1, 1, 0);
  
  while (pp.GetXPos() != 1 && pp.GetYPos() != 1){
    cout << "xpos: " << pp.GetXPos() << ", ypos: " << pp.GetYPos() << ", yaw: " << pp.GetYaw() << endl; 
    robot.ReadIfWaiting();
  }
  
  cout << "I am at (" << pp.GetXPos() << "," << pp.GetYPos() << ")" << endl ; 
  cout << "arrived at destination" << endl;
  

  //* Using SetSpeed
    
  // initial state: STRAIGHT
  state = STRAIGHT;
  pp.ResetOdometry();
  pp.SetSpeed(1, 0);
  
  while (stateChanges < 8) {
    robot.ReadIfWaiting(); // read from proxies
    
    cout << (state == STRAIGHT ? "straight" : "turning") 
	 << "::displacement = " 
	 << (state == STRAIGHT ? pp.GetXPos() : pp.GetYaw()) << endl;
    cout << ( inMotion() ? "moving" : "not moving" ) << endl; 
  
    switch (state) {
    case STRAIGHT: {
      if (pp.GetXPos() >= distance) {
	cout << "I'm there!" << endl;
	if (inMotion()) {
	  pp.SetSpeed(0,0); // stop
	  cout << "stopping" << endl;
	  while (inMotion()) {
	    robot.ReadIfWaiting();
	    cout << "readifwaiting called" << endl;
	  }
	} else {
	  cout << "stopped" << endl;
	  state = TURNING;
	  stateChanges++;
	  pp.ResetOdometry();
	  while (pp.GetXPos() != 0) {
	    robot.ReadIfWaiting();
	    cout << "readifwaiting called since x position is 0" << endl;
	  }
	  pp.SetSpeed(0, 1.8); // turn
	  cout << "started turning" << endl;
	}
      }
    } break;
    case TURNING: {
      if (pp.GetYaw() >= theta) {
	cout << "facing where I wanted to face" << endl;
	if (inMotion()) {
	  pp.SetSpeed(0,0); // stop
	  cout << "stopping" << endl;
	  while (inMotion()) {
	    robot.ReadIfWaiting();
	    cout << "readifwaiting called" << endl;
	  }
	} else {
	  state = STRAIGHT;
	  stateChanges++;
	  pp.ResetOdometry();
	  while (pp.GetYaw() != 0) {
	    robot.ReadIfWaiting();
	  }
	  pp.SetSpeed(1, 0); // straight
	}
      }
    } break;
    default: break;
    }
  }
  

//* Using GoTo
/*
  pp.ResetOdometry();
  stateChanges = 0;
  pp.GoTo(1,0,0);
  cout << "go forward 1 m" << endl;
  while (stateChanges < 4) {
    switch (stateChanges) {
    case 0: {
      if (pp.GetXPos() == 1 && pp.GetYPos() == 0) {
	cout << "went forward for 1m, turning 90 and going for another m" << endl;
	++stateChanges;
	pp.GoTo(1,1,M_PI_2);
      }
    } break;
    case 1: {
      if (pp.GetXPos() == 1 && pp.GetYPos() == 1) {
	cout << "went forward for 1m, turned 90 and went for another m. doing that again" << endl;
	++stateChanges;
	pp.GoTo(0,1,M_PI);
      }
    } break;
    case 2: {
      if (pp.GetXPos() == 0 && pp.GetYPos() == 1) {
	cout << "I made a large U facing the start point. heading to start point." << endl;
	++stateChanges;
	pp.GoTo(0,0,0);
      }
    } break;
    case 3: {
      if (pp.GetXPos() == 0 && pp.GetYPos() == 0) {
	cout << "reached start point stopping and exiting." << endl;
	++stateChanges;
	pp.SetSpeed(0,0);
	exit(0);
      }
    } break;
    default: break;
    }
    robot.Read();
    }*/
 
  return 0;
}
