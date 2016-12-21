/*
 * Controller.cpp
 *
 */

#include "Controller.h"
#include <iostream> 
#include <fstream>
#include <cmath>
using namespace std;

#define CTRL_DEBUG false 

Controller::Controller(PlayerClient* p, string lab, string ty) : pCli(p) {
  mot = new Motion(); 
  mot->setPosition2dProxy(pCli); 

  ofname = lab + "-samples.dat";
}

Controller::Controller(string pcH, int pcP, string csH, int csP, string lab, string ty) {
  try {
    pCli = new PlayerClient(pcH, pcP); 

    mot = new Motion(); 
    mot->setPosition2dProxy(pCli); 

    cMan = new CommunicationManager(*pCli, mot, lab, ty);
    
    if (!cMan->Connect(csH, csP)) {
      cerr << "Failed to establish a connection to the Central Server.\n"
	   << "Central Server hostname: " << csH << "\n"
	   << "Central Server port: " << csP << endl;
      exit(1);
    }
  }
  catch (PlayerError){
    cerr << "Failed to establish a connection to the Player Server.\n"
	 << "Robot: " << lab << ", type:" << ty << "\n"
	 << "Player Server hostname: " << pcH << "\n"
	 << "Player Server port: " << pcP << endl;
    exit(1);
  }
 
  ofname = lab + "-samples.dat";
}

void Controller::sampleLinearMotion() {
  // open file to write
  ofstream outfile(ofname.c_str(), ios::out); 
  if ( !outfile ) {
    cerr << "Could not open the file: " << ofname << endl;
    exit(1);    
  }

  const int REPEAT = 35;
  Position trans(75, 0, 0);                 // fwd 75cm
  safe_square_length = 75; 
  Position rot(0, 0, Utils::toRadians(-120));    // rotate 120 degrees right
  
  // go to starting position
  Position initpos(center_x - trans.getX()/2, center_y - trans.getX()/2, Utils::toRadians(89));

  if ( !isClose(initpos) )
    moveToInitPosition(initpos);

  // triangle
  cout << "In position! Sampling..." << endl;
  for ( int i = 0 ; i < REPEAT; i++ ){
    cout << "repetitions left : " << REPEAT - i << endl ; 
    for ( int j = 0 ; j < 3; j++ ) {
      bool recorded = false; 
      do {
	move(trans);
	recorded = writeMove(trans, outfile);  
	if ( !recorded ) 
	  cout << "recording failed" << "\t isInitPosFresh(): " << mot->isInitPosFresh() <<  "\t isFinalPosFresh(): " << mot->isFinalPosFresh() << endl; 
	move(rot); 
	// if out of safe zone
	if ( !isInSafeZone() ) 
	  moveToInitPosition(initpos);
      } while ( !recorded );
    }
  }

  // close file
  outfile.close();
}

void Controller::sampleRotationalMotion() {
  // open file to write
  ofstream outfile(ofname.c_str(), ios::app); 
  if ( !outfile ) {
    cerr << "Could not open the file: " << ofname << endl;
    exit(1);    
  }

  const int REPEAT = 20; 

  // go to start location
  Position initpos(center_x, center_y, Utils::toRadians(90));
  if ( !isClose(initpos) )
    moveToInitPosition(initpos); 

  int deltaTheta[] = {8, 22, 37, 52, 75, 90, 125, 179};
  for ( int j = 0 ; j < REPEAT; j++ ){
    cout << "repetitions left : " << REPEAT - j << endl ; 
    for ( unsigned int i = 0; i < 8; i++) {
      Position rot(0,0,Utils::toRadians(deltaTheta[i]));
      bool recorded = false; 
      do {
	move(rot); 
	recorded = writeMove(rot, outfile); 
	if ( !recorded ) 
	  cout << "recording failed" << "\t isInitPosFresh(): " << mot->isInitPosFresh() <<  "\t isFinalPosFresh(): " << mot->isFinalPosFresh() << endl; 
      } while ( !recorded );

      rot.setTheta(-1 * rot.getTheta()); 
      recorded = false ; 
      do {
	move(rot); 
	recorded = writeMove(rot, outfile); 
	if ( !recorded ) 
	  cout << "recording failed" << "\t isInitPosFresh(): " << mot->isInitPosFresh() <<  "\t isFinalPosFresh(): " << mot->isFinalPosFresh() << endl; 
      } while ( !recorded ); 
    }
    // if out of safe zone
    if ( !isInSafeZone() ) 
      moveToInitPosition(initpos);
  }

  // close file
  outfile.close();
}

/*!
  Returns true if the Current Position is within a "safe" area.
  The safe area is considered to be a fixed distance from the designated center_x and center_y
*/ 
bool Controller::isInSafeZone() {
  bool inZone = false; 
  Position cPos = mot->getPosition(); 
  if ( cPos.getX() <= center_x + safe_square_length &&   
       cPos.getX() >= center_x - safe_square_length &&
       cPos.getY() <= center_y + safe_square_length && 
       cPos.getY() >= center_y - safe_square_length )
    inZone = true; 
  return inZone; 
}


/*!

 */ 
bool Controller::isClose(Position p){
  bool close = false; 
  Position cPos = mot->getPosition(); 

  // if the robot is within 10 cm distance from its destination
  if ( Utils::get_euclidian_distance(cPos.getX(), cPos.getY(), p.getX(), p.getY()) <= 10 ) {
    double angleDiff = abs(cPos.getTheta() - p.getTheta()) ;
    if ( angleDiff > M_PI )
      angleDiff = 2 * M_PI - angleDiff ; 
    if ( angleDiff < Utils::toRadians(10) ){
      cout << "close enough" << endl ; 
      close = true; 
    }
  }
  return close; 
}

Position Controller::convertToRobotCoordinates(Position mapPos){
  Position currPos = mot->getPosition();  // in cm, radians

  // delta x, y from map perspective
  int tx = mapPos.getX() - currPos.getX() ;
  int ty = mapPos.getY() - currPos.getY() ;
  double theta = currPos.getTheta();
  
  // transform map ( x, y ) to robot ( x', y' )
  double nx = tx * cos(theta) + ty * sin(theta); 
  double ny = -tx * sin(theta) + ty * cos(theta); 

  // after updating x and y modify the final position according to the current orientation
  double deltaTheta = mapPos.getTheta() - theta; 
  Position dest( (int) nx, (int) ny, deltaTheta);

  return dest; 
}

void Controller::moveToInitPosition(Position initpos){
  Position currPos, initxy = initpos, initorient = initpos; 

  // First get close to x y position
  do {
    cout << "Out of safe zone moving to initial position" << endl;
    currPos = mot->getPosition(); 
    initxy.setTheta(Utils::calcHeading(initxy.getX() - currPos.getX(), initxy.getY() - currPos.getY())); 
    move(convertToRobotCoordinates(initxy)); 
  } while ( !isClose(initxy) ); 

  // once close to the position, modify orientation
  do {
    cout << "Correcting heading according to initial position" << endl; 
    currPos = mot->getPosition(); 
    initorient.setX(currPos.getX());
    initorient.setY(currPos.getY());
    move(convertToRobotCoordinates(initorient)); 
  } while ( !isClose(initorient) ); 
}

void Controller::move(Position p) {
  if ( !mot->isDestinationSet() ){
    cout << "\t Relative(" << p.getX() << ", " << p.getY() << ", " << Utils::toDegrees(p.getTheta()) << ")";
    mot->move(p); 

    while ( !mot->isMoveCompleted() ){}

    startPos = mot->getInitialPosition(); 
    endPos = mot->getFinalPosition(); 
    cout << "\tStart(" << startPos.getX() << ", " << startPos.getY() << ", " << Utils::toDegrees(startPos.getTheta()) << ")"
	 << "\tEnd(" << endPos.getX() << ", " << endPos.getY() << ", " << Utils::toDegrees(endPos.getTheta()) << ")" << endl;
  }
}


bool Controller::writeMove(Position p, ofstream& outfile){
  bool success = mot->isInitPosFresh() && mot->isFinalPosFresh(); 

  if ( success ) 
    // write to file < move-x move-y move-theta init-x init-y init-theta final-x final-y final-theta
    outfile << p.getX() << " " << p.getY() << " " << p.getTheta() << " "
	    << startPos.getX() << " " << startPos.getY() << " " << startPos.getTheta() << " "
	    << endPos.getX() << " " << endPos.getY() << " " << endPos.getTheta() << endl;

  return success; 
}
