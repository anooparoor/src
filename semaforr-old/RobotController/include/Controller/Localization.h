/*
 * Localization.h
 *
 */

#ifndef INTERFACE_TO_LOCALIZATION_H_
#define INTERFACE_TO_LOCALIZATION_H_

#include "boost/thread/mutex.hpp"
#include "boost/date_time/posix_time/posix_time.hpp"

#include "libplayerc++/playerc++.h"
#include <unistd.h>
#include "Observation.h"
#include "CameraObservation.h"
#include "Move.h"
#include "Position.h"
#include "Map.h"
#include "MonteCarlo.h"
#include "ObservationBlob.h"

using namespace PlayerCc;

class Localization {
public:
  Localization(Map * map, int fieldOfVision);

  void update();
  void move(Position relativePosition);
  Position getPosition();

  double getConfidence() {
    if ( usingStage || isSimulatingGPS() ) 
      return 1;
    robotMutex.lock();
    double confidence = 0.0; 
    confidence = debugger->getConfidence();
    robotMutex.unlock();
    return confidence;
  }
  
  MonteCarlo * getMonteCarlo() {
    return mc;
  }
  
  Map * getMap() { return map; }
  
  void setMCDebugger(){
    debugger = new MonteCarloDebugger(); 
    mc->setDebugger(debugger); 
  }
  
  MonteCarloDebugger* getMCDebugger(){ return debugger; }
  
  void setObservationVariance(double observationVariance) {
    this->observationVariance = observationVariance;
  }

  bool isSimulatingGPS() { return simulateGPS; } 
  void setSimulateGPS(bool b) { simulateGPS = b; }
  
  /* used in simulateGPS mode. */
  /* this function discards bad readings coming from the camera */
  void setLatestPosition(Position p); 
  void setCurrentPosition(Position p) { 
    currentPos = p; 
  } 
  /* if new campose received since we start moving return true. >= because time is in seconds, not accurate */
  bool isPositionChangedSinceMove(){
    return ( latestPosTime >= startPosTime ) ;
  }
  /* find out the movement between start and latest positions */
  Move getMoveDiffStartLatest(){
    //string label = "\tLocalization.h::getMoveDiffStartLatest()> ";

    //cout << label << "StartPos(" << startPos.getX() << "," << startPos.getY() << "," << startPos.getTheta() 
    //<< ")\tlatestPos(" << latestPos.getX() << "," << latestPos.getY() << "," << latestPos.getTheta() << endl; 
    
    return getMoveDiff(startPos, latestPos); 
  } 

  /* For differential drive only, no y movement */
  Move getMoveDiff(Position start, Position end){
    double deltaXY = Utils::get_euclidian_distance(start.getX(), start.getY(), end.getX(), end.getY());
    double deltaYaw = Utils::calcAngleDifference(start.getTheta(), end.getTheta());
    return Move(deltaXY, 0, deltaYaw); 
  }

  /* returns true if robot can travel between two positions within the time difference */
  bool positionChangePossible(Position startPos, int startPosTime, Position endPos, int endPosTime);
  
  /* end simulateGPS mode */

  void setErrorModel(double trans_mean, double trans_var, bool transAvailable, 
		     double drift_mean, double drift_var, bool driftAvailable, 
		     double rot_left_mean, double rot_left_var, bool rotLeftAvailable, 
		     double rot_right_mean, double rot_right_var, bool rotRightAvailable){
    if ( transAvailable ) 
      mc->setTranslationError(trans_mean, trans_var); 
    if ( driftAvailable )
      mc->setDriftError(drift_mean, drift_var); 
    if ( rotLeftAvailable )
      mc->setRotationLeftError(rot_left_mean, rot_left_var); 
    if ( rotRightAvailable ) 
      mc->setRotationRightError(rot_right_mean, rot_right_var); 

    if ( transAvailable && driftAvailable ) {
      errorTranslateAvailable = true; 
    }
    if ( rotLeftAvailable && rotRightAvailable ) {
      errorRotationAvailable = true; 
    }
  }
  
  Move addError(Move); 

  void setSpeed(double, double, double);
  void moveToMapPosition(int, int);
  void positionToMotion(double, double, double, char*, int*);
  void motion( char, char ); // sklar

  bool isMoveCompleted();

  bool isDestinationReached() { return destinationReached; }
  bool isDestinationSet();

  bool isFound() { return foundItem; }
  void resetDestinationInfo(); 

  void setBlobFinderProxy(PlayerClient*);
  BlobfinderProxy* getBlobfinderProxy() { return bfp; }
  void setPosition2dProxy(PlayerClient*);
  void setLocalizeProxy(PlayerClient*);   // for stage

  LocalizeProxy* getLocalizeProxy() { return lcp; }   // for stage
  player_pose2d_t readPosition(LocalizeProxy*);       // for stage

  void setCameraProxy(PlayerClient*);
  CameraProxy* getCameraProxy() { return cam; }
  void updateStagePosition();

  // move to Perception.cpp
  void printBlobColor(player_blobfinder_blob);
  void printBlobInfo(observationBlob);
  int getBlobColor(player_blobfinder_blob blob);

  void updateCameraObservations(CameraObservation);  

  // MOVE TO Controller. 
  // This function returns true if the intended move fell short in rotation significantly. 
  // Refers to overheating of the motors
  bool isTired() { return tired; } 
  void rested() { tired = false; }

  Move getLastMove();
  
  void setPlayerClient(PlayerClient *_pCli) { pCli = _pCli; }

protected:
  MonteCarlo * mc;
  MonteCarloDebugger * debugger; 
  CameraProxy * cp;
  BlobfinderProxy * bfp;
  Position2dProxy * p2d;
  CameraProxy * cam;
  
  LocalizeProxy * lcp;
  bool usingStage;

  bool usingGotoAngle;
  bool usingGotoLine;
  double angleToRotate;
  double destinationX;
  double destinationY;

  //Map properties
  int mapHeight;
  int mapLength;
  int heightShift;		
  int lengthShift;

  player_pose2d_t  pose;   // For handling localization data

  // Used for moving robot to position instead of Player GoTo when using Stage
  void GoTo(double,double,double);
  // Used for checking robot motion when using Stage
  void checkRotation();
  void checkDistance();

  // copy in Perception.cpp 
  boost::mutex robotMutex;

  Position currentPos;
  Position latestPos;   // latest camera position received. used in simulateGPS mode
  Position suspectPos; 
  Position startPos; 
  Position previousMove; 
  bool destinationReached; 
  Position destination;

  int latestPosTime;
  int startPosTime; 
  int suspectPosTime; 

  Map * map;
  int fov;						//the field of vision in degrees
  vector<Observation*> obs;
  vector<CameraObservation> received_camobs;
  vector<CameraObservation> in_process_camobs;
  double observationVariance;
  
  bool foundItem ;

  // if overhead camera info will be used as ground truth this value is true 
  bool simulateGPS;    

  // if the errorModel is available for a robot (if successfully read from robot.conf file) these values are  true
  bool errorTranslateAvailable; 
  bool errorRotationAvailable; 
  // if the error model is available these values are used when simulateGPS is true, otherwise particle filter takes charge
  double trans_mean; 
  double drift_mean; 
  double rot_left_mean; 
  double rot_right_mean; 

  bool moveCompleted;

  //void readData();
  void updateObservations();
//  Move getLastMove();
  //void checkDestinationReached(); 
  
  Position convertToRobotCoordinates(Position mapPos); 
  Position convertToMapCoordinates(Position robotPos); 
  Position convertToStageMapCoordinates(Position robotPos);

  // Move to Controller. false while robot's motors are working fine during turns, true otherwise.
  bool tired; 
  bool isRotationSuccessful(); 
  int numFailedRotations ;
  int suspectPosCount;

  PlayerClient *pCli;

  // Expiration time for FORR-based moves
  boost::posix_time::ptime move_expiration;

};

#endif /* INTERFACE_TO_LOCALIZATION_H_ */
