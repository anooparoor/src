/*
 * Localization.cpp
 *
 */

#include "Localization.h"
#include "definitions.h"

#define ITL_DEBUG false


/**
 * Localization constructor
 *
 */
Localization::Localization( Map * map, int fieldOfVision ){
  // this way of locking and unlocking the mutex is error prone in the face of possible
  // exceptions thrown in between. instead use boost::mutex:scoped_lock lock(robotMutex) 
  // this doesn't require unlocking and releases lock upon object leaving scope. However,
  // it will only become a danger if there are multiple threads accessing these functions
  // for the same robot.
  robotMutex.lock();

  mc = new MonteCarlo(map, 250, 10, 0.2);
  mc->setRandomCoefficients(Utils::toRadians(10), 10, Utils::toRadians(20), 20);
  observationVariance = 0;

  setMCDebugger();
	
  this->map = map;
  this->fov = fieldOfVision;
  
  currentPos = Position(0, 0, 0);
  latestPos = Position(0, 0, 0);
  suspectPos = Position(0, 0, 0);
  startPos = Position(0, 0, 0);

  destination = Position(0, 0, 0);
  destinationReached = false; 

  foundItem = false; 
  
  simulateGPS = true;
  //simulateGPS = false;

  errorTranslateAvailable = false; 
  errorRotationAvailable = false;

  usingStage = false;
  usingGotoAngle = usingGotoLine = false;
  angleToRotate = destinationX = destinationY = 0.0;

  mapHeight = map->getHeight();
  mapLength = map->getLength();
  heightShift = mapHeight/2;
  lengthShift = mapLength/2;

  moveCompleted = true; 

  tired = false;
  numFailedRotations = 0; 
  suspectPosCount = 0;

  latestPosTime = 0; 
  startPosTime = 0; 
  suspectPosTime = 0; 

  robotMutex.unlock();

} // end of Localization() constructor


/**
 * setBlobFinderProxy()
 *
 */
void Localization::setBlobFinderProxy(PlayerClient* pc) { 
  bfp = new BlobfinderProxy(pc, 0); 
} // end of setBlobFinderProxy()


/**
 * setPosition2dProxy()
 *
 */
void Localization::setPosition2dProxy(PlayerClient* pc) { 
  p2d = new Position2dProxy(pc, 0); 
} // end of setPosition2dProxy()


/**
 * setCameraProxy()
 *
 */
void Localization::setCameraProxy(PlayerClient* pc) { 
  cam = new CameraProxy(pc, 0); 
} // end of setCameraProxy()


/**
 * setLocalizeProxy()
 *
 */
void Localization::setLocalizeProxy(PlayerClient* pc) { 
  lcp = new LocalizeProxy(pc, 0);
  usingStage = true;
} // end of setLocalizeProxy()


/**
 * readPosition()
 *
 */
player_pose2d_t Localization::readPosition( LocalizeProxy * lp ) {

  player_localize_hypoth_t hypothesis;
  player_pose2d_t          pose;
  uint32_t                 hCount;

  // Need some messing around to avoid a crash when the proxy is
  // starting up.

  hCount = lp->GetHypothCount();

  if(hCount > 0) {
    hypothesis = lp->GetHypoth(0);
    pose = hypothesis.mean;
  }

  return pose;
} // end of readPosition()


/**
 * getPosition()
 *
 */
Position Localization::getPosition() {
  Position pos(0,0,0) ;
    
  if( usingStage ) {
    updateStagePosition();
    pos = currentPos;
  }
  else if ( isSimulatingGPS() ) {            
    pos = currentPos;
  }
  else {
    pos = debugger->getPosition(); // in (cm)
  }
  
  return pos;
} // end of getPosition()


/**
 * updateStagePosition()
 *
 */
void Localization::updateStagePosition(){
  pose = readPosition(lcp);
  
  //Convert to map Coordinates
  currentPos = convertToStageMapCoordinates(Position(pose.px, pose.py, pose.pa));

  //  cout << "updateStagePostion: currentPos = (" << currentPos.getX() << ", " << currentPos.getY() << ", " << currentPos.getTheta() << ")" << endl;
} // end of updateStagePosition()


/**
 * update()
 *
 * Updates the observations and the mc filter if the robot is on the
 * move or changed it's position since last update 
 *
 */
void Localization::update() {
  string label = "Localization::update()> ";

  if (!usingStage) {
    
    Move lastMove = getLastMove();
    Move lastGPSMove = lastMove; 
    
    bool moved = ( lastMove.getX() + lastMove.getY() + lastMove.getTheta() != 0 );

    // This block needs to be re-tested with a good motion error model.
    if( moved ){

      if ( ITL_DEBUG){ 
	cout << label << "Current Position is: ("
	     << currentPos.getX() << "," << currentPos.getY() << "," << currentPos.getTheta() 
	     << "<" << Utils::toDegrees(currentPos.getTheta()) << "> )" << endl;
	cout << label << "Moved: (" << lastMove.getX() << "," << lastMove.getY() << "," << lastMove.getTheta()
	     << "<" << Utils::toDegrees(lastMove.getTheta()) << "> )" << endl;
	}

      // if the position information has been changed via CAMPOSE
      if ( isPositionChangedSinceMove() ){

	if ( ITL_DEBUG) 
	  cout << label << "Position information has been changed by CAMPOSE since we started the move" << endl; 

	// returns the difference between the starting position of the move and the latest camera position 
	Move moveBtwLatestAndStart = getMoveDiffStartLatest(); 
	
	if ( ITL_DEBUG) 
	  cout << label << "Difference between the starting position and the latest camera position in terms of Move: ("
	       << moveBtwLatestAndStart.getX() << "," << moveBtwLatestAndStart.getY() << "," << moveBtwLatestAndStart.getTheta() << ")" << endl;

	// the effect of the lastMove on our position: difference between the lastMove and the already updated portion of it via CAMPOSE
	Move moveSinceLatest( lastMove.getX()-moveBtwLatestAndStart.getX(), 0, lastMove.getTheta()-moveBtwLatestAndStart.getTheta()); 
	lastGPSMove = moveSinceLatest;
	
	if ( ITL_DEBUG) 
	  cout << label << "Difference between lastMove and the move until a camera information is received: ("
	       << moveSinceLatest.getX() << "," << moveSinceLatest.getY() << "," << moveSinceLatest.getTheta() << ")" << endl;

      }
    }

    // if the overhead camera will be accepted as ground truth
    if ( isSimulatingGPS() ){     
      if(moved){
	
	lastGPSMove = addError(lastGPSMove); 

	if ( ITL_DEBUG )
	  cout << label << "Added error. lastGPSMove : " 
	       << "(" << lastGPSMove.getX() << "," << lastGPSMove.getY() << "," << lastGPSMove.getTheta() 
	       << "<" << Utils::toDegrees(lastGPSMove.getTheta()) << "> )" << endl;

	// if the camera information hasn't been received recently the odometry information is relied upon
	if ( time(NULL) - latestPosTime > 1 ) {

	  // Hopefully fix the problem where a robot freezes when it's in the middle of
	  // a move and falls back to odometry. -Eric
	  moveCompleted = true;

	  currentPos.moveRelative(lastGPSMove);

	  
	  if ( ITL_DEBUG) 
	    cout << label << "Current Position is now: ("
		 << currentPos.getX() << "," << currentPos.getY() << "," << currentPos.getTheta() 
		 << "<" << Utils::toDegrees(currentPos.getTheta()) << "> )" << endl;
	}
      }
    }
    // run particle filter + CHECK: at this point the lastMove is passed directly to the particle filter. 
    // since the odometry information is returned once the motion is complete and in between the start and
    // end of the motion the camera may send lots of messages, by the time we receive the odometry info, 
    // particles may have already moved towards the direction of motion based on likelihood. 
    // In other words, we might be feeding obsolete odometry information to particle filter
    else {                               
      updateObservations(); 
      
      if ( obs.size() > 0 || lastMove.getX() + lastMove.getY() + lastMove.getTheta() != 0 ) {
	mc->updateFilter(lastMove, obs);    
	obs.clear(); 
	in_process_camobs.clear();
      }
    }
    
    /* For both particle filter and simulate GPS modes
     * If move is complete, reset flags and check if motors are overheated 
     * Move the motor overheat detection part to Controller
     */
    if ( isMoveCompleted() && isDestinationSet() ) {
      if ( !isRotationSuccessful() ) {
	if ( numFailedRotations > 3 ) {
	  tired = true; 
	  numFailedRotations = 0; 
	}
	else {
	  numFailedRotations++;
	}
      }
      else {
	numFailedRotations = 0; 
      }
    }
    
  }  // end !usingStage
  // if usingStage
  else  {
    if(usingGotoAngle)
      checkRotation();
    
    if(usingGotoLine)
      checkDistance();  
    
    Move lastMove = getLastMove();
    
    //    if(lastMove.getX() + lastMove.getY() + lastMove.getTheta() != 0)
    // currentPos.moveRelative(lastMove);
  }
} // end of update()


/**
 * move()
 *
 */
void Localization::move( Position relativePosition ) {

  string label = "\tLocalization::move(destination)> ";

  if ( ITL_DEBUG) 
    cout << label << "moving to : (" << relativePosition.getX() << ", " 
	 << relativePosition.getY() << ", " << relativePosition.getTheta() << "<" 
	 << Utils::toDegrees(relativePosition.getTheta()) << "> ) " << endl; 

  if ( ITL_DEBUG ) 
    cout << label << "locking robotMutex" << endl; 

  robotMutex.lock();

  if ( ITL_DEBUG ) 
    cout << label << "Acquired mutex lock" << endl; 

  moveCompleted = false;
  resetDestinationInfo();

  // convert x, y (cm) to (m)
  destination = Position(relativePosition.getX() / 100.0,
			 relativePosition.getY() / 100.0, 
			 relativePosition.getTheta());

  
  if (ITL_DEBUG)
    cout << label << "new dest(" 
	 << destination.getX() << "," 
	 << destination.getY() << "," 
	 << destination.getTheta() << ")" << endl; 
  

  // record starting position
  startPos = getPosition(); 
  startPosTime = time(NULL);

  if(usingStage){
    GoTo(destination.getX(), destination.getY(), destination.getTheta());
  }
  else{
    p2d->GoTo(destination.getX(), destination.getY(), destination.getTheta());
  }

  if ( ITL_DEBUG ) 
    cout << label << "Releasing mutex lock" << endl; 

  robotMutex.unlock();
} // end of move()


/**
 * setSpeed()
 *
 */
void Localization::setSpeed(double xs, double ys, double ts){
  robotMutex.lock();
  resetDestinationInfo(); 
  if ( xs < 0 ) 
    destination = Position(-1000, 0, 0); 
  else if ( xs > 0 )
    destination = Position(1000, 0, 0); 
  else if ( ys < 0 ) 
    destination = Position(0, -1000, 0); 
  else if ( ys > 0 )
    destination = Position(0, 1000, 0); 
  else if ( ts < 0 ) 
    destination = Position(0, 0, -M_PI * 100); 
  else if ( ts > 0 )
    destination = Position(0, 0, M_PI * 100); 
  
  if(usingStage){
    p2d->SetSpeed( xs/100, ys/100, ts/100 );
  }
  else{
    //    p2d->SetSpeed(xs, ys, ts); // sklar
    p2d->SetVelHead( xs, ys, ts);
  }
  robotMutex.unlock();
} // end of setSpeed()


/**
 * isDestinationSet()
 *
 */
bool Localization::isDestinationSet() {
  robotMutex.lock();
  bool destSet = !(destination == Position(0,0,0));
  robotMutex.unlock(); 
  return destSet; 
} // end of isDestinationSet()


/**
 * getLastMove()
 *
 */
Move Localization::getLastMove() {
  Move lastMove;
  string label = "\t\tITL::getLastMove()> " ;
  
  robotMutex.lock();

  if (destination == Position(0, 0, 0)){
    lastMove = Move(0, 0, 0);
//    moveCompleted = true;
  }
  else {
    double p2dX = p2d->GetXPos(); 
    double p2dY = p2d->GetYPos(); 
    double p2dYaw = p2d->GetYaw(); 

    if ( p2d->IsFresh() 
	 && ( p2dX + p2dY + p2dYaw ) != 0 
	 && !( p2dX == previousMove.getX() && p2dY == previousMove.getY() && p2dYaw == previousMove.getTheta() )){
      
      p2d->NotFresh();

      
      if (ITL_DEBUG)
	cout << "\t" << label << "previousMove : (" << previousMove.getX() << ", " 
	     << previousMove.getY() << ", " << previousMove.getTheta() << ")" << endl; 
      
      double delta_x = 0.0, delta_y = 0.0, delta_theta = 0.0;
      
      if (p2dX != 0) 
	delta_x = p2dX - previousMove.getX();
      if (p2dY != 0)
	delta_y = p2dY - previousMove.getY();
      if (p2dYaw != 0)
	delta_theta = p2dYaw - previousMove.getTheta();
      
      
      if (ITL_DEBUG)
	cout << label << "new motion since the last odometry reading : (" << delta_x << ", " 
	     << delta_y << ", " << delta_theta << ")" << endl; 
  

      // convert to cm 
      // in order to update the particle locations properly we have to convert 
      double delta = 0.0; 
      if ( delta_x + delta_y != 0 ) {
	delta = sqrt( pow(delta_x * 100, 2) + pow(delta_y * 100, 2) );       
	if ( delta_x < 0 && delta_y == 0 ) 
	  delta *= -1;
      }

      lastMove = Move(delta, 0, delta_theta);
      
      if (ITL_DEBUG) 
	cout << label << "lastMove: Move(" << delta << ",0," << delta_theta << ")" << endl;

      previousMove.setX(p2dX); 
      previousMove.setY(p2dY);   
      previousMove.setTheta(p2dYaw);
      
    }
    else {  // position not fresh so we didn't move
      lastMove = Move(0,0,0);
    }
  
    bool destReached = false;

    // check if the destination is reached. This is based on odometry not actual position.
    if ( !usingStage ) {
      if ( abs(destination.getX() - previousMove.getX()) < 0.01 
	   && abs(destination.getY() - previousMove.getY()) < 0.01 
	   && abs(destination.getTheta() - previousMove.getTheta()) < 0.03 ){
	destReached = true;
      }
    }
    else {
      if ( abs(destination.getX() - previousMove.getX()) < 0.10 
	   && abs(destination.getY() - previousMove.getY()) < 0.10 
	   && abs(destination.getTheta() - previousMove.getTheta()) < 0.3 ){
	destReached = true;
      }
    }

    bool movingTooLong = false;

    if ( time(NULL) - startPosTime >= 10 )
      movingTooLong = true;

    if ( destReached || movingTooLong ) {

      if ( !isSimulatingGPS() ){
	if (ITL_DEBUG) 
	  cout << label << "destination reached." << endl;
        if(usingStage)
	  //	  p2d->SetSpeed(0,0,0);
	  p2d->SetVelHead(0,0,0);
      }
      else {   // why don't we set moveCompleted to true if it is not simulating GPS ? 
	if (ITL_DEBUG) cout << label << "move completed." << endl;
	moveCompleted = true; 
      }
    }

  }

  //if ( ITL_DEBUG ) 
  //cout << label << "Releasing mutex lock" << endl; 

  robotMutex.unlock();

  return lastMove;
} // end of getLastMove()


/**
 * addError()
 *
 */
Move Localization::addError(Move m) {
  double x = 0.0, theta = 0.0; 

  // This bit adds the error for the motion that is read from the robot.conf file
  // Requires more testing. At the moment it causes jumps in the robot motion
  /*
  if (( m.getX() != 0 ) && errorTranslateAvailable ){
    x = (m.getX() * trans_mean);
    theta = (m.getX() * drift_mean);  
  }
  if ( m.getTheta() != 0 && errorRotationAvailable ){
    if ( m.getTheta() > 0 ) 
      theta += (m.getTheta() * rot_left_mean ); 
    else
      theta += (m.getTheta() * rot_right_mean ); 
  }
  */

  // if for some reason we don't get the error for x, use original move
  if ( x == 0.0 ) x = m.getX();
  // if for some reason we don't get the error for theta, use original move
  if ( theta == 0.0 ) theta = m.getTheta(); 

  return Move(x, m.getY(), theta); 
} // end of addError()


/**
 * resetDestinationInfo()
 *
 */
void Localization::resetDestinationInfo() {
  destination = Position(0, 0, 0);

  // How many times we 
  int wait_counter = 0;

  //  p2d->ResetOdometry();
  //  p2d->SetOdometry(0,0,0);
  //  if ( p2d->GetXPos() == 0 && p2d->GetYPos() == 0 && p2d->GetYaw() == 0 ) 
  //      previousMove = Position(0, 0, 0);

  p2d->ResetOdometry();

  // The following loop isn't necessary for Stage

  if (! usingStage) {
    while ( p2d->GetXPos() != 0 || p2d->GetYPos() != 0 || p2d->GetYaw() != 0 ) {
      pCli->Read();
      
      usleep(500000);
      
      wait_counter++;
      
      if (wait_counter % 30 == 0) {
	//    cout << "p2d->ResetOdometry() " << endl ;
	cout << "p2d->GetXPos(): " << p2d->GetXPos() << endl ; 
	cout << "p2d->GetYPos(): " << p2d->GetYPos() << endl ; 
	cout << "p2d->GetYaw(): " << p2d->GetYaw() << endl ; 
	
	p2d->ResetOdometry();
      }
    }
  }

  previousMove = Position(0, 0, 0);

  //  if ( ITL_DEBUG ) { 
  //    cout << "p2d->ResetOdometry() " << endl ;
  //    cout << "p2d->GetXPos(): " << p2d->GetXPos() << endl ; 
  //    cout << "p2d->GetYPos(): " << p2d->GetYPos() << endl ; 
  //    cout << "p2d->GetYaw(): " << p2d->GetYaw() << endl ; 
    //  }
  if ( usingStage ) 
    //    p2d->SetSpeed(0,0,0);
    p2d->SetVelHead(0,0,0);
  destinationReached = false;
} // end of resetDestinationInfo()


/**
 * moveToMapPosition()
 *
 */
void Localization::moveToMapPosition(int x, int y){
  Position pos = convertToRobotCoordinates(Position(x, y, 0)); 
  move(pos);
} // end of moveToMapPosition()


/**
 * convertToRobotCoordinates()
 *
 */
Position Localization::convertToRobotCoordinates(Position mapPos){
  Position currPos = getPosition();  // in cm

  
  if ( ITL_DEBUG )
    cout << "currPos(" 
	 << currPos.getX() << ", " 
	 << currPos.getY() << ", "
	 << currPos.getTheta() << "-Degrees:" << Utils::toDegrees(currPos.getTheta()) << ")" << endl;


  // delta x, y from map perspective
  int tx = mapPos.getX() - currPos.getX() ;
  int ty = mapPos.getY() - currPos.getY() ;
  double theta = currPos.getTheta();
  
  // transform map ( x, y ) to robot ( x', y' )
  double nx = tx * cos(theta) + ty * sin(theta); 
  double ny = -tx * sin(theta) + ty * cos(theta); 
  double ntheta = Utils::calcHeading(nx, ny);
  
  Position dest( (int) nx, 
		 (int) ny,
		 ntheta);

  
  if ( ITL_DEBUG )
    cout << "destination relative to robot (" 
	 << dest.getX() << ", " 
	 << dest.getY() << ", " 
	 << dest.getTheta() << "-Degrees:" << Utils::toDegrees(dest.getTheta()) << ")" << endl;


  return dest; 
} // end of convertToRobotCoordinates()



/*! Not implemented yet
 *  \brief Converts a given position from robots perspective ( 15cm away, 30 degrees to my left ) 
 *         to map coordinates
 *			   
 *  \return a position on map
 */
Position Localization::convertToMapCoordinates(Position robotPos){
  return Position(0,0,0);
} // end of convertToMapCoordinates()


/**
 * convertToStageMapCoordinates()
 *
 */
Position Localization::convertToStageMapCoordinates(Position robotPos){
  // transform robot ( x, y ) to map ( x', y' )
  double nx = robotPos.getX()*100 + lengthShift;
  double ny = robotPos.getY()*100 + heightShift; 

  Position conv( (int) nx, 
		 (int) ny,
		 robotPos.getTheta());
  return conv; 
} // end of convertToStageMapCoordinates()


/**
 * GoTo()
 *
 * ADDED TO USE GOTO OF STAGE
 *
 */
void Localization::GoTo(double gx, double gy, double gt) {
  if(gx!=0 && gt==0){
    //    p2d->SetSpeed(gx, gy, 0); // take y out TEST IT OUT
    p2d->SetVelHead(gx, gy, 0);
  }
  else if(gx==0 	&& gy==0 && gt!=0){
    //    p2d->SetSpeed(0,0,gt);
    p2d->SetVelHead(0, 0, gt);
  }
  else{
    usingGotoAngle = true;
    angleToRotate = gt;
    destinationX = gx;
    destinationY = gy;
    if(fabs(angleToRotate)>0.4){
      if(angleToRotate>0)
	//	p2d->SetSpeed(0,0,0.4);
	p2d->SetVelHead(0,0,0.6);
      else
	//	p2d->SetSpeed(0,0,-0.4);
	p2d->SetVelHead(0,0,-0.6);
    }
    else{
      //      p2d->SetSpeed(0,0,gt);
      p2d->SetVelHead(0,0,gt);
    }
  }
} // end of GoTo()


/**
 * checkRotation()
 *
 */
void Localization::checkRotation() {
   double p2dYaw = p2d->GetYaw(); 
   if(abs(p2dYaw-angleToRotate)<0.03){
     //      p2d->SetSpeed(.05,0,0);
     p2d->SetVelHead(.05,0,0); // sklar
      usingGotoAngle = false;
      usingGotoLine = true;
   }
} // end of checkRotation()


/**
 * checkDistance()
 *
 */
void Localization::checkDistance() {
   double p2dX = p2d->GetXPos(); 
   double p2dY = p2d->GetYPos();

   //   cout << "p2dX: " << p2dX << ", p2dY: " << p2dY << endl;

   if( abs(p2dX-destinationX) < 0.02 && abs(p2dY-destinationY) < 0.02){
      usingGotoLine = false;
   }

} // end of checkDistance()


/**
 * updateObservations()
 *
 */
void Localization::updateObservations() {
  string label = "\tLocalization::updateObservations()> ";

  if ( ITL_DEBUG ) 
    cout << label << "locking robotMutex" << endl; 

  robotMutex.lock();

  if ( ITL_DEBUG ) 
    cout << label << "Acquired mutex lock" << endl; 

  if ( !received_camobs.empty() ) {
    in_process_camobs.push_back(received_camobs.front());
    obs.push_back(&in_process_camobs.front());
    received_camobs.clear(); 
  }

  if ( ITL_DEBUG ) 
    cout << label << "Releasing mutex lock" << endl; 

  robotMutex.unlock();
} // end of updateObservations()


/**
 * updateCameraObservations()
 *
 */
void Localization::updateCameraObservations(CameraObservation cobs){
  string label = "\tLocalization::updateCameraObservations(CameraObservation)> ";

  if ( ITL_DEBUG ) 
    cout << label << "locking robotMutex" << endl; 

  robotMutex.lock();

  if ( ITL_DEBUG ) 
    cout << label << "Acquired mutex lock" << endl; 

  received_camobs.clear(); 
  received_camobs.push_back(cobs);

  if ( ITL_DEBUG ) 
    cout << label << "Releasing mutex lock" << endl; 

  robotMutex.unlock();
} // end of updateCameraObservations()


/**
 * getBlobColor()
 *
 * this function should get the values for the colors from the player
 * colors file. Currently it is hard coded the color values are
 * display values designated by the user in the colors file, not the
 * actual values observed by the blob finder proxy. In other words,
 * blob.color returns the display value not the actual rgb observed. 
 *
 */
int Localization::getBlobColor(player_blobfinder_blob blob) {
  uint32_t color = blob.color;
  int b = color % 256;
  color = color / 256;
  int g = color % 256;
  color = color / 256;
  int r = color % 256;
  
  if (r == 255 && g == 0 && b == 255)
    return COLOR_PINK;
  else if (r == 255 && g == 255 && b == 0)
    return COLOR_YELLOW;
  else if (r == 0 && g == 255 && b == 0)
    return COLOR_GREEN;
  else if (r == 0 && g == 0 && b == 255)
    return COLOR_BLUE;
  else if (r == 255 && g == 125 && b == 0)
    return COLOR_ORANGE;
  else {
    return -1;
  }
} // end of getBlobColor()


/**
 * isRotationSuccessful()
 *
 * Returns true if the rotation after a move command yields at least
 * half the intended rotation, false otherwise.
 *
 */
bool Localization::isRotationSuccessful(){
  
  double targeted_rotation = abs(destination.getTheta() - startPos.getTheta()); 
  if ( targeted_rotation > M_PI ) 
    targeted_rotation -= M_PI; 

  double delta_rotation = abs(getPosition().getTheta() - startPos.getTheta()); 
  if ( delta_rotation > M_PI ) 
    delta_rotation -= M_PI; 

  /*
  if (ITL_DEBUG){
    cout << "\t targeted_rotation:  " << Utils::toDegrees(targeted_rotation) << endl;
    cout << "\t delta_rotation: " << Utils::toDegrees(delta_rotation) << endl; 
  }
  */

  // if the turn angle is too small ( < 20 degrees) return true. we are not reading positions that accurately yet. 
  if ( targeted_rotation < 0.35 ) 
    return true;

  if ( targeted_rotation > (2 * delta_rotation) )
    return false;
 
  return true; 
} // end of isRotationSuccessful()


/**
 * setLatestPosition()
 *
 */
void Localization::setLatestPosition(Position p){
  string label = "\t\tLocalization::setLatestPosition> " ; 
  
  int currentTime = time(NULL);  
  if (ITL_DEBUG) cout << label << "(" << p.getX() << "," << p.getY() << "," << p.getTheta() << 
		   ") currentTime: " << currentTime << ", latestPosTime: " << latestPosTime << endl ;
  
  if ( positionChangePossible(latestPos, latestPosTime, p, currentTime) ) {
    latestPosTime = currentTime; 
    latestPos = p; 
    setCurrentPosition(p);
  }
  else {
    if ( suspectPosCount == 0 ) {
      suspectPos = p ; 
      suspectPosTime = currentTime; 
      suspectPosCount++; 
      if (ITL_DEBUG) cout << label << "Discarding latest position, cannot travel that far" << endl; 
    }
    else if ( suspectPosCount <= 2 && positionChangePossible(suspectPos, suspectPosTime, p, currentTime)){
      suspectPos = p ; 
      suspectPosTime = currentTime; 
      suspectPosCount++; 
      if (ITL_DEBUG) cout << label << "Received a suspicious position info count:" << suspectPosCount << endl; 
    }

    // third time we receive a campose which is far away from our latest reliable position, we start believing in it ( bad hack )
    if ( suspectPosCount > 2 ) {
      if (ITL_DEBUG) cout << label << "Can't be a coincidence, I'm updating my position " << suspectPosCount << endl; 
      latestPos = p ; 
      latestPosTime = currentTime;
      suspectPosCount = 0; 
      setCurrentPosition(p);
    }
  }
} // end of setLatestPosition()


/**
 * positionChangePossible()
 *
 */
bool Localization::positionChangePossible(Position startPos, int startPosTime, Position endPos, int endPosTime){
  string label = "\t\tLocalization::positionChangePossible> " ; 

  double maxTravelDistPerSecond = 30 ;   // in cm

  double timeDiff = (endPosTime - startPosTime);  
  double threshold; 
  (timeDiff != 0) ? threshold = maxTravelDistPerSecond * timeDiff : threshold = maxTravelDistPerSecond ;
  double distFromStartPos = Utils::get_euclidian_distance(startPos.getX(), startPos.getY(), endPos.getX(), endPos.getY());

  return ( distFromStartPos < threshold ) ; 
} // end of positionChangePossible()


/**
 * motion()
 *
 * cmd | left | right | translates to:
 * ----+------+-------+-----------------------------------
 * 'F' |  127 |   127 | forward
 * 'a' | -127 |   127 | in-place turn, anti-clockwise
 * 'A' |    0 |   127 | wide turn, anti-clockwise
 * ----+------+-------+-----------------------------------
 * 'c' |  127 |  -127 | in-place turn, clockwise
 * 'B' | -127 |  -127 | backward
 * n/a |    0 |  -127 | wide turn, clockwise
 * ----+------+-------+-----------------------------------
 * 'C' |  127 |     0 | wide turn, clockwise
 * n/a | -127 |     0 | wide turn, counter-clockwise
 * 'H' |    0 |     0 | halt (stop)
 * ----+------+-------+-----------------------------------
 *
 * modifier values:
 * '1' = very short
 * '2' = short
 * '3' = medium
 * '4' = long
 * '5' = extra long
 *
 */
void Localization::motion( char cmd, char modifier ) {
  const int MOTOR_ON_POS = 127;
  const int MOTOR_ON_POS_SMALL = 35;
  const int MOTOR_ON_NEG = -127;
  const int MOTOR_ON_NEG_SMALL = -35;
  const int MOTOR_OFF = 0;
  const int V_TIME = 5;
  const int S_TIME = 10;
  const int M_TIME = 20;
  const int L_TIME = 40;
  const int X_TIME = 100;
  int xs, ys, ts;
  switch( cmd ) {
  case 'F': // forward
    xs = MOTOR_ON_POS;
    ys = MOTOR_ON_POS;
    break;
  case 'B': // backward
    xs = MOTOR_ON_NEG;
    ys = MOTOR_ON_NEG;
    break; 
  case 'c': // in-place turn, clockwise
    xs = MOTOR_ON_POS;
    ys = MOTOR_ON_NEG;
    break; 
  case 'C': // wide turn, clockwise
    xs = MOTOR_ON_POS;
    //    ys = MOTOR_OFF;
    ys = MOTOR_ON_POS_SMALL;
    break; 
  case 'a': // in-place turn, anti-clockwise
    xs = MOTOR_ON_NEG;
    ys = MOTOR_ON_POS;
    break; 
  case 'A': // wide turn, anti-clockwise
    //    xs = MOTOR_OFF;
    xs = MOTOR_ON_POS_SMALL;
    ys = MOTOR_ON_POS;
    break; 
  case 'H': // halt (stop)
  default:
    xs = MOTOR_OFF;
    ys = MOTOR_OFF;
  } // end of switch cmd
  switch( modifier ) {
  case '1':
    ts = V_TIME;
    break;
  case '2':
    ts = S_TIME;
    break;
  case '3':
    ts = M_TIME;
    break;
  case '4':
    ts = L_TIME;
    break;
  case '5':
    ts = X_TIME;
    break;
  default:
    ts = 0;
  } // end of modifier switch

  moveCompleted = false;
  resetDestinationInfo();

  p2d->SetSpeed( xs, ys, ts );

  using namespace boost::posix_time;

  move_expiration = ptime(microsec_clock::local_time()) + milliseconds(ts * 15) + milliseconds(250);

} // end of motion()

bool Localization::isMoveCompleted() {

  using namespace boost::posix_time;
  
  ptime now(microsec_clock::local_time());

  if ( now >= move_expiration ) {
    moveCompleted = true;
  }
 
  return moveCompleted ; 
}
