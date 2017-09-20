
#include <AgentState.h>
#include <iostream>

/*
Position AgentState::getExpectedPositionAfterActions(Position initialPosition, vector<FORRAction> actionList){
  Position currentPosition = initialPosition;
  Position nextPosition;
  for(int i = 0 ; i < actionList.size(); i++){
    FORRAction action = actionList[i];
    nextPosition = getExpectedPositionAfterAction(currentPosition, action);
    // cout << "AgentState :: getExpectedPositionAfterActions >> nextposition " << nextPosition.getX() << " " << nextPosition.getY() << endl;
    currentPosition = nextPosition;
  }
  return nextPosition;
}
*/

Position AgentState::getExpectedPositionAfterAction(FORRAction action){
  Position result; 
  int intensity = action.parameter;
  FORRActionType type = action.type;
  Position initialPosition = getCurrentPosition();
  
  switch(type){
  case FORWARD:
    result = afterLinearMove(initialPosition, move[intensity]);
    break;
  case RIGHT_TURN:
    result = afterAngularMove(initialPosition, -rotate[intensity]);
    break;
  case LEFT_TURN:
    result = afterAngularMove(initialPosition, rotate[intensity]);
    break;
  case PAUSE:
    result = initialPosition;
    break;
  }
  return result;
}
  

Position AgentState::afterLinearMove(Position initialPosition, double distance){
  if(distance > getDistanceToForwardObstacle()) distance = getDistanceToForwardObstacle();

  double new_x = initialPosition.getX() + (distance * cos(initialPosition.getTheta()));
  double new_y = initialPosition.getY() + (distance * sin(initialPosition.getTheta()));
  
  return Position(new_x, new_y, initialPosition.getTheta());
}


Position AgentState::afterAngularMove(Position initialPosition, double angle){

  double new_angle = (angle + initialPosition.getTheta());
  
  double distance = getDistanceToObstacle(new_angle);
  //max is the maximum look ahead in meters 
  double max = move[numMoves-1];
  if(distance > max) distance = max;

  double new_x = initialPosition.getX() + (distance * cos(new_angle));
  double new_y = initialPosition.getY() + (distance * sin(new_angle));
  
  return Position(new_x, new_y, new_angle);
}


//return the distance to obstacle when the robot is in POS using the current laser scan
double AgentState::getDistanceToNearestObstacle(Position pos){ 
   double min_dis = 1000000;
   for(int i = 0; i < laserEndpoints.size(); i++){
	double x = laserEndpoints[i].get_x();
 	double y = laserEndpoints[i].get_y();
	double dist = pos.getDistance(Position(x,y,0));
	if(dist < min_dis)
		min_dis = dist; 
   }
   return min_dis;
}


//Sees if the laser scan intersects with a segment created by 2
//trailpoints
bool AgentState::canSeeSegment(CartesianPoint point1, CartesianPoint point2){
  CartesianPoint curr(currentPosition.getX(),currentPosition.getY());
  return canSeeSegment(laserEndpoints, curr, point1, point2);
}

//Sees if the laser scan intersects with a segment created by 2
//trailpoints
bool AgentState::canSeeSegment(vector<CartesianPoint> givenLaserEndpoints, CartesianPoint laserPos, CartesianPoint point1, CartesianPoint point2){
  CartesianPoint intersection_point(0,0);
  bool canSeeSegment = false;
  for(int i = 0; i < givenLaserEndpoints.size(); i++){
    //this can be cleaned up or put into one function, but need to convert the distance to a linesegment to calculate
    //the distance from a point to a line
    CartesianPoint endpoint = givenLaserEndpoints[i];
    
    LineSegment distance_vector_line = LineSegment(laserPos, endpoint);
    LineSegment trail_segment = LineSegment(point1, point2);
    if(do_intersect(distance_vector_line, trail_segment, intersection_point)){
      canSeeSegment = true;
	break;
    }
  }
  //else, not visible
  return canSeeSegment;
}


//returns true if there is a point that is "visible" by the wall distance vectors to some epsilon.  
//A point is visible if the distance to a wall distance vector line is < epsilon.
bool AgentState::canSeePoint(vector<CartesianPoint> givenLaserEndpoints, CartesianPoint laserPos, CartesianPoint point){
  ROS_DEBUG_STREAM("AgentState:canSeePoint() , robot pos " << laserPos.get_x() << "," << laserPos.get_y() << " target " <<
	point.get_x() << "," << point.get_y()); 
  double epsilon = canSeePointEpsilon;
  ROS_DEBUG_STREAM("Number of laser endpoints " << givenLaserEndpoints.size()); 
  bool canSeePoint = false;
  double ab = laserPos.get_distance(point);
  for(int i = 0; i < givenLaserEndpoints.size(); i++){
    //ROS_DEBUG_STREAM("Laser endpoint : " << givenLaserEndpoints[i].get_x() << "," << givenLaserEndpoints[i].get_y());
    double ac = laserPos.get_distance(givenLaserEndpoints[i]);
    double bc = givenLaserEndpoints[i].get_distance(point);
    if(((ab + bc) - ac) < epsilon){
      //cout << "Distance vector endpoint visible: ("<<laserEndpoints[i].get_x()<<","<< laserEndpoints[i].get_y()<<")"<<endl; 
      //cout << "Distance: "<<distance_to_point<<endl;
      canSeePoint = true;
      break;
    }
  }
  //else, not visible
  return canSeePoint;
}

//returns true if there is a point that is "visible" by the wall distance vectors.  
//A point is visible if the distance to the nearest wall distance vector lines is > distance to the point.
bool AgentState::canAccessPoint(vector<CartesianPoint> givenLaserEndpoints, CartesianPoint laserPos, CartesianPoint point){
  ROS_DEBUG_STREAM("AgentState:canAccessPoint() , robot pos " << laserPos.get_x() << "," << laserPos.get_y() << " target " << point.get_x() << "," << point.get_y()); 
  ROS_DEBUG_STREAM("Number of laser endpoints " << givenLaserEndpoints.size()); 
  bool canSeePoint = false;
  double distLaserPosToPoint = laserPos.get_distance(point);
  double point_direction = atan2((point.get_y() - laserPos.get_y()), (point.get_x() - laserPos.get_x()));
  int index = 0;
  double min_angle = 100000;

  for(int i = 0; i < givenLaserEndpoints.size(); i++){
    //ROS_DEBUG_STREAM("Laser endpoint : " << givenLaserEndpoints[i].get_x() << "," << givenLaserEndpoints[i].get_y());
    double laser_direction = atan2((givenLaserEndpoints[i].get_y() - laserPos.get_y()), (givenLaserEndpoints[i].get_x() - laserPos.get_x()));
    if(abs(laser_direction - point_direction) < min_angle){
      //ROS_DEBUG_STREAM("Laser Direction : " << laser_direction << ", Point Direction : " << point_direction);
      min_angle = abs(laser_direction - point_direction);
      index = i;
    }
  }
  //ROS_DEBUG_STREAM("Min angle : " << min_angle << ", " << index);
  int numFree = 0;
  for(int i = -2; i < 3; i++) {
    double distLaserEndPointToLaserPos = givenLaserEndpoints[index+i].get_distance(laserPos);
    //ROS_DEBUG_STREAM("Distance Laser EndPoint to Laser Pos : " << distLaserEndPointToLaserPos << ", Distance Laser Pos to Point : " << distLaserPosToPoint);
    if (distLaserEndPointToLaserPos > distLaserPosToPoint) {
      numFree++;
    }
  }
  //ROS_DEBUG_STREAM("Number farther than point : " << numFree);
  if (numFree > 4) {
    canSeePoint = true;
  }
  //else, not visible
  return canSeePoint;
}


std::pair < std::vector<CartesianPoint>, std::vector< vector<CartesianPoint> > > AgentState::getCleanedTrailMarkers(){
	std::pair < std::vector<CartesianPoint>, std::vector<vector<CartesianPoint> > > cleanedMarker;
	// Change position history into CartesianPoint format
	vector <CartesianPoint> pos_history;
	for(std::vector<Position>::iterator it = currentTask->getPositionHistory()->begin() ; it != currentTask->getPositionHistory()->end(); ++it){
		CartesianPoint pos(it->getX(),it->getY());
		pos_history.push_back(pos);
	}
	// Initialize trail vectors
	std::vector<CartesianPoint> trailPositions;
	std::vector< vector<CartesianPoint> > trailLaserEndpoints;
	//Get laser history
	std::vector< vector<CartesianPoint> > laser_endpoints = *(currentTask->getLaserHistory());
	// Push first point in path to trail
	trailPositions.push_back(pos_history[0]);
	trailLaserEndpoints.push_back(laser_endpoints[0]);
	// Find the furthest point on path that can be seen from current position, push that point to trail and then move to that point
	for(int i = 0; i < pos_history.size(); i++){
		//cout << "First point: " << pos_history[i].get_x() << " " << pos_history[i].get_y() << endl;
		for(int j = pos_history.size()-1; j > i; j--){
			//cout << pos_history[j].get_x() << " " << pos_history[j].get_y() << endl;
			//if(canSeePoint(laser_endpoints[i], pos_history[i], pos_history[j])) {
      //if(canSeePoint(laser_endpoints[i], pos_history[i], pos_history[j]) and canSeePoint(laser_endpoints[j], pos_history[j], pos_history[i])) {
      if(canAccessPoint(laser_endpoints[i], pos_history[i], pos_history[j])) {
				cout << "CanAccessPoint is true" << endl;
				cout << "Next point: " << pos_history[j].get_x() << " " << pos_history[j].get_y() << endl;
				trailPositions.push_back(pos_history[j]);
				trailLaserEndpoints.push_back(laser_endpoints[j]);
				i = j-1;
			}
		}
	}
  if (pos_history[pos_history.size()-1].get_x() != trailPositions[trailPositions.size()-1].get_x() or pos_history[pos_history.size()-1].get_y() != trailPositions[trailPositions.size()-1].get_y()) {
    trailPositions.push_back(pos_history.back());
    trailLaserEndpoints.push_back(laser_endpoints.back());
  }
	cout << pos_history[pos_history.size()-1].get_x() << " " << pos_history[pos_history.size()-1].get_y() << endl;
	cout << trailPositions[trailPositions.size()-1].get_x() << " " << trailPositions[trailPositions.size()-1].get_y() << endl;
	
	cleanedMarker.first = trailPositions;
	cleanedMarker.second = trailLaserEndpoints;
	return cleanedMarker;
}


//returns true if there is a point that is "visible" by the wall distance vectors to some epsilon.  
//A point is visible if the distance to a wall distance vector line is < epsilon.
bool AgentState::canSeePoint(CartesianPoint point){
  CartesianPoint curr(currentPosition.getX(),currentPosition.getY());
  return canSeePoint(laserEndpoints, curr, point);
}

void AgentState::transformToEndpoints(){
    ROS_DEBUG("Convert laser scan to endpoints");
    double start_angle = currentLaserScan.angle_min;
    double increment = currentLaserScan.angle_increment;
    laserEndpoints.clear();

    double r_x = currentPosition.getX();
    double r_y = currentPosition.getY();
    double r_ang = currentPosition.getTheta();
    CartesianPoint current_point(r_x,r_y);
  
    for(int i = 0 ; i < currentLaserScan.ranges.size(); i++){
       Vector v = Vector(current_point, start_angle + r_ang, currentLaserScan.ranges[i]);
       CartesianPoint endpoint = v.get_endpoint();
       start_angle = start_angle + increment;
       laserEndpoints.push_back(endpoint);
    }    
}

double AgentState::getDistanceToObstacle(double rotation_angle){
	//ROS_DEBUG("In getDistanceToObstacle");
	// one increment in the laser range scan is 1/3 degrees, i.e 0.005817 in radians  
	int index = (int)(rotation_angle/(laserScanRadianIncrement));
	//ROS_DEBUG("In getDistanceToObstacle after index");
	//shift the index in the positive
	index = index + 330;
	//ROS_DEBUG("In getDistanceToObstacle after index shift");
	if(index < 0) index = 0;
	//ROS_DEBUG("In getDistanceToObstacle after first if");
	if(index > 659) index = 659;
	//ROS_DEBUG("In getDistanceToObstacle after second if");
	//cout << index << " " << currentLaserScan.ranges.size() << endl;
	if(currentLaserScan.ranges.size() == 0) { return maxLaserRange; }
	//cout << currentLaserScan.ranges[index] << endl;
	
	double r_x = currentPosition.getX();
	double r_y = currentPosition.getY();
	double r_ang = currentPosition.getTheta();
	CartesianPoint current_point(r_x,r_y);
	double r = robotFootPrint+robotFootPrintBuffer; // fetch robot's footprint plus 0.1 meter buffer
	
	Vector v1 = Vector(current_point, r_ang+(M_PI/2), r);
	Vector v2 = Vector(current_point, r_ang-(M_PI/2), r);
	
	Vector parallel1 = Vector(v1.get_endpoint(), r_ang, maxLaserRange);
	Vector parallel2 = Vector(v2.get_endpoint(), r_ang, maxLaserRange);
	
	Vector laserVector = Vector(current_point, r_ang+rotation_angle, maxLaserRange);
	
	CartesianPoint intersectionPoint = CartesianPoint();
	
	if(do_intersect(parallel1, laserVector, intersectionPoint)){
		if(intersectionPoint.get_distance(current_point) < currentLaserScan.ranges[index]){
			//cout << "25" << endl;
			return maxLaserRange;
		}
		else {
			//cout << currentLaserScan.ranges[index] << endl;
			return currentLaserScan.ranges[index];
		}
	}
	else if(do_intersect(parallel2, laserVector, intersectionPoint)){
		if(intersectionPoint.get_distance(current_point) < currentLaserScan.ranges[index]){
			//cout << "25" << endl;
			return maxLaserRange;
		}
		else {
			//cout << currentLaserScan.ranges[index] << endl;
			return currentLaserScan.ranges[index];
		}
	}
	else {
		//cout << currentLaserScan.ranges[index] << endl;
		return currentLaserScan.ranges[index];
	}
}


FORRAction AgentState::maxForwardAction(){
 	ROS_DEBUG("In maxforwardaction");
	double error_margin = maxForwardActionBuffer; // margin from obstacles
	double view = maxForwardActionSweepAngle; // +view radians to -view radians view
	//double view = 0.7854;
	double min_distance = getDistanceToObstacle(view);
	//cout << min_distance << endl;
	for(double angle = (-1)*view; angle < view; angle = angle + 0.005){
		//cout << angle << endl;
		double distance = getDistanceToObstacle(angle);
		if(distance < min_distance){
			min_distance = distance;
		} 
	}
	
	min_distance = min_distance - error_margin;

	ROS_DEBUG_STREAM("Forward obstacle distance " << min_distance);
  for (int i = numMoves-1; i > 0; i--) {
    if(min_distance > move[i]) {
      return FORRAction(FORWARD,i);
    }
  }
  return FORRAction(FORWARD,0);
	/*if(min_distance > move[5])
		return FORRAction(FORWARD,5);
	else if(min_distance > move[4] && min_distance <= move[5])
		return FORRAction(FORWARD,4);
	else if(min_distance > move[3] && min_distance <= move[4])
		return FORRAction(FORWARD,3);
	else if(min_distance > move[2] && min_distance <= move[3])
		return FORRAction(FORWARD,2);
	else if(min_distance > move[1] && min_distance <= move[2])
		return FORRAction(FORWARD,1);
	else
		return FORRAction(FORWARD,0);*/

}


FORRAction AgentState::get_max_allowed_forward_move(){
  FORRAction max_forward(FORWARD, numMoves-1);
  cout << " Number of vetoed actions : " << vetoedActions->size() << endl;
  for(int intensity = 1; intensity <= numMoves; intensity++){
    if(vetoedActions->find(FORRAction(FORWARD,intensity)) != vetoedActions->end()){
      max_forward.type = FORWARD;
      max_forward.parameter = intensity - 1;
      break;
    }
  }
  return max_forward;
}


// returns an Action that takes the robot closest to the target
FORRAction AgentState::moveTowards(){
    ROS_DEBUG("AgentState :: In moveTowards");
    double distance_from_target = currentPosition.getDistance(currentTask->getX(), currentTask->getY());
    ROS_DEBUG_STREAM("Distance from target : " << distance_from_target);
    // compute the angular difference between the direction to the target and the current robot direction
    double robot_direction = currentPosition.getTheta();
    double goal_direction = atan2((currentTask->getY() - currentPosition.getY()), (currentTask->getX() - currentPosition.getX()));
    
    double required_rotation = goal_direction - robot_direction;

    ROS_DEBUG_STREAM("Robot direction : " << robot_direction << ", Goal Direction : " << goal_direction << ", Required rotation : " << required_rotation);
    if(required_rotation > M_PI)
      required_rotation = required_rotation - (2*M_PI);
    if(required_rotation < -M_PI)
      required_rotation = required_rotation + (2*M_PI);
    //cout << "Robot direction : " << robot_direction << ", Goal Direction : " << goal_direction << ", Required rotation : " << required_rotation << endl;
    ROS_DEBUG_STREAM("Robot direction : " << robot_direction << ", Goal Direction : " << goal_direction << ", Required rotation : " << required_rotation);
    // if the angular difference is greater than smallest turn possible 
    // pick the right turn to allign itself to the target
    
    FORRAction decision;
    int rotIntensity=0;
    while(fabs(required_rotation) > rotate[rotIntensity] and rotIntensity < numRotates) {
      rotIntensity++;
    }
    ROS_DEBUG_STREAM("Rotation Intensity : " << rotIntensity);
    if (rotIntensity > 1) {
      if (required_rotation < 0){
        decision = FORRAction(RIGHT_TURN, rotIntensity-1);
      }
      else {
        decision = FORRAction(LEFT_TURN, rotIntensity-1);
      }
    }
    else {
      int intensity=0;
      while(distance_from_target > move[intensity] and intensity < numMoves) {
        intensity++;
      }
      ROS_DEBUG_STREAM("Move Intensity : " << intensity);
      int max_allowed = maxForwardAction().parameter;
      if(intensity > max_allowed){
          intensity = max_allowed;
      }
      decision = FORRAction(FORWARD, intensity);
    }

    /*if(fabs(required_rotation) > rotate[1]){
      if( required_rotation > rotate[1] && required_rotation <= rotate[2])
        decision = FORRAction(LEFT_TURN, 1);
      else if( required_rotation > rotate[2] && required_rotation <= rotate[3])
        decision = FORRAction(LEFT_TURN, 2);
      else if(required_rotation > rotate[3] && required_rotation <= rotate[4])
        decision = FORRAction(LEFT_TURN, 3);
      else if(required_rotation > rotate[4])
        decision = FORRAction(LEFT_TURN, 4);
      else if( required_rotation < -rotate[1] && required_rotation >= -rotate[2])
        decision = FORRAction(RIGHT_TURN, 1);
      else if( required_rotation < -rotate[2] && required_rotation >= -rotate[3])
        decision = FORRAction(RIGHT_TURN, 2);
      else if(required_rotation < -rotate[3] && required_rotation >= -rotate[4])
        decision = FORRAction(RIGHT_TURN, 3);
      else if(required_rotation < -rotate[4])
        decision = FORRAction(RIGHT_TURN, 4);
    }
    else{
      int intensity;
      if(distance_from_target <= move[1])
        intensity = 0;
      else if(distance_from_target <= move[2])
        intensity = 1;
      else if(distance_from_target <= move[3])
        intensity = 2;
      else if(distance_from_target <= move[4])
        intensity = 3;
      else if(distance_from_target <= move[5])
        intensity = 4;
      else
        intensity = 5;
      int max_allowed = maxForwardAction().parameter;
      if(intensity > max_allowed){
        intensity = max_allowed;
      }
      decision = FORRAction(FORWARD, intensity);
    }*/
    
    ROS_INFO_STREAM("Action choosen : " << decision.type << "," << decision.parameter);
    return decision;
}

  void AgentState::setAgentStateParameters(double val1, double val2, double val3, double val4, double val5, double val6, double val7){
    canSeePointEpsilon = val1;
    laserScanRadianIncrement = val2;
    robotFootPrint = val3;
    robotFootPrintBuffer = val4;
    maxLaserRange = val5;
    maxForwardActionBuffer = val6;
    maxForwardActionSweepAngle = val7;
  }
