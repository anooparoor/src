
#include <AgentState.h>
#include <iostream>

Position AgentState::getExpectedPositionAfterAction(FORRAction action){
  Position expectedPosition;
  return expectedPosition;
}
  

double AgentState::getForwardObstacleDistance(){
   int scan_lines = currentLaserScan.ranges.size();	
   return currentLaserScan.ranges[scan_lines/2];
}

Position AgentState::getExpectedPositionAfterActions(vector<FORRAction> actions){

  Position expectedPosition;
  return expectedPosition;

}

//return the distance to obstacle after taking an action
double AgentState::getDistanceToObstacle(FORRAction action){
   //Position p1 = getExpectedPositionAfterAction(action);
   //return p1.getDistance();
   return 0;
}

//return the distance to obstacle from the robots current location
double AgentState::getDistanceToObstacle(){

   
   return 0;
}


FORRAction AgentState::get_max_allowed_forward_move(){
  FORRAction max_forward(FORWARD, 5);
  cout << " Number of vetoed actions : " << vetoedActions->size() << endl;
  for(int intensity = 1; intensity <= 5 ; intensity++){
    if(vetoedActions->find(FORRAction(FORWARD,intensity)) != vetoedActions->end()){
      max_forward.type = FORWARD;
      max_forward.parameter = intensity - 1;
      break;
    }
  }
  return max_forward;
}


bool AgentState::isTargetInSight(){
  return true;
}


double AgentState::get_move_length(FORRAction forward_move){
  switch(forward_move.parameter){
  case 1:
    return 3;
  case 2:
    return 7;
  case 3:
    return 20;
  case 4:
    return 25;
  case 5: 
    return 105;
  default:
    return 0;
  }
}


double getForwardObstacleDistance(){


  return 0;

}

FORRAction AgentState::singleMoveTowardsTarget(){
    FORRAction max_forward_move = get_max_allowed_forward_move();
    cout << "Target in sight , victory advisor active" << endl;
    double distance_from_target = get_euclidian_distance(currentPosition.getX(), currentPosition.getY(), currentTask->getX(), currentTask->getY());
    cout << "Distance from target : " << distance_from_target << endl;
    // compute the angular difference between the direction to the target and the current robot direction
    double robot_direction = currentPosition.getTheta();
    double goal_direction = atan2((currentTask->getY() - currentPosition.getY()), (currentTask->getX() - currentPosition.getX()));
    //if(task->getX() >= curr_pos.getX())
    //goal_direction = goal_direction + 3.1415;
    //goal_direction = goal_direction - 3.1415;
    double required_rotation = goal_direction - robot_direction;
    if(required_rotation > 3.39)
      required_rotation = required_rotation - 6.283;
    if(required_rotation < -3.39)
      required_rotation = required_rotation + 6.283;
    //cout << "Robot direction : " << robot_direction << ", Goal Direction : " << goal_direction << ", Required rotation : " << required_rotation << endl;
    // if the angular difference is greater than smallest turn possible 
    // pick the right turn to allign itself to the target
    
    FORRAction decision;

    if(fabs(required_rotation) > 0.3048){
      //cout << "Rotation move made " << endl;
      if( required_rotation > 0.1548 && required_rotation <= 0.3048)
	decision = FORRAction(LEFT_TURN, 1);
      else if( required_rotation > 0.3048 && required_rotation <= 0.65)
	decision = FORRAction(LEFT_TURN, 2);
      else if( required_rotation > 0.65 && required_rotation <= 1.3)
	decision = FORRAction(LEFT_TURN, 3);
      else if(required_rotation > 1.3 && required_rotation <= 3.39)
	decision = FORRAction(LEFT_TURN, 4);
      else if( required_rotation < -0.1548 && required_rotation >= -0.3048)
	decision = FORRAction(RIGHT_TURN, 1);
      else if( required_rotation < -0.3048 && required_rotation >= -0.65)
	decision = FORRAction(RIGHT_TURN, 2);
      else if( required_rotation < -0.65 && required_rotation >= -1.3)
	decision = FORRAction(RIGHT_TURN, 3);
      else if(required_rotation < -1.3 && required_rotation >= -3.39)
	decision = FORRAction(RIGHT_TURN, 4);
    }
    else if(max_forward_move.parameter == 5 || distance_from_target < get_move_length(max_forward_move)){
      int intensity;
      //cout << "Forward move made" << endl;
      if(distance_from_target <= 3)
	intensity = 0;
      else if(distance_from_target <= 7 )
	intensity = 1;
      else if(distance_from_target <= 20)
	intensity = 2;
      else if(distance_from_target <= 25)
	intensity = 3;
      else if(distance_from_target <= 105)
	intensity = 4;
      else
	intensity = 5;
      decision = FORRAction(FORWARD,intensity);
      cout << "Intensity of the forward move : " << intensity << endl;
    }
    return decision;
}


vector<double> AgentState::get_angle_difference_vector(){
  // create vector that goes from robot to target
  double rotations[] = {0, .1548, -.1548, .3048, -.3048, .65, -.65, 1.3, -1.3};
  double robot_target_x = currentTask->getX() - currentPosition.getX();
  double robot_target_y = currentTask->getY() - currentPosition.getY();
  double robot_yaw = currentPosition.getTheta(); 
  double angle_difference; // difference between our robot-target and directional vector of movement
  vector<double> angle_difference_vector;

  //initialize angle_difference_vector
  for(int j = 0; j < 18; j++)
    angle_difference_vector.push_back(0);
  
  // all I need is angle of that vector with x axis in my system
  double robot_target_angle = acos(robot_target_x/sqrt((robot_target_x*robot_target_x + robot_target_y * robot_target_y)));
  // weird but this is how our HR coordinate system works
  if(robot_target_y < 0)
    robot_target_angle *= -1;
  // cout << "Robot-target angle: " << robot_target_angle << endl;
  for(int i = 0; i < 9; ++i){
    // this is for forward
    angle_difference = robot_target_angle - (robot_yaw + rotations[i]);
    angle_difference = normalize_angle(angle_difference);
    //cout << angle_difference << " ";
    angle_difference_vector[i] = angle_difference;
    // this is for going backwards along the same rotation
    if(angle_difference > 0)
      angle_difference = M_PI + robot_yaw - angle_difference;
    else
      angle_difference = M_PI + robot_yaw + angle_difference;
    //cout << endl;
    angle_difference = normalize_angle(angle_difference);
    angle_difference_vector[17 - i] = angle_difference;
  }
  return angle_difference_vector;
}

double AgentState::normalize_angle(double angle){
  int quotient;
  if(angle > 2*M_PI){
    quotient = ceil(angle / (2*M_PI));
    angle = angle - quotient * (2*M_PI);
  }
  else if(angle < -2 * M_PI){
    quotient = floor(angle / (-2*M_PI));
    angle = angle + quotient * (2*M_PI);
  }
  if(angle > M_PI)
    angle = angle - 2 * M_PI;
  if(angle < -M_PI)
    angle = angle + 2 * M_PI;
  return angle;
}

double AgentState::getDistanceToTarget(){
   return currentPosition.getDistance(currentTask->getX(), currentTask->getY());
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
  for(int i = 0; i < givenLaserEndpoints.size(); i++){
    //this can be cleaned up or put into one function, but need to convert the distance to a linesegment to calculate
    //the distance from a point to a line
    CartesianPoint endpoint = givenLaserEndpoints[i];
    
    LineSegment distance_vector_line = LineSegment(laserPos, endpoint);
    LineSegment trail_segment = LineSegment(point1, point2);
    if(do_intersect(distance_vector_line, trail_segment, intersection_point)){
      return true;
    }
  }
  //else, not visible
  return false;
}


//returns true if there is a point that is "visible" by the wall distance vectors to some epsilon.  
//A point is visible if the distance to a wall distance vector line is < epsilon.
bool AgentState::canSeePoint(vector<CartesianPoint> givenLaserEndpoints, CartesianPoint laserPos, CartesianPoint point, double epsilon){
  for(int i = 0; i < givenLaserEndpoints.size(); i++){
    //this can be cleaned up or put into one function, but need to convert the distance to a linesegment to calculate
    //the distance from a point to a line
    LineSegment l = LineSegment(laserPos, givenLaserEndpoints[i]);
    double distance_to_point = distance(point, l);
    if(distance_to_point < epsilon){
      //cout << "Distance vector endpoint visible: ("<<laserEndpoints[i].get_x()<<","<< laserEndpoints[i].get_y()<<")"<<endl; 
      //cout << "Distance: "<<distance_to_point<<endl;
      return true;
    }
  }
  //else, not visible
  return false;
}



//returns true if there is a point that is "visible" by the wall distance vectors to some epsilon.  
//A point is visible if the distance to a wall distance vector line is < epsilon.
bool AgentState::canSeePoint(CartesianPoint point, double epsilon){
  CartesianPoint curr(currentPosition.getX(),currentPosition.getY());
  return canSeePoint(laserEndpoints, curr, point, epsilon);
}

void AgentState::transformToEndpoints(){
    double start_angle = currentLaserScan.angle_min;
    double increment = currentLaserScan.angle_increment;

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


