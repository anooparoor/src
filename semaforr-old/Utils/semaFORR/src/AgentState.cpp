
#include <AgentState.h>
#include <iostream>


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
Position AgentState::getExpectedPositionAfterAction(Position initialPosition, FORRAction action){
  
  Position result; 
  int intensity = action.parameter;
  FORRActionType type = action.type;
  
  //Estimate of the moves made by slavisa after running the robots on stage multiple times could be different for physical robots
  // <type:parameter> : value (either angular rotation or forward or backward moves)
  //    <NOOP,*> : 0
  //    <FORWARD,1;2;3;4;5> : 3,7,20,25,105
  //    <BACKWARD,1;2;3;4;5> : -3,-7,-20,-25,-105
  //    <RIGHT_TURN,1;2;3;4;5> :
  // Slavisa this part has to be moved to reading from the config file
  double tmp_rotations[] = {.1548, .3048, .65, 1.3, 3.39};  
  int tmp_movements[] = {3, 7, 20, 25, 105};
  
  switch(type){
  case NOOP:
    result = initialPosition;
    break;
  case FORWARD:
    result = afterLinearMove(initialPosition, tmp_movements[intensity-1]);
    break;
  case BACKWARD:
    result = afterLinearMove(initialPosition, -tmp_movements[intensity-1]);
    break;
  case RIGHT_TURN:
    result = afterAngularMove(initialPosition, -tmp_rotations[intensity-1]);
    break;
  case LEFT_TURN:
    result = afterAngularMove(initialPosition, +tmp_rotations[intensity-1]);
    break;
    //currently wide turn is not used in the system
  case WIDE_RIGHT_TURN:
    result = initialPosition;
    break;
    //currently wide turn is not used in the system
  case WIDE_LEFT_TURN:
    result = initialPosition;
    break;
  case PAUSE:
    result = initialPosition;
  case HALT:
    result = initialPosition;
    break;
  }
  return result;
}
  

Position AgentState::afterLinearMove(Position initialPosition, double distance){
  
  double new_x = initialPosition.getX() + (distance * cos(initialPosition.getTheta()));
  double new_y = initialPosition.getY() + (distance * sin(initialPosition.getTheta()));
  
  return Position(new_x, new_y, initialPosition.getTheta());
}


Position AgentState::afterAngularMove(Position initialPosition, double angle){

  double new_angle = (angle + initialPosition.getTheta());
  return Position(initialPosition.getX(), initialPosition.getY(), new_angle);
}


/*
// Descriptive 2. find distance to obstacle related code!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

void DescriptiveManager::create_wall_proximity_vector(string type_parameters){

  vector<double> wallDistanceVec;
  cout << "Received message " << type_parameters << endl;

  string robot_id = type_parameters;
  string position = ((current_robot_position.find(robot_id))->second);
  vector<string> posstr;
  boost::split(posstr, position, boost::is_any_of(" "));

  double pos_x = atof((posstr[0]).c_str());
  double pos_y = atof((posstr[1]).c_str());
  double robot_yaw = atof((posstr[2]).c_str());

  Vector robot = Vector(CartesianPoint(pos_x, pos_y), robot_yaw, 1000);
  double new_angle;
  double max_move_len = 150;


  for (unsigned i = 0; i < rotations.size(); ++i){

    // this magic number 1000 should be some constant like the length 
    // of the diagonal in our currently known map
    new_angle = turn_robot(robot, rotations[i]);
    Vector turned_robot = Vector(CartesianPoint(pos_x, pos_y), new_angle, max_move_len);
    
    // this robot_width should be read from the configuration file and put as member variable in DM
    const int robot_width = 25;
    CartesianPoint left = turned_robot.get_point(robot_width/2, 3.14/2);
    CartesianPoint right = turned_robot.get_point(robot_width/2, -3.14/2);
    //pair<CartesianPoint, CartesianPoint> casting_points = get_points_at_distance(turned_robot, robot_width);
    Vector turned_robot_1 = Vector(left, new_angle, max_move_len);
    Vector turned_robot_2 = Vector(right, new_angle, max_move_len);
    //LineSegment left_right = LineSegment(turned_robot_1.get_point(max_move_len, 0), turned_robot_2.get_point(max_move_len, 0));

    //cout << (left_right.get_endpoints()).first.get_x() << "," << (left_right.get_endpoints()).first.get_y() << "  " << (left_right.get_endpoints()).second.get_x() << "," << (left_right.get_endpoints()).second.get_y() << endl;
    cout << (turned_robot.get_endpoint()).get_x() << "," << (turned_robot.get_endpoint()).get_y() << endl;
    cout << (turned_robot.get_origin()).get_x() << "," << (turned_robot.get_origin()).get_y() << endl;

    cout << (turned_robot_1.get_endpoint()).get_x() << "," << (turned_robot_1.get_endpoint()).get_y() << endl;
    cout << (turned_robot_2.get_endpoint()).get_x() << "," << (turned_robot_2.get_endpoint()).get_y() << endl;
    //cout << "Origin:" << endl;
    cout << (turned_robot_1.get_origin()).get_x() << "," << (turned_robot_1.get_origin()).get_y() << endl;
    cout << (turned_robot_2.get_origin()).get_x() << "," << (turned_robot_2.get_origin()).get_y() << endl;

    //LineSegment lr = LineSegment(turned_robot_1.get_endpoint(), turned_robot_2.get_origin())
    //LineSegment rl = LineSegment(
    double min_distance = 1000;
    CartesianPoint intersection_1, intersection_2, intersection_3;
    double collision_distance_1, collision_distance_2,collision_distance_3, collision_min;
    
    pair<CartesianPoint, CartesianPoint> a_wall;

    for (unsigned int j = 0; j < maze.size(); ++j)  {
      collision_distance_1 = 1000;
      collision_distance_2 = 1000;
      collision_distance_3 = 1000;
      CartesianPoint first = maze[j].get_endpoints().first;
      CartesianPoint second = maze[j].get_endpoints().second;
      //cout << "wall no: " << j << " : " << first.get_x() << "," << first.get_y() << " --- " << second.get_x() <<"," << second.get_y() << endl;
      a_wall = maze[j].get_endpoints();
      LineSegment wall = LineSegment(a_wall.first, a_wall.second);
      
      if(do_intersect(turned_robot_1, wall, intersection_1))
	collision_distance_1 = distance(intersection_1, turned_robot.get_origin());
      //cout << "inter: "  << " : " << intersection_1.get_x() << "," << intersection_1.get_y() << endl;
      if(do_intersect(turned_robot_2, wall, intersection_2))
	collision_distance_2 = distance(intersection_2, turned_robot.get_origin());
      //cout << "inter: "  << " : " << intersection_2.get_x() << "," << intersection_2.get_y() << endl;
      if(do_intersect(turned_robot, wall, intersection_3))
	collision_distance_3 = distance(intersection_3, turned_robot.get_origin());
      //cout << "inter: "  << " : " << intersection_3.get_x() << "," << intersection_3.get_y() << endl;
      //cout << collision_distance_1 << " " << collision_distance_2 << " " << collision_distance_3 << endl;
      collision_min = min(collision_distance_1, min(collision_distance_2, collision_distance_3));
      if(collision_min < min_distance)
	min_distance = collision_min;
      
      //cout << "Checking for collision with wall :" <<  min_distance  << endl;
    }
    wallDistanceVec.push_back(min_distance);
  }  
  string value = vector_to_string(wallDistanceVec); 
  std::map<string,string>::iterator it;
  it = distance_from_obstacles_desc.find(robot_id);
  if(it!=distance_from_obstacles_desc.end())
    {
      distance_from_obstacles_desc.erase(it);
    }
  distance_from_obstacles_desc.insert ( std::pair<string,string>(robot_id,value) );
}

void DescriptiveManager::addEdgeBuffers(){
  pair<FORRWall, FORRWall> edgeWalls;
  vector<FORRWall> wallsList; 
  for(int i = 0 ; i < maze.size(); i++){
    edgeWalls = getEdgeBuffer(maze[i]);
    wallsList.push_back(edgeWalls.first);
    wallsList.push_back(edgeWalls.second);
  }
  for(int i = 0 ; i < wallsList.size(); i++){
    maze.push_back(wallsList[i]);
  }
  cout << "Adding " << wallsList.size() <<" buffer walls to the walls vector" << endl;
  return;
}


pair<FORRWall, FORRWall> DescriptiveManager::getEdgeBuffer(FORRWall a_wall){
  double wall_len = 40;
  cout << "calling get edge buffer" << endl;
  pair <FORRWall, FORRWall > edgeBufferPair;
  CartesianPoint firstEnd = a_wall.get_endpoints().first;
  CartesianPoint secondEnd = a_wall.get_endpoints().second;
  Line line (firstEnd, secondEnd);
  double angle = atan(line.get_slope());
  Vector v1(firstEnd, angle, 5);
  Vector v2(secondEnd, angle, 5);
  FORRWall w1(v1.get_point(wall_len/2, 3.14/2),v1.get_point(wall_len/2, -3.14/2));
  FORRWall w2(v2.get_point(wall_len/2, 3.14/2),v2.get_point(wall_len/2, -3.14/2));
  edgeBufferPair.first = w1;
  edgeBufferPair.second = w2;
  return edgeBufferPair;
}

double DescriptiveManager::roundValue(double x, int round){

   x = x * round;
   return floor(x + 0.5) / round;
}


void DescriptiveManager::addFORRWalls(vector<MapWall> walls){

  for(int i = 0; i < walls.size();i++){
    
    FORRWall wall = FORRWall(walls[i].getX0(), walls[i].getY0(), walls[i].getX1(), walls[i].getY1());
    maze.push_back(wall);
  }
}
// END of descriptive 2. find distance to obstacle !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! END

// Start of code for next target point descrptive........
void DescriptiveManager::getNextTargetPoint(string type_parameters)
{
     CartesianPoint next_target;
     cout << "DM: getNextTargetPoint " << endl;
       // << target_points[0].xCoo << " " << target_points[0].yCoo << endl;
     if(target_points.empty())
     {
       next_target.set_x(10000);
       next_target.set_y(10000);    
     }
     else
     {
        next_target = target_points[0];
        target_points.erase(target_points.begin());
     }
     cout << "DM in getNextTargetPoint " << endl;
     string next_target_string = to_string(next_target.get_x()) + " " + to_string(next_target.get_y()); 
     
     //save the target point in current_target map
     std::map<string,string>::iterator it;
     it = current_target.find(type_parameters);
     if(it!=current_target.end())
       {
	 current_target.erase(it);
       }
     current_target.insert( std::pair<string,string>(type_parameters,next_target_string) );
}


// Start of code for advisor weight descriptive

void DescriptiveManager::findAdvisorWeight(string type_parameters)
{
  // log name is same as the robot id which is same as the session id of the robot controller...
  string robot_id = type_parameters;
  rswl.learnWeights(robot_id);
  vector<double> weights = rswl.activeAdvisorWeights;
  vector<double> commentCount = rswl.activeAdvisorCommentCount;
  
  for (int i = 0; i < weights.size(); i++){
    weights[i] = weights[i]/commentCount[i];
  }
  string value = vector_to_string(weights);
  
  //update the weights in the descriptive manager data structure..
  std::map<string,string>::iterator it;
  it = advisor_weight_desc.find(robot_id);
  if(it!=advisor_weight_desc.end())
    {
      advisor_weight_desc.erase(it);
    }
  advisor_weight_desc.insert ( std::pair<string,string>(robot_id,value) );   
}


vector<double> DescriptiveManager::target_distance_calculation(Vector robot, CartesianPoint target_position){
  
  vector<double> result;

  //TODO
  // ultimately I would like this to be a map not a vector
  for(int i = 0; i < 5; ++i)
    result.push_back(target_distance_going_forward(robot, target_position, i));
  for(int i = 0; i < 5; ++i)
    result.push_back(target_distance_going_backward(robot, target_position, i));
  for(int i = 0; i < 5; ++i)
    result.push_back(target_distance_turning_right(robot, target_position, i));
  for(int i = 0; i < 5; ++i)
    result.push_back(target_distance_turning_left(robot, target_position, i));
  

  return result;
}


double DescriptiveManager::target_distance_going_forward(Vector robot, CartesianPoint target_position, int intensity){

  CartesianPoint robot_position = robot.get_origin();
  double robot_yaw = robot.get_angle();
  CartesianPoint new_position;

  new_position = position_after_move(Vector(robot_position, robot_yaw, movements[intensity]));
    
  return distance(new_position, target_position);
}



double DescriptiveManager::target_distance_going_backward(Vector robot, CartesianPoint target_position, int intensity){
  
  CartesianPoint robot_position = robot.get_origin();
  double robot_yaw = robot.get_angle();
  CartesianPoint new_position;
  
  new_position = position_after_move(Vector(robot_position, turn_robot(robot, -M_PI), movements[intensity]));

  return distance(new_position, target_position);
}


double DescriptiveManager::target_distance_turning_right(Vector robot, CartesianPoint target_position, int intensity){
  
  double new_yaw;
  CartesianPoint robot_position = robot.get_origin();

  // Well turns are more complicated than this, because we can move 
  // forward or backward, so we have to simulate both movements
  // and pick one that will get agent closer to the target
  new_yaw = turn_robot(robot, rotations[2 * (intensity+1)]);
  
  // TODO
  // this value to be defined from configuration file
  // let's make it a constant MAX_DISTANCE or something like it      
  double min_distance = 1000;
  
  double new_distance_fwd;
  //double new_distance_bck;

  CartesianPoint new_position;
  for(int j = 0; j < movements.size(); j++){
      new_position = position_after_move(Vector(robot_position, new_yaw, movements[j]));
      //TODO
      // add the same thing for backward move
      // just turn yaw for 180 degrees
      
      new_distance_fwd = distance(new_position, target_position);
      // TODO
      // add this calculation for backward move if wished

      if (new_distance_fwd < min_distance)
	min_distance = new_distance_fwd;
  }
  return min_distance;
}

 
double DescriptiveManager::target_distance_turning_left(Vector robot, CartesianPoint target_position, int intensity){
   
  double new_yaw;
  CartesianPoint robot_position = robot.get_origin();

  new_yaw = turn_robot(robot, rotations[2 * intensity + 1]);

  // TODO
  // this value to be defined from configuration file
  // let's make it a constant MAX_DISTANCE or something like it      
    double min_distance = 1000;
      
    double new_distance_fwd;
    //double new_distance_back;

    CartesianPoint new_position;
    for(int j = 0; j < movements.size(); j++){
      new_position = position_after_move(Vector(robot_position, new_yaw, movements[j]));
      //TODO
      // add the same thing for backward move
      // just turn yaw for 180 degrees
      
      new_distance_fwd = distance(new_position, target_position);
      // TODO
      // add this calculation for backward move if wished

      if (new_distance_fwd < min_distance)
	min_distance = new_distance_fwd;
    }
    return min_distance;
}
  


double DescriptiveManager::turn_robot(Vector robot, double angle){
  
  double new_angle = robot.get_angle() + angle;
  new_angle = Utils::normalizeAngle(new_angle);
  //cout << "Robot direction: " << robot.get_angle() << endl;
  //cout << "Rotation direction: " << angle << endl;
  //cout << "New direction :" << new_angle << endl;
  return new_angle;
}


void DescriptiveManager::set_target(CartesianPoint new_target){
  target_points.push_back(new_target);
}


CartesianPoint DescriptiveManager::get_target(){
  return target_points[0];
}
*/
