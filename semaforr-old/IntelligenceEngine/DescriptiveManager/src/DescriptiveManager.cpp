/* DescriptiveManager.cpp
 *
 * Handles descriptives.
 *
 * Copyright 2013 HRTeam.
 * License:
 * Created: Dec 6,2013
 * Original authors: ANoop Aroor <anoop.a.rao@gmail.com>
 */
#include <iostream>
#include "DescriptiveManager.h"
#include <cmath>
#include <vector>
#include <fstream>

#include <boost/algorithm/string.hpp>

using namespace std;

//constructor
DescriptiveManager::DescriptiveManager(Map map, string advisor_file, string rswl_file)
  : message_handler(message_handler) , rswl(advisor_file,rswl_file) {
    addFORRWalls(map.getWalls());
    addEdgeBuffers();
    
    for(int i = 1 ; i <= maze.size(); i++){
      CartesianPoint first = maze[i].get_endpoints().first;
      CartesianPoint second = maze[i].get_endpoints().second;
      //cout << "wall no: " << i << " : " << first.get_x() << "," << first.get_y() << " --- " << second.get_x() <<"," << second.get_y() << endl;
    }
    cout << "DM:DM >>descriptive manager constructor" << endl;
    // Slavisa this part has to be moved to reading from the config file
    double tmp_rotations[] = {0, .1548, -.1548, .3048, -.3048, .65, -.65, 1.3, -1.3, 3.39, -3.39, 3.14};  
    int tmp_movements[] = {3, 7, 20, 25, 105};
    for(int i = 0; i < 12; ++i)
      rotations.push_back(tmp_rotations[i]);
    for(int i = 0; i < 5; ++i)
      movements.push_back(tmp_movements[i]);
}

//Destructor
DescriptiveManager::~DescriptiveManager() {

}

void DescriptiveManager::set_message_handler(DManagerMessageHandler *msg_handler){
  message_handler = msg_handler;
} 

//This the function which is called when the DM thread is invoked , and this function works as a separate thread
void DescriptiveManager::operator()() {

  // Part of the code which initializes all static details like map , forr walls etc, into the raw_details
  
  // initialize targets , currently hard coded, should get it from some file..
 // initTargets();

  // add buffers to the walls in the maze and store them in the internal
  // representation in maze structure
  
  while( true ) {
    update();
    //std::cout << "Descriptive Manager thread "<< std::endl;
    usleep(1000);
    //std::cout.flush();
  }
}

//The update function checks for the incoming messages using the message handler
void DescriptiveManager::update() {
	//std::cout << "calling check inbox" << std::endl;
	message_handler->check_inbox();
	//std::cout.flush();
}

// update or create a new the descriptive based on the message received
void DescriptiveManager::update_raw_details(string type, string type_parameters, string value){
if(type == "position")
   {
        //parse the details from the detail_value string
        string robot_id = type_parameters;
        string position = value; //current point (x,y) and yaw
        
        //find and remove earlier position details and update the new position into the map
        std::map<string,string>::iterator it;
        it = current_robot_position.find(robot_id);
        if(it!=current_robot_position.end())
        {
        	current_robot_position.erase(it);
        }
        current_robot_position.insert ( std::pair<string,string>(robot_id,position) );
   }
if(type == "last_action")
   {
       string robot_id = type_parameters;
       std::map<string,string>::iterator it;
        it = last_action_desc.find(robot_id);
        if(it!=last_action_desc.end())
        {
        	last_action_desc.erase(it);
        }
        last_action_desc.insert ( std::pair<string,string>(type_parameters,value) );
   }
if(type == "current_target")
   {
       string robot_id = type_parameters;
       std::map<string,string>::iterator it;
        it = current_target.find(robot_id);
        if(it!=current_target.end())
        {
        	current_target.erase(it);
        }
        current_target.insert ( std::pair<string,string>(type_parameters,value) );
   }
}

//Check if the descriptive is stale and if not returns the descriptive, and if it is state then recomputes the descriptives
string DescriptiveManager::get_descriptive(string type, string type_parameters)
{
  //cout << "computing descriptives, in get_descriptive" << endl;
    if(is_stale(type,type_parameters) == true)
    {
        compute(type,type_parameters);
    }
    string value = extract_value(type,type_parameters);
    return value;
}

string DescriptiveManager::extract_value(string type,string type_parameters)
{
  //cout << "computing descriptives, in extract_values" << endl;
     string value;
     std::map<string,string>::iterator it;
     string robot_id = type_parameters;   
     if(type == "target_point")
     {     
          it = current_target.find(robot_id);
     }
     else if(type == "distance_from_target")
     {
          it = distance_from_target_desc.find(robot_id);
     }
     else if(type == "last_action")
     { 
          it = last_action_desc.find(robot_id);
     }
     else if(type == "distance_from_obstacles")
     { 
          it = distance_from_obstacles_desc.find(robot_id);
     }
     else if(type == "advisor_weight")
     {
          it = advisor_weight_desc.find(robot_id);
     }
     else if(type == "team_pose")
     {
          // all pose descriptive is the first global descriptive and is the same for any robot at a particular instance of time, hence it is 
          // stored in a string instead of a map.
       return team_pose_desc;
     }
     else if(type == "veto_forward_moves_advisor_data")
     { 
          it = distance_from_obstacles_desc.find(robot_id);
     }

     value = it->second;
     return value;
} 

// function to compute and update the descriptive table from the raw details
void DescriptiveManager::compute(string type, string type_parameters)
{
  //cout << "computing descriptives, in compute_values" << endl;
     if(type == "target_point")
       {
	 getNextTargetPoint(type_parameters);    
       }
     else if(type == "distance_from_target")
       {
	 findDistanceToTarget(type_parameters);
       }
     else if(type == "last_action")
       {
       //findLastAction(type_parameters); no such computation is required as the last_action is recieved in the format that is needed
       }
     else if(type == "team_pose")
       {
	 findTeamPose(type_parameters);
       }
     else if(type == "distance_from_obstacles")
       {
	 create_wall_proximity_vector(type_parameters);
       }
     else if(type == "advisor_weight")
       {
	 findAdvisorWeight(type_parameters);
       }
     else if(type == "veto_forward_moves_advisor_data")
       {
	 //cout << "Then on to this one " << endl;
	 create_wall_proximity_vector(type_parameters);
       }
}

void DescriptiveManager::add_new_description(string description){
  advisor_descriptions.push_back(description);
}
int DescriptiveManager::get_number_of_advisors(){
  return advisor_descriptions.size();
}

string DescriptiveManager::get_n_description(int n){
  
  string value;

  if(n < 0 || n > advisor_descriptions.size()){
    std::cerr << "Illegal number " << endl;
    value = "";
  }    
  else
    value = advisor_descriptions.at(n);

  return value;
} 

// checks if the descriptive which is being requested is stale or not
bool DescriptiveManager::is_stale(string type,string type_parameters){
     return true;
}

// generates a descriptive string such that the first pose on the string is the position of the robot requesting the descriptive.
void DescriptiveManager::findTeamPose(string type_parameters)
{
  string robot_id = type_parameters;
  //cout << "DM: before calling second " << endl;
  string position = ((current_robot_position.find(robot_id))->second);
  team_pose_desc = position;
  
  std::map<string,string>::iterator it;
  for(it = current_robot_position.begin(); it != current_robot_position.end(); it++) {
    if(it->first != robot_id){
      team_pose_desc = team_pose_desc + " " + it->second;
    }
    //cout << "DM : team pose desc: " << team_pose_desc << endl;
  }
}


// code from slavisa's fake descriptive @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
// Descriptive find distance to target related code!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

void DescriptiveManager::findDistanceToTarget(string type_parameters){

  // Here is how they are stored: first 5 are for forward movements
  // second 5 are for backward movements
  // next 5 are for rotation to the right                
  // xlast five are for rotation to the left
  //cout << "In function findDistanceToTarget" << endl;
  string value;
  string robot_id = type_parameters;
  string position = ((current_robot_position.find(robot_id))->second);
  vector<string> posstr;
  boost::split(posstr, position, boost::is_any_of(" "));
  string xstring = posstr[0];
  string ystring = posstr[1];
  string yawstring = posstr[2];

  double myX = atof(xstring.c_str());
  double myY = atof(ystring.c_str());
  double robotYaw = atof(yawstring.c_str());
  Vector robot = Vector(CartesianPoint(myX, myY), robotYaw, 1);

  string target = ((current_target.find(robot_id))->second);
  vector<string> tarstr;
  boost::split(tarstr, target, boost::is_any_of(" "));
  double targetX = atof((tarstr[0]).c_str());
  double targetY = atof((tarstr[1]).c_str());  
  CartesianPoint current_target = CartesianPoint(targetX, targetY);

  //cout << "DM: before pushing target distances " << endl;
  vector<double> targetDistances;
  
  targetDistances = target_distance_calculation(robot, current_target);

  value = vector_to_string(targetDistances);  
  std::map<string,string>::iterator it;
  it = distance_from_target_desc.find(robot_id);
  if(it!=distance_from_target_desc.end())
  {
        distance_from_target_desc.erase(it);
  }
  distance_from_target_desc.insert ( std::pair<string,string>(robot_id,value) );
  //cout << "DM: finished computing distance to the target" << endl;
}


string DescriptiveManager::vector_to_string(vector<double> input)
{
        string output;
        std::stringstream ss;
        
	for(int i =0; i < input.size();i++)
	{
                ss.str( std::string() );
                ss.clear();
                ss << input[i];
                ss.precision(2);
                ss << std::fixed;
                if(i == 0)
			output = ss.str();
                else
            		output = output + " " + ss.str();
	}
        return output;
}

// Descriptive 1. find distance to target related code!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! END!!






//******************************************************
//******************************************************





// Descriptive 2. find distance to obstacle related code!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

void DescriptiveManager::create_wall_proximity_vector(string type_parameters){

  vector<double> wallDistanceVec;
  //cout << "Received message " << type_parameters << endl;

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
    const int robot_width = 30;
    CartesianPoint left = turned_robot.get_point(robot_width/2, 3.14/2);
    CartesianPoint right = turned_robot.get_point(robot_width/2, -3.14/2);
 
    Vector turned_robot_1 = Vector(left, new_angle, max_move_len);
    Vector turned_robot_2 = Vector(right, new_angle, max_move_len);

    //cout << (left_right.get_endpoints()).first.get_x() << "," << (left_right.get_endpoints()).first.get_y() << "  " << (left_right.get_endpoints()).second.get_x() << "," << (left_right.get_endpoints()).second.get_y() << endl;
    //cout << (turned_robot.get_endpoint()).get_x() << "," << (turned_robot.get_endpoint()).get_y() << endl;
    //cout << (turned_robot.get_origin()).get_x() << "," << (turned_robot.get_origin()).get_y() << endl;

    //cout << (turned_robot_1.get_endpoint()).get_x() << "," << (turned_robot_1.get_endpoint()).get_y() << endl;
    //cout << (turned_robot_2.get_endpoint()).get_x() << "," << (turned_robot_2.get_endpoint()).get_y() << endl;
    //cout << "Origin:" << endl;
    //cout << (turned_robot_1.get_origin()).get_x() << "," << (turned_robot_1.get_origin()).get_y() << endl;
    //cout << (turned_robot_2.get_origin()).get_x() << "," << (turned_robot_2.get_origin()).get_y() << endl;

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
  //cout << "Adding " << wallsList.size() <<" buffer walls to the walls vector" << endl;
  return;
}


pair<FORRWall, FORRWall> DescriptiveManager::getEdgeBuffer(FORRWall a_wall){
  double wall_len = 25;
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
     //cout << "DM: getNextTargetPoint " << endl;
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
     //cout << "DM in getNextTargetPoint " << endl;
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
  

CartesianPoint DescriptiveManager::position_after_move(Vector robot){
  
  CartesianPoint robot_position = robot.get_origin();
  double robot_yaw = robot.get_angle();
  
  double new_x = robot_position.get_x() + robot.get_intensity() * cos(robot_yaw);
  double new_y = robot_position.get_y() + robot.get_intensity() * sin(robot_yaw);
  
  return CartesianPoint(new_x, new_y);
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


