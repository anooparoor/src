// Implementation file which contains the functions of the class AvoidRobots.h
// Tier 1 advisor which vetoes actions which will cause collision with other robots..
// anoop.a.rao@gmail.com
// date : 3rd april 2014

#include "FORRGeometry.h"
#include <math.h>
#include "FORRAction.h"
#include "Beliefs.h"
#include "Position.h"
#include "AvoidRobots.h"

void AvoidRobots::avoid_robots(Beliefs *beliefs){
  cout << "Starting avoid robot advisor" << endl;
  set<FORRAction> *vetoed_actions = beliefs->vetoedActions;
  vector<Position> team_pose = beliefs->teamPose;
  vector<double> team_max_len;
  //cout << "check for inline collision" << endl;
  beliefs->inCollisionMode = false;
  // Case 1: If a robot is in front of me and facing other direction , reduce the max len to the distance between the two 
  // Case 2: If a robot is in front of me and is facing me , then reduce the max len to half the distance between the two
  veto_inline_collisions(beliefs);

  // data structure to represent the collision graph is adjacency list
  list<Vertex> *collision_graph = new list<Vertex>;
  //cout << "starting collision graph fucntion" << endl;
  generate_collision_graph(team_pose, collision_graph);
  //cout << "Collision Avoidance :: Collision graph created : size:" << collision_graph->size() << endl;
  //cout << "Collision Avoidance :: vertex list :" << endl;
  std::list<Vertex>::iterator iterator;
  for(iterator = (*collision_graph).begin(); iterator!= (*collision_graph).end(); iterator++){
    (*iterator).printdetails();
  } 
  if(collision_graph->size() > 1){
    // This function takes in the vetoed_actions collision set and uses the collision set to find the best possible subset to get the vetoed actions, if 
    // my robot id is one of the choose ones to make collision work then, actions are vetoed and updated.
    beliefs->inCollisionMode = true;
    veto_angular_collisions(collision_graph, vetoed_actions, team_pose[0]);
  }
  
}

void AvoidRobots::veto_inline_collisions(Beliefs *beliefs){
  set<FORRAction> *vetoed_actions = beliefs->vetoedActions;
  vector<Position> team_pose = beliefs->teamPose;
  Position my_robot_pose = team_pose[0];
  Position test_robot;
  double buffer = 30;
  double allowed_intensity = 5;
  for(int i = 1; i < team_pose.size(); i++){
    test_robot = team_pose[i];
    // checks if my_robot is going to hit the test robot directly , if the test robot stays in its position
    double collision_distance = check_inline_collision(my_robot_pose, test_robot);
    // checks if test robot is going to hit me directly if the I stay in my current position 
    double collision_distance2 = check_inline_collision(test_robot, my_robot_pose);
    cout << "collision distance " << collision_distance << endl;
    if(collision_distance <= 150){
      cout << "InlineCollision: I am facing other robot" << endl;
      collision_distance = collision_distance - buffer; 
      //case when both robots are facing each other
      if(collision_distance2 <= 150){
	cout << "InlineCollision::Both robots are facing each other" << endl;
	collision_distance = collision_distance/2;
      }
      //case when the current robot is facing the other robot
      else{
	//cout << "InlineCollision:: Only I am facing another robot" << endl;
      }
      double temp_allowed_intensity = 5;
      if(collision_distance > 105)
	temp_allowed_intensity = 5;
      else if(collision_distance > 45)
	temp_allowed_intensity = 4;
      else if(collision_distance > 20 )
	temp_allowed_intensity = 3;
      else if(collision_distance > 7)
	temp_allowed_intensity = 2;
      else if(collision_distance > 3)
	temp_allowed_intensity = 1;
      else
	temp_allowed_intensity = 0;
      //cout << "allowed intensity for current test robot" << temp_allowed_intensity << endl;
      if(allowed_intensity > temp_allowed_intensity)
	allowed_intensity = temp_allowed_intensity;
    }
    else{
      //cout << "InlineCollision: I am not colliding with the current test robot" << endl;
    }
  }
  if(allowed_intensity <= 4)
    beliefs->inCollisionMode = true;

  for(int i = allowed_intensity + 1; i < 6; ++i)  
    vetoed_actions->insert(FORRAction(FORWARD, i));
  //cout << "All forward moves above intensity "  <<  allowed_intensity  <<  "  are banned for robot due to inline collision!! " << my_robot_pose.getX() << " , " <<my_robot_pose.getY() << endl;
  return;
}
/*
bool AvoidRobots::is_facing( Position robot1, Position robot2){
  // returns true if robot1 is facing robot2
  bool is_facing = false;
  double robot1_angle = robot1.getTheta();
  double connecting_angle = atan2(robot2.getY() - robot1.getY(), robot2.getX() - robot1.getX());
  cout << "robot1_angle " << robot1_angle << endl;
  cout << "connecting_angle " << connecting_angle << endl;
  cout << "is_facing:: Angle between robot1 direction and collision line" << (robot1_angle - connecting_angle) << endl;
  if(abs(robot1_angle - connecting_angle) < 3.14/3)
    is_facing = true;
  return is_facing;
}
*/

double AvoidRobots::check_inline_collision(Position robot1_pos, Position robot2_pos){

  double robot_side_len = 25;

  double free_space = 1000;
  double robot1_maxlen = 135;
  double robot2_maxlen = 135;
  double robot_diagonal_len = robot_side_len * (1.414);
  // get a right and left vector for robot1 and robot2
  Vector robot1(robot1_pos.getX(), robot1_pos.getY(), robot1_pos.getTheta(), robot1_maxlen);
  Vector robot2(robot2_pos.getX(), robot2_pos.getY(), robot2_pos.getTheta(), robot_side_len);

  CartesianPoint left_center_robot1(robot1.get_point(robot_side_len/2, 1.57));
  Vector vleft_center_robot1 (left_center_robot1, robot1.get_angle(), robot1_maxlen);
  CartesianPoint left_front_robot1(vleft_center_robot1.get_endpoint());

  CartesianPoint right_center_robot1(robot1.get_point(robot_side_len/2, -1.57));
  Vector vright_center_robot1 (right_center_robot1, robot1.get_angle(), robot1_maxlen);
  CartesianPoint right_front_robot1(vright_center_robot1.get_endpoint());

  CartesianPoint left_back_robot2(robot2.get_point(robot_diagonal_len/2, 2.35));
  Vector vleft_back_robot2 (left_back_robot2, robot2.get_angle(), robot2_maxlen);
  CartesianPoint left_face_robot2(vleft_back_robot2.get_point(robot_side_len, 0));

  CartesianPoint right_back_robot2(robot2.get_point(robot_diagonal_len/2, -2.35));
  Vector vright_back_robot2 (right_back_robot2, robot2.get_angle(), robot2_maxlen);
  CartesianPoint right_face_robot2(vright_back_robot2.get_point(robot_side_len, 0));

  //cout << "robot1 position" << endl;
  //cout << robot1_pos.getX() << "," << robot1_pos.getY() << endl;
  //cout << robot1_pos.getX() + (100 * cos(robot1_pos.getTheta())) << "," <<  robot1_pos.getY() + (100 * sin(robot1_pos.getTheta())) << endl;

  //cout << "robot1 original vector" << endl;

  //print_point(robot1.get_origin());
  //print_point(robot1.get_endpoint());

  
  //cout << "robot1: " << endl;
  //print_point(left_back_robot1);
  //print_point(left_front_robot1);
  //print_point(right_back_robot1);
  //print_point(right_front_robot1);
  

  //cout << "robot2 position" << endl;
  //cout << robot2_pos.getX() << "," << robot2_pos.getY() << endl;
  //cout << robot2_pos.getX() + (100 * cos(robot2_pos.getTheta())) << "," <<  robot2_pos.getY() + (100 * sin(robot2_pos.getTheta())) << endl;


  //cout << "robot2 original vector" << endl;

  //print_point(robot2.get_origin());
  //print_point(robot2.get_endpoint());

  //cout << "robot2: " << endl;

  //print_point(left_back_robot2);
  //print_point(left_face_robot2);
  //print_point(right_back_robot2);
  //print_point(right_face_robot2);
  
  vector<LineSegment> robot1_zone;
  robot1_zone.push_back(LineSegment(left_front_robot1, left_center_robot1));
  robot1_zone.push_back(LineSegment(right_front_robot1, right_center_robot1));
  robot1_zone.push_back(LineSegment(left_center_robot1, right_center_robot1));
  robot1_zone.push_back(LineSegment(left_front_robot1, right_front_robot1));
  
  vector<LineSegment> robot2_zone;
  robot2_zone.push_back(LineSegment(left_face_robot2, left_back_robot2));
  robot2_zone.push_back(LineSegment(right_face_robot2, right_back_robot2));
  robot2_zone.push_back(LineSegment(left_back_robot2, right_back_robot2));
  robot2_zone.push_back(LineSegment(left_face_robot2, right_face_robot2));
  
  CartesianPoint intersection_point;
  double min_free_space = 1000;

  for(int i = 0 ; i < robot1_zone.size(); i++){
    for(int j = 0 ; j < robot2_zone.size(); j++){
      //print_point(robot1_zone[i].get_endpoints().first);
      //print_point(robot1_zone[i].get_endpoints().second);
      //print_point(robot2_zone[j].get_endpoints().first);
      //print_point(robot2_zone[j].get_endpoints().second);
      if(do_intersect(robot1_zone[i], robot2_zone[j], intersection_point)){
	//cout << "intersection found" << endl;
	free_space = distance(intersection_point, robot1.get_origin());
	//cout << free_space << endl;
	if(free_space < min_free_space)
	  min_free_space = free_space;
      }
    }
  }
  cout << "Inline : possible collision dectected, with free space : " << min_free_space <<  endl; 
  return (min_free_space);
}

void AvoidRobots::print_point(CartesianPoint point){
  cout << point.get_x() << "," << point.get_y() << endl;
  return;
}


bool AvoidRobots::generate_collision_graph(vector<Position> team_pose, list<Vertex> *collision_graph){
  //check 
  cout << "teampose size : " << team_pose.size() << endl;
  Position my_robot_pose = team_pose[0];
  list<Position> to_be_tested;
  to_be_tested.push_back(my_robot_pose);
  list<Position> already_tested;
  while(to_be_tested.size() > 0){
    //cout << "Size of to_be_tested: " << to_be_tested.size() << endl;
    Position test_robot = to_be_tested.front();
    std::list<Position>::iterator findIter = std::find(already_tested.begin(), already_tested.end(), test_robot);
    if(findIter == already_tested.end()){
      double weight = 1000;
      for(int i = 0; i < team_pose.size(); i++){
        if(!checkequal(test_robot, team_pose[i])){
	  //cout << "calling check for collision on " << test_robot.getX() << "," << test_robot.getY() << " and " << team_pose[i].getX() << "'" << team_pose[i].getY() << endl;
	  double temp_weight = check_for_collision(test_robot, team_pose[i]);
	  // if check_for_collision returns 1000 , implies no collision between the 2 robots in question
	  if(temp_weight != 1000){
	    // update the weight if its better
	    //cout << "collision detected : weight(distance from point of intersection) for test_robot:" << temp_weight << endl;
	    if(temp_weight < weight){
	      weight = temp_weight;
	    }
            //cout << "adding the new robot into the to_be_tested list " << endl;
	    // add the colliding robot into the to_be_tested list and add the collision info into the collision_graph
	    to_be_tested.push_back(team_pose[i]);
	    add_collision_info(collision_graph, test_robot, team_pose[i]);
	  }
	  else{
	    //cout << "no collision" << endl;
	  }
        }
      }
      //cout << "final weight after collision checking : " << weight << endl;
      add_weight_info(collision_graph, test_robot, weight);
      already_tested.push_back(test_robot);
    }
    to_be_tested.pop_front();
  }
}

// use slavisa new FORRGeometry compute collision 
double AvoidRobots::check_for_collision(Position robot1_pos, Position robot2_pos){

  double robot_side_len = 25;

  double robot_diagonal_len = robot_side_len * 1.414;
  double max_move_len = 135;
  double free_space = 1000;
  // get a right and left vector for robot1 and robot2
  Vector robot1(robot1_pos.getX(), robot1_pos.getY(), robot1_pos.getTheta(), max_move_len);
  Vector robot2(robot2_pos.getX(), robot2_pos.getY(), robot2_pos.getTheta(), max_move_len);

  CartesianPoint left_back_robot1(robot1.get_point(robot_diagonal_len, 2.35));
  Vector vleft_back_robot1 (left_back_robot1, robot1.get_angle(), max_move_len);
  CartesianPoint left_front_robot1(vleft_back_robot1.get_point(max_move_len, 0));
  //CartesianPoint left_middle_robot1(vleft_back_robot1.get_point(0, 

  CartesianPoint right_back_robot1(robot1.get_point(robot_diagonal_len, -2.35));
  Vector vright_back_robot1 (right_back_robot1, robot1.get_angle(), max_move_len);
  CartesianPoint right_front_robot1(vright_back_robot1.get_point(max_move_len, 0));

  CartesianPoint left_back_robot2(robot2.get_point(robot_diagonal_len, 2.35));
  Vector vleft_back_robot2 (left_back_robot2, robot2.get_angle(), max_move_len);
  CartesianPoint left_front_robot2(vleft_back_robot2.get_point(max_move_len, 0));

  CartesianPoint right_back_robot2(robot2.get_point(robot_diagonal_len, -2.35));
  Vector vright_back_robot2 (right_back_robot2, robot2.get_angle(), max_move_len);
  CartesianPoint right_front_robot2(vright_back_robot2.get_point(max_move_len, 0));
  
  vector<LineSegment> robot1_zone;
  robot1_zone.push_back(LineSegment(left_back_robot1, left_front_robot1));
  robot1_zone.push_back(LineSegment(right_back_robot1, right_front_robot1));
  robot1_zone.push_back(LineSegment(left_back_robot1, right_back_robot1));
  robot1_zone.push_back(LineSegment(left_front_robot1, right_front_robot1));
  
  vector<LineSegment> robot2_zone;
  robot2_zone.push_back(LineSegment(left_back_robot2, left_front_robot2));
  robot2_zone.push_back(LineSegment(right_back_robot2, right_front_robot2));
  robot2_zone.push_back(LineSegment(left_back_robot2, right_back_robot2));
  robot2_zone.push_back(LineSegment(left_front_robot2, right_front_robot2));
  
  CartesianPoint intersection_point;
  double min_free_space = 1000;

  for(int i = 0 ; i < robot1_zone.size(); i++){
    for(int j = 0 ; j < robot2_zone.size(); j++){
      if(do_intersect(robot1_zone[i], robot2_zone[j], intersection_point)){	
	free_space = distance(intersection_point, robot1.get_origin());
	if(free_space < min_free_space)
	  min_free_space = free_space;
      }
    }
  }
  //cout << "possible collision dectected, with free space : " << min_free_space <<  endl;
   
  return (min_free_space);
}

double AvoidRobots::distance(CartesianPoint first, CartesianPoint second){
  return sqrt((first.get_x() - second.get_x())*(first.get_x() - second.get_x()) + (first.get_y() - second.get_y())*(first.get_y() - second.get_y()));
}

double AvoidRobots::distance(Position first, Position second){
  return sqrt((first.getX() - second.getX())*(first.getX() - second.getX()) + (first.getY() - second.getY())*(first.getY() - second.getY()));
}

void AvoidRobots::add_collision_info(list<Vertex> *collision_graph ,Position robot1,Position robot2){
  //cout << "------start of add collision infor ---- " << endl;
  list<Vertex>::iterator iter = collision_graph->begin();
  bool already_in_graph = false;
  for(iter = collision_graph->begin(); iter != collision_graph->end(); iter++){
    if(checkequal((*iter).robot, robot1)){
      already_in_graph = true;
      break;
    }
  }
  if(already_in_graph == false){
    cout << "adding new vertex" << endl;
    Vertex new_vertex;
    new_vertex.robot = robot1;
    new_vertex.adj.push_back(robot2); 
    collision_graph->push_back(new_vertex);
  }
  else if(already_in_graph == true){
    list<Position>::iterator posIter;
    std::list<Position>::iterator findIter = std::find((*iter).adj.begin(), (*iter).adj.end(), robot2);
    if(findIter == (*iter).adj.end()){
      (*iter).adj.push_back(robot2);
    }
  }
  //cout << "-------end of add collision info --------" << endl;
}


bool AvoidRobots::checkequal(Position position1, Position position2){
  //cout << "In function checkequal " << endl;
  //cout << position1.getX() << "," << position1.getY() << " " << position2.getX() << "," << position2.getY() << endl; 
  bool isEqual = false;
  if((position1.getX() == position2.getX()) && (position1.getY() == position2.getY())){
    //cout << "positions are equal" << endl;
    isEqual = true;
  }
  return isEqual;
}

void AvoidRobots::add_weight_info(list<Vertex> *collision_graph ,Position robot, double weight){
  //cout << "--------start of add weight info ----------" << endl;
  list<Vertex>::iterator iter = collision_graph->begin();
  for(iter = collision_graph->begin(); iter != collision_graph->end(); iter++){
    if(checkequal((*iter).robot, robot)){
      (*iter).weight = weight;
      break;
    }
  }
  //cout << "----end of add weight info ------------------" << endl;
}


bool AvoidRobots::veto_angular_collisions(list<Vertex> *collision_graph, set<FORRAction> *vetoed_actions ,Position robot){
  // convert the list of vertices in the collision graph into set
  //cout << "in fucntion veto_angular_collisions" << endl;
  set< set <Vertex> > pSet = generatePowerSet(collision_graph);
  //cout << pSet.size() << " subsets generated" << endl;
  // generates all possible subsets of collision_set vertices and saves it into pSet
  
  set< set <Vertex> >::iterator pSet_iter;
  set<Vertex> maximal_independent_set;
  double maximal_weight = 0; 
  for(pSet_iter = pSet.begin(); pSet_iter != pSet.end(); pSet_iter++){
    if(is_independent(*pSet_iter)){
      double temp_total_weight = compute_weight(*pSet_iter);
      //cout << "Total Weight of subset :" << temp_total_weight;
      if(temp_total_weight > maximal_weight){
	maximal_weight = temp_total_weight;
        maximal_independent_set = (*pSet_iter);
      }
    }
  }
  //cout << "Final maximum weight independent set:" << endl;
  set<Vertex>::iterator maximal_iter;
  for(maximal_iter = maximal_independent_set.begin(); maximal_iter != maximal_independent_set.end(); maximal_iter++){
    maximal_iter->printdetails();
  }
  //cout << "Weight of final MIS: " << maximal_weight << endl;
 
  // if robot belongs to the choosen independent set
  if(!in_maximal_set(maximal_independent_set, robot)){
    list<Vertex>::iterator iter;
    double distance_from_intersection = 0;
    for(iter = collision_graph->begin(); iter != collision_graph->end(); iter++){
      if((iter->robot).getX() == robot.getX() && (iter->robot).getY() == robot.getY()){
	distance_from_intersection = iter->weight;
	break;
      }
    }
    // veto forward actions
    int allowed_intensity;
    int buffer = 30;
    distance_from_intersection = distance_from_intersection - buffer;
    if(distance_from_intersection > 105)
      allowed_intensity = 5;
    else if(distance_from_intersection > 45)
      allowed_intensity = 4;
    else if(distance_from_intersection > 20 )
      allowed_intensity = 3;
    else if(distance_from_intersection > 7)
      allowed_intensity = 2;
    else if(distance_from_intersection > 3)
      allowed_intensity = 1;
    else
      allowed_intensity = 0;

    for(int i = allowed_intensity + 1; i < 6; ++i)  
      vetoed_actions->insert(FORRAction(FORWARD, i));
    //cout << "All forward moves above intensity "  <<  allowed_intensity  <<  "  are banned for robot " << robot.getX() << " , " <<robot.getY() <<  " due to angular collision" << endl;	
  }
  else{
    //cout << "Robot" << robot.getX() << "," <<robot.getY() << " will not be vetoed" << endl;
  }
}

bool AvoidRobots::in_maximal_set(set<Vertex> maximal_independent_set, Position robot){
  //cout << "In function in_maximal_set " << endl;
  set<Vertex>::iterator setIter;
  bool in_set = false;
  for(setIter = maximal_independent_set.begin();setIter != maximal_independent_set.end(); setIter++){
    if(checkequal(setIter->robot, robot)){
      //cout << "checked with robot: " << endl;
      setIter->printdetails();
      in_set = true;
    }
  }
  //cout << "Current robot belongs to the the maximal_independent_set" << endl;
  return in_set;
}

double AvoidRobots::compute_weight(set<Vertex> independent_set){
  set<Vertex>::iterator setIter;
  double weight = 0;
  for(setIter = independent_set.begin();setIter != independent_set.end(); setIter++){
    weight = weight + (105 - setIter->weight);
  }
  //cout << "the weight computed for the independent set is " << weight << endl;
  return weight;
}

bool AvoidRobots::is_independent(set<Vertex> input_set){
  set<Vertex>::iterator setIter;
  set<Vertex>::iterator setIter2;
  bool is_independent = true;
  for(setIter = input_set.begin(); setIter != input_set.end(); setIter++){
    Position mypose = setIter->robot;
    for(setIter2 = input_set.begin(); setIter2 != input_set.end(); setIter2++){
      list<Position> adj = setIter2->adj;
      std::list<Position>::iterator findIter = std::find(adj.begin(), adj.end(), mypose);
      if(findIter != adj.end()){
	is_independent = false;
        //cout << "The set is not an independent set" << endl;
        return is_independent;
      }
    }
  }
  //cout << "the set is an independent set" << endl;
  return is_independent;
}

set < set < Vertex > > AvoidRobots::generatePowerSet( list<Vertex> *inputSet ){
  //cout << "In function generate power set "<< endl;
  vector<Vertex> inputVector;
  set <set < Vertex > > pSet;

  list<Vertex>::iterator inputIter;
  for(inputIter = inputSet->begin(); inputIter != inputSet->end() ; inputIter++){
    inputVector.push_back(*inputIter);
  }

  set<Vertex> currentSet;

  unsigned long int power = (1 << inputVector.size());
  for(unsigned long i = 0 ; i < power; i++){
    unsigned long working = i;
    currentSet.clear();
    for(int j = 0; j < sizeof(unsigned long)*8; ++j){
      if(working & 1){
	currentSet.insert(inputVector[j]);
        inputVector[j].printdetails();
      }
      working >>=1;
    }
    //cout << "--- Set number " << i << endl;
    pSet.insert(currentSet);  
  }
  return pSet;
}
