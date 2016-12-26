#include <Beliefs.h>

/*
 * this is fake monolithic descriptive that will perform all
 * the computation that will be required by advisors.
 * It will be broken into several individual descriptives later on.
 * This will enable me to create lean advisers and the communication between
 * them and this fake descriptive will be implemented through Message Handler.
 *
 * Created on: Sep. 11, 2013
 * Last modified on: Mar. 27, 2014
 *
 *  Created by Slavisa Djukic <sdjukic@hunter.cuny.edu>
 */
  

# include "Map.h"
# include "Utils.h"
# include <cmath>
# include <math.h>

# include <iostream>
# include <fstream>
# include <string>
# include "FORRGeometry.h"

// function that will initialize arrays and vectors that are members
// of Beliefs class, by zeroing out them.
void Beliefs::initVectors(){
  for(int i = 0; i < 20; i++){
    if(i > 10)
      targetDistanceVector.push_back(0);
    else{
      targetDistanceVector.push_back(0);
      wallDistanceVector.push_back(0);
    }
  }
  //cout << "InitVectors : after initializing targetdistancevector and wallDistanceVector" << endl;
  lastAction = FORRAction(HALT, 0); 
}

void Beliefs::update(CartesianPoint newPosition, CartesianPoint oldPosition, double yw){
  // first check if robot changed its position or yaw
  Position robotPosition = getCurrentPosition();
  //cout << "Prev x: "<< oldPosition.get_x();
  //cout << "Prev y: "<< oldPosition.get_y();
  //cout << "Current x: "<<newPosition.get_x()<<endl;
  //cout << "Current y: "<<newPosition.get_y()<<endl;

  if(newPosition.get_x() != oldPosition.get_x() || newPosition.get_y() != oldPosition.get_y()
     || yw != robotPosition.getTheta()){
  // also mark square that this location belongs to as visited one more time
    //cout << "In update: inside if condition  after getting currentPosition" << endl;
    //cout << "Old x: "<< oldPosition.get_x();
    //cout << "old y: "<< oldPosition.get_y();
    double grid_x = oldPosition.get_x() / square_size;
    double grid_y = oldPosition.get_y() / square_size;
    
  // visited_locations[square_x][square_y] += 
  visited_grid[grid_x][grid_y] += 1;
  
  Position np = Position(newPosition.get_x(), newPosition.get_y(),yw);
  setCurrentPosition(np);
  //cout << "In update : end of update" << endl;
}


vector<double> Beliefs::get_angle_difference_vector(){
  // create vector that goes from robot to target
  double rotations[] = {0, .1548, -.1548, .3048, -.3048, .65, -.65, 1.3, -1.3};
  double robot_target_x = getCurrentTask()->getX() - (getCurrentPosition().getX());
  double robot_target_y = getCurrentTask()->getY() - (getCurrentPosition().getY());
  double robot_yaw = getCurrentPosition().getTheta(); 
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

double Beliefs::normalize_angle(double angle){
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


int Beliefs::times_at_location(CartesianPoint location){
  int square_x = location.get_x() / square_size;
  int square_y = location.get_y() / square_size;

  if(square_x > locations_size || square_y > locations_size){
    cout << "ERROR: out of map " << endl;
    exit(1);
  }
  return visited_locations[square_x][square_y];
}


void Beliefs::create_locations(){
  const int MAZE = 600; // this is magic number for maze
  locations_size = MAZE / square_size;
  visited_locations = new double*[locations_size];
  for(int i = 0; i < locations_size; ++i)
    visited_locations[i] = new double[locations_size];

  // zero out all the values
  for(int i = 0; i < locations_size; ++i)
    for(int j = 0; j < locations_size; ++j)
      visited_locations[i][j] = 0;
}


void Beliefs::display_visited_map(){
  
  for(int j = boxes_height-1; j >=0; j--){
    
    for(int i = 0; i < boxes_width; i++){
      //cout << visited_grid[i][j] << " ";
    }
    //cout << endl;
  }
  
  //cout << "Square size: " << square_size<<endl;
  
}


void Beliefs::location_lookup(){
  int movements[] = {3, 7, 20, 25, 105};
  int newX, newY, square_x, square_y;
  Position robot_position = getCurrentPosition();
  double robot_yaw = robot_position.getTheta();
  // determine where is the robot at the moment
  square_x = robot_position.getX() / square_size;
  square_y = robot_position.getY() / square_size;
   // used for all rotations
  targetDistanceVector[0] = visited_locations[square_x][square_y];
  //cout << "In FD: for Explorer: 0 "  << targetDistances[0] << endl;
       
  for(int i = 1; i < 6; ++i){
    // going forward
    newX = robot_position.getX() + movements[i-1]  * cos(robot_yaw);  
    newY = robot_position.getY() + movements[i-1]  * sin(robot_yaw);

    if(newX < 600 && newX > 0 && newY < 600 && newY > 0){// make sure you are in the map
      square_x = newX / square_size;
      square_y = newY / square_size;
      // reusing targetDistances vector
      targetDistanceVector[i] = visited_locations[square_x][square_y];
    }
    else
      // this line ultimately does not matter since this step will be commented by Tier1
      targetDistanceVector[i] = 10; // if step is out of the map, magic value
    // going backward
    newX = robot_position.getX() - movements[i-1]  * cos(robot_yaw);
    newY = robot_position.getY() - movements[i-1]  * sin(robot_yaw);
    if(newX < 600 && newX > 0 && newY < 600 && newY > 0){
      // determine where is the robot at the moment
      square_x = newX / square_size;
      square_y = newY / square_size;
      targetDistanceVector[i+5] = visited_locations[square_x][square_y];
    }
    else
      // this line ultimately does not matter since this step will be commented by Tier1
      targetDistanceVector[i] = 10; // if step is out of the map, magic value
    //cout << "In FD: for Explorer: " << i << "  " << targetDistances[i] << endl;
  }
}

double Beliefs::get_visited_locations_value(Position expectedPosition){
  double x = expectedPosition.getX();
  double y = expectedPosition.getY();
  int mapHeight = getMap()->getHeight();
  int mapWidth = getMap()->getLength();
  //cout << "In get_visited_locations_value: "<<endl;
  //cout << "x before lowering: "<<x << endl;
  //cout << "y before lowering: "<<y << endl;
  
  if(x > mapWidth) x = mapWidth-1;
  if(y > mapHeight) y = mapHeight-1;
  //cout << "x after lowering: "<<x << endl;
  //cout << "y after lowering: "<<y << endl;

  if((x > 0) && (x < mapWidth) && (y > 0) && (y < mapHeight)){
    //cout << "Int x:" <<(int)(x/square_size)<<", Int y: "<< (int)(y/square_size) <<endl;
    //cout << "Visited grid width: " << visited_grid.size()<<endl;
    //cout << "Visited grid height: "<< visited_grid[0].size()<<endl;
    return visited_grid[(int)(x/square_size)][(int)(y/square_size)];
  }
  else return 10;

}



void Beliefs::set_visited_grid(int width, int height){
  //cout << "Entered set visited grid."<<endl;
  //cout << "Square size: " << square_size<<endl;
  boxes_width = ceil(width/square_size*1.0);
  boxes_height = ceil(height/square_size*1.0);
  
  for(int i = 0; i < boxes_width; i++){
    vector<int> row;
    for(int j = 0; j < boxes_height; j++){
      row.push_back(0);
    }
    visited_grid.push_back(row);
  }
  //cout << "Exit set visited grid."<<endl;

}

void Beliefs::reset_visited_grid(){
  for(int i = 0; i < visited_grid.size(); i++){
    for(int j = 0; j < visited_grid[i].size(); j++){
      visited_grid[i][j] = 0;
    }

  }
}






//to be used by the rotation advisors.  takes in a rotation action and looks to see 
//what the maximum following linear move could be (given the wall distance vectors and
//how they match up to the rotations)
int Beliefs::get_maximum_intensity_following_linear_move(FORRAction action){
  //cout <<"In get_maximum_intensity_following_linear_move"<<endl;
  //double rots[] = {0, .1548, -.1548, .3048, -.3048, .65, -.65, 1.3, -1.3, 3.39, -3.39};
  int movements[] = {3,7,20,45,105};
  FORRActionType type = action.type;
  int parameter = action.parameter;
  double distance;
  
  //get correct wallDistanceVector value corresponding to the action and intensity
  if(type == LEFT_TURN) distance = wallDistanceVector[(parameter * 2) -1];
  else if(type == RIGHT_TURN) distance = wallDistanceVector[parameter * 2];
  
  //cout << "Distance of current wall vector inside get_maximum_intensity_following<>:: "<<distance<<endl;
  
  if(distance > 105) return 5; //distance to wall is greater than max move, so return max move
  else {
    //loop through distances, starting with the greatest, to see which one is accessible
    //check to see if the distance is in the interval between the distances, i.e.
    //is greater than the lowest value (because we checked to see if it's greater than 105 already)
    for(int i = 3; i >= 0; i--){
      if(distance > movements[i]){
	return i+1; // i+1 because movement[i] = intensity of i+1
      }    
    }
  
    //default, means that distance < 3, so return intensity 0 (note: this does not exist as an
    //intensity, just to identify that it found no value);
    return 0;
  }
}


void Beliefs::turn_to_side(){
  if(wallDistanceVector[0] > 120)
    m_side = 'n'; // keep going forward
  else{
    double average_left = 0, average_right = 0;
    // note that I am not using the whole array, I want to discard
    // backward move and rotation with intensity 5
    for(int i = 1; i < 9; ++i)
      if(i % 2 == 1)
	average_left += wallDistanceVector[i];
      else
	average_right += wallDistanceVector[i];

    average_left /= 4;
    average_right /= 4;
    if(average_left >= average_right)
      m_side = 'l';
    else
      m_side = 'r';
  }
}


FORRAction Beliefs::not_opposite_data(){
      return getLastAction();
}

//gets a gate if it connects to the target region, otherwise returns a cartesian point of -1,-1
CartesianPoint Beliefs::get_target_gate(double wall_distance){
  int targetPositionX = getCurrentTask()->getX();
  int targetPositionY = getCurrentTask()->getY();
  int mapLength = getMap()->getLength();
  int mapHeight = getMap()->getHeight();
  int targetRegion = gates.region(targetPositionX, targetPositionY, mapLength, mapHeight);
  int currentRegion = gates.region(getCurrentPosition().getX(), getCurrentPosition().getY(), mapLength, mapHeight);
  double min_distance_to_gate = 10000; //want a close gate (heuristic)
  // used to give a gate that
  CartesianPoint return_gate;
  for(int i = 0; i < gates.getSize(); i++){
    CartesianPoint point_from = gates.getGate(i).point_from;
    CartesianPoint point_to = gates.getGate(i).point_to;
    double distance_to_gate_to = Utils::get_euclidian_distance(point_to.get_x(), point_to.get_y(), 
							       getCurrentPosition().getX(), getCurrentPosition().getY());
    double distance_to_gate_from = Utils::get_euclidian_distance(point_to.get_x(), point_to.get_y(), 
								 getCurrentPosition().getX(), getCurrentPosition().getY());
    if((gates.getGate(i).quad_to == targetRegion) && (gates.getGate(i).quad_from == currentRegion)){
      if(distance_to_gate_to < wall_distance)
	return point_to; // want to get to the other side of the gate first
      else if(distance_to_gate_from < wall_distance)
	return point_from;
    }
  }
  return CartesianPoint(-1,-1); // no gate exists within wall distance vectors

}

//returns the number of gates within some radius of position p 
int Beliefs::size_of_gate_cluster(Position p, int radius){
  int gate_count = 0;
  double distance = 0;
  for(int i = 0; i < gates.getSize(); i++){
    distance = Utils::get_euclidian_distance(p.getX(), p.getY(), gates.getGate(i).point_from.get_x(), 
					     gates.getGate(i).point_from.get_y());
    if(distance < radius)
      gate_count++;
  }

  return gate_count;

}



//removes gates saved if both ends of the gate are in the same circle
void Beliefs::clear_gates_in_circles(){
  for(unsigned int i = 0; i < gates.getSize(); i++){
    for(unsigned int j = 0; j < (abstractMap.getCircles()).size(); j++){
      //if both ends of the gate are in the same circle, remove it
      if(((abstractMap.getCircles())[j].inCircle(gates.getGate(i).point_from)) &&
	 ((abstractMap.getCircles())[j].inCircle(gates.getGate(i).point_to))){
	gates.remove_gate(i);
	i = i - 1;
      }
    }
  }
}


//removes gates that are more than epsilon distance away from a collection of cleaned paths
void Beliefs::clear_gates_far_from_paths(string filename){
  std::vector<Gate> new_gates;
  //cout << "Entered clear gates far from paths." << endl;
  std::ifstream pathstream;
  std::vector<LineSegment>lines;
  pathstream.open(filename.c_str());
  string fileLine;
  bool first_values = true;
  while(!pathstream.eof()){
    getline(pathstream, fileLine);
    stringstream ss(fileLine);
    int x1,y1,x2,y2 = 0;
  
    string first_x, first_y;
    string second_x, second_y;
    string third_x, third_y; // this is because I need the first_* pointers to be ahead not behind
    while(ss>>first_x){
      ss >> first_y;
      if(first_values && !ss.eof()){
	ss >> second_x;
	ss >> second_y;
	//first_values = false;
      }
      x1 = atoi(first_x.c_str());
      y1 = atoi(first_y.c_str());
      CartesianPoint p1 = CartesianPoint(x1,y1);
      LineSegment current_path_line;
      if(first_values){
	x2 = atoi(second_x.c_str());
	y2 = atoi(second_y.c_str());
	third_x = second_x;
	third_y = second_y;

	first_values = false;
      }
      else{
	x2 = atoi(third_x.c_str());
	y2 = atoi(third_y.c_str());
	third_x = first_x;
	third_y = first_y;
      }
      printf("x1:%d, y1:%d, x2:%d, y2:%d \n",x1,y1,x2,y2);
      CartesianPoint p2 = CartesianPoint(x2,y2);
      current_path_line = LineSegment(p1,p2);
      lines.push_back(current_path_line);
            
      }
    }
   double distance_to_point = 0;
   for(unsigned int i = 0; i < gates.getSize(); i++){
     for(unsigned int j = 0; j < lines.size(); j++){
      Gate g = gates.getGate(i);
	CartesianPoint gate_point = g.point_from;
	distance_to_point = distance(gate_point, lines[j]);
	if(distance_to_point < .5){
	  new_gates.push_back(g);
	  cout << "Close gate found! At distance" << distance_to_point << endl;
	  cout << "Adding ("<<gate_point.get_x()<<","<< gate_point.get_y()<<")"<<endl;
	  cout << "i = "<<i<<","<<"j="<<j<<endl;
	  break;
	}
     }
   }


  gates.setGates(new_gates);  
}



CartesianPoint Beliefs::get_nearby_gate_exit(){
  double distance = 0;
  for(unsigned int i = 0; i < gates.getSize(); i++){
    distance = Utils::get_euclidian_distance(getCurrentPosition().getX(), getCurrentPosition().getY(), gates.getGate(i).point_from.get_x(),gates.getGate(i).point_from.get_y());
    //gate exists less than 20 pixels away
    if(distance < 30)
      return gates.getGate(i).point_to;
  }
  
  
  
  //else, no gate found, return -1,-1
  return CartesianPoint(-1,-1);
      
}
