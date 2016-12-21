/* DescriptiveManager.h
 *
 * Handles descriptives.
 *
 * Copyright 2013 HRTeam.
 * License:
 * Created: Dec 6,2013
 * Original authors: ANoop Aroor <anoop.a.rao@gmail.com>
 */

#ifndef DESCRIPTIVEMANAGER_H
#define DESCRIPTIVEMANAGER_H

#include <map>
#include <time.h>
#include <list>
#include <cstdlib>
#include <string>
#include "FORRWall.h"
#include "FORRGeometry.h"
#include "Map.h"
#include "rswl.h"

#include "DManagerMessageHandler.h"
#include "definitions.h"

using std::string;
using std::map;

class DManagerMessageHandler;
class DescriptiveManager{

 public:
  DescriptiveManager(Map arena_map);
  ~DescriptiveManager();
  
  void operator()();

  //update function updates the raw detials using any incoming message which is being received from the central server
  void update_raw_details(string type, string type_parameters, string value);

  //returns the fresh descriptive, whose type is specified in the input parameter, which is being requested by the agent
  string get_descriptive(string type, string type_parameters);
  
  //set a pointer to DescriptiveManagermessage handler
  void set_message_handler(DManagerMessageHandler *); 
  
  //add new description read from file
  void add_new_description(string description);

  //next two functions are for testing purposes
  int get_number_of_advisors();

  string get_n_description(int n);

  // for using my test 
  void set_target(CartesianPoint new_target);

  CartesianPoint get_target();

private:
  DManagerMessageHandler *message_handler;

  bool hasMoreTargets;
  
  // processed details which are sent to the agent requesting it, these are updated using the compute function in the Descriptive class
  map<string,string> distance_from_target_desc;
  map<string,string> distance_from_obstacles_desc;
  map<string,string> last_action_desc;
  map<string,string> advisor_weight_desc;
 
  // global advisor in string format which contains the current position of all the robots in the maze
  string team_pose_desc;

  // compute function using the input to get the type of 
  void compute(string type, string type_parameters);
  bool is_stale(string type, string type_parameters);
  string extract_value(string type,string type_parameters);
  
  //raw details which are required to be updated periodically
  map<string, string> current_robot_position;
  map<string, string> current_target;

  //static details
  Map maze_map;
  vector<FORRWall> maze;
  vector<CartesianPoint> target_points;
  RSWL rswl; 

  //update is used to ask the message handler to check for any mails in the inbox
  void update();

  //specific functions to compute the descriptives
  void findDistanceToTarget(string type_parameters);
  void create_wall_proximity_vector(string type_parameters);
  void findAdvisorWeight(string type_parameters);

  double roundValue(double x, int round);
  void getNextTargetPoint(string type_parameters);
  void addFORRWalls(vector<MapWall> walls);
  void findTeamPose(string type_parameters);
  
  // utilities
  string vector_to_string(vector<double> input);  

  // This members should be read from the configuration file, not
  // to be magic numbers like this.
  vector<double> rotations;   // physical abilites of robots to rotate
  vector<int> movements;      // physical ability of robot to go forward/backward

  /*
   * Set of functions that will return the distance from the target
   * after robot performs move
   */
  double target_distance_going_forward(Vector robot, CartesianPoint target_position, int intensity);

  double target_distance_going_backward(Vector robot, CartesianPoint target_position, int intensity);

  double target_distance_turning_right(Vector robot, CartesianPoint target_position, int intensity);

  double target_distance_turning_left(Vector robot, CartesianPoint target_position, int intensity);

  // Function that will call the above functions  
  vector<double> target_distance_calculation(Vector robot, CartesianPoint target_position);


  /*
   * This function will return coordinats of the point where we estimate
   * robot will end up after linear move described by the vector that
   * is representing robot.
   */
  CartesianPoint position_after_move(Vector robot);

  /*
   * Function in charge of changing vector representation of the
   * robot after the turn. In our implementation positive angles 
   * mean turn to the left and negative turn to the right.
   */
  double turn_robot(Vector robot, double angle);

  //TODO there is the same function in Utils/..Utils.h
  /*
   * Function that will normalize angle of robot orientation to
   * confirm with specifications of our system that angles of 
   * orientation range from -PI to PI.
   */
  double normalize_angle(double angle);
  
  // Structure that holds data for instantiating Tier3Advisors; read from config file
  vector<string> advisor_descriptions;
};

#endif
