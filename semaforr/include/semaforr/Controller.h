#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <map>
#include <set>
#include <string>
#include <time.h>
#include <limits.h>
#include <cstdlib>
#include <time.h>
#include <math.h>
#include <vector>
#include <utility>

// SemaFORR
#include "Beliefs.h"
#include "Tier1Advisor.h"
#include "Tier3Advisor.h"

#include <fstream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_datatypes.h>


// Forward-declare Controller so the typedef below can reference it
class Controller;
typedef std::vector<Tier3Advisor*>::iterator advisor3It;

// ROS Controller class 
class Controller {
public:

  Controller(ros::NodeHandle &nh, string, string, string);
  Beliefs *getBeliefs() { return beliefs; }
  
  //main sense decide loop, receives the input messages and calls the FORRDecision function
  FORRAction decide();


private:

  //FORR decision loop and tiers
  FORRAction FORRDecision();
  
  //Tier 1 advisors are called here
  bool tierOneDecision(FORRAction *decision);

  //Tier 3 advisors are called here
  void tierThreeDecision(FORRAction *decision);
  

  // learns the spatial model and updates the beliefs
  //void learnSpatialModel();

  void initialize_advisors(string);
  void initialize_tasks(string);
  void initialize_robot(string);
  
  // Knowledge component of robot
  Beliefs *beliefs;

  // An ordered list of advisors that are consulted by Controller::FORRDecision
  Tier1Advisor *tier1;
  std::vector<Tier3Advisor*> tier3Advisors;
  
  // Checks if a given advisor is active
  bool isAdvisorActive(string advisorName);

  
  //! The ROS node handle 
  ros::NodeHandle nh_;
  //! We will be publishing to the "/cmd_vel" topic to issue commands
  ros::Publisher cmd_vel_pub_;
  ros::Subscriber sub_laser_;
  ros::Subscriber sub_pose_;
  ros::Subscriber sub_crowd_pose_;

  // Message handler functions in controller  
  void updateLaserScan(const sensor_msgs::LaserScan &scan);
  //void updateCrowdPose();
  void updatePose(const geometry_msgs::PoseStamped &pose);

};
  
#endif /* CONTROLLER_H */
