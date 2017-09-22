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
#include "FORRActionStats.h"
#include "PathPlanner.h"

#include <fstream>
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>


// Forward-declare Controller so the typedef below can reference it
class Controller;
typedef std::vector<Tier3Advisor*>::iterator advisor3It;

// ROS Controller class 
class Controller {
public:

  Controller(string, string, string, string, string);
  
  //main sense decide loop, receives the input messages and calls the FORRDecision function
  FORRAction decide();

  FORRActionStats *getCurrentDecisionStats() { return decisionStats; }
  void clearCurrentDecisionStats() { decisionStats = new FORRActionStats();}

  //Update state of the agent using sensor readings 
  void updateState(Position current, sensor_msgs::LaserScan laserscan, geometry_msgs::PoseArray crowdpose);

  //Returns the state of the robots mission (True 
  bool isMissionComplete();

  // getter for beliefs
  Beliefs *getBeliefs() { return beliefs; }

  // getter for planner
  PathPlanner *getPlanner() { return planner; }

private:

  //FORR decision loop and tiers
  FORRAction FORRDecision();

  FORRActionStats *decisionStats = new FORRActionStats();
  
  //Tier 1 advisors are called here
  bool tierOneDecision(FORRAction *decision);

  //Tier 3 advisors are called here
  void tierThreeDecision(FORRAction *decision);

  //Check influence of tier 3 Advisors
  void tierThreeAdvisorInfluence();

  // learns the spatial model and updates the beliefs
  void learnSpatialModel(AgentState *agentState);

  void initialize_advisors(std::string);
  void initialize_tasks(std::string);
  void initialize_params(std::string);
  void initialize_planner(std::string,std::string, int &l, int &h);
  
  // Knowledge component of robot
  Beliefs *beliefs;

  // An ordered list of advisors that are consulted by Controller::FORRDecision
  Tier1Advisor *tier1;
  PathPlanner *planner;
  std::vector<Tier3Advisor*> tier3Advisors;
  
  // Checks if a given advisor is active
  bool isAdvisorActive(string advisorName);

  double canSeePointEpsilon, laserScanRadianIncrement, robotFootPrint, robotFootPrintBuffer, maxLaserRange, maxForwardActionBuffer, maxForwardActionSweepAngle;
  double arrMove[100];
  double arrRotate[100];
  int moveArrMax, rotateArrMax;
  int taskDecisionLimit;
  bool trailsOn;
  bool conveyorsOn;
  bool regionsOn;
  bool doorsOn;
  bool aStarOn;
  bool firstTaskAssigned;
};
  
#endif /* CONTROLLER_H */
