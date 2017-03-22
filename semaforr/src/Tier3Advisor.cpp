/*
 *
 * Implementation of Advisors 
 *
 * Created on: Jun. 27, 2013
 * Last modified: April 14, 2013
 * created by Slavisa Djukic <sdjukic@hunter.cuny.edu>
 */

# include "Tier3Advisor.h"
# include "FORRAction.h"
# include <cmath>
# include <iostream>
# include "FORRWall.h"
# include <cstdlib>
# include <time.h>
# include <utility>

using std::set;
  
// Constructor for Tier3Advisor
Tier3Advisor::Tier3Advisor(Beliefs *beliefs_para, string name_para, string description_para, double weight_para, double *magic_init, bool is_active)  { 
  beliefs = beliefs_para;
  name = name_para;
  description = description_para;
  weight = weight_para;
  active = is_active;
  // load all magic numbers for this advisor
  for (int i = 0; i < 4; ++i)
    auxiliary_constants[i] = magic_init[i];
  //cout << "after initializing auxilary constants" << endl;
}

// Destructor
Tier3Advisor::~Tier3Advisor() {};

// this function will produce comment strengths for all possible actions 
// robot can make at the certain moment
// it does it by successively calling actionComment method for each action
// No arguments are necessary because actions are stored as member variable
// in advisor 
// It returns map that maps action to comment strength
std::map <FORRAction, double> Tier3Advisor::allAdvice(){
  set<FORRAction> *vetoed_actions = beliefs->getAgentState()->getVetoedActions();
  set<FORRAction> *action_set;

  bool inRotateMode;
  cout << "Decision Count : " << beliefs->getAgentState()->getCurrentTask()->getDecisionCount() << endl;
  
  if(beliefs->getAgentState()->getCurrentTask()->getDecisionCount() % 2 == 0)  
        inRotateMode = true;
  else  
        inRotateMode = false;

  cout << "Rotation mode : " << inRotateMode << endl;

  if(inRotateMode){
	action_set = beliefs->getAgentState()->getRotationActionSet();
  }
  else{
	action_set = beliefs->getAgentState()->getForwardActionSet();
  }

  std::map <FORRAction, double> result;
 
  std::size_t foundr = (this->get_name()).find("Rotation");
  // If the advisor is linear and the agent is in rotation mode
  if((foundr == std::string::npos) and inRotateMode){
	cout << "Advisor is linear and agent is in rotation mode" << endl; 
      return result;
  }
  // If the advisor is rotation and the agent is in linear mode
  if((foundr != std::string::npos) and !inRotateMode){
	cout << "Advisor is rotation and agent is in linear mode" << endl;
      return result;
  }

  double adviceStrength;
  FORRAction forrAction;
  //std::cout << this->agent_name << ": in allAdvice function .. comments are as following:" << std::endl;
  set<FORRAction>::iterator actionIter;
  for(actionIter = action_set->begin(); actionIter != action_set->end(); actionIter++){
    forrAction = *actionIter;
    cout << forrAction.type << " " << forrAction.parameter << endl;
    if(vetoed_actions->find(forrAction) != vetoed_actions->end())// is this action vetoed
      continue;
    adviceStrength = this->actionComment(forrAction);
    std::cout << "Advisor name :"  << this->get_name() << " Strength: " << adviceStrength << " Action Type:" << forrAction.type << " " << "Action intensity " << forrAction.parameter << std::endl;
    result[forrAction] = adviceStrength;
  } 
  //if(result.size() > 1){
  //normalize(&result);
  rank(&result);
  //}
  return result;
}


//normalizing from 0 to 10
void Tier3Advisor::
normalize(map <FORRAction, double> * result){
  double max = -1000, min = 1000;
  map<FORRAction, double>::iterator itr;
  for(itr = result->begin(); itr != result->end() ; itr++){
    if(max < itr->second)  max = itr->second;
    if(min > itr->second)  min = itr->second;
  } 
  if(max != min){
    double norm_factor = (max - min)/10;
    for(itr = result->begin(); itr != result->end() ; itr++){
      cout << "Before : " << itr->second << endl;
      itr->second = (itr->second - min)/norm_factor;
    }
  }
}

//rank the comments 
void Tier3Advisor::
rank(map <FORRAction, double> *result){
  //cout << "start compute advisor preference function" << endl;
  std::set<double> distinctComments;
  map<FORRAction, double>::iterator iter1, iter2;
  for(iter1 = result->begin(); iter1 != result->end(); iter1++){
    distinctComments.insert(iter1->second);
  }
  for(iter2 = result->begin(); iter2 != result->end(); iter2++){
    double rank = result->size();
    double test = iter2->second;
    set<double>::iterator iter; 
    for(iter = distinctComments.begin(); iter != distinctComments.end(); iter++){
      if(test < (*iter)){
	rank--;
      }
    }
    iter2->second = rank;
  }
}

// factory definition
Tier3Advisor* Tier3Advisor::makeAdvisor(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active){
  if(name == "Greedy")
    return new Tier3Greedy(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "ObstacleAvoid")
    return new Tier3AvoidObstacle(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "CloseIn")
    return new Tier3CloseIn(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "BigStep")
    return new Tier3BigStep(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "AvoidLeaf")
    return new Tier3AvoidLeaf(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "AvoidLeafRotation")
    return new Tier3AvoidLeafRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "Explorer")
    return new Tier3Explorer(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "BaseLine")
    return new Tier3BaseLine(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "GreedyRotation")
    return new Tier3GreedyRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "ObstacleAvoidRotation")
    return new Tier3AvoidObstacleRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "CloseInRotation")
    return new Tier3CloseInRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "BigStepRotation")
    return new Tier3BigStepRotation(beliefs, name, description, weight, magic_init, is_active);
  //else if(name == "GoAroundRotation")
    //return new Tier3GoAroundRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "ExplorerRotation")
    return new Tier3ExplorerRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "BaseLineRotation")
    return new Tier3BaseLineRotation(beliefs, name, description, weight, magic_init, is_active);
  //else if(name == "AvoidRobotRotation")
    //return new Tier3AvoidRobotRotation(beliefs, name, description, weight, magic_init, is_active);
  //else if(name == "AvoidRobot")
    //return new Tier3AvoidRobot(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "ExitFinderLinear")
    return new Tier3ExitFinderLinear(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "ExitFinderRotation")
    return new Tier3ExitFinderRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "RegionFinderLinear")
    return new Tier3RegionFinderLinear(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "RegionFinderRotation")
    return new Tier3RegionFinderRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "WaypointFinderLinear")
    return new Tier3WaypointFinderLinear(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "WaypointFinderRotation")
    return new Tier3WaypointFinderRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "TrailFinderLinear")
    return new Tier3TrailFinderLinear(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "TrailFinderRotation")
    return new Tier3TrailFinderRotation(beliefs, name, description, weight, magic_init, is_active);

  else 
    std::cout << "No such advisor " << std::endl;
}

// Constructors 
Tier3Greedy::Tier3Greedy (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3AvoidObstacle::Tier3AvoidObstacle(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3CloseIn::Tier3CloseIn(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3BigStep::Tier3BigStep(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};  
Tier3AvoidLeaf::Tier3AvoidLeaf(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3Explorer::Tier3Explorer(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3BaseLine::Tier3BaseLine (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
//Tier3AvoidRobot::Tier3AvoidRobot (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 

Tier3GreedyRotation::Tier3GreedyRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3AvoidObstacleRotation::Tier3AvoidObstacleRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3CloseInRotation::Tier3CloseInRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3BigStepRotation::Tier3BigStepRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
//Tier3GoAroundRotation::Tier3GoAroundRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3AvoidLeafRotation::Tier3AvoidLeafRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3ExplorerRotation::Tier3ExplorerRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3BaseLineRotation::Tier3BaseLineRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
//Tier3AvoidRobotRotation::Tier3AvoidRobotRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3ExitFinderLinear::Tier3ExitFinderLinear (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3ExitFinderRotation::Tier3ExitFinderRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3RegionFinderLinear::Tier3RegionFinderLinear (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3RegionFinderRotation::Tier3RegionFinderRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3WaypointFinderLinear::Tier3WaypointFinderLinear (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3WaypointFinderRotation::Tier3WaypointFinderRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3TrailFinderLinear::Tier3TrailFinderLinear (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3TrailFinderRotation::Tier3TrailFinderRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
 


// Default dummy constructors
Tier3Greedy::Tier3Greedy(): Tier3Advisor() {};
Tier3AvoidObstacle::Tier3AvoidObstacle(): Tier3Advisor() {};
Tier3CloseIn::Tier3CloseIn(): Tier3Advisor() {};
Tier3BigStep::Tier3BigStep(): Tier3Advisor() {};
Tier3AvoidLeaf::Tier3AvoidLeaf(): Tier3Advisor() {};
Tier3Explorer::Tier3Explorer(): Tier3Advisor() {};
//Tier3AvoidRobot::Tier3AvoidRobot(): Tier3Advisor() {};
Tier3BaseLine::Tier3BaseLine(): Tier3Advisor() {};

Tier3GreedyRotation::Tier3GreedyRotation(): Tier3Advisor() {};
Tier3AvoidObstacleRotation::Tier3AvoidObstacleRotation(): Tier3Advisor() {};
Tier3CloseInRotation::Tier3CloseInRotation(): Tier3Advisor() {};
Tier3BigStepRotation::Tier3BigStepRotation(): Tier3Advisor() {};
//Tier3GoAroundRotation::Tier3GoAroundRotation(): Tier3Advisor() {};
Tier3AvoidLeafRotation::Tier3AvoidLeafRotation(): Tier3Advisor() {};
Tier3ExplorerRotation::Tier3ExplorerRotation(): Tier3Advisor() {};
//Tier3AvoidRobotRotation::Tier3AvoidRobotRotation(): Tier3Advisor() {};
Tier3BaseLineRotation::Tier3BaseLineRotation(): Tier3Advisor() {};

Tier3ExitFinderLinear::Tier3ExitFinderLinear(): Tier3Advisor() {};
Tier3ExitFinderRotation::Tier3ExitFinderRotation(): Tier3Advisor() {};
Tier3RegionFinderLinear::Tier3RegionFinderLinear(): Tier3Advisor() {};
Tier3RegionFinderRotation::Tier3RegionFinderRotation(): Tier3Advisor() {};
Tier3WaypointFinderLinear::Tier3WaypointFinderLinear(): Tier3Advisor() {};
Tier3WaypointFinderRotation::Tier3WaypointFinderRotation(): Tier3Advisor() {};
Tier3TrailFinderLinear::Tier3TrailFinderLinear(): Tier3Advisor() {};
Tier3TrailFinderRotation::Tier3TrailFinderRotation(): Tier3Advisor() {};


// vote to go through an extrance to a region containing the target

double Tier3RegionFinderLinear::actionComment(FORRAction action){
  cout << "In region finder linear " << endl;
  double result;
  vector<FORRCircle> circles = beliefs->getSpatialModel()->getAbstractMap()->getCircles();
  int robotCircle=-1,targetCircle=-1;
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());

  // check the preconditions for activating the advisor
  for(int i = 0; i < circles.size() ; i++){
    // check if the target point is in circle
    if(circles[i].inCircle(targetPoint.get_x(), targetPoint.get_y())){
      targetCircle = i;
    }
    // check if the rob_pos is in a circle and the circle has atleast one exit
    if(circles[i].inCircle(curr_pos.getX(), curr_pos.getY())){
      robotCircle = i;
    }
  }

  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  
  cout << "Robot Circle : " << circles[robotCircle].getCenter().get_x() << " " << circles[robotCircle].getCenter().get_y() << endl;
    
  vector<FORRExit> exits = circles[robotCircle].getExits();

  vector<FORRCircle> nearCircles;
  for(int i = 0; i < exits.size() ; i++){
    FORRCircle test = circles[exits[i].getExitCircle()];
    std::vector<FORRCircle>::iterator it = std::find(nearCircles.begin(),nearCircles.end(), test);
    if(nearCircles.empty() or (it == nearCircles.end())){
      nearCircles.push_back(test);
      cout << "Neighbour Circle : " << test.getCenter().get_x() << " " << test.getCenter().get_y() << endl;
    }
  }

  cout << "#Neighbors found :" << nearCircles.size() << endl;

  double metric = 0;
  for(int i = 0; i < nearCircles.size(); i++){
    if(beliefs->getSpatialModel()->getAbstractMap()->isLeaf(nearCircles[i]) and nearCircles[i].inCircle(targetPoint.get_x(), targetPoint.get_y())){
      cout << "RegionFinder: Found Target Region !" << endl;
      metric += pow(expectedPosition.getDistance(nearCircles[i].getCenter().get_x(), nearCircles[i].getCenter().get_y()), 2);
    }
  }
  return metric * (-1);
}


void Tier3RegionFinderLinear::set_commenting(){

  cout << "In region finder linear set commenting " << endl;
  vector<FORRCircle> circles = beliefs->getSpatialModel()->getAbstractMap()->getCircles();
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  bool targetInCircle = false;
  bool currPosInCircleWithExit = false;
  int robotCircle=-1, targetCircle = -1;
  
  // check the preconditions for activating the advisor
  for(int i = 0; i < circles.size() ; i++){
    // check if the target point is in circle
    if(circles[i].inCircle(targetPoint.get_x(), targetPoint.get_y())){
      targetInCircle = true;
      targetCircle = i;
    }
    // check if the rob_pos is in a circle and the circle has atleast one exit
    if(circles[i].inCircle(curr_pos.getX(), curr_pos.getY()) and ((circles[i]).getExits().size() >= 1)){
      currPosInCircleWithExit = true;
      robotCircle = i;
    }
  }

  if(targetInCircle == true and currPosInCircleWithExit == true and robotCircle != targetCircle)
    advisor_commenting = true;
  else
    advisor_commenting = false;
}


double Tier3RegionFinderRotation::actionComment(FORRAction action){

  cout << "In region finder rotation " << endl;

  double result;
  vector<FORRCircle> circles = beliefs->getSpatialModel()->getAbstractMap()->getCircles();
  int robotCircle = -1,targetCircle = -1;
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  bool targetInCircle = false;
  bool currPosInCircleWithExit = false;

  // check the preconditions for activating the advisor
  for(int i = 0; i < circles.size() ; i++){
    // check if the rob_pos is in a circle and the circle has atleast one exit
    if(circles[i].inCircle(curr_pos.getX(), curr_pos.getY()) and ((circles[i]).getExits().size() >= 1)){
      currPosInCircleWithExit = true;
      robotCircle = i;
    }
  }

  vector<FORRAction> actionList;
  actionList.push_back(action);

  FORRAction max_forward_move = FORRAction(FORWARD,5);
  actionList.push_back(max_forward_move);

  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterActions(actionList);
  
  vector<FORRExit> exits = circles[robotCircle].getExits();
  
  vector<FORRCircle> nearCircles;
  for(int i = 0; i < exits.size() ; i++){
    FORRCircle test = circles[exits[i].getExitCircle()];
    std::vector<FORRCircle>::iterator it = std::find(nearCircles.begin(),nearCircles.end(), test);
    if(nearCircles.empty() or (it == nearCircles.end())){
      nearCircles.push_back(test);
    }
  }

  double metric = 0;
  for(int i = 0; i < nearCircles.size(); i++){
    if(beliefs->getSpatialModel()->getAbstractMap()->isLeaf(nearCircles[i]) and nearCircles[i].inCircle(targetPoint.get_x(), targetPoint.get_y())){
      cout << "RegionFinderRotation: Found Target Region !" << endl;
      metric += pow(expectedPosition.getDistance(nearCircles[i].getCenter().get_x(), nearCircles[i].getCenter().get_y()), 2);
    }
  }

  return metric * (-1);
}

void Tier3RegionFinderRotation::set_commenting(){

  cout << "In region finder rotation set commenting " << endl;
  vector<FORRCircle> circles = beliefs->getSpatialModel()->getAbstractMap()->getCircles();
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  bool targetInCircle = false;
  bool currPosInCircleWithExit = false;
  int robotCircle=-1, targetCircle = -1;
  
  // check the preconditions for activating the advisor
  for(int i = 0; i < circles.size() ; i++){
    // check if the target point is in circle
    if(circles[i].inCircle(targetPoint.get_x(), targetPoint.get_y())){
      targetInCircle = true;
      targetCircle = i;
    }
    // check if the rob_pos is in a circle and the circle has atleast one exit
    if(circles[i].inCircle(curr_pos.getX(), curr_pos.getY()) and ((circles[i]).getExits().size() >= 1)){
      currPosInCircleWithExit = true;
      robotCircle = i;
    }
  }

  if(targetInCircle == true and currPosInCircleWithExit == true and robotCircle != targetCircle)
    advisor_commenting = true;
  else
    advisor_commenting = false;
}

double Tier3ExitFinderLinear::actionComment(FORRAction action){
  double result=0;
  vector<FORRCircle> circles = beliefs->getSpatialModel()->getAbstractMap()->getCircles();
  
  int robotCircle=-1;
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
 
  // check the preconditions for activating the advisor
  for(int i = 0; i < circles.size() ; i++){
    // check if the rob_pos is in a circle and the circle has atleast one exit
    if(circles[i].inCircle(curr_pos.getX(), curr_pos.getY())){
      robotCircle = i;
    }
  }

  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  
  cout << "Robot Circle : " << circles[robotCircle].getCenter().get_x() << " " << circles[robotCircle].getCenter().get_y() << endl;
  vector<FORRExit> ext_exits = circles[robotCircle].getExtExits();

  for(int i = 0 ; i < ext_exits.size(); i++){
    result += expectedPosition.getDistance(ext_exits[i].getExitPoint().get_x(), ext_exits[i].getExitPoint().get_y());
  }

  return result*(-1);
}


void Tier3ExitFinderLinear::set_commenting(){

  vector<FORRCircle> circles = beliefs->getSpatialModel()->getAbstractMap()->getCircles();
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  bool targetInCircle = false;
  bool currPosInCircleWithExit = false;
  int robotCircle=-1, targetCircle = -1;
  
  // check the preconditions for activating the advisor
  for(int i = 0; i < circles.size() ; i++){
    // check if the target point is in circle
    if(circles[i].inCircle(targetPoint.get_x(), targetPoint.get_y())){
      targetInCircle = true;
      targetCircle = i;
    }
    // check if the rob_pos is in a circle and the circle has atleast one exit
    if(circles[i].inCircle(curr_pos.getX(), curr_pos.getY()) and ((circles[i]).getExits().size() >= 1)){
      currPosInCircleWithExit = true;
      robotCircle = i;
    }
  }
  if(currPosInCircleWithExit == true and robotCircle != targetCircle)
    advisor_commenting = true;
  else
    advisor_commenting = false;
}


double Tier3ExitFinderRotation::actionComment(FORRAction action){
  double result=0;
  vector<FORRCircle> circles = beliefs->getSpatialModel()->getAbstractMap()->getCircles();
  
  int robotCircle=-1;
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
 
  // check the preconditions for activating the advisor
  for(int i = 0; i < circles.size() ; i++){
    // check if the rob_pos is in a circle and the circle has atleast one exit
    if(circles[i].inCircle(curr_pos.getX(), curr_pos.getY())){
      robotCircle = i;
    }
  }

  vector<FORRAction> actionList;
  actionList.push_back(action);
  
  FORRAction max_forward_move = FORRAction(FORWARD,5);
  actionList.push_back(max_forward_move);
  
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterActions(actionList);

  cout << "Robot Circle : " << circles[robotCircle].getCenter().get_x() << " " << circles[robotCircle].getCenter().get_y() << endl;
  vector<FORRExit> ext_exits = circles[robotCircle].getExtExits();

  for(int i = 0 ; i < ext_exits.size(); i++){
    result += expectedPosition.getDistance(ext_exits[i].getExitPoint().get_x(), ext_exits[i].getExitPoint().get_y());
  }

  return result*(-1);
}

void Tier3ExitFinderRotation::set_commenting(){
  vector<FORRCircle> circles = beliefs->getSpatialModel()->getAbstractMap()->getCircles();
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  bool targetInCircle = false;
  bool currPosInCircleWithExit = false;
  int robotCircle=-1, targetCircle = -1;
  
  // check the preconditions for activating the advisor
  for(int i = 0; i < circles.size() ; i++){
    // check if the target point is in circle
    if(circles[i].inCircle(targetPoint.get_x(), targetPoint.get_y())){
      targetInCircle = true;
      targetCircle = i;
    }
    // check if the rob_pos is in a circle and the circle has atleast one exit
    if(circles[i].inCircle(curr_pos.getX(), curr_pos.getY()) and ((circles[i]).getExits().size() >= 1)){
      currPosInCircleWithExit = true;
      robotCircle = i;
    }
  }
  if(currPosInCircleWithExit == true and robotCircle != targetCircle)
    advisor_commenting = true;
  else
    advisor_commenting = false;
  
}

//CloseIn : When target is nearby, distance is within 80 units, go towards it!
double Tier3CloseIn::actionComment(FORRAction action){
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  
  double task_x = beliefs->getAgentState()->getCurrentTask()->getX();
  double task_y = beliefs->getAgentState()->getCurrentTask()->getY();

  double newDistance = expectedPosition.getDistance(task_x, task_y);
  
  return newDistance *(-1);
}


// this advisor is active only when robot is 80px away from the target
void Tier3CloseIn::set_commenting(){
  double distanceToTarget = beliefs->getAgentState()->getDistanceToTarget();
  if(distanceToTarget > 80)
    advisor_commenting = false;
  else
    advisor_commenting = true;
}


double Tier3CloseInRotation::actionComment(FORRAction action){
  double newDistance;

  vector<FORRAction> actionList;
  actionList.push_back(action);

  FORRAction max_forward_move = FORRAction(FORWARD,5);
  actionList.push_back(max_forward_move);
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterActions(actionList);
  
  double task_x = beliefs->getAgentState()->getCurrentTask()->getX();
  double task_y = beliefs->getAgentState()->getCurrentTask()->getY();

  newDistance = expectedPosition.getDistance(task_x, task_y);

  return newDistance*(-1);
}


void Tier3CloseInRotation::set_commenting(){
  double distanceToTarget = beliefs->getAgentState()->getDistanceToTarget();
  if(distanceToTarget > 80)
    advisor_commenting = false;
  else
    advisor_commenting = true;
}

/*
double Tier3AvoidRobot::actionComment(FORRAction action){
  double result;
  //max_len is used as range
  double range = 100;
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  double sumOfTeammateDistances = beliefs->getTeamState()->getSumOfTeammateDistances(expectedPosition, range);  
  return sumOfTeammateDistances;
}

// On when there are more than one robots in the field
void Tier3AvoidRobot::set_commenting(){
  if((beliefs->beliefs->getTeamState()->getTeamPose()).size() <= 1)
    advisor_commenting = false;
  else
    advisor_commenting = true;
}

double Tier3AvoidRobotRotation::actionComment(FORRAction action){
  double result;
  double range = 100;
  vector<FORRAction> actionList;
  actionList.push_back(action);
  FORRAction max_forward_move = FORRAction(FORWARD,5);
  actionList.push_back(max_forward_move);
  
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterActions(actionList);
  double sumOfTeammateDistances = beliefs->getTeamState()->getSumOfTeammateDistances(expectedPosition, range);
  return sumOfTeammateDistances;
}

void Tier3AvoidRobotRotation::set_commenting(){
  if((beliefs->getTeamState()->getTeamPose()).size() <= 1)
    advisor_commenting = false;
  else
    advisor_commenting = true;
}
*/

//Wants to make moves that keep the robot as far as possible from the obstacles
double Tier3AvoidObstacle::actionComment(FORRAction action){

  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  double distanceToObstacle = beliefs->getAgentState()->getDistanceToNearestObstacle(expectedPosition);
  return distanceToObstacle;
}

// always on
void Tier3AvoidObstacle::set_commenting(){
  advisor_commenting = true;
}

double Tier3AvoidObstacleRotation::actionComment(FORRAction action){

  vector<FORRAction> actionList;
  actionList.push_back(action);
  actionList.push_back(FORRAction(FORWARD,5));
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterActions(actionList);
  double obstacleDistance = beliefs->getAgentState()->getDistanceToNearestObstacle(expectedPosition);
  
  return obstacleDistance;
}

void Tier3AvoidObstacleRotation::set_commenting(){
  advisor_commenting = true;
}


double Tier3Greedy::actionComment(FORRAction action){
  
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  
  double task_x = beliefs->getAgentState()->getCurrentTask()->getX();
  double task_y = beliefs->getAgentState()->getCurrentTask()->getY();

  double newDistance = expectedPosition.getDistance(task_x, task_y);
  
  return newDistance *(-1);
}

// always on
void Tier3Greedy::set_commenting(){
  advisor_commenting = true;
}

double Tier3GreedyRotation::actionComment(FORRAction action){
  double newDistance;

  vector<FORRAction> actionList;
  actionList.push_back(action);

  FORRAction max_forward_move = FORRAction(FORWARD,5);
  actionList.push_back(max_forward_move);
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterActions(actionList);
  
  double task_x = beliefs->getAgentState()->getCurrentTask()->getX();
  double task_y = beliefs->getAgentState()->getCurrentTask()->getY();

  newDistance = expectedPosition.getDistance(task_x, task_y);

  return newDistance*(-1);
}

void Tier3GreedyRotation::set_commenting(){
  advisor_commenting = true;
}

// Comment strength function for BigStep advisor
double Tier3BigStep::actionComment(FORRAction action){
   
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);

  double cur_x = beliefs->getAgentState()->getCurrentPosition().getX();
  double cur_y = beliefs->getAgentState()->getCurrentPosition().getY();

  double distanceFromCurrentPosition = expectedPosition.getDistance(cur_x,cur_y);
  return distanceFromCurrentPosition;
}

// commenting only when robot is sufficiently far from the obstacle
void Tier3BigStep::set_commenting(){
  
  advisor_commenting = true;
  
}

double Tier3BigStepRotation::actionComment(FORRAction action){
   
  vector<FORRAction> actions;
  actions.push_back(action);
  actions.push_back(FORRAction(FORWARD, 5));

  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterActions(actions);

  double cur_x = beliefs->getAgentState()->getCurrentPosition().getX();
  double cur_y = beliefs->getAgentState()->getCurrentPosition().getY();

  double distanceFromCurrentPosition = expectedPosition.getDistance(cur_x,cur_y);
  return distanceFromCurrentPosition;
}

void Tier3BigStepRotation::set_commenting(){
  
  advisor_commenting = true;
  
}


// always on
void Tier3AvoidLeaf::set_commenting(){
  //cout << "In avoid leaf set commenting " << endl;
  vector<FORRCircle> circles = beliefs->getSpatialModel()->getAbstractMap()->getCircles();
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  bool targetInCircle = false;
  bool currPosInCircleWithExit = false;
  int robotCircle=-1, targetCircle = -1;
  
  // check the preconditions for activating the advisor
  for(int i = 0; i < circles.size() ; i++){
    // check if the target point is in circle
    if(circles[i].inCircle(targetPoint.get_x(), targetPoint.get_y())){
      targetInCircle = true;
      targetCircle = i;
    }
    // check if the rob_pos is in a circle and the circle has atleast one exit
    if(circles[i].inCircle(curr_pos.getX(), curr_pos.getY()) and ((circles[i]).getExits().size() >= 1)){
      currPosInCircleWithExit = true;
      robotCircle = i;
    }
  }

  if(currPosInCircleWithExit == true and robotCircle != targetCircle)
    advisor_commenting = true;
  else
    advisor_commenting = false;
}

double Tier3AvoidLeaf::actionComment(FORRAction action){
  //cout << "In Avoid leaf " << endl;
  double result;
  vector<FORRCircle> circles = beliefs->getSpatialModel()->getAbstractMap()->getCircles();
  int robotCircle=-1,targetCircle=-1;
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());

  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);

  // check the preconditions for activating the advisor
  for(int i = 0; i < circles.size() ; i++){
    // check if the target point is in circle
    if(circles[i].inCircle(targetPoint.get_x(), targetPoint.get_y())){
      targetCircle = i;
    }
    // check if the rob_pos is in a circle and the circle has atleast one exit
    if(circles[i].inCircle(curr_pos.getX(), curr_pos.getY())){
      robotCircle = i;
    }
  }
  
  vector<FORRExit> exits = circles[robotCircle].getExits();
  //cout << "Robot Circle : " << circles[robotCircle].getCenter().get_x() << " " << circles[robotCircle].getCenter().get_y() << endl;

  vector<FORRCircle> nearCircles;
  for(int i = 0; i < exits.size() ; i++){
    FORRCircle test = circles[exits[i].getExitCircle()];
    std::vector<FORRCircle>::iterator it = std::find(nearCircles.begin(),nearCircles.end(), test);
    if(nearCircles.empty() or (it == nearCircles.end())){
      nearCircles.push_back(test);
      //cout << "Neighbour Circle : " << test.getCenter().get_x() << " " << test.getCenter().get_y() << endl;
    }
  }

  //cout << "#neighbours found" << nearCircles.size() << endl; 

  double metric = 0;
  for(int i = 0; i < nearCircles.size(); i++){
    if(beliefs->getSpatialModel()->getAbstractMap()->isLeaf(nearCircles[i]) and !(nearCircles[i].inCircle(targetPoint.get_x(), targetPoint.get_y()))){
      //cout << "Avoid Leaf: Found deadend !" << endl;
      metric += pow(expectedPosition.getDistance(nearCircles[i].getCenter().get_x(), nearCircles[i].getCenter().get_y()), 2);
    }
  }

  return metric;
}

void Tier3AvoidLeafRotation::set_commenting(){
  //cout << "In region finder rotation set commenting " << endl;
   vector<FORRCircle> circles = beliefs->getSpatialModel()->getAbstractMap()->getCircles();
   Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
   Task *task = beliefs->getAgentState()->getCurrentTask();
   CartesianPoint targetPoint (task->getX() , task->getY());
   CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
   bool targetInCircle = false;
   bool currPosInCircleWithExit = false;
   int robotCircle=-1, targetCircle = -1;
   
   // check the preconditions for activating the advisor
   for(int i = 0; i < circles.size() ; i++){
     // check if the target point is in circle
     if(circles[i].inCircle(targetPoint.get_x(), targetPoint.get_y())){
       targetInCircle = true;
       targetCircle = i;
     }
     // check if the rob_pos is in a circle and the circle has atleast one exit
     if(circles[i].inCircle(curr_pos.getX(), curr_pos.getY()) and ((circles[i]).getExits().size() >= 1)){
       currPosInCircleWithExit = true;
       robotCircle = i;
     }
   }
   
   if(currPosInCircleWithExit == true and robotCircle != targetCircle)
     advisor_commenting = true;
   else
     advisor_commenting = false;
}

double Tier3AvoidLeafRotation::actionComment(FORRAction action){
  //cout << "In Avoid leaf Rotation" << endl;
  double result;
  vector<FORRCircle> circles = beliefs->getSpatialModel()->getAbstractMap()->getCircles();
  int robotCircle=-1,targetCircle=-1;
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());

  vector<FORRAction> actionList;
  actionList.push_back(action);
  actionList.push_back(FORRAction(FORWARD,5));

  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterActions(actionList);

  // check the preconditions for activating the advisor
  for(int i = 0; i < circles.size() ; i++){
    // check if the target point is in circle
    if(circles[i].inCircle(targetPoint.get_x(), targetPoint.get_y())){
      targetCircle = i;
    }
    // check if the rob_pos is in a circle and the circle has atleast one exit
    if(circles[i].inCircle(curr_pos.getX(), curr_pos.getY())){
      robotCircle = i;
    }
  }
  
  vector<FORRExit> exits = circles[robotCircle].getExits();
  //cout << "Robot Circle : " << circles[robotCircle].getCenter().get_x() << " " << circles[robotCircle].getCenter().get_y() << endl;

  vector<FORRCircle> nearCircles;
  for(int i = 0; i < exits.size() ; i++){
    FORRCircle test = circles[exits[i].getExitCircle()];
    std::vector<FORRCircle>::iterator it = std::find(nearCircles.begin(),nearCircles.end(), test);
    if(nearCircles.empty() or (it == nearCircles.end())){
      nearCircles.push_back(test);
      //cout << "Neighbour Circle : " << test.getCenter().get_x() << " " << test.getCenter().get_y() << endl;
    }
  }

  //cout << "#neighbours found" << nearCircles.size() << endl; 

  double metric = 0;
  for(int i = 0; i < nearCircles.size(); i++){
    if(beliefs->getSpatialModel()->getAbstractMap()->isLeaf(nearCircles[i]) and !(nearCircles[i].inCircle(targetPoint.get_x(), targetPoint.get_y()))){
      //cout << "Avoid leaf Rotation: Found deadend !" << endl;
      metric += pow(expectedPosition.getDistance(nearCircles[i].getCenter().get_x(), nearCircles[i].getCenter().get_y()), 2);
    }
  }

  return metric;
}

// always on
void Tier3Explorer::set_commenting(){
  advisor_commenting = true;
}

double Tier3Explorer::actionComment(FORRAction action){
 
  vector<Position> *positionHis = beliefs->getAgentState()->getPositionHistory();
 
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  int beta = 0;
  double totalForce = 0, distance = 0; 

  for(int i = 0; i < positionHis->size(); i++){
    distance = expectedPosition.getDistance((*positionHis)[i]);
    if(distance < 1)     distance = 1;
    totalForce += (1/distance);
  }
  return totalForce * (-1);
  
}

double Tier3BaseLine::actionComment(FORRAction action){
  //srand (time(NULL));
  //cout << "Baseline :::::::::::::::::::::::::::::::::::::::::::::::::::::::" << rand()%10 - 5 << endl;
  return rand()%10 - 5;
}

void Tier3BaseLine::set_commenting(){
  //srand (time(NULL));
  if(rand()%2 == 0)
    advisor_commenting = true;
  else
    advisor_commenting = false;
  //advisor_commenting = true;
}

void Tier3ExplorerRotation::set_commenting(){
  advisor_commenting = true;
}

double Tier3ExplorerRotation::actionComment(FORRAction action){
 
  vector<FORRAction> actionList;
  actionList.push_back(action);
  
  actionList.push_back(FORRAction(FORWARD,5));
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterActions(actionList);

  vector<Position> *positionHis = beliefs->getAgentState()->getPositionHistory();
  int beta = 0;
  double totalForce = 0, distance = 0;
 
  for(int i = 0; i < positionHis->size(); i++){
    distance = expectedPosition.getDistance((*positionHis)[i]);
    if(distance < 1)     distance = 1;
    totalForce += (1/distance);
  }
  return totalForce * (-1);
}

double Tier3BaseLineRotation::actionComment(FORRAction action){
  //srand (time(NULL));
  return rand()%10 - 5;
}

void Tier3BaseLineRotation::set_commenting(){
  //srand (time(NULL));
  if(rand()%2 == 0)
    advisor_commenting = true;
  else
    advisor_commenting = false;
  //advisor_commenting = true;
}



//votes for moves that will take it to an adjacent cell block, given by the overlay grid, 
//that has the most points in it.  
//The metric is the distance to the expected point times the grid value, because we want to pull
//the robot further distances, but at the same time want it to go to high-value grid cells
double Tier3WaypointFinderLinear::actionComment(FORRAction action){
 
  //cout << "Entered waypointfinder linear."<<endl;
  Position cur_pos = beliefs->getAgentState()->getCurrentPosition();
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  
  int grid_value = beliefs->getSpatialModel()->getWaypoints()->getGridValue(expectedPosition.getX(), expectedPosition.getY());
  
  double distance = cur_pos.getDistance(expectedPosition.getX(), expectedPosition.getY());
  
  //cout <<"Exit waypoint linear."<<endl;
  return distance * grid_value; //want larger grid values that are further away
}


void Tier3WaypointFinderLinear::set_commenting(){
  advisor_commenting = true;
}

double Tier3WaypointFinderRotation::actionComment(FORRAction action){
  //cout <<" Entered waypointfinder rotation." << endl;
  vector<FORRAction> actionList;
  actionList.push_back(action);
  actionList.push_back(FORRAction(FORWARD,5));
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterActions(actionList);

  int grid_value = beliefs->getSpatialModel()->getWaypoints()->getGridValue(expectedPosition.getX(), expectedPosition.getY());
  //cout << "grid value: "<<grid_value<<endl;

  Position cur_pos = beliefs->getAgentState()->getCurrentPosition();
  double distance = cur_pos.getDistance(expectedPosition.getX(), expectedPosition.getY());  
  return distance * grid_value;
}

void Tier3WaypointFinderRotation::set_commenting(){
  advisor_commenting = true;
}


double Tier3TrailFinderLinear::actionComment(FORRAction action){
   CartesianPoint target_trailmarker = beliefs->getSpatialModel()->getTrails()->getFurthestVisiblePointOnChosenTrail();
   //if out of line of sight of trail marker, vote the same value functionally turning off voting
   if(!beliefs->getSpatialModel()->getTrails()->canSeeTrail()){
     return 0;
   }

  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  double newDistance = expectedPosition.getDistance(beliefs->getAgentState()->getCurrentPosition());
  return newDistance *(-1); 
}

//only turn on if a trail is active
void Tier3TrailFinderLinear::set_commenting(){
  if(beliefs->getSpatialModel()->getTrails()->getChosenTrail() == -1){
    advisor_commenting = false;
  }
  else{
      advisor_commenting = true;
  }
}

double Tier3TrailFinderRotation::actionComment(FORRAction action){
  Position cur_pos = beliefs->getAgentState()->getCurrentPosition();
  CartesianPoint target_trailmarker = beliefs->getSpatialModel()->getTrails()->getFurthestVisiblePointOnChosenTrail();
   //if out of line of sight of trail marker, vote the same value in effect turning off voting
  //can_see_trail is set in getFurthestVisiblePointOnChosenTrail
   if(!beliefs->getSpatialModel()->getTrails()->canSeeTrail()){
     return 0;
   }

  vector<FORRAction> actionList;
  actionList.push_back(action);
  actionList.push_back(FORRAction(FORWARD,5));

  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterActions(actionList);
  double newDistance = expectedPosition.getDistance(cur_pos);
  return newDistance *(-1); 
}


//only turn on if a trail is active
void Tier3TrailFinderRotation::set_commenting(){
  if(beliefs->getSpatialModel()->getTrails()->getChosenTrail() == -1){
    advisor_commenting = false;
  }
  else{
    advisor_commenting = true;
  }
}
