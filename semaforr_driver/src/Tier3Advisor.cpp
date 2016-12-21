/*
 *
 * Implementation of Advisors 
 *
 * Created on: Jun. 27, 2013
 * Last modified: April 14, 2013
 * created by Slavisa Djukic <sdjukic@hunter.cuny.edu>
 */

# include "Map.h"
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
Tier3Advisor::Tier3Advisor(Beliefs *beliefs_para, string name, string description, vector<double> weight, double *magic_init, bool is_active)  { 
  //  cout << "In constructor of tier 3 advisor" << endl;
  beliefs = beliefs_para;


  //  cout << "Initializing the allActions" << endl;

  set<FORRAction> all_actions;

  std::size_t found = (name).find("Rotation");
  if (found!=std::string::npos){
    for(int i = 1; i < 6; i++){
      all_actions.insert(FORRAction(LEFT_TURN, i));
      all_actions.insert(FORRAction(RIGHT_TURN, i));
    }
  }
  else{
    for(int i = 1; i< 6; i++){
      all_actions.insert(FORRAction(FORWARD, i));
    }
    all_actions.insert(FORRAction(PAUSE,0));
  }
  
  // cout << "End of initializing actions: size: " << all_actions.size() << endl;

  agentActions = all_actions;
  agent_name = name;
  agentDescription = description;
  advisorWeight = weight;
  advisor_active = is_active;
  //  cout << "before initializing movements and rotations" << endl;

  int mmts[] = {3, 7, 20, 25, 105}; 
  double rots[] =  {0, .1548, -.1548, .3048, -.3048, .65, -.65, 1.3, -1.3, 3.39, -3.39}; 

  for( int i = 0 ; i < 5; i++)
    movements[i] = mmts[i];
  for(int i = 0; i < 11; i++)
    rotations[i] = rots[i];

  cout << "before initializing auxilary constants" << endl;
  // load all magic numbers for this advisor
  for (int i = 0; i < 4; ++i)
    auxiliary_constants[i] = magic_init[i];
}

// Destructor
Tier3Advisor::~Tier3Advisor() {};

// this function will produce comment strengths for all possible actions 
// robot can make at the certain moment
// it does it by successively calling actionComment method for each action
// No arguments are necessary because actions are stored as member variable
// in advisor 
// It returns map that maps action to comment strength
std::map <FORRAction, double> Tier3Advisor::allAdvice(set<FORRAction> vetoed_actions){
  std::map <FORRAction, double> result;
  double adviceStrength;
  FORRAction forrAction;
  //std::cout << this->agent_name << ": in allAdvice function .. comments are as following:" << std::endl;
  set<FORRAction>::iterator actionIter;
  for(actionIter = agentActions.begin(); actionIter != agentActions.end(); actionIter++){
    forrAction = *actionIter;
    std::size_t foundr = (this->get_name()).find("Rotation");
    if( ((forrAction.type == LEFT_TURN or forrAction.type == RIGHT_TURN ) and (foundr == std::string::npos)) or (forrAction.type == FORWARD and foundr != std::string::npos))
      break;
    if(vetoed_actions.find(forrAction) != vetoed_actions.end())// is this action vetoed
      continue;
    adviceStrength = this->actionComment(forrAction);
    //std::cout << "Advisor name :"  << this->get_name() << " Strength: " << adviceStrength << " Action Type:" << forrAction.type << " " << "Action intensity " << forrAction.parameter << std::endl;
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
Tier3Advisor* Tier3Advisor::makeAdvisor(Beliefs *beliefs, string name, string description, vector<double> weight, double *magic_init, bool is_active){
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
  else if(name == "GoAroundRotation")
    return new Tier3GoAroundRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "ExplorerRotation")
    return new Tier3ExplorerRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "BaseLineRotation")
    return new Tier3BaseLineRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "AvoidRobotRotation")
    return new Tier3AvoidRobotRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "AvoidRobot")
    return new Tier3AvoidRobot(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "GateFinderLinear")
    return new Tier3GateFinderLinear(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "GateFinderRotation")
    return new Tier3GateFinderRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "ExitFinderLinear")
    return new Tier3ExitFinderLinear(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "ExitFinderRotation")
    return new Tier3ExitFinderRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "RegionFinderLinear")
    return new Tier3RegionFinderLinear(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "RegionFinderRotation")
    return new Tier3RegionFinderRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "PortalHopperLinear")
    return new Tier3PortalHopperLinear(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "PortalHopperRotation")
    return new Tier3PortalHopperRotation(beliefs, name, description, weight, magic_init, is_active);
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
Tier3Greedy::Tier3Greedy (Beliefs *beliefs, string name, string description, vector<double> weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3AvoidObstacle::Tier3AvoidObstacle(Beliefs *beliefs, string name, string description, vector<double> weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3CloseIn::Tier3CloseIn(Beliefs *beliefs, string name, string description, vector<double> weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3BigStep::Tier3BigStep(Beliefs *beliefs, string name, string description, vector<double> weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};  
Tier3AvoidLeaf::Tier3AvoidLeaf(Beliefs *beliefs, string name, string description, vector<double> weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3Explorer::Tier3Explorer(Beliefs *beliefs, string name, string description, vector<double> weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3BaseLine::Tier3BaseLine (Beliefs *beliefs, string name, string description, vector<double> weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3AvoidRobot::Tier3AvoidRobot (Beliefs *beliefs, string name, string description, vector<double> weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 

Tier3GreedyRotation::Tier3GreedyRotation (Beliefs *beliefs, string name, string description, vector<double> weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3AvoidObstacleRotation::Tier3AvoidObstacleRotation(Beliefs *beliefs, string name, string description, vector<double> weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3CloseInRotation::Tier3CloseInRotation(Beliefs *beliefs, string name, string description, vector<double> weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3BigStepRotation::Tier3BigStepRotation(Beliefs *beliefs, string name, string description, vector<double> weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3GoAroundRotation::Tier3GoAroundRotation(Beliefs *beliefs, string name, string description, vector<double> weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3AvoidLeafRotation::Tier3AvoidLeafRotation(Beliefs *beliefs, string name, string description, vector<double> weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3ExplorerRotation::Tier3ExplorerRotation(Beliefs *beliefs, string name, string description, vector<double> weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3BaseLineRotation::Tier3BaseLineRotation (Beliefs *beliefs, string name, string description, vector<double> weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3AvoidRobotRotation::Tier3AvoidRobotRotation (Beliefs *beliefs, string name, string description, vector<double> weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3GateFinderLinear::Tier3GateFinderLinear (Beliefs *beliefs, string name, string description, vector<double> weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3GateFinderRotation::Tier3GateFinderRotation (Beliefs *beliefs, string name, string description, vector<double> weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3ExitFinderLinear::Tier3ExitFinderLinear (Beliefs *beliefs, string name, string description, vector<double> weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3ExitFinderRotation::Tier3ExitFinderRotation (Beliefs *beliefs, string name, string description, vector<double> weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3RegionFinderLinear::Tier3RegionFinderLinear (Beliefs *beliefs, string name, string description, vector<double> weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3RegionFinderRotation::Tier3RegionFinderRotation (Beliefs *beliefs, string name, string description, vector<double> weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3PortalHopperLinear::Tier3PortalHopperLinear (Beliefs *beliefs, string name, string description, vector<double> weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3PortalHopperRotation::Tier3PortalHopperRotation (Beliefs *beliefs, string name, string description, vector<double> weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3WaypointFinderLinear::Tier3WaypointFinderLinear (Beliefs *beliefs, string name, string description, vector<double> weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3WaypointFinderRotation::Tier3WaypointFinderRotation (Beliefs *beliefs, string name, string description, vector<double> weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3TrailFinderLinear::Tier3TrailFinderLinear (Beliefs *beliefs, string name, string description, vector<double> weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3TrailFinderRotation::Tier3TrailFinderRotation (Beliefs *beliefs, string name, string description, vector<double> weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
 


// Default dummy constructors
Tier3Greedy::Tier3Greedy(): Tier3Advisor() {};
Tier3AvoidObstacle::Tier3AvoidObstacle(): Tier3Advisor() {};
Tier3CloseIn::Tier3CloseIn(): Tier3Advisor() {};
Tier3BigStep::Tier3BigStep(): Tier3Advisor() {};
Tier3AvoidLeaf::Tier3AvoidLeaf(): Tier3Advisor() {};
Tier3Explorer::Tier3Explorer(): Tier3Advisor() {};
Tier3AvoidRobot::Tier3AvoidRobot(): Tier3Advisor() {};
Tier3BaseLine::Tier3BaseLine(): Tier3Advisor() {};

Tier3GreedyRotation::Tier3GreedyRotation(): Tier3Advisor() {};
Tier3AvoidObstacleRotation::Tier3AvoidObstacleRotation(): Tier3Advisor() {};
Tier3CloseInRotation::Tier3CloseInRotation(): Tier3Advisor() {};
Tier3BigStepRotation::Tier3BigStepRotation(): Tier3Advisor() {};
Tier3GoAroundRotation::Tier3GoAroundRotation(): Tier3Advisor() {};
Tier3AvoidLeafRotation::Tier3AvoidLeafRotation(): Tier3Advisor() {};
Tier3ExplorerRotation::Tier3ExplorerRotation(): Tier3Advisor() {};
Tier3AvoidRobotRotation::Tier3AvoidRobotRotation(): Tier3Advisor() {};
Tier3BaseLineRotation::Tier3BaseLineRotation(): Tier3Advisor() {};
Tier3GateFinderLinear::Tier3GateFinderLinear(): Tier3Advisor() {};
Tier3GateFinderRotation::Tier3GateFinderRotation(): Tier3Advisor() {};
Tier3ExitFinderLinear::Tier3ExitFinderLinear(): Tier3Advisor() {};
Tier3ExitFinderRotation::Tier3ExitFinderRotation(): Tier3Advisor() {};
Tier3RegionFinderLinear::Tier3RegionFinderLinear(): Tier3Advisor() {};
Tier3RegionFinderRotation::Tier3RegionFinderRotation(): Tier3Advisor() {};
Tier3PortalHopperLinear::Tier3PortalHopperLinear(): Tier3Advisor() {};
Tier3PortalHopperRotation::Tier3PortalHopperRotation(): Tier3Advisor() {};
Tier3WaypointFinderLinear::Tier3WaypointFinderLinear(): Tier3Advisor() {};
Tier3WaypointFinderRotation::Tier3WaypointFinderRotation(): Tier3Advisor() {};
Tier3TrailFinderLinear::Tier3TrailFinderLinear(): Tier3Advisor() {};
Tier3TrailFinderRotation::Tier3TrailFinderRotation(): Tier3Advisor() {};


// vote to go through an extrance to a region containing the target

double Tier3RegionFinderLinear::actionComment(FORRAction action){
  cout << "In region finder linear " << endl;
  double result;
  vector<FORRCircle> circles = (beliefs->abstractMap).getCircles();
  int robotCircle=-1,targetCircle=-1;
  Position curr_pos = beliefs->getCurrentPosition();
  Task *task = beliefs->getCurrentTask();
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

  Position expectedPosition = beliefs->getExpectedPositionAfterAction(action);
  
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
    if(beliefs->abstractMap.isLeaf(nearCircles[i]) and nearCircles[i].inCircle(targetPoint.get_x(), targetPoint.get_y())){
      cout << "RegionFinder: Found Target Region !" << endl;
      metric += pow(expectedPosition.getDistance(nearCircles[i].getCenter().get_x(), nearCircles[i].getCenter().get_y()), 2);
    }
  }

  return metric * (-1);

}


void Tier3RegionFinderLinear::set_commenting(){

  cout << "In region finder linear set commenting " << endl;
  vector<FORRCircle> circles = (beliefs->abstractMap).getCircles();
  Position curr_pos = beliefs->getCurrentPosition();
  Task *task = beliefs->getCurrentTask();
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
  vector<FORRCircle> circles = (beliefs->abstractMap).getCircles();
  int robotCircle = -1,targetCircle = -1;
  Task *task = beliefs->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  Position curr_pos = beliefs->getCurrentPosition();
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
  
  int max_forward_intensity = beliefs->get_maximum_intensity_following_linear_move(action);
  FORRAction max_forward_move = FORRAction(FORWARD,max_forward_intensity);
  actionList.push_back(max_forward_move);

  Position expectedPosition = beliefs->getExpectedPositionAfterActions(actionList);
  
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
    if(beliefs->abstractMap.isLeaf(nearCircles[i]) and nearCircles[i].inCircle(targetPoint.get_x(), targetPoint.get_y())){
      cout << "RegionFinderRotation: Found Target Region !" << endl;
      metric += pow(expectedPosition.getDistance(nearCircles[i].getCenter().get_x(), nearCircles[i].getCenter().get_y()), 2);
    }
  }

  return metric * (-1);
}

void Tier3RegionFinderRotation::set_commenting(){

  cout << "In region finder rotation set commenting " << endl;
  vector<FORRCircle> circles = (beliefs->abstractMap).getCircles();
  Position curr_pos = beliefs->getCurrentPosition();
  Task *task = beliefs->getCurrentTask();
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
  vector<FORRCircle> circles = (beliefs->abstractMap).getCircles();
  
  int robotCircle=-1;
  Position curr_pos = beliefs->getCurrentPosition();
 
  // check the preconditions for activating the advisor
  for(int i = 0; i < circles.size() ; i++){
    // check if the rob_pos is in a circle and the circle has atleast one exit
    if(circles[i].inCircle(curr_pos.getX(), curr_pos.getY())){
      robotCircle = i;
    }
  }

  Position expectedPosition = beliefs->getExpectedPositionAfterAction(action);
  
  cout << "Robot Circle : " << circles[robotCircle].getCenter().get_x() << " " << circles[robotCircle].getCenter().get_y() << endl;
  vector<FORRExit> ext_exits = circles[robotCircle].getExtExits();

  for(int i = 0 ; i < ext_exits.size(); i++){
    result += expectedPosition.getDistance(ext_exits[i].getExitPoint().get_x(), ext_exits[i].getExitPoint().get_y());
  }

  return result*(-1);
}


void Tier3ExitFinderLinear::set_commenting(){

  vector<FORRCircle> circles = (beliefs->abstractMap).getCircles();
  Position curr_pos = beliefs->getCurrentPosition();
  Task *task = beliefs->getCurrentTask();
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
  vector<FORRCircle> circles = (beliefs->abstractMap).getCircles();
  
  int robotCircle=-1;
  Position curr_pos = beliefs->getCurrentPosition();
 
  // check the preconditions for activating the advisor
  for(int i = 0; i < circles.size() ; i++){
    // check if the rob_pos is in a circle and the circle has atleast one exit
    if(circles[i].inCircle(curr_pos.getX(), curr_pos.getY())){
      robotCircle = i;
    }
  }

  vector<FORRAction> actionList;
  actionList.push_back(action);
  
  int max_forward_intensity = beliefs->get_maximum_intensity_following_linear_move(action);
  FORRAction max_forward_move = FORRAction(FORWARD,max_forward_intensity);
  actionList.push_back(max_forward_move);
  
  Position expectedPosition = beliefs->getExpectedPositionAfterActions(actionList);

  cout << "Robot Circle : " << circles[robotCircle].getCenter().get_x() << " " << circles[robotCircle].getCenter().get_y() << endl;
  vector<FORRExit> ext_exits = circles[robotCircle].getExtExits();

  for(int i = 0 ; i < ext_exits.size(); i++){
    result += expectedPosition.getDistance(ext_exits[i].getExitPoint().get_x(), ext_exits[i].getExitPoint().get_y());
  }

  return result*(-1);
}

void Tier3ExitFinderRotation::set_commenting(){
  vector<FORRCircle> circles = (beliefs->abstractMap).getCircles();
  Position curr_pos = beliefs->getCurrentPosition();
  Task *task = beliefs->getCurrentTask();
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


//TODO Maybe I need sigmoid to normalize the values to the interval [-5, 5]
double Tier3CloseIn::actionComment(FORRAction action){

  // breaking FORRAction function parameter into its components
  // as defined in FORRAction.h
  FORRActionType actionType = action.type;
  int intensity = action.parameter;
  double comment_strength;

  // Here decision vector for CloseIn advisor is constructed as follows
  // first 5 components are x and y coordinate of robot position, respectively
  // followed by its yaw and x and y coordinate of target 
  double myX = (beliefs->getCurrentPosition()).getX();
  double myY = (beliefs->getCurrentPosition()).getY();
  double yaw = (beliefs->getCurrentPosition()).getTheta();
  double targetX = (beliefs->getCurrentTask())->getX();
  double targetY = (beliefs->getCurrentTask())->getY();
  double distanceToTarget = beliefs->getDistanceToTarget();
 
  // here I want to calculate the distance of robot from target
  double newTargetX = targetX - myX;
  double newTargetY =  targetY - myY; 

  //short l_sign;
  // if the target is above the robot it will be positive angle of rotation
  char sign;
  if(newTargetY > 0) 
    sign = 1;
  else
    sign = -1;
  // now we make dot product of target vector with positive part
  // of x axis, which is only x component of target coordinate
  // since x axis vector is (1, 0)
  double targetAngle = sign * acos(newTargetX / sqrt(newTargetX*newTargetX + newTargetY*newTargetY)); 
  // see which difference is less
  // the one to the right or to the left
  // it is different way of calculating things
  double angleDifference = (abs(targetAngle - yaw) > M_PI)? 2 * M_PI - (targetAngle - yaw): targetAngle - yaw; 

  double rotation; // need it to calculate comment on rotation
  switch(actionType){
  case FORWARD: 
    //l_sign = 1;
    comment_strength = 5 * (movements[intensity-1]/distanceToTarget) * cos(angleDifference);
    break;

  case BACKWARD:
    auxiliary_constants[0] = -1;
    comment_strength = 5 * (movements[intensity-1]/distanceToTarget) * cos(angleDifference);
    break;

  case RIGHT_TURN:
    // what is our rotation
    rotation = rotations[2*intensity];
    comment_strength = 5 - 5 * ((angleDifference - rotation)/angleDifference);
    break;
  case LEFT_TURN:
    rotation = rotations[2*intensity - 1];
    comment_strength = 5 - 5 * ((angleDifference - rotation)/angleDifference);
    break;
  case WIDE_RIGHT_TURN:
  case WIDE_LEFT_TURN:
    comment_strength = -5;
    break;
  default: // everything else
    comment_strength = 0;
  }

  if(comment_strength > 5)
    comment_strength = 5 - comment_strength;
  if(comment_strength < -5)
    comment_strength = -5;
  //std::cout << "In CloseIn  Action:" << actionType << " intensity:" << intensity << " => " << comment_strength << std::endl;  
  return comment_strength;
}

double Tier3CloseInRotation::actionComment(FORRAction action){

  // breaking FORRAction function parameter into its components
  // as defined in FORRAction.h
  FORRActionType actionType = action.type;
  int intensity = action.parameter;
  double comment_strength;

  // Here decision vector for CloseIn advisor is constructed as follows
  // first 5 components are x and y coordinate of robot position, respectively
  // followed by its yaw and x and y coordinate of target 

  double myX = (beliefs->getCurrentPosition()).getX();
  double myY = (beliefs->getCurrentPosition()).getY();
  double yaw = (beliefs->getCurrentPosition()).getTheta();
  double targetX = (beliefs->getCurrentTask())->getX();
  double targetY = (beliefs->getCurrentTask())->getY();
 
  // here I want to calculate the distance of robot from target
  double newTargetX = targetX - myX;
  double newTargetY =  targetY - myY; 

  //short l_sign;
  // if the target is above the robot it will be positive angle of rotation
  char sign;
  if(newTargetY > 0) 
    sign = 1;
  else
    sign = -1;

  // now we make dot product of target vector with positive part
  // of x axis, which is only x component of target coordinate
  // since x axis vector is (1, 0)
  double targetAngle = sign * acos(newTargetX / sqrt(newTargetX*newTargetX + newTargetY*newTargetY)); 
  // see which difference is less
  // the one to the right or to the left
  // it is different way of calculating things
  double angleDifference = (abs(targetAngle - yaw) > M_PI)? 2 * M_PI - (targetAngle - yaw): targetAngle - yaw; 

  double rotation; // need it to calculate comment on rotation
  switch(actionType){
   case RIGHT_TURN:
     // what is our rotation
     rotation = rotations[2*intensity];
     comment_strength = 5 - 5 * ((angleDifference - rotation)/angleDifference);
     break;
  case LEFT_TURN:
    rotation = rotations[2*intensity - 1];
    comment_strength = 5 - 5 * ((angleDifference - rotation)/angleDifference);
    break;
  case WIDE_RIGHT_TURN:
  case WIDE_LEFT_TURN:
    comment_strength = -5;
    break;
  default: // everything else
    comment_strength = 0;
  }

  if(comment_strength > 5)
    comment_strength = 5 - comment_strength;
  if(comment_strength < -5)
    comment_strength = -5;
  //std::cout << "In CloseIn  Action:" << actionType << " intensity:" << intensity << " => " << comment_strength << std::endl;  
  return comment_strength;
}

// this advisor is active only when robot is 80px away from the target
void Tier3CloseIn::set_commenting(){
  double distanceToTarget = beliefs->getDistanceToTarget();
  if(distanceToTarget > auxiliary_constants[0])
    advisor_commenting = false;
  else
    advisor_commenting = true;
}

void Tier3CloseInRotation::set_commenting(){
  double distanceToTarget = beliefs->getDistanceToTarget();
  if(distanceToTarget > auxiliary_constants[0])
    advisor_commenting = false;
  else
    advisor_commenting = true;
}

double Tier3AvoidRobot::actionComment(FORRAction action){
  double result;
  //max_len is used as range
  double range = 100;
  Position expectedPosition = beliefs->getExpectedPositionAfterAction(action);
  double sumOfTeammateDistances = beliefs->getSumOfTeammateDistances(expectedPosition, range);  
  return sumOfTeammateDistances;;
}

// On when there are more than one robots in the field
void Tier3AvoidRobot::set_commenting(){
  if((beliefs->teamPose).size() <= 1)
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
  
  Position expectedPosition = beliefs->getExpectedPositionAfterActions(actionList);
  double sumOfTeammateDistances = beliefs->getSumOfTeammateDistances(expectedPosition, range);
  return sumOfTeammateDistances;
}

void Tier3AvoidRobotRotation::set_commenting(){
  if((beliefs->teamPose).size() <= 1)
    advisor_commenting = false;
  else
    advisor_commenting = true;
}



// This puppy here will calculate its comment strength for an action
// with the respect to how close it takes agent to the obstacle
// Values for the decisionVector are calculated in FakeDescriptive, as for all advisors
// and they are the distance from the closest obstacle in the direction of possible
// movements of the robot
double Tier3AvoidObstacle::actionComment(FORRAction action){
  double result;
  // decompose FORRAction struct into its components
  FORRActionType actionType = action.type;
  int intensity = action.parameter;
  vector<double> wallDistanceVector = beliefs->wallDistanceVector;
  // set maximum distance here as default
  double obstacleDistance;
  
  // the key for producing the comment strengts is again decisionVector
  // values for this advisor are packed as follows:
  // first element is the distance from the obstacle in the current orientation of the robot
  // so it is used to calculate comment strengts for all forward movements
  // last value in the decisionVector is distance from the obstacle 180 degrees from the 
  // current orientation so it is used for all backward movements
  // in between values are packed as follows
  // in the even positions in the vector we have distance to closest obstacle
  // to the right (for right turns) and for odd indexes are distances to obstacle
  // to the left, where intensity of the rotation determines index (explained below).
  switch(actionType){
    case NOOP:
    case PAUSE:
    case HALT:
      return 0;

    case FORWARD:
      // in this case we use the first value of distance vector, which gives us
      // the distance of the obstacle that is in the direction where robot is heading.
      obstacleDistance = wallDistanceVector[0] - movements[intensity-1];
      break;

    case BACKWARD:
      // same thing as forward, only this time we need the last value in the decisionVector array
      //obstacleDistance = wallDistanceVector[11] - movements[intensity-1];
      break;

    case RIGHT_TURN:
      // again simple to produce since we have distance vector
      // right turn is decreasing the value of Theta so we take 
      // even indexed values of decisionVector
      //obstacleDistance = wallDistanceVector[intensity * 2];
      break;

    case LEFT_TURN:
      // same as LEFT_TURN but this time we take odd indexed
      // value in decisionVector
      //obstacleDistance = decisionVector[intensity * 2 - 1];
      break;

    // No calculation here so far
    case WIDE_RIGHT_TURN:
    case WIDE_LEFT_TURN:
      return -5;
  } 
  // this is Gompertz sigmoid (look up Wikipedia)
  // a = 10 which is upper asymptote (then all we have to do is subtract 5 from it to
  // produce values in the range [-5, 5])
  // b and c are negative and b = -4 which is y displacement aka how far to the left/right curve is
  // and c = -.22 is growht rate aka how steep curve is (higher c the steeper curve is)
  // here I am subtracting 45 from the obstacleDistance which should give me some buffer from the obstacle
  // NOTE: c is pretty low number, I think it should be higher, aka make steeper curve

  result = auxiliary_constants[0]*exp(auxiliary_constants[1] * exp(auxiliary_constants[2] * (obstacleDistance - 45))) - 5;
  //std::cout << "In Obstacle avoid " << actionType << " " << intensity << " D= " << obstacleDistance << " => " << result << std::endl;
  
  return result;
}

// always on
void Tier3AvoidObstacle::set_commenting(){
  advisor_commenting = true;
}

double Tier3AvoidObstacleRotation::actionComment(FORRAction action){
  double result;

  FORRActionType actionType = action.type;
  int intensity = action.parameter;
  vector<double> wallDistanceVector = beliefs->wallDistanceVector;

  double obstacleDistance;
  
  switch(actionType){
    case NOOP:
    case PAUSE:
    case HALT:
      return 0;

    case RIGHT_TURN:
      obstacleDistance = wallDistanceVector[intensity * 2];
      break;

    case LEFT_TURN:
      obstacleDistance = wallDistanceVector[intensity * 2 - 1];
      break;

    case WIDE_RIGHT_TURN:
    case WIDE_LEFT_TURN:
      return -5;
  } 

  result = auxiliary_constants[0]*exp(auxiliary_constants[1] * exp(auxiliary_constants[2] * (obstacleDistance - 45))) - 5;
  //std::cout << "In Obstacle avoid " << actionType << " " << intensity << " D= " << obstacleDistance << " => " << result << std::endl;
  
  return result;
}

void Tier3AvoidObstacleRotation::set_commenting(){
  advisor_commenting = true;
}


double Tier3Greedy::actionComment(FORRAction action){
  
  Position expectedPosition = beliefs->getExpectedPositionAfterAction(action);
  double newDistance = beliefs->getDistanceToTarget(expectedPosition);
  
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

  int max_forward_intensity = beliefs->get_maximum_intensity_following_linear_move(action);
  FORRAction max_forward_move = FORRAction(FORWARD,max_forward_intensity);
  actionList.push_back(max_forward_move);
  Position expectedPosition = beliefs->getExpectedPositionAfterActions(actionList);
  
  newDistance = beliefs->getDistanceToTarget(expectedPosition);

  return newDistance*(-1);
}

void Tier3GreedyRotation::set_commenting(){
  advisor_commenting = true;
}

// Comment strength function for BigStep advisor
double Tier3BigStep::actionComment(FORRAction action){
   
  // in this case aStep will be value from decision vector
  // which will have the same value as decision vector for
  // wallAvoid aka the length of ray from robot to the obstacle in
  // the direction of the movement. The point is that this advisor "comes for free"
  // calculations are already made.

  // this is value of the size of the step robot is making for some action
  double aStep; 
  vector<double> wallDistanceVector = beliefs->wallDistanceVector;
  // decompose action struct into its components
  FORRActionType actionType = action.type;
  int intensity = action.parameter;

  switch(actionType){
  
  case NOOP:
  case PAUSE:
  case HALT:
    return 0;
    
    // here I calculate step robot will make for given action and intensity
    // it will be either the size of that step or the distance to obstacle in
    // that direction, whichever is smaller. Values for decision vector are
    // the same as for AvoidObstacle.    
  case FORWARD:
    aStep = (wallDistanceVector[0] > movements[intensity - 1]) ? movements[intensity-1] : wallDistanceVector[0];
    break;
    
  case BACKWARD: 
    aStep = (wallDistanceVector[11] > movements[intensity - 1]) ? movements[intensity - 1] : wallDistanceVector[11];
    break;
    
    // note that I don't have -1 in index of movements, because I eliminated
    // the rotation with intensity 5 and I would like to favor other rotation
  case RIGHT_TURN:
    aStep = (wallDistanceVector[intensity * 2] > movements[4]) ? movements[4] : wallDistanceVector[intensity * 2];
    break;
    
  case LEFT_TURN:
    aStep = (wallDistanceVector[intensity*2 - 1] > movements[4]) ? movements[4] : wallDistanceVector[intensity * 2 - 1];
    break;
    
    // No calculation here so far
  case WIDE_RIGHT_TURN:
  case WIDE_LEFT_TURN:
    return -5;
  }
  // a way to favor big steps in that direction (divide step with biggest possible step
  // this means there are no negative comment strengths at the moment
  // NOTE: Do I need finer distinction? aka spread out this comment strength more.
  return 5*( aStep/ movements[4]);
}

// commenting only when robot is sufficiently far from the obstacle
void Tier3BigStep::set_commenting(){
  //double distanceToTarget = beliefs->getDistanceToTarget();
  //if(distanceToTarget > auxiliary_constants[0])
  advisor_commenting = true;
  //else
  //advisor_commenting = false;
}

double Tier3BigStepRotation::actionComment(FORRAction action){
   
  double aStep; 

  FORRActionType actionType = action.type;
  int intensity = action.parameter;
  vector<double> wallDistanceVector = beliefs->wallDistanceVector;
  switch(actionType){
  
  case NOOP:
  case PAUSE:
  case HALT:
    return 0;
    
  case RIGHT_TURN:
    aStep = (wallDistanceVector[intensity * 2] > movements[4]) ? movements[4] : wallDistanceVector[intensity * 2];
    break;
    
  case LEFT_TURN:
    aStep = (wallDistanceVector[intensity*2 - 1] > movements[4]) ? movements[4] : wallDistanceVector[intensity * 2 - 1];
    break;
    
  case WIDE_RIGHT_TURN:
  case WIDE_LEFT_TURN:
    return -5;
  }

  return 5*( aStep/ movements[4]);
}

void Tier3BigStepRotation::set_commenting(){
  //double distanceToTarget = beliefs->getDistanceToTarget();
  //if(distanceToTarget > auxiliary_constants[0])
  advisor_commenting = true;
  //else
  //advisor_commenting = false;
}

double Tier3GoAroundRotation::actionComment(FORRAction action){
  // break up action into its components
  FORRActionType actionType = action.type;
  int intensity = action.parameter;
  vector<double> wallDistanceVector = beliefs->wallDistanceVector;
  beliefs->turn_to_side();

  double comment_strength; // this is value advisor will return

  switch(actionType){
  
  case NOOP:
  case PAUSE:
  case HALT:
    comment_strength = 0;
    break;
    /*
     * The only rotation that does not follow the pattern is the one with intensity 5.
     * Have to decide what to do with it.
      */
  case RIGHT_TURN:
    if(beliefs->m_side == 'r'){
      if(wallDistanceVector[0] > auxiliary_constants[0])
	comment_strength = 5 - abs(intensity - 1);
      else if(wallDistanceVector[0] > auxiliary_constants[1])
	comment_strength = 5 - abs(intensity - 2);
      else if(wallDistanceVector[0] > auxiliary_constants[2])
	comment_strength = 5 - abs(intensity - 3);
      else 
	comment_strength = 5 - abs(intensity - 4);
    }
    else
      comment_strength = -2;
    break;
  case LEFT_TURN:
    if(beliefs->m_side != 'r' && beliefs->m_side != 'n'){
      if(wallDistanceVector[0] > auxiliary_constants[0])
	comment_strength = 1 - abs(intensity - 1);
      else if(wallDistanceVector[0] > auxiliary_constants[1])
	comment_strength = 5 - abs(intensity - 2);
      else if(wallDistanceVector[0] > auxiliary_constants[2])
	comment_strength = 5 - abs(intensity - 3);
      else 
	comment_strength = 5 - abs(intensity - 4);
    }
    else
      comment_strength = -2;
    break;
  case WIDE_RIGHT_TURN:
  case WIDE_LEFT_TURN:
    comment_strength = -5;
  }
  
  return comment_strength;
}
  
void Tier3GoAroundRotation::set_commenting(){
  beliefs->turn_to_side();
  if(beliefs->m_side == 'n')
    advisor_commenting = false;
  else
    advisor_commenting = true;
}

// always on
void Tier3AvoidLeaf::set_commenting(){
  //cout << "In avoid leaf set commenting " << endl;
  vector<FORRCircle> circles = (beliefs->abstractMap).getCircles();
  Position curr_pos = beliefs->getCurrentPosition();
  Task *task = beliefs->getCurrentTask();
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
  vector<FORRCircle> circles = (beliefs->abstractMap).getCircles();
  int robotCircle=-1,targetCircle=-1;
  Position curr_pos = beliefs->getCurrentPosition();
  Task *task = beliefs->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());

  Position expectedPosition = beliefs->getExpectedPositionAfterAction(action);

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
    if(beliefs->abstractMap.isLeaf(nearCircles[i]) and !(nearCircles[i].inCircle(targetPoint.get_x(), targetPoint.get_y()))){
      //cout << "Avoid Leaf: Found deadend !" << endl;
      metric += pow(expectedPosition.getDistance(nearCircles[i].getCenter().get_x(), nearCircles[i].getCenter().get_y()), 2);
    }
  }

  return metric;
}

void Tier3AvoidLeafRotation::set_commenting(){
  //cout << "In region finder rotation set commenting " << endl;
   vector<FORRCircle> circles = (beliefs->abstractMap).getCircles();
   Position curr_pos = beliefs->getCurrentPosition();
   Task *task = beliefs->getCurrentTask();
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
  vector<FORRCircle> circles = (beliefs->abstractMap).getCircles();
  int robotCircle=-1,targetCircle=-1;
  Position curr_pos = beliefs->getCurrentPosition();
  Task *task = beliefs->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());

  vector<FORRAction> actionList;
  actionList.push_back(action);
  
  int max_forward_intensity = beliefs->get_maximum_intensity_following_linear_move(action);
  FORRAction max_forward_move = FORRAction(FORWARD,max_forward_intensity);
  actionList.push_back(max_forward_move);

  Position expectedPosition = beliefs->getExpectedPositionAfterActions(actionList);

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
    if(beliefs->abstractMap.isLeaf(nearCircles[i]) and !(nearCircles[i].inCircle(targetPoint.get_x(), targetPoint.get_y()))){
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
  // break up action into its components
  FORRActionType actionType = action.type;
  int intensity = action.parameter;
  vector<Position> *positionHis = beliefs->positionHistory;
  double comment_strength; // this is value advisor will return
  Position expectedPosition = beliefs->getExpectedPositionAfterAction(action);
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
 
  FORRActionType actionType = action.type;
  int intensity = action.parameter;

  double comment_strength; // this is value advisor will return
  //vector<double> targetDistanceVector = beliefs->targetDistanceVector;
  vector<FORRAction> actionList;
  actionList.push_back(action);
  
  int max_forward_intensity = beliefs->get_maximum_intensity_following_linear_move(action);
  FORRAction max_forward_move = FORRAction(FORWARD,max_forward_intensity);
  actionList.push_back(max_forward_move);
  Position expectedPosition = beliefs->getExpectedPositionAfterActions(actionList);

  vector<Position> *positionHis = beliefs->positionHistory;
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


//votes strongly for moves that take it either to a gate that is connected to the target quadrant
//or to a cluster of gates 
double Tier3GateFinderLinear::actionComment(FORRAction action){
  vector<double> wallDistanceVector = beliefs->wallDistanceVector;
  double comment_strength;
  CartesianPoint gate_position = beliefs->get_target_gate(wallDistanceVector[0]);
  double distance_to_gate, new_distance;
  //if a gate exists
   
  
  FORRActionType actionType = action.type;
  int intensity = action.parameter;

  Position expectedPosition = beliefs->getExpectedPositionAfterAction(action);
  distance_to_gate = Utils::get_euclidian_distance(gate_position.get_x(), gate_position.get_y(),
							    beliefs->getCurrentPosition().getX(), beliefs->getCurrentPosition().getY());
  if(gate_position.get_x()!= -1){

    //add 100 to make it stronger than the second case
    comment_strength = 1 / Utils::get_euclidian_distance(expectedPosition.getX(), expectedPosition.getX(), 
							 beliefs->getCurrentTask()->getX(),beliefs->getCurrentTask()->getY());
 
  }

  else comment_strength = 0;

   
  //if(action.parameter < 4) comment_strength = 0; // prevent small moves

  return  comment_strength;
}
void Tier3GateFinderLinear::set_commenting(){
  int mapheight = beliefs->getMap()->getHeight();
  int maplength = beliefs->getMap()->getLength();

  int curr_region = beliefs->gates.region(beliefs->getCurrentPosition().getX(), beliefs->getCurrentPosition().getY(),
					  maplength, mapheight);
  int target_region = beliefs->gates.region(beliefs->getCurrentTask()->getX(), beliefs->getCurrentTask()->getY(),
					    maplength, mapheight);
  if(curr_region == target_region) advisor_commenting = false;
  else advisor_commenting = true;

}


double Tier3GateFinderRotation::actionComment(FORRAction action){
  vector<double> wallDistanceVector = beliefs->wallDistanceVector;
  double comment_strength;
  //if a gate exists 
  vector<FORRAction> actionList;
  actionList.push_back(action);

  int max_forward_intensity = beliefs->get_maximum_intensity_following_linear_move(action);
  FORRAction max_forward_move = FORRAction(FORWARD,max_forward_intensity);
  double distance_to_gate;
  actionList.push_back(max_forward_move);
  
  
  //cout << " ------------ !!!! IN GATE FINDER ROTATION !!!! ------------" <<endl;
  
  FORRActionType actionType = action.type;
  int intensity = action.parameter;
  
  Position expectedPosition = beliefs->getExpectedPositionAfterActions(actionList);
  
  double wall_distance = 0;
  
  //find the corresponding forward wall distance projection for each rotation 
  if(action.type == 3){
    if(action.parameter == 1)
      wall_distance = wallDistanceVector[1];
    else if(action.parameter == 2)
      wall_distance = wallDistanceVector[3];
    else if(action.parameter == 3)
      wall_distance = wallDistanceVector[5];
    else if(action.parameter == 4)
      wall_distance = wallDistanceVector[7];
    else if(action.parameter == 5)
      wall_distance = wallDistanceVector[9];
    
  }
  else if(action.type == 4){
    if(action.parameter == 1)
      wall_distance = wallDistanceVector[2];
    else if(action.parameter == 2)
      wall_distance = wallDistanceVector[4];
    else if(action.parameter == 3)
      wall_distance = wallDistanceVector[6];
    else if(action.parameter == 4)
      wall_distance = wallDistanceVector[8];
    else if(action.parameter == 5)
      wall_distance = wallDistanceVector[10];
  }
  CartesianPoint gate_position = beliefs->get_target_gate(wall_distance);
  distance_to_gate = Utils::get_euclidian_distance(gate_position.get_x(), gate_position.get_y(),
						    beliefs->getCurrentPosition().getX(), beliefs->getCurrentPosition().getY());
   
   if((gate_position.get_x()!= -1) && (distance_to_gate < wall_distance)){
    comment_strength = 1 / Utils::get_euclidian_distance(expectedPosition.getX(), expectedPosition.getX(), 
							 beliefs->getCurrentTask()->getX(),beliefs->getCurrentTask()->getY());
   }
   else comment_strength = 0;

  
  return comment_strength;
}

void Tier3GateFinderRotation::set_commenting(){
  
  int mapheight = beliefs->getMap()->getHeight();
  int maplength = beliefs->getMap()->getLength();

  int curr_region = beliefs->gates.region(beliefs->getCurrentPosition().getX(), beliefs->getCurrentPosition().getY(),
					  maplength, mapheight);
  int target_region = beliefs->gates.region(beliefs->getCurrentTask()->getX(), beliefs->getCurrentTask()->getY(),
					    maplength, mapheight);
  if(curr_region == target_region) advisor_commenting = false;
  else advisor_commenting = true;

}



//evanusa note
//votes to move from one end of the pipe to the other.  
//NOTE: TO FIX: Unlike get_target_gate, get_nearby_gate returns the Gate itself not a cartesianpoint
double Tier3PortalHopperLinear::actionComment(FORRAction action){
  
  FORRActionType actionType = action.type;
  int intensity = action.parameter;
  double comment_strength;
  double distance_to_other_end, new_distance = 0;
  
  
  CartesianPoint nearby_gate_exit = beliefs->get_nearby_gate_exit();
  if((nearby_gate_exit.get_x() != -1) && (!beliefs->get_moved_across_pipe())){
    distance_to_other_end = Utils::get_euclidian_distance(beliefs->getCurrentPosition().getX(), beliefs->getCurrentPosition().getY(),
							  nearby_gate_exit.get_x(), nearby_gate_exit.get_y());
    Position expectedPosition = beliefs->getExpectedPositionAfterAction(action);
    new_distance = Utils::get_euclidian_distance(expectedPosition.getX(), expectedPosition.getY(), nearby_gate_exit.get_x(),
						 nearby_gate_exit.get_y());
    beliefs->set_moved_across_pipe(true);
    return new_distance * (-1);
    //cout << "-----IN PORTAL HOPPER LINEAR, NEAR GATE! comment strength: " << comment_strength << endl;
  }
  else{
    comment_strength = 0;
    //reset flag, didn't move through pipe this turn
    //cout << "------IN PORTAL HOPPER LINEAR, NOT NEAR GATE. ----- " << endl;
    beliefs->set_moved_across_pipe(false);
  }

  return comment_strength;

}


//only comment if the robot is not in the target quadrant
void Tier3PortalHopperLinear::set_commenting(){
  int length = beliefs->getMap()->getLength();
  int height = beliefs->getMap()->getHeight();
  int target_region = beliefs->gates.region(beliefs->getCurrentTask()->getX(),beliefs->getCurrentTask()->getY(),
					    length, height);

  int curr_region = beliefs->gates.region(beliefs->getCurrentPosition().getX(), beliefs->getCurrentPosition().getY(),
					  length, height);

  if(target_region == curr_region)
    advisor_commenting = false;
  else
    advisor_commenting = true;

}


double Tier3PortalHopperRotation::actionComment(FORRAction action){
  
  FORRActionType actionType = action.type;
  int intensity = action.parameter;
  double comment_strength;
  double distance_to_other_end, new_distance = 0;
  vector<FORRAction> actionList;
  actionList.push_back(action);
  FORRAction max_forward_move = FORRAction(FORWARD,5);
  actionList.push_back(max_forward_move);
  
  
  CartesianPoint nearby_gate_exit = beliefs->get_nearby_gate_exit();
  if((nearby_gate_exit.get_x() != -1)){
    distance_to_other_end = Utils::get_euclidian_distance(beliefs->getCurrentPosition().getX(), beliefs->getCurrentPosition().getY(),
							  nearby_gate_exit.get_x(), nearby_gate_exit.get_y());
    Position expectedPosition = beliefs->getExpectedPositionAfterActions(actionList);
    new_distance = Utils::get_euclidian_distance(expectedPosition.getX(), expectedPosition.getY(), nearby_gate_exit.get_x(),
						 nearby_gate_exit.get_y());
    
    if(distance_to_other_end != 0)
      comment_strength = (5 * ((distance_to_other_end - new_distance) / distance_to_other_end));
    else comment_strength = 0;

    //set the flag to true, so as to not move along the pipe ain
    // beliefs->set_moved_across_pipe(true);

    //cout << "-----IN PORTAL HOPPER ROTATION, NEAR GATE! comment strength: " << comment_strength << endl;
    //cout << "distance to other end: " << distance_to_other_end << endl;
    //cout << "new distance: " << new_distance << endl;
  }
  else{
    comment_strength = 0;
    //reset flag, didn't move through pipe this turn
    // beliefs->set_moved_across_pipe(false);
  }
  if(comment_strength > 5) comment_strength = 5;
  if(comment_strength < -5) comment_strength = -5;
  return comment_strength;

}

//only comment if the robot is not in the target quadrant
void Tier3PortalHopperRotation::set_commenting(){
  int length = beliefs->getMap()->getLength();
  int height = beliefs->getMap()->getHeight();
  int target_region = beliefs->gates.region(beliefs->getCurrentTask()->getX(),beliefs->getCurrentTask()->getY(),
					    length, height);

  int curr_region = beliefs->gates.region(beliefs->getCurrentPosition().getX(), beliefs->getCurrentPosition().getY(),
					  length, height);

  if(target_region == curr_region)
    advisor_commenting = false;
  else
    advisor_commenting = true;

}



//votes for moves that will take it to an adjacent cell block, given by the overlay grid, 
//that has the most points in it.  
//The metric is the distance to the expected point times the grid value, because we want to pull
//the robot further distances, but at the same time want it to go to high-value grid cells
double Tier3WaypointFinderLinear::actionComment(FORRAction action){
 
  //cout << "Entered waypointfinder linear."<<endl;
  Position expectedPosition = beliefs->getExpectedPositionAfterAction(action);
  
  int grid_value = beliefs->Waypoints.getGridValue(expectedPosition.getX(), expectedPosition.getY());
  
  double distance = Utils::get_euclidian_distance(beliefs->getCurrentPosition().getX(), beliefs->getCurrentPosition().getY(),
						  expectedPosition.getX(), expectedPosition.getY());
  
  //cout <<"Exit waypoint linear."<<endl;
  return distance * grid_value; //want larger grid values that are further away
}


void Tier3WaypointFinderLinear::set_commenting(){
  advisor_commenting = true;


}

double Tier3WaypointFinderRotation::actionComment(FORRAction action){
  vector<double> wallDistanceVector = beliefs->wallDistanceVector;
  //cout <<" Entered waypointfinder rotation." << endl;
  vector<FORRAction> actionList;
  actionList.push_back(action);
  int max_forward_intensity = beliefs->get_maximum_intensity_following_linear_move(action);
  FORRAction max_forward_move = FORRAction(FORWARD,max_forward_intensity);
  actionList.push_back(max_forward_move);
  Position expectedPosition = beliefs->getExpectedPositionAfterActions(actionList);
  
  //cout << "before grid value."<<endl;
  //cout << "Expected position x: "<<expectedPosition.getX()<<", Expected position y: "<<expectedPosition.getY();
 
  if((expectedPosition.getX() < 0) || (expectedPosition.getX() > beliefs->getMap()->getLength()) ||
     (expectedPosition.getY() < 0) || (expectedPosition.getY() > beliefs->getMap()->getHeight())){
    return -10000.0;
    
  }
  

  int grid_value = beliefs->Waypoints.getGridValue(expectedPosition.getX(), expectedPosition.getY());
  //cout << "grid value: "<<grid_value<<endl;
  double distance = Utils::get_euclidian_distance(beliefs->getCurrentPosition().getX(), beliefs->getCurrentPosition().getY(),
						  expectedPosition.getX(), expectedPosition.getY());
  
  

  //cout << "Exit waypoint rotation."<<endl;
  
  //cout << "Wall distance: "<<wall_distance<<endl;
  //cout << "Distance after rotation: "<<distance<<endl;
  
  return distance * grid_value;
 
}

void Tier3WaypointFinderRotation::set_commenting(){
  advisor_commenting = true;
}




double Tier3TrailFinderLinear::actionComment(FORRAction action){
   vector<double> wallDistance = beliefs->wallDistanceVector;
   CartesianPoint curr = CartesianPoint(beliefs->getCurrentPosition().getX(), beliefs->getCurrentPosition().getY());
   CartesianPoint target_trailmarker = beliefs->trail_vectors.getFurthestVisiblePointOnChosenTrail(curr, wallDistance);
   //if out of line of sight of trail marker, vote the same value functionally turning off voting
   if(!beliefs->trail_vectors.canSeeTrail()){
     return 0;
   }

   
  Position expectedPosition = beliefs->getExpectedPositionAfterAction(action);
  //double newDistance = beliefs->getDistanceToTarget(expectedPosition);
  double newDistance = Utils::get_euclidian_distance(expectedPosition.getX(), expectedPosition.getY(),
						  curr.get_x(), curr.get_y());
  
  
  return newDistance *(-1);
}

//only turn on if a trail is active
void Tier3TrailFinderLinear::set_commenting(){
  if(beliefs->trail_vectors.getChosenTrail() == -1){
    advisor_commenting = false;
      }
  else{
      advisor_commenting = true;
  }
}

double Tier3TrailFinderRotation::actionComment(FORRAction action){
  vector<double> wallDistance = beliefs->wallDistanceVector;

  CartesianPoint curr = CartesianPoint(beliefs->getCurrentPosition().getX(), beliefs->getCurrentPosition().getY());
  CartesianPoint target_trailmarker = beliefs->trail_vectors.getFurthestVisiblePointOnChosenTrail(curr, wallDistance);
   //if out of line of sight of trail marker, vote the same value in effect turning off voting
  //can_see_trail is set in getFurthestVisiblePointOnChosenTrail
   if(!beliefs->trail_vectors.canSeeTrail()){
     return 0;
   }


  vector<FORRAction> actionList;
  actionList.push_back(action);
  int max_forward_intensity = beliefs->get_maximum_intensity_following_linear_move(action);
  FORRAction max_forward_move = FORRAction(FORWARD,max_forward_intensity);

  Position expectedPosition = beliefs->getExpectedPositionAfterActions(actionList);

  double newDistance = Utils::get_euclidian_distance(expectedPosition.getX(), expectedPosition.getY(),
						  curr.get_x(), curr.get_y());
  
  
  return newDistance *(-1);

 
}


//only turn on if a trail is active
void Tier3TrailFinderRotation::set_commenting(){
  if(beliefs->trail_vectors.getChosenTrail() == -1){
    advisor_commenting = false;
      }
  else{
    advisor_commenting = true;
  }
}
