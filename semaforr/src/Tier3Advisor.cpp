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
# include <cstdlib>
# include <time.h>
# include <utility>
# include <limits>
# include <sensor_msgs/LaserScan.h>

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

  bool inRotateMode = beliefs->getAgentState()->getRotateMode();
  cout << "Decision Count : " << beliefs->getAgentState()->getCurrentTask()->getDecisionCount() << endl;
  
  cout << "Rotation mode : " << inRotateMode << endl;
  /*action_set = beliefs->getAgentState()->getActionSet();
  inRotateMode = true;*/
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
  normalize(&result);
  //rank(&result);
  //standardize(&result);
  //}
  return result;
}


//normalizing from 0 to 10
void Tier3Advisor::
normalize(map <FORRAction, double> * result){
  double max = -std::numeric_limits<double>::infinity(), min = std::numeric_limits<double>::infinity();
  map<FORRAction, double>::iterator itr;
  for(itr = result->begin(); itr != result->end() ; itr++){
    if(max < itr->second)  max = itr->second;
    if(min > itr->second)  min = itr->second;
  } 
  std::cout << "Inside normalize " << max << " " << min << endl;
  if(max != min and result->size() > 1){
    double norm_factor = (max - min)/10;
    for(itr = result->begin(); itr != result->end() ; itr++){
      cout << "Before : " << itr->second << endl;
      itr->second = (itr->second - min)/norm_factor;
    }
  } else {
    result->begin()->second = 0;
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

//standardize the comments by converting to a z-score
void Tier3Advisor::
standardize(map <FORRAction, double> * result){
  double mean = 0, count = 0, stdDev = 0;
  map<FORRAction, double>::iterator itr;
  for(itr = result->begin(); itr != result->end() ; itr++){
    mean += itr->second;
    count++;
  }
  mean = (mean / count);

  for(itr = result->begin(); itr != result->end() ; itr++){
    stdDev += pow((itr->second - mean), 2);
  }
  stdDev = sqrt(stdDev / count);
  if(stdDev != 0) {
    for(itr = result->begin(); itr != result->end() ; itr++){
      cout << "Before : " << itr->second << endl;
      itr->second = ((itr->second - mean)/stdDev);
    }
  }
  else {
    for(itr = result->begin(); itr != result->end() ; itr++){
      cout << "Before : " << itr->second << endl;
      itr->second = 0;
    }
  }
}

// factory definition
Tier3Advisor* Tier3Advisor::makeAdvisor(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active){
  if(name == "Greedy")
    return new Tier3Greedy(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "ElbowRoom")
    return new Tier3ElbowRoom(beliefs, name, description, weight, magic_init, is_active);
  //else if(name == "CloseIn")
    //return new Tier3CloseIn(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "BigStep")
    return new Tier3BigStep(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "Unlikely")
    return new Tier3Unlikely(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "UnlikelyRotation")
    return new Tier3UnlikelyRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "UnlikelyField")
    return new Tier3UnlikelyField(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "UnlikelyFieldRotation")
    return new Tier3UnlikelyFieldRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "Explorer")
    return new Tier3Explorer(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "ExplorerEndPoints")
    return new Tier3ExplorerEndPoints(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "BaseLine")
    return new Tier3BaseLine(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "GreedyRotation")
    return new Tier3GreedyRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "ElbowRoomRotation")
    return new Tier3ElbowRoomRotation(beliefs, name, description, weight, magic_init, is_active);
  //else if(name == "CloseInRotation")
    //return new Tier3CloseInRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "BigStepRotation")
    return new Tier3BigStepRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "GoAroundRotation")
    return new Tier3GoAroundRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "ExplorerRotation")
    return new Tier3ExplorerRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "ExplorerEndPointsRotation")
    return new Tier3ExplorerEndPointsRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "BaseLineRotation")
    return new Tier3BaseLineRotation(beliefs, name, description, weight, magic_init, is_active);
  //else if(name == "AvoidRobotRotation")
    //return new Tier3AvoidRobotRotation(beliefs, name, description, weight, magic_init, is_active);
  //else if(name == "AvoidRobot")
    //return new Tier3AvoidRobot(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "ExitLinear")
    return new Tier3ExitLinear(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "ExitRotation")
    return new Tier3ExitRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "ExitFieldLinear")
    return new Tier3ExitFieldLinear(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "ExitFieldRotation")
    return new Tier3ExitFieldRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "ExitClosest")
    return new Tier3ExitClosest(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "ExitClosestRotation")
    return new Tier3ExitClosestRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "RegionLeaverLinear")
    return new Tier3RegionLeaverLinear(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "RegionLeaverRotation")
    return new Tier3RegionLeaverRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "EnterLinear")
    return new Tier3EnterLinear(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "EnterRotation")
    return new Tier3EnterRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "EnterExit")
    return new Tier3EnterExit(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "EnterExitRotation")
    return new Tier3EnterExitRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "ConveyLinear")
    return new Tier3ConveyLinear(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "ConveyRotation")
    return new Tier3ConveyRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "TrailerLinear")
    return new Tier3TrailerLinear(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "TrailerRotation")
    return new Tier3TrailerRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "EnterDoorLinear")
    return new Tier3EnterDoorLinear(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "EnterDoorRotation")
    return new Tier3EnterDoorRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "ExitDoorLinear")
    return new Tier3ExitDoorLinear(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "ExitDoorRotation")
    return new Tier3ExitDoorRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "AccessLinear")
    return new Tier3AccessLinear(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "AccessRotation")
    return new Tier3AccessRotation(beliefs, name, description, weight, magic_init, is_active);
  //else if(name == "NeighborDoorLinear")
    //return new Tier3NeighborDoorLinear(beliefs, name, description, weight, magic_init, is_active);
  //else if(name == "NeighborDoorRotation")
    //return new Tier3NeighborDoorRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "LearnSpatialModel")
    return new Tier3LearnSpatialModel(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "LearnSpatialModelRotation")
    return new Tier3LearnSpatialModelRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "ActiveLearner")
    return new Tier3ActiveLearner(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "ActiveLearnerRotation")
    return new Tier3ActiveLearnerRotation(beliefs, name, description, weight, magic_init, is_active);
  else 
    std::cout << "No such advisor " << std::endl;
}

// Constructors 
Tier3Greedy::Tier3Greedy (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3ElbowRoom::Tier3ElbowRoom(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
//Tier3CloseIn::Tier3CloseIn(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3BigStep::Tier3BigStep(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};  
Tier3Unlikely::Tier3Unlikely(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3UnlikelyField::Tier3UnlikelyField(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3Explorer::Tier3Explorer(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3ExplorerEndPoints::Tier3ExplorerEndPoints(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3BaseLine::Tier3BaseLine (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
//Tier3AvoidRobot::Tier3AvoidRobot (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 

Tier3GreedyRotation::Tier3GreedyRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3ElbowRoomRotation::Tier3ElbowRoomRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
//Tier3CloseInRotation::Tier3CloseInRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3BigStepRotation::Tier3BigStepRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3GoAroundRotation::Tier3GoAroundRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3UnlikelyRotation::Tier3UnlikelyRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3UnlikelyFieldRotation::Tier3UnlikelyFieldRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3ExplorerRotation::Tier3ExplorerRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3ExplorerEndPointsRotation::Tier3ExplorerEndPointsRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3BaseLineRotation::Tier3BaseLineRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
//Tier3AvoidRobotRotation::Tier3AvoidRobotRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3ExitLinear::Tier3ExitLinear (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3ExitRotation::Tier3ExitRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3ExitFieldLinear::Tier3ExitFieldLinear (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3ExitFieldRotation::Tier3ExitFieldRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3ExitClosest::Tier3ExitClosest (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3ExitClosestRotation::Tier3ExitClosestRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3RegionLeaverLinear::Tier3RegionLeaverLinear (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3RegionLeaverRotation::Tier3RegionLeaverRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3EnterLinear::Tier3EnterLinear (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3EnterRotation::Tier3EnterRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3EnterExit::Tier3EnterExit (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3EnterExitRotation::Tier3EnterExitRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3ConveyLinear::Tier3ConveyLinear (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3ConveyRotation::Tier3ConveyRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3TrailerLinear::Tier3TrailerLinear (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3TrailerRotation::Tier3TrailerRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3EnterDoorLinear::Tier3EnterDoorLinear (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3EnterDoorRotation::Tier3EnterDoorRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3ExitDoorLinear::Tier3ExitDoorLinear (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3ExitDoorRotation::Tier3ExitDoorRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3AccessLinear::Tier3AccessLinear (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3AccessRotation::Tier3AccessRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
//Tier3NeighborDoorLinear::Tier3NeighborDoorLinear (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
//Tier3NeighborDoorRotation::Tier3NeighborDoorRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3LearnSpatialModel::Tier3LearnSpatialModel (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3LearnSpatialModelRotation::Tier3LearnSpatialModelRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3ActiveLearner::Tier3ActiveLearner (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3ActiveLearnerRotation::Tier3ActiveLearnerRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
 


// Default dummy constructors
Tier3Greedy::Tier3Greedy(): Tier3Advisor() {};
Tier3ElbowRoom::Tier3ElbowRoom(): Tier3Advisor() {};
//Tier3CloseIn::Tier3CloseIn(): Tier3Advisor() {};
Tier3BigStep::Tier3BigStep(): Tier3Advisor() {};
Tier3Unlikely::Tier3Unlikely(): Tier3Advisor() {};
Tier3UnlikelyField::Tier3UnlikelyField(): Tier3Advisor() {};
Tier3Explorer::Tier3Explorer(): Tier3Advisor() {};
Tier3ExplorerEndPoints::Tier3ExplorerEndPoints(): Tier3Advisor() {};
//Tier3AvoidRobot::Tier3AvoidRobot(): Tier3Advisor() {};
Tier3BaseLine::Tier3BaseLine(): Tier3Advisor() {};

Tier3GreedyRotation::Tier3GreedyRotation(): Tier3Advisor() {};
Tier3ElbowRoomRotation::Tier3ElbowRoomRotation(): Tier3Advisor() {};
//Tier3CloseInRotation::Tier3CloseInRotation(): Tier3Advisor() {};
Tier3BigStepRotation::Tier3BigStepRotation(): Tier3Advisor() {};
Tier3GoAroundRotation::Tier3GoAroundRotation(): Tier3Advisor() {};
Tier3UnlikelyRotation::Tier3UnlikelyRotation(): Tier3Advisor() {};
Tier3UnlikelyFieldRotation::Tier3UnlikelyFieldRotation(): Tier3Advisor() {};
Tier3ExplorerEndPointsRotation::Tier3ExplorerEndPointsRotation(): Tier3Advisor() {};
//Tier3AvoidRobotRotation::Tier3AvoidRobotRotation(): Tier3Advisor() {};
Tier3BaseLineRotation::Tier3BaseLineRotation(): Tier3Advisor() {};

Tier3ExitLinear::Tier3ExitLinear(): Tier3Advisor() {};
Tier3ExitRotation::Tier3ExitRotation(): Tier3Advisor() {};
Tier3ExitFieldLinear::Tier3ExitFieldLinear(): Tier3Advisor() {};
Tier3ExitFieldRotation::Tier3ExitFieldRotation(): Tier3Advisor() {};
Tier3ExitClosest::Tier3ExitClosest(): Tier3Advisor() {};
Tier3ExitClosestRotation::Tier3ExitClosestRotation(): Tier3Advisor() {};
Tier3RegionLeaverLinear::Tier3RegionLeaverLinear(): Tier3Advisor() {};
Tier3RegionLeaverRotation::Tier3RegionLeaverRotation(): Tier3Advisor() {};
Tier3EnterLinear::Tier3EnterLinear(): Tier3Advisor() {};
Tier3EnterRotation::Tier3EnterRotation(): Tier3Advisor() {};
Tier3EnterExit::Tier3EnterExit(): Tier3Advisor() {};
Tier3EnterExitRotation::Tier3EnterExitRotation(): Tier3Advisor() {};
Tier3ConveyLinear::Tier3ConveyLinear(): Tier3Advisor() {};
Tier3ConveyRotation::Tier3ConveyRotation(): Tier3Advisor() {};
Tier3TrailerLinear::Tier3TrailerLinear(): Tier3Advisor() {};
Tier3TrailerRotation::Tier3TrailerRotation(): Tier3Advisor() {};
Tier3EnterDoorLinear::Tier3EnterDoorLinear(): Tier3Advisor() {};
Tier3EnterDoorRotation::Tier3EnterDoorRotation(): Tier3Advisor() {};
Tier3ExitDoorLinear::Tier3ExitDoorLinear(): Tier3Advisor() {};
Tier3ExitDoorRotation::Tier3ExitDoorRotation(): Tier3Advisor() {};
Tier3AccessLinear::Tier3AccessLinear(): Tier3Advisor() {};
Tier3AccessRotation::Tier3AccessRotation(): Tier3Advisor() {};
//Tier3NeighborDoorLinear::Tier3NeighborDoorLinear(): Tier3Advisor() {};
//Tier3NeighborDoorRotation::Tier3NeighborDoorRotation(): Tier3Advisor() {};
Tier3LearnSpatialModel::Tier3LearnSpatialModel(): Tier3Advisor() {};
Tier3LearnSpatialModelRotation::Tier3LearnSpatialModelRotation(): Tier3Advisor() {};
Tier3ActiveLearner::Tier3ActiveLearner(): Tier3Advisor() {};
Tier3ActiveLearnerRotation::Tier3ActiveLearnerRotation(): Tier3Advisor() {};


// vote to go through an extrance to a region containing the target

double Tier3EnterLinear::actionComment(FORRAction action){
  cout << "In enter linear " << endl;
  double result;
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  int robotRegion=-1,targetRegion=-1;
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());

  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetRegion = i;
    }
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY())){
      robotRegion = i;
    }
  }

  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  
  cout << "Robot Region : " << regions[robotRegion].getCenter().get_x() << " " << regions[robotRegion].getCenter().get_y() << endl;
    
  vector<FORRExit> exits = regions[robotRegion].getExits();

  vector<FORRRegion> nearRegions;
  for(int i = 0; i < exits.size() ; i++){
    FORRRegion test = regions[exits[i].getExitRegion()];
    std::vector<FORRRegion>::iterator it = std::find(nearRegions.begin(),nearRegions.end(), test);
    if(nearRegions.empty() or (it == nearRegions.end())){
      nearRegions.push_back(test);
      cout << "Neighbour Region : " << test.getCenter().get_x() << " " << test.getCenter().get_y() << endl;
    }
  }

  cout << "#Neighbors found :" << nearRegions.size() << endl;

  double metric = 0;
  for(int i = 0; i < nearRegions.size(); i++){
    if(nearRegions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      cout << "Enter: Found Target Region !" << endl;
      metric += abs((expectedPosition.getDistance(nearRegions[i].getCenter().get_x(), nearRegions[i].getCenter().get_y())) - nearRegions[i].getRadius());
    }
  }
  return metric * (-1);
}


void Tier3EnterLinear::set_commenting(){

  cout << "In enter linear set commenting " << endl;
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  bool targetInRegion = false;
  bool currPosInRegionWithExit = false;
  int robotRegion=-1, targetRegion = -1;
  
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetInRegion = true;
      targetRegion = i;
    }
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY()) and ((regions[i]).getExits().size() >= 1)){
      currPosInRegionWithExit = true;
      robotRegion = i;
    }
  }

  if(targetInRegion == true and currPosInRegionWithExit == true and robotRegion != targetRegion)
    advisor_commenting = true;
  else
    advisor_commenting = false;
}


double Tier3EnterRotation::actionComment(FORRAction action){

  cout << "In enter rotation " << endl;

  double result;
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  int robotRegion = -1,targetRegion = -1;
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  bool targetInRegion = false;
  bool currPosInRegionWithExit = false;

  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY()) and ((regions[i]).getExits().size() >= 1)){
      currPosInRegionWithExit = true;
      robotRegion = i;
    }
  }

 
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  
  vector<FORRExit> exits = regions[robotRegion].getExits();
  
  vector<FORRRegion> nearRegions;
  for(int i = 0; i < exits.size() ; i++){
    FORRRegion test = regions[exits[i].getExitRegion()];
    std::vector<FORRRegion>::iterator it = std::find(nearRegions.begin(),nearRegions.end(), test);
    if(nearRegions.empty() or (it == nearRegions.end())){
      nearRegions.push_back(test);
    }
  }

  double metric = 0;
  for(int i = 0; i < nearRegions.size(); i++){
    if(nearRegions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      cout << "EnterRotation: Found Target Region !" << endl;
      metric += abs((expectedPosition.getDistance(nearRegions[i].getCenter().get_x(), nearRegions[i].getCenter().get_y())) - nearRegions[i].getRadius());
    }
  }

  return metric * (-1);
}

void Tier3EnterRotation::set_commenting(){

  cout << "In enter rotation set commenting " << endl;
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  bool targetInRegion = false;
  bool currPosInRegionWithExit = false;
  int robotRegion=-1, targetRegion = -1;
  
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetInRegion = true;
      targetRegion = i;
    }
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY()) and ((regions[i]).getExits().size() >= 1)){
      currPosInRegionWithExit = true;
      robotRegion = i;
    }
  }

  if(targetInRegion == true and currPosInRegionWithExit == true and robotRegion != targetRegion)
    advisor_commenting = true;
  else
    advisor_commenting = false;
}

double Tier3EnterExit::actionComment(FORRAction action){
  cout << "In enter exit linear " << endl;
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  int targetRegion=-1;
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());

  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetRegion = i;
    }
  }

  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
      
  vector<FORRExit> exits = regions[targetRegion].getExits();

  double metric = std::numeric_limits<double>::infinity();
  for(int i = 0; i < exits.size(); i++){
    if (expectedPosition.getDistance(exits[i].getExitPoint().get_x(), exits[i].getExitPoint().get_y()) < metric) {
      metric = expectedPosition.getDistance(exits[i].getExitPoint().get_x(), exits[i].getExitPoint().get_y());
    }
  }
  return metric * (-1);
}


void Tier3EnterExit::set_commenting(){

  cout << "In enter exit linear set commenting " << endl;
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  bool targetInRegion = false;
  bool currPosInRegionWithExit = false;
  int robotRegion=-1, targetRegion = -1;
  
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetInRegion = true;
      targetRegion = i;
    }
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY()) and ((regions[i]).getExits().size() >= 1)){
      currPosInRegionWithExit = true;
      robotRegion = i;
    }
  }

  if(targetInRegion == true and robotRegion != targetRegion)
    advisor_commenting = true;
  else
    advisor_commenting = false;
}


double Tier3EnterExitRotation::actionComment(FORRAction action){
  cout << "In enter exit rotation " << endl;
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  int targetRegion=-1;
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());

  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetRegion = i;
    }
  }

  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
      
  vector<FORRExit> exits = regions[targetRegion].getExits();

  double metric = std::numeric_limits<double>::infinity();
  for(int i = 0; i < exits.size(); i++){
    if (expectedPosition.getDistance(exits[i].getExitPoint().get_x(), exits[i].getExitPoint().get_y()) < metric) {
      metric = expectedPosition.getDistance(exits[i].getExitPoint().get_x(), exits[i].getExitPoint().get_y());
    }
  }
  return metric * (-1);
}

void Tier3EnterExitRotation::set_commenting(){

  cout << "In region finder rotation set commenting " << endl;
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  bool targetInRegion = false;
  bool currPosInRegionWithExit = false;
  int robotRegion=-1, targetRegion = -1;
  
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetInRegion = true;
      targetRegion = i;
    }
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY()) and ((regions[i]).getExits().size() >= 1)){
      currPosInRegionWithExit = true;
      robotRegion = i;
    }
  }

  if(targetInRegion == true and robotRegion != targetRegion)
    advisor_commenting = true;
  else
    advisor_commenting = false;
}

double Tier3ExitLinear::actionComment(FORRAction action){
  double result=0;
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  
  int robotRegion=-1;
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
 
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY())){
      robotRegion = i;
    }
  }

  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  
  cout << "Robot Region : " << regions[robotRegion].getCenter().get_x() << " " << regions[robotRegion].getCenter().get_y() << endl;
  vector<FORRExit> ext_exits = regions[robotRegion].getExtExits();

  for(int i = 0 ; i < ext_exits.size(); i++){
    result += expectedPosition.getDistance(ext_exits[i].getExitPoint().get_x(), ext_exits[i].getExitPoint().get_y());
  }

  return result*(-1);
}


void Tier3ExitLinear::set_commenting(){

  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  bool targetInRegion = false;
  bool currPosInRegionWithExit = false;
  int robotRegion=-1, targetRegion = -1;
  
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetInRegion = true;
      targetRegion = i;
    }
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY()) and ((regions[i]).getExits().size() >= 1)){
      currPosInRegionWithExit = true;
      robotRegion = i;
    }
  }
  if(currPosInRegionWithExit == true and robotRegion != targetRegion)
    advisor_commenting = true;
  else
    advisor_commenting = false;
}


double Tier3ExitRotation::actionComment(FORRAction action){
  double result=0;
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  
  int robotRegion=-1;
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
 
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY())){
      robotRegion = i;
    }
  }
  
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);

  cout << "Robot Region : " << regions[robotRegion].getCenter().get_x() << " " << regions[robotRegion].getCenter().get_y() << endl;
  vector<FORRExit> ext_exits = regions[robotRegion].getExtExits();

  for(int i = 0 ; i < ext_exits.size(); i++){
    result += expectedPosition.getDistance(ext_exits[i].getExitPoint().get_x(), ext_exits[i].getExitPoint().get_y());
  }

  return result*(-1);
}

void Tier3ExitRotation::set_commenting(){
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  bool targetInRegion = false;
  bool currPosInRegionWithExit = false;
  int robotRegion=-1, targetRegion = -1;
  
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetInRegion = true;
      targetRegion = i;
    }
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY()) and ((regions[i]).getExits().size() >= 1)){
      currPosInRegionWithExit = true;
      robotRegion = i;
    }
  }
  if(currPosInRegionWithExit == true and robotRegion != targetRegion)
    advisor_commenting = true;
  else
    advisor_commenting = false;
  
}

double Tier3ExitFieldLinear::actionComment(FORRAction action){
  double result=0;
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  
  int robotRegion=-1;
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
 
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY())){
      robotRegion = i;
    }
  }

  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  
  cout << "Robot Region : " << regions[robotRegion].getCenter().get_x() << " " << regions[robotRegion].getCenter().get_y() << endl;
  vector<FORRExit> ext_exits = regions[robotRegion].getExtExits();

  for(int i = 0 ; i < ext_exits.size(); i++){
    result += (1 / expectedPosition.getDistance(ext_exits[i].getExitPoint().get_x(), ext_exits[i].getExitPoint().get_y()));
  }

  return result;
}


void Tier3ExitFieldLinear::set_commenting(){

  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  bool targetInRegion = false;
  bool currPosInRegionWithExit = false;
  int robotRegion=-1, targetRegion = -1;
  
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetInRegion = true;
      targetRegion = i;
    }
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY()) and ((regions[i]).getExits().size() >= 1)){
      currPosInRegionWithExit = true;
      robotRegion = i;
    }
  }
  if(currPosInRegionWithExit == true and robotRegion != targetRegion)
    advisor_commenting = true;
  else
    advisor_commenting = false;
}


double Tier3ExitFieldRotation::actionComment(FORRAction action){
  double result=0;
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  
  int robotRegion=-1;
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
 
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY())){
      robotRegion = i;
    }
  }
  
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);

  cout << "Robot Region : " << regions[robotRegion].getCenter().get_x() << " " << regions[robotRegion].getCenter().get_y() << endl;
  vector<FORRExit> ext_exits = regions[robotRegion].getExtExits();

  for(int i = 0 ; i < ext_exits.size(); i++){
    result += (1 / expectedPosition.getDistance(ext_exits[i].getExitPoint().get_x(), ext_exits[i].getExitPoint().get_y()));
  }

  return result;
}

void Tier3ExitFieldRotation::set_commenting(){
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  bool targetInRegion = false;
  bool currPosInRegionWithExit = false;
  int robotRegion=-1, targetRegion = -1;
  
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetInRegion = true;
      targetRegion = i;
    }
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY()) and ((regions[i]).getExits().size() >= 1)){
      currPosInRegionWithExit = true;
      robotRegion = i;
    }
  }
  if(currPosInRegionWithExit == true and robotRegion != targetRegion)
    advisor_commenting = true;
  else
    advisor_commenting = false;
  
}

double Tier3ExitClosest::actionComment(FORRAction action){
  double result=std::numeric_limits<double>::infinity();
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  
  int robotRegion=-1;
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
 
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY())){
      robotRegion = i;
    }
  }

  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  
  cout << "Robot Region : " << regions[robotRegion].getCenter().get_x() << " " << regions[robotRegion].getCenter().get_y() << endl;
  vector<FORRExit> exits = regions[robotRegion].getExits();

  for(int i = 0 ; i < exits.size(); i++){
    if (expectedPosition.getDistance(exits[i].getExitPoint().get_x(), exits[i].getExitPoint().get_y()) < result)
      result = expectedPosition.getDistance(exits[i].getExitPoint().get_x(), exits[i].getExitPoint().get_y());
  }

  return -result;
}


void Tier3ExitClosest::set_commenting(){

  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  bool targetInRegion = false;
  bool currPosInRegionWithExit = false;
  int robotRegion=-1, targetRegion = -1;
  
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetInRegion = true;
      targetRegion = i;
    }
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY()) and ((regions[i]).getExits().size() >= 1)){
      currPosInRegionWithExit = true;
      robotRegion = i;
    }
  }
  if(currPosInRegionWithExit == true and robotRegion != targetRegion)
    advisor_commenting = true;
  else
    advisor_commenting = false;
}


double Tier3ExitClosestRotation::actionComment(FORRAction action){
  double result=std::numeric_limits<double>::infinity();
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  
  int robotRegion=-1;
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
 
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY())){
      robotRegion = i;
    }
  }

  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  
  cout << "Robot Region : " << regions[robotRegion].getCenter().get_x() << " " << regions[robotRegion].getCenter().get_y() << endl;
  vector<FORRExit> exits = regions[robotRegion].getExits();

  for(int i = 0 ; i < exits.size(); i++){
    if (expectedPosition.getDistance(exits[i].getExitPoint().get_x(), exits[i].getExitPoint().get_y()) < result)
      result = expectedPosition.getDistance(exits[i].getExitPoint().get_x(), exits[i].getExitPoint().get_y());
  }

  return -result;
}

void Tier3ExitClosestRotation::set_commenting(){
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  bool targetInRegion = false;
  bool currPosInRegionWithExit = false;
  int robotRegion=-1, targetRegion = -1;
  
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetInRegion = true;
      targetRegion = i;
    }
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY()) and ((regions[i]).getExits().size() >= 1)){
      currPosInRegionWithExit = true;
      robotRegion = i;
    }
  }
  if(currPosInRegionWithExit == true and robotRegion != targetRegion)
    advisor_commenting = true;
  else
    advisor_commenting = false;
  
}

double Tier3RegionLeaverLinear::actionComment(FORRAction action){
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  
  int robotRegion=-1;
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
 
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY())){
      robotRegion = i;
    }
  }

  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  
  cout << "Robot Region : " << regions[robotRegion].getCenter().get_x() << " " << regions[robotRegion].getCenter().get_y() << " " << regions[robotRegion].getRadius() << endl;

  if(expectedPosition.getDistance(regions[robotRegion].getCenter().get_x(), regions[robotRegion].getCenter().get_y()) > regions[robotRegion].getRadius()) {
    return expectedPosition.getDistance(regions[robotRegion].getCenter().get_x(), regions[robotRegion].getCenter().get_y()) - regions[robotRegion].getRadius();
  } else {
    return (-1);
  }
}


void Tier3RegionLeaverLinear::set_commenting(){

  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  bool targetInRegion = false;
  bool currPosInRegionWithExit = false;
  int robotRegion=-1, targetRegion = -1;
  
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetInRegion = true;
      targetRegion = i;
    }
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY()) and ((regions[i]).getExits().size() >= 1)){
      currPosInRegionWithExit = true;
      robotRegion = i;
    }
  }
  if(currPosInRegionWithExit == true and robotRegion != targetRegion)
    advisor_commenting = true;
  else
    advisor_commenting = false;
}


double Tier3RegionLeaverRotation::actionComment(FORRAction action){
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  
  int robotRegion=-1;
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
 
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY())){
      robotRegion = i;
    }
  }
  
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);

  cout << "Robot Region : " << regions[robotRegion].getCenter().get_x() << " " << regions[robotRegion].getCenter().get_y() << " " << regions[robotRegion].getRadius() << endl;

  if(expectedPosition.getDistance(regions[robotRegion].getCenter().get_x(), regions[robotRegion].getCenter().get_y()) > regions[robotRegion].getRadius()) {
    return expectedPosition.getDistance(regions[robotRegion].getCenter().get_x(), regions[robotRegion].getCenter().get_y()) - regions[robotRegion].getRadius();
  } else {
    return (-1);
  }
}

void Tier3RegionLeaverRotation::set_commenting(){
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  bool targetInRegion = false;
  bool currPosInRegionWithExit = false;
  int robotRegion=-1, targetRegion = -1;
  
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetInRegion = true;
      targetRegion = i;
    }
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY()) and ((regions[i]).getExits().size() >= 1)){
      currPosInRegionWithExit = true;
      robotRegion = i;
    }
  }
  if(currPosInRegionWithExit == true and robotRegion != targetRegion)
    advisor_commenting = true;
  else
    advisor_commenting = false;
  
}
/*
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

  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  
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
*/
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
double Tier3ElbowRoom::actionComment(FORRAction action){

  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  double distanceToObstacle = beliefs->getAgentState()->getDistanceToNearestObstacle(expectedPosition);
  return distanceToObstacle;
}

// always on
void Tier3ElbowRoom::set_commenting(){
  advisor_commenting = true;
}

double Tier3ElbowRoomRotation::actionComment(FORRAction action){

  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  double obstacleDistance = beliefs->getAgentState()->getDistanceToNearestObstacle(expectedPosition);
  
  return obstacleDistance;
}

void Tier3ElbowRoomRotation::set_commenting(){
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

  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  
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

  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);

  double cur_x = beliefs->getAgentState()->getCurrentPosition().getX();
  double cur_y = beliefs->getAgentState()->getCurrentPosition().getY();

  double distanceFromCurrentPosition = expectedPosition.getDistance(cur_x,cur_y);
  return distanceFromCurrentPosition;
}

void Tier3BigStepRotation::set_commenting(){
  
  advisor_commenting = true;
  
}


// always on
void Tier3Unlikely::set_commenting(){
  //cout << "In avoid leaf set commenting " << endl;
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  bool targetInRegion = false;
  bool currPosInRegionWithExit = false;
  int robotRegion=-1, targetRegion = -1;
  
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetInRegion = true;
      targetRegion = i;
    }
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY()) and ((regions[i]).getExits().size() >= 1)){
      currPosInRegionWithExit = true;
      robotRegion = i;
    }
  }

  if(currPosInRegionWithExit == true and robotRegion != targetRegion)
    advisor_commenting = true;
  else
    advisor_commenting = false;
}

double Tier3Unlikely::actionComment(FORRAction action){
  //cout << "In Avoid leaf " << endl;
  double result;
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  std::vector< std::vector<Door> > doors = beliefs->getSpatialModel()->getDoors()->getDoors();
  int robotRegion=-1,targetRegion=-1;
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());

  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);

  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetRegion = i;
    }
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY())){
      robotRegion = i;
    }
  }
  
  vector<FORRExit> exits = regions[robotRegion].getExits();
  std::vector<Door> robotRegionDoors = doors[robotRegion];
  //cout << "Robot Region : " << regions[robotRegion].getCenter().get_x() << " " << regions[robotRegion].getCenter().get_y() << endl;

  vector<FORRRegion> nearRegions;
  for(int i = 0; i < exits.size() ; i++){
    FORRRegion test = regions[exits[i].getExitRegion()];
    std::vector<FORRRegion>::iterator it = std::find(nearRegions.begin(),nearRegions.end(), test);
    if(nearRegions.empty() or (it == nearRegions.end())){
      nearRegions.push_back(test);
      //cout << "Neighbour Region : " << test.getCenter().get_x() << " " << test.getCenter().get_y() << endl;
    }
  }

  //cout << "#neighbours found" << nearRegions.size() << endl; 

  double metric = 0;
  for(int i = 0; i < nearRegions.size(); i++){
    if(beliefs->getSpatialModel()->getRegionList()->isLeaf(nearRegions[i], robotRegionDoors.size()) and !(nearRegions[i].inRegion(targetPoint.get_x(), targetPoint.get_y()))){
      //cout << "Avoid Leaf: Found deadend !" << endl;
      metric += expectedPosition.getDistance(nearRegions[i].getCenter().get_x(), nearRegions[i].getCenter().get_y());
    }
  }

  return metric;
}

void Tier3UnlikelyRotation::set_commenting(){
  //cout << "In region finder rotation set commenting " << endl;
   vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
   Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
   Task *task = beliefs->getAgentState()->getCurrentTask();
   CartesianPoint targetPoint (task->getX() , task->getY());
   CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
   bool targetInRegion = false;
   bool currPosInRegionWithExit = false;
   int robotRegion=-1, targetRegion = -1;
   
   // check the preconditions for activating the advisor
   for(int i = 0; i < regions.size() ; i++){
     // check if the target point is in region
     if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
       targetInRegion = true;
       targetRegion = i;
     }
     // check if the rob_pos is in a region and the region has atleast one exit
     if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY()) and ((regions[i]).getExits().size() >= 1)){
       currPosInRegionWithExit = true;
       robotRegion = i;
     }
   }
   
   if(currPosInRegionWithExit == true and robotRegion != targetRegion)
     advisor_commenting = true;
   else
     advisor_commenting = false;
}

double Tier3UnlikelyRotation::actionComment(FORRAction action){
  //cout << "In Avoid leaf Rotation" << endl;
  double result;
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  std::vector< std::vector<Door> > doors = beliefs->getSpatialModel()->getDoors()->getDoors();
  int robotRegion=-1,targetRegion=-1;
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());

  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);

  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetRegion = i;
    }
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY())){
      robotRegion = i;
    }
  }
  
  vector<FORRExit> exits = regions[robotRegion].getExits();
  std::vector<Door> robotRegionDoors = doors[robotRegion];
  //cout << "Robot Region : " << regions[robotRegion].getCenter().get_x() << " " << regions[robotRegion].getCenter().get_y() << endl;

  vector<FORRRegion> nearRegions;
  for(int i = 0; i < exits.size() ; i++){
    FORRRegion test = regions[exits[i].getExitRegion()];
    std::vector<FORRRegion>::iterator it = std::find(nearRegions.begin(),nearRegions.end(), test);
    if(nearRegions.empty() or (it == nearRegions.end())){
      nearRegions.push_back(test);
      //cout << "Neighbour Region : " << test.getCenter().get_x() << " " << test.getCenter().get_y() << endl;
    }
  }

  //cout << "#neighbours found" << nearRegions.size() << endl; 

  double metric = 0;
  for(int i = 0; i < nearRegions.size(); i++){
    if(beliefs->getSpatialModel()->getRegionList()->isLeaf(nearRegions[i], robotRegionDoors.size()) and !(nearRegions[i].inRegion(targetPoint.get_x(), targetPoint.get_y()))){
      //cout << "Avoid leaf Rotation: Found deadend !" << endl;
      metric += expectedPosition.getDistance(nearRegions[i].getCenter().get_x(), nearRegions[i].getCenter().get_y());
    }
  }

  return metric;
}

void Tier3UnlikelyField::set_commenting(){
  //cout << "In avoid leaf set commenting " << endl;
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  bool targetInRegion = false;
  bool currPosInRegionWithExit = false;
  int robotRegion=-1, targetRegion = -1;
  
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetInRegion = true;
      targetRegion = i;
    }
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY()) and ((regions[i]).getExits().size() >= 1)){
      currPosInRegionWithExit = true;
      robotRegion = i;
    }
  }

  if(currPosInRegionWithExit == true and robotRegion != targetRegion)
    advisor_commenting = true;
  else
    advisor_commenting = false;
}

double Tier3UnlikelyField::actionComment(FORRAction action){
  //cout << "In Avoid leaf " << endl;
  double result;
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  std::vector< std::vector<Door> > doors = beliefs->getSpatialModel()->getDoors()->getDoors();
  int robotRegion=-1,targetRegion=-1;
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());

  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);

  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetRegion = i;
    }
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY())){
      robotRegion = i;
    }
  }
  
  vector<FORRExit> exits = regions[robotRegion].getExits();
  std::vector<Door> robotRegionDoors = doors[robotRegion];
  //cout << "Robot Region : " << regions[robotRegion].getCenter().get_x() << " " << regions[robotRegion].getCenter().get_y() << endl;

  vector<FORRRegion> nearRegions;
  for(int i = 0; i < exits.size() ; i++){
    FORRRegion test = regions[exits[i].getExitRegion()];
    std::vector<FORRRegion>::iterator it = std::find(nearRegions.begin(),nearRegions.end(), test);
    if(nearRegions.empty() or (it == nearRegions.end())){
      nearRegions.push_back(test);
      //cout << "Neighbour Region : " << test.getCenter().get_x() << " " << test.getCenter().get_y() << endl;
    }
  }

  //cout << "#neighbours found" << nearRegions.size() << endl; 

  double metric = 0;
  for(int i = 0; i < nearRegions.size(); i++){
    if(beliefs->getSpatialModel()->getRegionList()->isLeaf(nearRegions[i], robotRegionDoors.size()) and !(nearRegions[i].inRegion(targetPoint.get_x(), targetPoint.get_y()))){
      //cout << "Avoid Leaf: Found deadend !" << endl;
      metric += 1/(abs((expectedPosition.getDistance(nearRegions[i].getCenter().get_x(), nearRegions[i].getCenter().get_y())) - nearRegions[i].getRadius()));
    }
  }

  return metric * (-1);
}

void Tier3UnlikelyFieldRotation::set_commenting(){
  //cout << "In region finder rotation set commenting " << endl;
   vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
   Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
   Task *task = beliefs->getAgentState()->getCurrentTask();
   CartesianPoint targetPoint (task->getX() , task->getY());
   CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
   bool targetInRegion = false;
   bool currPosInRegionWithExit = false;
   int robotRegion=-1, targetRegion = -1;
   
   // check the preconditions for activating the advisor
   for(int i = 0; i < regions.size() ; i++){
     // check if the target point is in region
     if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
       targetInRegion = true;
       targetRegion = i;
     }
     // check if the rob_pos is in a region and the region has atleast one exit
     if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY()) and ((regions[i]).getExits().size() >= 1)){
       currPosInRegionWithExit = true;
       robotRegion = i;
     }
   }
   
   if(currPosInRegionWithExit == true and robotRegion != targetRegion)
     advisor_commenting = true;
   else
     advisor_commenting = false;
}

double Tier3UnlikelyFieldRotation::actionComment(FORRAction action){
  //cout << "In Avoid leaf Rotation" << endl;
  double result;
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  std::vector< std::vector<Door> > doors = beliefs->getSpatialModel()->getDoors()->getDoors();
  int robotRegion=-1,targetRegion=-1;
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());

  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);

  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetRegion = i;
    }
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY())){
      robotRegion = i;
    }
  }
  
  vector<FORRExit> exits = regions[robotRegion].getExits();
  std::vector<Door> robotRegionDoors = doors[robotRegion];
  //cout << "Robot Region : " << regions[robotRegion].getCenter().get_x() << " " << regions[robotRegion].getCenter().get_y() << endl;

  vector<FORRRegion> nearRegions;
  for(int i = 0; i < exits.size() ; i++){
    FORRRegion test = regions[exits[i].getExitRegion()];
    std::vector<FORRRegion>::iterator it = std::find(nearRegions.begin(),nearRegions.end(), test);
    if(nearRegions.empty() or (it == nearRegions.end())){
      nearRegions.push_back(test);
      //cout << "Neighbour Region : " << test.getCenter().get_x() << " " << test.getCenter().get_y() << endl;
    }
  }

  //cout << "#neighbours found" << nearRegions.size() << endl; 

  double metric = 0;
  for(int i = 0; i < nearRegions.size(); i++){
    if(beliefs->getSpatialModel()->getRegionList()->isLeaf(nearRegions[i], robotRegionDoors.size()) and !(nearRegions[i].inRegion(targetPoint.get_x(), targetPoint.get_y()))){
      //cout << "Avoid leaf Rotation: Found deadend !" << endl;
      metric += 1/(abs((expectedPosition.getDistance(nearRegions[i].getCenter().get_x(), nearRegions[i].getCenter().get_y())) - nearRegions[i].getRadius()));
    }
  }

  return metric * (-1);
}

// always on
void Tier3Explorer::set_commenting(){
  advisor_commenting = true;
}

double Tier3Explorer::actionComment(FORRAction action){
 
  vector<Position> *positionHis = beliefs->getAgentState()->getCurrentTask()->getPositionHistory();
 
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

void Tier3ExplorerEndPoints::set_commenting(){
  advisor_commenting = true;
}

double Tier3ExplorerEndPoints::actionComment(FORRAction action){
  vector< vector <CartesianPoint> > *laserHis = beliefs->getAgentState()->getCurrentTask()->getLaserHistory();
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  int beta = 0;
  double totalForce = 0, distance = 0; 

  for(int i = 0; i < laserHis->size(); i++){
  	//cout << laserHis->size() << endl;
  	for (int j = 0; j< ((*laserHis)[i]).size(); j++) {
  		//cout << ((*laserHis)[i]).size() << endl;
  		distance = expectedPosition.getDistance(((*laserHis)[i])[j].get_x(), ((*laserHis)[i])[j].get_y());
    	if(distance < 1)     distance = 1;
    	if(distance < 25) {
        totalForce += (1/distance);
      }
  	}
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
 
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);

  vector<Position> *positionHis = beliefs->getAgentState()->getCurrentTask()->getPositionHistory();
  int beta = 0;
  double totalForce = 0, distance = 0;
 
  for(int i = 0; i < positionHis->size(); i++){
    distance = expectedPosition.getDistance((*positionHis)[i]);
    if(distance < 1)     distance = 1;
    totalForce += (1/distance);
  }
  return totalForce * (-1);
}

void Tier3ExplorerEndPointsRotation::set_commenting(){
  advisor_commenting = true;
}

double Tier3ExplorerEndPointsRotation::actionComment(FORRAction action){
  vector< vector <CartesianPoint> > *laserHis = beliefs->getAgentState()->getCurrentTask()->getLaserHistory();
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  int beta = 0;
  double totalForce = 0, distance = 0; 

  for(int i = 0; i < laserHis->size(); i++){
  	//cout << laserHis->size() << endl;
  	for (int j = 0; j< ((*laserHis)[i]).size(); j++) {
  		//cout << ((*laserHis)[i]).size() << endl;
  		distance = expectedPosition.getDistance(((*laserHis)[i])[j].get_x(), ((*laserHis)[i])[j].get_y());
    	if(distance < 1)     distance = 1;
      if(distance < 25) {
        totalForce += (1/distance);
      }
  	}
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
double Tier3ConveyLinear::actionComment(FORRAction action){
 
  //cout << "Entered Convey linear."<<endl;
  Position cur_pos = beliefs->getAgentState()->getCurrentPosition();
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  
  int grid_value = beliefs->getSpatialModel()->getConveyors()->getGridValue(expectedPosition.getX(), expectedPosition.getY());
  
  double distance = cur_pos.getDistance(expectedPosition.getX(), expectedPosition.getY());
  
  //cout <<"Exit conveyor linear."<<endl;
  return distance * grid_value; //want larger grid values that are further away
}


void Tier3ConveyLinear::set_commenting(){
  advisor_commenting = true;
}

double Tier3ConveyRotation::actionComment(FORRAction action){
  cout <<" Entered Convey rotation." << endl;
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  cout <<" Expected position after action: " <<expectedPosition.getX() << " " << expectedPosition.getY() << endl;
  int grid_value = beliefs->getSpatialModel()->getConveyors()->getGridValue(expectedPosition.getX(), expectedPosition.getY());
  cout << "grid value: "<<grid_value<<endl;

  Position cur_pos = beliefs->getAgentState()->getCurrentPosition();
  double distance = cur_pos.getDistance(expectedPosition.getX(), expectedPosition.getY());  
  return distance * grid_value;
}

void Tier3ConveyRotation::set_commenting(){
  advisor_commenting = true;
}


double Tier3TrailerLinear::actionComment(FORRAction action){
   CartesianPoint target_trailmarker = beliefs->getSpatialModel()->getTrails()->getFurthestVisiblePointOnChosenTrail(beliefs->getAgentState());  
   //if out of line of sight of trail marker, vote the same value functionally turning off voting
   if(!beliefs->getSpatialModel()->getTrails()->canSeeTrail()){
     return 0;
   }

  Position target(target_trailmarker.get_x(),target_trailmarker.get_y(),0);

  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  double newDistance = expectedPosition.getDistance(target);
  cout << (-1)* newDistance << endl;
  return newDistance *(-1); 
}

//only turn on if a trail is active
void Tier3TrailerLinear::set_commenting(){
  // Searches for a trail if not found
  beliefs->getSpatialModel()->getTrails()->findNearbyTrail(beliefs->getAgentState());
  if(beliefs->getSpatialModel()->getTrails()->getChosenTrail() == -1){
    advisor_commenting = false;
  }
  else{
	// If the advisor is commenting that means a trail has been choosen and advisor will stick with it till the end of task
      advisor_commenting = true;
  }
}

double Tier3TrailerRotation::actionComment(FORRAction action){
   CartesianPoint target_trailmarker = beliefs->getSpatialModel()->getTrails()->getFurthestVisiblePointOnChosenTrail(beliefs->getAgentState());  
   //if out of line of sight of trail marker, vote the same value in effect turning off voting
  //can_see_trail is set in getFurthestVisiblePointOnChosenTrail
   if(!beliefs->getSpatialModel()->getTrails()->canSeeTrail()){
     return 0;
   }

  Position target(target_trailmarker.get_x(),target_trailmarker.get_y(),0);

  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  double newDistance = expectedPosition.getDistance(target);
  return newDistance *(-1); 
}


//only turn on if a trail is active
void Tier3TrailerRotation::set_commenting(){
  // Searches for a trail if not choosen yet
  beliefs->getSpatialModel()->getTrails()->findNearbyTrail(beliefs->getAgentState());
  if(beliefs->getSpatialModel()->getTrails()->getChosenTrail() == -1){
    advisor_commenting = false;
  }
  else{
    // If the advisor is commenting that means a trail has been choosen and advisor will stick with it till the end of task 
    advisor_commenting = true;
  }
}

double Tier3GoAroundRotation::actionComment(FORRAction action){
  // break up action into its components
  FORRActionType actionType = action.type;
  int intensity = action.parameter;
  double comment_strength;
  double avgRightDistanceVector = 0, avgLeftDistanceVector = 0;
  sensor_msgs::LaserScan laserScan = beliefs->getAgentState()->getCurrentLaserScan();
  // compute forward distance to obstacle
  double centerDistanceVector = ( laserScan.ranges[((laserScan.ranges.size()/2)-1)] + laserScan.ranges[((laserScan.ranges.size()/2))] ) / 2;
  // find average length of distance vectors on right and left sides
  for(int i = 0; i < ((laserScan.ranges.size()/2)); i++){
    double length = laserScan.ranges[i];
    avgRightDistanceVector += length;
  }
  for(int i = (laserScan.ranges.size()/2); i < (laserScan.ranges.size()-1); i++){
    double length = laserScan.ranges[i];
    avgLeftDistanceVector += length;
  }

  avgRightDistanceVector = avgRightDistanceVector / (laserScan.ranges.size()/2);
  avgLeftDistanceVector = avgLeftDistanceVector / (laserScan.ranges.size()/2);
  // Comment strength is computed as the intensity of the rotation divided by the distance to the forward obstacle
  if(avgRightDistanceVector > avgLeftDistanceVector && actionType == RIGHT_TURN) {
    comment_strength = (intensity / centerDistanceVector);
  }
  else if(avgRightDistanceVector < avgLeftDistanceVector && actionType == RIGHT_TURN) {
    comment_strength = -1 * (intensity / centerDistanceVector);
  }
  else if(avgRightDistanceVector > avgLeftDistanceVector && actionType == LEFT_TURN) {
    comment_strength = -1 * (intensity / centerDistanceVector);
  }
  else if(avgRightDistanceVector < avgLeftDistanceVector && actionType == LEFT_TURN) {
    comment_strength = (intensity / centerDistanceVector);
  }
  else if(avgRightDistanceVector = avgLeftDistanceVector) {
    comment_strength = (intensity / centerDistanceVector);
  }

  return comment_strength;
}
  
void Tier3GoAroundRotation::set_commenting(){
  advisor_commenting = true;
}

double Tier3EnterDoorLinear::actionComment(FORRAction action){
  std::vector< std::vector<Door> > doors = beliefs->getSpatialModel()->getDoors()->getDoors();
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  CartesianPoint expPosition (expectedPosition.getX(), expectedPosition.getY());
  int targetRegion = -1;
  double comment_strength = 0;

  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetRegion = i;
    }
  }
  // check if the expected position is in the target's region
  if(regions[targetRegion].inRegion(expPosition) == true){
    Circle region = Circle(regions[targetRegion].getCenter(), regions[targetRegion].getRadius());
    CartesianPoint intersectPoint = intersection_point(region, LineSegment(currentPosition, expPosition));
    double intersectPointAngle = beliefs->getSpatialModel()->getDoors()->calculateFixedAngle(regions[targetRegion].getCenter().get_x(), regions[targetRegion].getCenter().get_y(), intersectPoint.get_x(), intersectPoint.get_y());
    for(int i = 0; i < doors[targetRegion].size(); i++) {
      // check if the point that the robot crosses into the region goes through one of the doors
      double startPointAngle = beliefs->getSpatialModel()->getDoors()->calculateFixedAngle(regions[targetRegion].getCenter().get_x(), regions[targetRegion].getCenter().get_y(), doors[targetRegion][i].startPoint.getExitPoint().get_x(), doors[targetRegion][i].startPoint.getExitPoint().get_y());
      double endPointAngle = beliefs->getSpatialModel()->getDoors()->calculateFixedAngle(regions[targetRegion].getCenter().get_x(), regions[targetRegion].getCenter().get_y(), doors[targetRegion][i].endPoint.getExitPoint().get_x(), doors[targetRegion][i].endPoint.getExitPoint().get_y());
      if(intersectPointAngle <= endPointAngle and intersectPointAngle >= startPointAngle){
        comment_strength = -expPosition.get_distance(targetPoint);
      }
    }
  }

  return comment_strength;
}

void Tier3EnterDoorLinear::set_commenting(){
  cout << "In enter door linear set commenting " << endl;
  std::vector< std::vector<Door> > doors = beliefs->getSpatialModel()->getDoors()->getDoors();
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  bool targetInRegionWithDoor = false;
  bool currPosInTargetRegion = false;
  int targetRegion = -1;
  
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y()) and (doors[i].size() >= 1)){
      targetInRegionWithDoor = true;
      targetRegion = i;
      currPosInTargetRegion = regions[targetRegion].inRegion(curr_pos.getX(), curr_pos.getY());
    }
  }

  if(targetInRegionWithDoor == true and currPosInTargetRegion == false)
    advisor_commenting = true;
  else
    advisor_commenting = false;
}

double Tier3EnterDoorRotation::actionComment(FORRAction action){
  std::vector< std::vector<Door> > doors = beliefs->getSpatialModel()->getDoors()->getDoors();
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  CartesianPoint expPosition (expectedPosition.getX(), expectedPosition.getY());
  int targetRegion = -1;
  double comment_strength = 0;

  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetRegion = i;
    }
  }
  // check if the expected position is in the target's region
  if(regions[targetRegion].inRegion(expPosition) == true){
    Circle region = Circle(regions[targetRegion].getCenter(), regions[targetRegion].getRadius());
    CartesianPoint intersectPoint = intersection_point(region, LineSegment(currentPosition, expPosition));
    double intersectPointAngle = beliefs->getSpatialModel()->getDoors()->calculateFixedAngle(regions[targetRegion].getCenter().get_x(), regions[targetRegion].getCenter().get_y(), intersectPoint.get_x(), intersectPoint.get_y());
    for(int i = 0; i < doors[targetRegion].size(); i++) {
      // check if the point that the robot crosses into the region goes through one of the doors
      double startPointAngle = beliefs->getSpatialModel()->getDoors()->calculateFixedAngle(regions[targetRegion].getCenter().get_x(), regions[targetRegion].getCenter().get_y(), doors[targetRegion][i].startPoint.getExitPoint().get_x(), doors[targetRegion][i].startPoint.getExitPoint().get_y());
      double endPointAngle = beliefs->getSpatialModel()->getDoors()->calculateFixedAngle(regions[targetRegion].getCenter().get_x(), regions[targetRegion].getCenter().get_y(), doors[targetRegion][i].endPoint.getExitPoint().get_x(), doors[targetRegion][i].endPoint.getExitPoint().get_y());
      if(intersectPointAngle <= endPointAngle and intersectPointAngle >= startPointAngle){
        comment_strength = -expPosition.get_distance(targetPoint);
      }
    }
  }

  return comment_strength;
}

void Tier3EnterDoorRotation::set_commenting(){
  cout << "In enter door rotation set commenting " << endl;
  std::vector< std::vector<Door> > doors = beliefs->getSpatialModel()->getDoors()->getDoors();
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  bool targetInRegionWithDoor = false;
  bool currPosInTargetRegion = false;
  int targetRegion = -1;
  
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y()) and (doors[i].size() >= 1)){
      targetInRegionWithDoor = true;
      targetRegion = i;
      currPosInTargetRegion = regions[targetRegion].inRegion(curr_pos.getX(), curr_pos.getY());
    }
  }

  if(targetInRegionWithDoor == true and currPosInTargetRegion == false)
    advisor_commenting = true;
  else
    advisor_commenting = false;
}

double Tier3ExitDoorLinear::actionComment(FORRAction action){
  std::vector< std::vector<Door> > doors = beliefs->getSpatialModel()->getDoors()->getDoors();
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  CartesianPoint expPosition (expectedPosition.getX(), expectedPosition.getY());
  double comment_strength = 0;
  int robotRegion=-1;
 
  for(int i = 0; i < regions.size() ; i++){
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY())){
      robotRegion = i;
    }
  }

  cout << "Robot Region : " << regions[robotRegion].getCenter().get_x() << " " << regions[robotRegion].getCenter().get_y() << " " << regions[robotRegion].getRadius() << endl;

  double min_distance = std::numeric_limits<double>::infinity();
  for(int i = 0; i < doors[robotRegion].size(); i++) {
  	double doorDistance = beliefs->getSpatialModel()->getDoors()->distanceToDoor(expPosition, regions[robotRegion], doors[robotRegion][i]);
    if (doorDistance < min_distance){
      min_distance = doorDistance;
      comment_strength = ((double)(doors[robotRegion][i].str)) * (1 / doorDistance);
    }
  }

  // check if the robot gets out of the region
  /*if(expectedPosition.getDistance(regions[robotRegion].getCenter().get_x(), regions[robotRegion].getCenter().get_y()) > regions[robotRegion].getRadius()) {
    Circle region = Circle(regions[robotRegion].getCenter(), regions[robotRegion].getRadius());
    CartesianPoint intersectPoint = intersection_point(region, LineSegment(currentPosition, expPosition));
    double intersectPointAngle = beliefs->getSpatialModel()->getDoors()->calculateFixedAngle(regions[robotRegion].getCenter().get_x(), regions[robotRegion].getCenter().get_y(), intersectPoint.get_x(), intersectPoint.get_y());
    for(int i = 0; i < doors[robotRegion].size(); i++) {
      // check if the point that the robot crosses out of the region goes through one of the doors
      double startPointAngle = beliefs->getSpatialModel()->getDoors()->calculateFixedAngle(regions[robotRegion].getCenter().get_x(), regions[robotRegion].getCenter().get_y(), doors[robotRegion][i].startPoint.getExitPoint().get_x(), doors[robotRegion][i].startPoint.getExitPoint().get_y());
      double endPointAngle = beliefs->getSpatialModel()->getDoors()->calculateFixedAngle(regions[robotRegion].getCenter().get_x(), regions[robotRegion].getCenter().get_y(), doors[robotRegion][i].endPoint.getExitPoint().get_x(), doors[robotRegion][i].endPoint.getExitPoint().get_y());
      if(intersectPointAngle <= endPointAngle and intersectPointAngle >= startPointAngle){
        comment_strength = ((double)(doors[robotRegion][i].str)) * (expectedPosition.getDistance(regions[robotRegion].getCenter().get_x(), regions[robotRegion].getCenter().get_y()) - regions[robotRegion].getRadius());
      }
    }
  }*/
  return comment_strength;
}

void Tier3ExitDoorLinear::set_commenting(){
  std::vector< std::vector<Door> > doors = beliefs->getSpatialModel()->getDoors()->getDoors();
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  bool targetInRegion = false;
  bool currPosInRegionWithDoor = false;
  int robotRegion=-1, targetRegion = -1;
  
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetInRegion = true;
      targetRegion = i;
    }
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY()) and (doors[i].size() >= 1)){
      currPosInRegionWithDoor = true;
      robotRegion = i;
    }
  }
  if(currPosInRegionWithDoor == true and robotRegion != targetRegion)
    advisor_commenting = true;
  else
    advisor_commenting = false;
}

double Tier3ExitDoorRotation::actionComment(FORRAction action){
  std::vector< std::vector<Door> > doors = beliefs->getSpatialModel()->getDoors()->getDoors();
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  CartesianPoint expPosition (expectedPosition.getX(), expectedPosition.getY());
  double comment_strength = 0;
  int robotRegion=-1;
 
  for(int i = 0; i < regions.size() ; i++){
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY())){
      robotRegion = i;
    }
  }

  cout << "Robot Region : " << regions[robotRegion].getCenter().get_x() << " " << regions[robotRegion].getCenter().get_y() << " " << regions[robotRegion].getRadius() << endl;

  double min_distance = std::numeric_limits<double>::infinity();
  for(int i = 0; i < doors[robotRegion].size(); i++) {
  	double doorDistance = beliefs->getSpatialModel()->getDoors()->distanceToDoor(expPosition, regions[robotRegion], doors[robotRegion][i]);
    if (doorDistance < min_distance){
      min_distance = doorDistance;
      comment_strength = ((double)(doors[robotRegion][i].str)) * (1 / doorDistance);
    }
  }

  // check if the robot gets out of the region
  /*if(expectedPosition.getDistance(regions[robotRegion].getCenter().get_x(), regions[robotRegion].getCenter().get_y()) > regions[robotRegion].getRadius()) {
    Circle region = Circle(regions[robotRegion].getCenter(), regions[robotRegion].getRadius());
    CartesianPoint intersectPoint = intersection_point(region, LineSegment(currentPosition, expPosition));
    double intersectPointAngle = beliefs->getSpatialModel()->getDoors()->calculateFixedAngle(regions[robotRegion].getCenter().get_x(), regions[robotRegion].getCenter().get_y(), intersectPoint.get_x(), intersectPoint.get_y());
    for(int i = 0; i < doors[robotRegion].size(); i++) {
      // check if the point that the robot crosses out of the region goes through one of the doors
      double startPointAngle = beliefs->getSpatialModel()->getDoors()->calculateFixedAngle(regions[robotRegion].getCenter().get_x(), regions[robotRegion].getCenter().get_y(), doors[robotRegion][i].startPoint.getExitPoint().get_x(), doors[robotRegion][i].startPoint.getExitPoint().get_y());
      double endPointAngle = beliefs->getSpatialModel()->getDoors()->calculateFixedAngle(regions[robotRegion].getCenter().get_x(), regions[robotRegion].getCenter().get_y(), doors[robotRegion][i].endPoint.getExitPoint().get_x(), doors[robotRegion][i].endPoint.getExitPoint().get_y());
      if(intersectPointAngle <= endPointAngle and intersectPointAngle >= startPointAngle){
        comment_strength = ((double)(doors[robotRegion][i].str)) * (expectedPosition.getDistance(regions[robotRegion].getCenter().get_x(), regions[robotRegion].getCenter().get_y()) - regions[robotRegion].getRadius());
      }
    }
  }*/
  return comment_strength;
}

void Tier3ExitDoorRotation::set_commenting(){
  std::vector< std::vector<Door> > doors = beliefs->getSpatialModel()->getDoors()->getDoors();
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  bool targetInRegion = false;
  bool currPosInRegionWithDoor = false;
  int robotRegion=-1, targetRegion = -1;
  
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetInRegion = true;
      targetRegion = i;
    }
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY()) and (doors[i].size() >= 1)){
      currPosInRegionWithDoor = true;
      robotRegion = i;
    }
  }
  if(currPosInRegionWithDoor == true and robotRegion != targetRegion)
    advisor_commenting = true;
  else
    advisor_commenting = false;
}

double Tier3AccessLinear::actionComment(FORRAction action){
  double result=0;
  std::vector< std::vector<Door> > doors = beliefs->getSpatialModel()->getDoors()->getDoors();
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  CartesianPoint expPosition (expectedPosition.getX(), expectedPosition.getY());

  for(int i = 0; i < doors.size() ; i++){
    result += ((1 / (expPosition.get_distance(regions[i].getCenter()) - regions[i].getRadius()) ) * doors[i].size());
  }

  return result;
}

void Tier3AccessLinear::set_commenting(){
  std::vector< std::vector<Door> > doors = beliefs->getSpatialModel()->getDoors()->getDoors();
  bool atLeastOneDoor = false;
  for(int i = 0; i < doors.size(); i++){
    if(doors[i].size() >= 1){
      atLeastOneDoor = true;
    }
  }
  if(atLeastOneDoor == true) 
    advisor_commenting = true;
  else
    advisor_commenting = false;
}

double Tier3AccessRotation::actionComment(FORRAction action){
  double result=0;
  std::vector< std::vector<Door> > doors = beliefs->getSpatialModel()->getDoors()->getDoors();
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  CartesianPoint expPosition (expectedPosition.getX(), expectedPosition.getY());

  for(int i = 0; i < doors.size() ; i++){
    result += ((1 / (expPosition.get_distance(regions[i].getCenter()) - regions[i].getRadius()) ) * doors[i].size());
  }

  return result;
}

void Tier3AccessRotation::set_commenting(){
  std::vector< std::vector<Door> > doors = beliefs->getSpatialModel()->getDoors()->getDoors();
  bool atLeastOneDoor = false;
  for(int i = 0; i < doors.size(); i++){
    if(doors[i].size() >= 1){
      atLeastOneDoor = true;
    }
  }
  if(atLeastOneDoor == true) 
    advisor_commenting = true;
  else
    advisor_commenting = false;
}

/*double Tier3NeighborDoorLinear::actionComment(FORRAction action){
  double result=0;
  std::vector< std::vector<Door> > doors = beliefs->getSpatialModel()->getDoors()->getDoors();
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  CartesianPoint expPosition (expectedPosition.getX(), expectedPosition.getY());
  int targetRegion = -1;
  
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetRegion = i;
    }
  }

  vector<FORRExit> exits = regions[targetRegion].getExits();
  vector<FORRRegion> nearRegions;
  for(int i = 0; i < exits.size() ; i++){
    FORRRegion test = regions[exits[i].getExitRegion()];
    std::vector<FORRRegion>::iterator it = std::find(nearRegions.begin(),nearRegions.end(), test);
    if(nearRegions.empty() or (it == nearRegions.end())){
      nearRegions.push_back(test);
      result += ((1 / expPosition.get_distance(regions[exits[i].getExitRegion()].getCenter())) * doors[exits[i].getExitRegion()].size());
    }
  }

  return result;
}

void Tier3NeighborDoorLinear::set_commenting(){
  cout << "In neighbor door linear set commenting " << endl;
  std::vector< std::vector<Door> > doors = beliefs->getSpatialModel()->getDoors()->getDoors();
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  bool targetInRegion = false;
  bool targetHasNeighbors = false;
  int targetRegion = -1;
  
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetInRegion = true;
      targetRegion = i;
    }
  }
  if(targetInRegion == true) {
    vector<FORRExit> exits = regions[targetRegion].getExits();
    vector<FORRRegion> nearRegions;
    for(int i = 0; i < exits.size() ; i++){
      FORRRegion test = regions[exits[i].getExitRegion()];
      std::vector<FORRRegion>::iterator it = std::find(nearRegions.begin(),nearRegions.end(), test);
      if(nearRegions.empty() or (it == nearRegions.end())){
        nearRegions.push_back(test);
        cout << "Neighbour Region : " << test.getCenter().get_x() << " " << test.getCenter().get_y() << endl;
      }
    }
    cout << "#Neighbors found :" << nearRegions.size() << endl;
    if(nearRegions.size() >= 1) {
      targetHasNeighbors = true;
    }
  }

  if(targetInRegion == true and targetHasNeighbors == true)
    advisor_commenting = true;
  else
    advisor_commenting = false;
}

double Tier3NeighborDoorRotation::actionComment(FORRAction action){
  double result=0;
  std::vector< std::vector<Door> > doors = beliefs->getSpatialModel()->getDoors()->getDoors();
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  CartesianPoint expPosition (expectedPosition.getX(), expectedPosition.getY());
  int targetRegion = -1;
  
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetRegion = i;
    }
  }

  vector<FORRExit> exits = regions[targetRegion].getExits();
  vector<FORRRegion> nearRegions;
  for(int i = 0; i < exits.size() ; i++){
    FORRRegion test = regions[exits[i].getExitRegion()];
    std::vector<FORRRegion>::iterator it = std::find(nearRegions.begin(),nearRegions.end(), test);
    if(nearRegions.empty() or (it == nearRegions.end())){
      nearRegions.push_back(test);
      result += ((1 / expPosition.get_distance(regions[exits[i].getExitRegion()].getCenter())) * doors[exits[i].getExitRegion()].size());
    }
  }

  return result;
}

void Tier3NeighborDoorRotation::set_commenting(){
  cout << "In neighbor door linear set commenting " << endl;
  std::vector< std::vector<Door> > doors = beliefs->getSpatialModel()->getDoors()->getDoors();
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  bool targetInRegion = false;
  bool targetHasNeighbors = false;
  int targetRegion = -1;
  
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetInRegion = true;
      targetRegion = i;
    }
  }
  if(targetInRegion == true) {
    vector<FORRExit> exits = regions[targetRegion].getExits();
    vector<FORRRegion> nearRegions;
    for(int i = 0; i < exits.size() ; i++){
      FORRRegion test = regions[exits[i].getExitRegion()];
      std::vector<FORRRegion>::iterator it = std::find(nearRegions.begin(),nearRegions.end(), test);
      if(nearRegions.empty() or (it == nearRegions.end())){
        nearRegions.push_back(test);
        cout << "Neighbour Region : " << test.getCenter().get_x() << " " << test.getCenter().get_y() << endl;
      }
    }
    cout << "#Neighbors found :" << nearRegions.size() << endl;
    if(nearRegions.size() >= 1) {
      targetHasNeighbors = true;
    }
  }

  if(targetInRegion == true and targetHasNeighbors == true)
    advisor_commenting = true;
  else
    advisor_commenting = false;
}*/

void Tier3LearnSpatialModel::set_commenting(){
  advisor_commenting = true;
}

double Tier3LearnSpatialModel::actionComment(FORRAction action){
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  CartesianPoint expPosition (expectedPosition.getX(), expectedPosition.getY());
  double result;
  int robotRegion = -1;

  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  for(int i = 0; i < regions.size() ; i++){
    // check if the expected position is in region
    if(regions[i].inRegion(expPosition.get_x(), expPosition.get_y())){
      robotRegion = i;
    }
  }
  if(robotRegion > (-1)){
    result = -1.0 * (beliefs->getSpatialModel()->getConveyors()->getMaxGridValue());
    cout << "LearnSpatialModel INSIDE REGION : " << result << endl;
  }
  else {
    result = -(beliefs->getSpatialModel()->getConveyors()->getAverageGridValue(expectedPosition.getX(), expectedPosition.getY()));
    cout << "LearnSpatialModel OUTSIDE REGION : " << result << endl;
  }
  return result;
}

void Tier3LearnSpatialModelRotation::set_commenting(){
  advisor_commenting = true;
}

double Tier3LearnSpatialModelRotation::actionComment(FORRAction action){
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  CartesianPoint expPosition (expectedPosition.getX(), expectedPosition.getY());
  double result;
  int robotRegion = -1;

  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  for(int i = 0; i < regions.size() ; i++){
    // check if the expected position is in region
    if(regions[i].inRegion(expPosition.get_x(), expPosition.get_y())){
      robotRegion = i;
    }
  }
  if(robotRegion > (-1)){
    result = -1.0 * (beliefs->getSpatialModel()->getConveyors()->getMaxGridValue());
    cout << "LearnSpatialModel INSIDE REGION : " << result << endl;
  }
  else {
    result = -(beliefs->getSpatialModel()->getConveyors()->getAverageGridValue(expectedPosition.getX(), expectedPosition.getY()));
    cout << "LearnSpatialModel OUTSIDE REGION : " << result << endl;
  }
  return result;
}

void Tier3ActiveLearner::set_commenting(){
  advisor_commenting = true;
}

double Tier3ActiveLearner::actionComment(FORRAction action){
  vector< vector<CartesianPoint> > all_trace = beliefs->getAgentState()->getAllTrace();
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  CartesianPoint expPosition (expectedPosition.getX(), expectedPosition.getY());
  double totalForce = 0, distance = 0; 

  for(int i = 0; i < all_trace.size(); i++){
    for(int j = 0; j < all_trace[i].size(); j++) {
      distance = expPosition.get_distance(all_trace[i][j]);
      if(distance < 1)     distance = 1;
      totalForce += (1/distance);
    }
  }

  vector<Position> *positionHis = beliefs->getAgentState()->getCurrentTask()->getPositionHistory();
  for(int i = 0; i < positionHis->size(); i++){
    distance = expectedPosition.getDistance((*positionHis)[i]);
    if(distance < 1)     distance = 1;
    totalForce += (1/distance);
  }

  return totalForce * (-1);
}

void Tier3ActiveLearnerRotation::set_commenting(){
  advisor_commenting = true;
}

double Tier3ActiveLearnerRotation::actionComment(FORRAction action){
  vector< vector<CartesianPoint> > all_trace = beliefs->getAgentState()->getAllTrace();
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  CartesianPoint expPosition (expectedPosition.getX(), expectedPosition.getY());
  double totalForce = 0, distance = 0; 

  for(int i = 0; i < all_trace.size(); i++){
    for(int j = 0; j < all_trace[i].size(); j++) {
      distance = expPosition.get_distance(all_trace[i][j]);
      if(distance < 1)     distance = 1;
      totalForce += (1/distance);
    }
  }

  vector<Position> *positionHis = beliefs->getAgentState()->getCurrentTask()->getPositionHistory();
  for(int i = 0; i < positionHis->size(); i++){
    distance = expectedPosition.getDistance((*positionHis)[i]);
    if(distance < 1)     distance = 1;
    totalForce += (1/distance);
  }

  return totalForce * (-1);
}