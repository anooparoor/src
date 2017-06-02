/*
 * Controller.cpp
 *
 */
          
#include "Controller.h"
#include "FORRGeometry.h"
#include <unistd.h>

#include <deque>
#include <iostream> 
#include <fstream>
#include <math.h>
#include <time.h>
#include <vector>
#include <string>
#include <sstream>



using namespace std;

#define CTRL_DEBUG true

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Read from the config file and intialize advisors and weights and spatial learning modules based on the advisors
//
//
void Controller::initialize_advisors(string filename){
 
    string fileLine;
    string advisor_name, advisor_description;
    bool advisor_active;
    double advisor_weight = 1;
    double parameters[4];
    std::ifstream file(filename.c_str());
    ROS_DEBUG_STREAM("Reading read_advisor_file:" << filename);
    if(!file.is_open()){
	ROS_DEBUG("Unable to locate or read advisor config file!");
    }
    //read advisor names and parameters from the config file and create new advisor objects
    while(getline(file, fileLine)){
       if(fileLine[0] == '#')  // skip comment lines
          continue;
       else{
          std::stringstream ss(fileLine);
	  std::istream_iterator<std::string> begin(ss);
	  std::istream_iterator<std::string> end;
	  std::vector<std::string> vstrings(begin, end);
	  advisor_name = vstrings[0];
	  advisor_description = vstrings[1];
	  if(vstrings[2] == "t")
       		advisor_active = true;
     	  else
      		advisor_active = false;
	  advisor_weight = atof(vstrings[3].c_str());
	  parameters[0]= atof(vstrings[4].c_str());
     	  parameters[1] = atof(vstrings[5].c_str());
          parameters[2] = atof(vstrings[6].c_str());
          parameters[3] = atof(vstrings[7].c_str());
          tier3Advisors.push_back(Tier3Advisor::makeAdvisor(getBeliefs(), advisor_name, advisor_description, advisor_weight, parameters, advisor_active));
       }
     }
     
     ROS_DEBUG_STREAM("" << tier3Advisors.size() << " advisors registered.");
     for(unsigned i = 0; i < tier3Advisors.size(); ++i)
      	ROS_DEBUG_STREAM("Created advisor " << tier3Advisors[i]->get_name() << " with weight: " << tier3Advisors[i]->get_weight());

     //CONVEYORS = isAdvisorActive("WaypointFinderLinear");
     //REGIONS = isAdvisorActive("ExitFinderLinear");
     //TRAILS = isAdvisorActive("TrailLinear");
}



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Read from the config file and intialize robot parameters
//
//
void Controller::initialize_actions(string filename){
// robot intial position
// robot laser sensor range, span and increment
// robot action <-> semaFORR decision
}



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Read from the config file and intialize tasks
//
//
void Controller::initialize_tasks(string filename){
    string fileLine;
    std::ifstream file(filename.c_str());
    ROS_DEBUG_STREAM("Reading read_task_file:" << filename);
    //cout << "Inside file in tasks " << endl;
    if(!file.is_open()){
	  ROS_DEBUG("Unable to locate or read task config file!");
    }
    while(getline(file, fileLine)){
       //cout << "Inside while in tasks" << endl;
       if(fileLine[0] == '#')  // skip comment lines
          continue;
       else{
          std::stringstream ss(fileLine);
	  std::istream_iterator<std::string> begin(ss);
	  std::istream_iterator<std::string> end;
	  std::vector<std::string> vstrings(begin, end);
	  double x = atof(vstrings[0].c_str());
     	  double y = atof(vstrings[1].c_str());
	  beliefs->getAgentState()->addTask(x,y);
	  ROS_DEBUG_STREAM("Added task " << x << ", " << y);
       }
     }
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Initialize the controller and setup messaging to ROS
//
//
Controller::Controller(string advisor_config, string task_config, string action_config){

            // Initialize the agent's 'beliefs' of the world state with the map and nav
            // graph and spatial models
            beliefs = new Beliefs(120,120,2);

            // Initialize advisors and weights from config file
            initialize_advisors(advisor_config);

	    // Initialize the tasks from a config file
	    initialize_tasks(task_config);

	    // Initialize robot parameters from a config file
	    //initialize_actions(action_config);

	    // Initialize current task
	    beliefs->getAgentState()->setCurrentTask(beliefs->getAgentState()->getNextTask());
	    
	    tier1 = new Tier1Advisor(beliefs);
}


// Function which takes sensor inputs and updates it for semaforr to use for decision making, and updates task status
void Controller::updateState(Position current, sensor_msgs::LaserScan laser_scan){
      beliefs->getAgentState()->setCurrentSensor(current, laser_scan);
	//*********** Goal reached, switch task and learn spatial model from previous task ********************************
  	if (beliefs->getAgentState()->isTaskComplete()){
		ROS_DEBUG("Target Achieved!!");
		//Learn spatial model only on tasks completed successfully
		learnSpatialModel(beliefs->getAgentState());
		//Clear existing task
    		beliefs->getAgentState()->finishTask();
		if(beliefs->getAgentState()->getAgenda().size() > 0){
			beliefs->getAgentState()->setCurrentTask(beliefs->getAgentState()->getNextTask());
		}
		return;
	}
	//********************* Decision limit reached, skip task ***************************************  
  	if(beliefs->getAgentState()->getCurrentTask()->getDecisionCount() > 500){
		ROS_DEBUG("Controller.cpp decisionCount > 500 , skipping task");
    		beliefs->getAgentState()->skipTask();
		if(beliefs->getAgentState()->getAgenda().size() > 0){
			beliefs->getAgentState()->setCurrentTask(beliefs->getAgentState()->getNextTask());
		}
		return;
  	}
}


// Function which returns the mission status
bool Controller::isMissionComplete(){
 	return beliefs->getAgentState()->isMissionComplete();
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Main robot decision making engine, return decisions that would lead the robot to complete its mission
// Manages switching tasks and stops if the robot is taking too long
//
FORRAction Controller::decide() {
        ROS_DEBUG("Entering decision loop");
	return FORRDecision(); 
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Update spatial model after every task 
//
//

void Controller::learnSpatialModel(AgentState* agentState){
 Task* completedTask = agentState->getCurrentTask();
 vector<Position> *pos_hist = completedTask->getPositionHistory();
 vector< vector<CartesianPoint> > *laser_hist = completedTask->getLaserHistory();
 vector< vector<CartesianPoint> > all_trace = beliefs->getAgentState()->getAllTrace();
 vector< vector<CartesianPoint> > trails_trace = beliefs->getSpatialModel()->getTrails()->getTrailsPoints();
 bool trails = true;
 bool conveyors = true;
 bool regions = true;
 
  if(trails){
    	beliefs->getSpatialModel()->getTrails()->updateTrails(agentState);
	beliefs->getSpatialModel()->getTrails()->resetChosenTrail();
  }
  if(conveyors){
	beliefs->getSpatialModel()->getWaypoints()->populateGridFromPositionHistory(pos_hist);
  }
  if(regions){
	beliefs->getSpatialModel()->getAbstractMap()->learnRegions(pos_hist, laser_hist);
	beliefs->getSpatialModel()->getAbstractMap()->clearAllExits();
	beliefs->getSpatialModel()->getAbstractMap()->learnExits(all_trace);
	beliefs->getSpatialModel()->getAbstractMap()->learnExits(trails_trace);
  }
}



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// SemaFORR decision workflow
//
//
FORRAction Controller::FORRDecision()
{  
    ROS_DEBUG("In FORR decision");
    FORRAction *decision = new FORRAction();
    // Basic semaFORR three tier decision making architecture 
    
    if(!tierOneDecision(decision)){
	ROS_DEBUG("Decision to be made by t3!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
	//decision->type = FORWARD;
	//decision->parameter = 5;
	tierThreeDecision(decision);
	decision->decisionTier = 3;
    }
    cout << "decisionTier = " << decision->decisionTier << endl;
    ROS_DEBUG("After decision made");
    beliefs->getAgentState()->getCurrentTask()->incrementDecisionCount();
    ROS_DEBUG("After incrementDecisionCount");
    //beliefs->getAgentState()->getCurrentTask()->saveDecision(*decision);
    ROS_DEBUG("After saveDecision");
    beliefs->getAgentState()->clearVetoedActions();
    ROS_DEBUG("After clearVetoedActions");
    return *decision;
}



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Generate tier 1 decision
//
//
bool Controller::tierOneDecision(FORRAction *decision){
  //decision making tier1 advisor
  bool decisionMade = false;
  if(tier1->advisorVictory(decision)){ 
	ROS_INFO_STREAM("Advisor victory has made a decision " << decision->type << " " << decision->parameter);
	decision->decisionTier = 1;
	decisionMade = true;	
  }
  else{
  	// group of vetoing tier1 advisors which adds to the list of vetoed actions
	ROS_INFO("Advisor avoid wall will veto actions");
  	tier1->advisorAvoidWalls();
	ROS_INFO("Advisor not opposite will veto actions");
  	tier1->advisorNotOpposite();
  }
  set<FORRAction> *vetoedActions = beliefs->getAgentState()->getVetoedActions();
  std::stringstream vetoList;
  set<FORRAction>::iterator it;
  for(it = vetoedActions->begin(); it != vetoedActions->end(); it++){
    vetoList << it->type << " " << it->parameter << ";";
  }
  decision->vetoedActions = vetoList.str();
  cout << "vetoedActions = " << decision->vetoedActions << " " << vetoList.str() << endl;
  return decisionMade;
}



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Generate tier 3 decision
//
//
void Controller::tierThreeDecision(FORRAction *decision){
  std::map<FORRAction, double> comments;
  // This map will aggregate value of all advisers
  std::map<FORRAction, double> allComments;

  // typedef to make for declaration that iterates over map shorter
  typedef map<FORRAction, double>::iterator mapIt;

  // vector of all the actions that got max comment strength in iteration
  vector<FORRAction> best_decisions;
  
  double rotationBaseline, linearBaseline;
  for (advisor3It it = tier3Advisors.begin(); it != tier3Advisors.end(); ++it){
    Tier3Advisor *advisor = *it;
    //if(advisor->is_active() == true)
      //cout << advisor->get_name() << " : " << advisor->get_weight() << endl;
    if(advisor->get_name() == "RotationBaseLine") rotationBaseline = advisor->get_weight();
    if(advisor->get_name() == "BaseLine")         linearBaseline   = advisor->get_weight();
  }
       
  std::stringstream advisorsList;
  std::stringstream advisorCommentsList;
  cout << "processing advisors::"<< endl;
  for (advisor3It it = tier3Advisors.begin(); it != tier3Advisors.end(); ++it){
    Tier3Advisor *advisor = *it; 
    cout << advisor->get_name() << endl;
    // check if advisor should make a decision
    advisor->set_commenting();
    if(advisor->is_active() == false){
      cout << advisor->get_name() << " is inactive " << endl;
      continue;
    }
    if(advisor->is_commenting() == false){
      cout << advisor->get_name() << " is not commenting " << endl;
      continue;
    }

    advisorsList << advisor->get_name() << " " << advisor->get_weight() << " " << advisor->is_active() << " " << advisor->is_commenting() << ";";

    cout << "Before commenting " << endl;
    comments = advisor->allAdvice();
    cout << "after commenting " << endl;
    // aggregate all comments

    for(mapIt iterator = comments.begin(); iterator != comments.end(); iterator++){
      //cout << "comment : " << (iterator->first.type) << (iterator->first.parameter) << " " << (iterator->second) << endl;
      // If this is first advisor we need to initialize our final map
      float weight;
      //cout << "Agenda size :::::::::::::::::::::::::::::::::: " << beliefs->getAgenda().size() << endl;
      cout << "<" << advisor->get_name() << "," << iterator->first.type << "," << iterator->first.parameter << "> : " << iterator->second << endl; 
      weight = advisor->get_weight();
      cout << "Weight for this advisor : " << weight << endl;
      advisorCommentsList << advisor->get_name() << " " << iterator->first.type << " " << iterator->first.parameter << " " << iterator->second << ";";
      if( allComments.find(iterator->first) == allComments.end()){
	    allComments[iterator->first] =  iterator->second * weight;
      }
      else{
	    allComments[iterator->first] += iterator->second * weight;
      }
    }
  } 
  
  // Loop through map advisor created and find command with the highest vote
  double maxAdviceStrength = -1000;
  for(mapIt iterator = allComments.begin(); iterator != allComments.end(); iterator++){
    //cout << "Values are : " << iterator->first.type << " " << iterator->first.parameter << " with value: " << iterator->second << endl;
    if(iterator->second > maxAdviceStrength){
      maxAdviceStrength = iterator->second;
    }
  }
  //cout << "Max vote strength " << maxAdviceStrength << endl;
  
  for(mapIt iterator = allComments.begin(); iterator!=allComments.end(); iterator++){
    if(iterator->second == maxAdviceStrength)
      best_decisions.push_back(iterator->first);
  }
  
  //cout << "There are " << best_decisions.size() << " decisions that got the highest grade " << endl;
  if(best_decisions.size() == 0){
      (*decision) = FORRAction(PAUSE,0);
  }
  for(unsigned i = 0; i < best_decisions.size(); ++i)
      cout << "Action type: " << best_decisions.at(i).type << " parameter: " << best_decisions.at(i).parameter << endl;
    
  //generate random number using system clock as seed
  srand(time(NULL));
  int random_number = rand() % (best_decisions.size());
    
  (*decision) = best_decisions.at(random_number);
  decision->advisors = advisorsList.str();
  decision->advisorComments = advisorCommentsList.str();
  cout << " advisors = " << decision->advisors << "\nadvisorComments = " << decision->advisorComments << endl;
}



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Checks if an T3 advisor is active
//
//
bool Controller::
isAdvisorActive(string advisorName){
  bool isActive = false;
  
  for (advisor3It it = tier3Advisors.begin(); it != tier3Advisors.end(); ++it){
    Tier3Advisor *advisor = *it;
    if(advisor->is_active() == true && advisor->get_name() == advisorName)
      isActive = true;
  }
  
  return isActive;
}



