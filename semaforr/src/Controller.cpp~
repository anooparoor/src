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
    double advisor_weight;
    double parameters[4];
    ifstream file(filename.c_str());
    cout << "Inside file in read_advisor_file " << endl;
    //read advisor names and parameters from the config file and create new advisor objects
    while(!file.eof()){
       getline(file, fileLine);
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
	  parameters[0]= atof(vstrings[4].c_str());
     	  parameters[1] = atof(vstrings[5].c_str());
          parameters[2] = atof(vstrings[6].c_str());
          parameters[3] = atof(vstrings[7].c_str());
          tier3Advisors.push_back(Tier3Advisor::makeAdvisor(getBeliefs(), advisor_name, advisor_description, advisor_weight, parameters, advisor_active));
       }
     }
     
     cout << tier3Advisors.size() << " advisors registered." << endl;
     for(unsigned i = 0; i < tier3Advisors.size(); ++i)
      	cout << "Created advisor " << tier3Advisors[i]->get_name() << " with weight: " << tier3Advisors[i]->get_weight() << endl;

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
    ifstream file(filename.c_str());
    cout << "Inside file in tasks " << endl;
    while(!file.eof()){
       getline(file, fileLine);
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
            beliefs = new Beliefs();

            // Initialize advisors and weights from config file
            //initialize_advisors(advisor_config);

	    // Initialize the tasks from a config file
	    //initialize_tasks(task_config);

	    // Initialize robot parameters from a config file
	    //initialize_actions(action_config);
	    // create a dummy task for testing
	    beliefs->getAgentState()->addTask(20,19);
	    beliefs->getAgentState()->addTask(25,19);
	    beliefs->getAgentState()->addTask(30,19);
	    // Initialize currnent task
	    beliefs->getAgentState()->setCurrentTask(beliefs->getAgentState()->getNextTask());
	    
	    tier1 = new Tier1Advisor(beliefs);
}


// Function which takes sensor inputs and updates it for semaforr to use for decision making, and updates task status
void Controller::updateState(Position current, sensor_msgs::LaserScan laser_scan){
      beliefs->getAgentState()->setCurrentPosition(current);
      beliefs->getAgentState()->setCurrentLaserScan(laser_scan);
	//*********** Goal reached, switch task and learn spatial model from previous task ********************************
  	if (beliefs->getAgentState()->getDistanceToTarget() < 0.3){
		ROS_DEBUG("Target Achieved!!");
    		beliefs->getAgentState()->finishTask();
		return;
	}
	//********************* Decision limit reached, skip task ***************************************  
  	if(beliefs->getAgentState()->getCurrentTask()->getDecisionCount() > 250){
		ROS_DEBUG("Controller.cpp decisionCount > 250 , skipping task");
    		beliefs->getAgentState()->skipTask();
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
/*
void Controller::learnSpatialModel(){

 if(TRAILS){
	vector<double> wallDistance = beliefs->wallDistanceVector;
	CartesianPoint currPoint = CartesianPoint(beliefs->getCurrentPosition().getX(), beliefs->getCurrentPosition().getY());
	Task *task = beliefs->getCurrentTask();
	CartesianPoint targetPoint = CartesianPoint(task->getX() , task->getY());
	beliefs->trail_vectors.findNearbyTrail(targetPoint, currPoint, wallDistance);
  }
  beliefs->abstractMap.save(beliefs->wallDistanceVector, position);
  
  if(TRAILS){
    //if a trail has not been found yet 
    if(beliefs->trail_vectors.getChosenTrail() == -1){
      vector<double> wallDistance = beliefs->wallDistanceVector;
      CartesianPoint currPoint = CartesianPoint(beliefs->getCurrentPosition().getX(), beliefs->getCurrentPosition().getY());
      Task *task = beliefs->getCurrentTask();
      CartesianPoint targetPoint = CartesianPoint(task->getX() , task->getY());
      beliefs->trail_vectors.findNearbyTrail(targetPoint, currPoint, wallDistance);
    }
  }

  //  	beliefs->trail_vectors.printTrails();

  previousX = position.getX();
  previousY = position.getY();
  previousTheta = position.getTheta();
     // *********************** start of learning spacial cognition *****************

    if(TRAILS || CONVEYORS || REGIONS){

      cout << "Learning gates, exits and waypoints after finishing a task" << endl;
      gettimeofday(&learning_timer, NULL);
      learning_timer_before  = learning_timer.tv_sec*1000000 + learning_timer.tv_usec;
      
      //learn waypoints by reading from paths.conf
      if(CONVEYORS){
	beliefs->Waypoints.clearWaypoints();
	beliefs->Waypoints.populateGridFromFullLine();
      }
      // read from paths.conf
      beliefs->getRunTrace()->read_trace_from_file();
     
      if(REGIONS){
	beliefs->abstractMap.learnExits(beliefs->getRunTrace()->getAllTrace());
	beliefs->abstractMap.learnRegions(); 
      }

      
      if(TRAILS){
	cout << "Before update trails."<<endl;
	beliefs->trail_vectors.updateTrails();
	beliefs->trail_vectors.printTrails();
	cout << "After update trails."<<endl;
      }
      
      // log the learned values
      int interval = 10;
      if(beliefs->getAgenda().size()%interval == 0){
	ostringstream ostr;
	ostr << beliefs->getAgenda().size();
	string period = ostr.str();

	if(REGIONS)
	  (beliefs->abstractMap).saveCirclesIntoFile("regions_" + name + "_" + period  + ".conf");

	if(CONVEYORS)
	  beliefs->Waypoints.outputWaypoints("waypoints_" + name + "_" + period + ".conf");
      }
    }
}
*/


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
	decision->type = FORWARD;
	decision->parameter = 5;	
	//tierThreeDecision(decision);
    }
    
    //decision->type = RIGHT_TURN;
    //decision->parameter = 4;
    beliefs->getAgentState()->getCurrentTask()->incrementDecisionCount();
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
	decisionMade = true;	
  }
  else{
  	// group of vetoing tier1 advisors which adds to the list of vetoed actions
	ROS_INFO("Advisor avoid wall will veto actions");
  	tier1->advisorAvoidWalls();
  	//tier1->advisorNotOpposite();
  }
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
    if(advisor->is_active() == true)
      //cout << advisor->get_name() << " : " << advisor->get_weight() << endl;
    if(advisor->get_name() == "RotationBaseLine") rotationBaseline = advisor->get_weight();
    if(advisor->get_name() == "BaseLine")         linearBaseline   = advisor->get_weight();
  }
       
  cout << "processing advisors::"<< endl;
  for (advisor3It it = tier3Advisors.begin(); it != tier3Advisors.end(); ++it){
    Tier3Advisor *advisor = *it; 
    //cout << advisor->get_name() << endl;
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
    //cout << "Before commenting " << endl;
    comments = advisor->allAdvice();
    //cout << "after commenting " << endl;
    // aggregate all comments

    for(mapIt iterator = comments.begin(); iterator != comments.end(); iterator++){
      //cout << "comment : " << (iterator->first.type) << (iterator->first.parameter) << " " << (iterator->second) << endl;
      // If this is first advisor we need to initialize our final map
      float weight;
      //cout << "Agenda size :::::::::::::::::::::::::::::::::: " << beliefs->getAgenda().size() << endl;
      cout << "<" << advisor->get_name() << "," << iterator->first.type << "," << iterator->first.parameter << "> : " << iterator->second << endl; 
      weight = advisor->get_weight();
      cout << "Weight for this advisor : " << weight << endl;
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



