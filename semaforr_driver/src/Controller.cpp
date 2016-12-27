/*
 * Controller.cpp
 *
 */
          
#include "Controller.h"
#include "RobotControllerMessageHandler.h"
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
// Initialize the controller and setup messaging to ROS
//
//
Controller::Controller(ros::NodeHandle &nh){

   	    // Initialize ROS handle and ROS publish and subscribe handles
	    nh_ = nh;
	    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 100);
	    sub_laser_ = nh_.subscribe("laser_scan", 1000, updateLaserScan);
  	    sub_pose_ = nh_.subscribe("pose", 1000, updatePose);
  	    sub_crowd_pose_ = nh_.subscribe("crowd_pose", 1000, updateCrowdPose);
            destErrorThreshold = 7;

            // Initialize the agent's 'beliefs' of the world state with the map and nav
            // graph
            beliefs = new Beliefs();
	    
            // Initialize advisors and weights
            initilize_advisors();

	    // Initialize the tasks
	    initialize_tasks();

	    // Initialize robot parameters
	    initialize_robots();
}



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Read from the config file and intialize advisors and weights and spatial learning modules based on the advisors
//
//
void Controller::initialize_FORRAdvisors(string filename){

    advisors.push_back(&Controller::advisorVictory);
 
    string fileLine;
    string advisor_name, advisor_description;
    bool advisor_active;
    double advisor_weight;
    double parameters[4];
    cout << "Inside file in read_advisor_file " << endl;
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
	  if(args[2] == "t")
       		advisor_active = true;
     	  else
      		advisor_active = false;
	  parameters[0]= atof(args[4].c_str());
     	  parameters[1] = atof(args[5].c_str());
          parameters[2] = atof(args[6].c_str());
          parameters[3] = atof(args[7].c_str());
	  controller->add_advisor(Tier3Advisor::makeAdvisor(beliefs, advisor_name, advisor_description, advisor_weight, parameters, advisor_active));
       }
     }
     
     cout << tier3Advisors.size() << " advisors registered." << endl;
     for(unsigned i = 0; i < tier3Advisors.size(); ++i)
      	cout << "Created advisor " << tier3Advisors[i]->get_name() << " with weight: " << tier3Advisors[i]->get_weight() << endl;

     CONVEYORS = isAdvisorActive("WaypointFinderLinear");
     REGIONS = isAdvisorActive("ExitFinderLinear");
     TRAILS = isAdvisorActive("TrailLinear");
}



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Read from the config file and intialize robot parameters
//
//
void Controller::initialize_robots(){
// robot intial position
// robot laser sensor range, span and increment
// robot action <-> semaFORR decision
}



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Read from the config file and intialize tasks
//
//
void Controller::initialize_tasks(){
    string fileLine;
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
	  double x = atof(args[0].c_str());
     	  double y = atof(args[1].c_str());
	  beliefs->
       }
     }
}



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Callback function for laser_scan message
//
//
void Controller::updateLaserScan(){


}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Callback function for pose message
//
//
void Controller::updatePose(){


}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Callback function for crowd_pose message
//
//
void Controller::updateCrowdPose(){
 

}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Main robot control loop Sense -> decide -> return decision
//
//
void Controller::decide() {
      // this will call the callback function associated with pose, laser scan and crowd_pose
      ros::spinOnce();

      cycleCounter++;
      
        //********************* Decision limit reached, skip task ***************************************  
  	if(decisionCount > 250){
    		beliefs->finishTask();
    		wall_distance_vectors.clear();
    		beliefs->positionHistory = new vector<Position>;
    		cout << "Time out !! giving up on the target point" << endl;
    		beliefs->reset_visited_grid();
    		return;
  	}
  
  	//*********************** Goal reached, switch task and learn from previous task ********************************
  	if (beliefs->getDistanceToTarget() < 7){
    		beliefs->finishTask();
    		beliefs->reset_visited_grid();
    		beliefs->trail_vectors.resetChosenTrail();
    		//beliefs->visited_points.clear();
    		beliefs->positionHistory = new vector<Position>;
    		cout << "target point reached" << endl;
	}
      
      // Make decision for the current task
      FORRDecision();
}





//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Pushes advisor into the advisor stack 
//
//
void Controller::add_advisor(Tier3Advisor* advisor){
  tier3Advisors.push_back(advisor);
}








//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Receive message from \pose ros topic and based on the position of the robot and the task, setup next task
//
//
void Controller::updatePose(){
  cout << "Begining a new task" << endl;
  start = std::clock();
  
  CartesianPoint target_point = CartesianPoint(beliefs->getCurrentTask()->getX(), beliefs->getCurrentTask()->getY());
  
  //stores the lines created by the theta of the robot and the walldistance vectors
  //the first 2 values stored are the x,y coordinate of the robot
  vector<double> wallDistanceSegments;
  
  vector<double> wallDistance = beliefs->wallDistanceVector;
  
  wallDistanceSegments.push_back(previousX);
  wallDistanceSegments.push_back(previousY);
  double wall_distance;
  double rots[] =  {0, .1548, -.1548, .3048, -.3048, .65, -.65, 1.3, -1.3, 3.39, -3.39};
  rswl_log << "distancevectors : " << previousX << " " << previousY << " ";
  for(int i = 0; i < wallDistance.size(); i++){
    if(wallDistance[i] > 105) wall_distance = 105; else wall_distance = wallDistance[i]; // prevents distance of 1000
    Vector line_from_rotation(previousX, previousY, previousTheta + rots[i], wall_distance);
    CartesianPoint endpoint = line_from_rotation.get_endpoint();
    wallDistanceSegments.push_back(endpoint.get_x()); 
    rswl_log << endpoint.get_x() << " ";
    wallDistanceSegments.push_back(endpoint.get_y());
    rswl_log << endpoint.get_y() << " ";
  }
  rswl_log << endl;
  wall_distance_vectors.push_back(wallDistanceSegments);
  wallDistanceSegments.clear();
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Update spatial model after every task
//
//
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



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Update advisor weights after every task
//
//
void Controller::learnWeights(){   
    normalize_weights();
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// SemaFORR decision workflow
//
//
void Controller::FORRDecision()
{  
  bool decisionMade = false;
  FORRAction decision;
  // Basic semaFORR three tier decision making architecture 
    if(no previous plan){
  	if(tierOneDecision()){
	     return;
        }
	if(tierTwoDecision()){
	     return;
        }
  	tierThreeDecision();
    }
}



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Generate tier 1 decision
//
//
bool Controller::tierOneDecision(){
  advisor_avoid_walls(beliefs);

  for(int i = 2; i < 6; ++i)
    (beliefs->vetoedActions)->insert(FORRAction(PAUSE, i));
  (beliefs->vetoedActions)->insert(FORRAction(LEFT_TURN, 1));
  (beliefs->vetoedActions)->insert(FORRAction(RIGHT_TURN, 1));

  AvoidRobots avoidRobot;
  avoidRobot.avoid_robots(beliefs);
  //cout << "After invoking avoid robots." << endl;
  
  
  // Consult Tier-1 advisors in sequence************************
  cout << "Calling Tier 1 advisors .  Number of tier 1 advisors registered : " << advisors.size() << endl; 
  for (advisorFuncIt it = advisors.begin(); it != advisors.end(); ++it) {
    AdvisorFunc advisor = *it;
    // Weird syntax, but this actually calls the advisor function
    decisionMade = (this->*advisor)(beliefs);
    
    // If any cleanup/post-processing is required, do it here. Otherwise
    // just break.
    if(decisionMade) {
      decision = beliefs->getDecision();
      cout << "Decision about to be executed: " << decision.type << " , " << decision.parameter << endl;
      //cin.get();
      executeDecision(decision);
      return;
    }
    else {
      //if(CTRL_DEBUG) cout << "Advisor did NOT handle the condition." << endl;
    }
  }
  advisorNotOpposite(beliefs);
}



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Generate tier 3 decision
//
//

bool Controller::tierThreeDecision(){
  std::map<FORRAction, double> comments;
  // This map will aggregate value of all advisers
  std::map<FORRAction, double> allComments;

  // typedef to make for declaration that iterates over map shorter
  typedef map<FORRAction, double>::iterator mapIt;

  // vector of all the actions that got max comment strength in iteration
  vector<FORRAction> best_decisions;

  (beliefs->vetoedActions)->insert(FORRAction(LEFT_TURN, 5));
  (beliefs->vetoedActions)->insert(FORRAction(RIGHT_TURN, 5));
  // if decisionNo is odd then rotate else move forward decisionCount = odd => rotate, first move is to rotate
  if(decisionCount % 2 == 1){
    for(int i = 1; i < 6; ++i){
      (beliefs->vetoedActions)->insert(FORRAction(FORWARD, i));
      (beliefs->vetoedActions)->insert(FORRAction(PAUSE, 0));
    }
  }
  else{
    for(int i = 1; i < 6; ++i)
      (beliefs->vetoedActions)->insert(FORRAction(RIGHT_TURN, i));
    for(int i = 1; i < 6; ++i)
      (beliefs->vetoedActions)->insert(FORRAction(LEFT_TURN, i));
  }

  set<FORRAction> *va = beliefs->vetoedActions;
  FORRAction max_forward_move = get_max_allowed_forward_move(*va);
  rswl_log << "MaxForwardMove : " << max_forward_move.parameter << endl; 
  
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
      rswl_log << "<"<< advisor->get_name() <<",1,1> : notcommenting" << endl;
      continue;
    }
    //cout << "Before commenting " << endl;
    comments = advisor->allAdvice(*(beliefs->vetoedActions));
    //cout << "after commenting " << endl;
    // aggregate all comments

    for(mapIt iterator = comments.begin(); iterator != comments.end(); iterator++){
      //cout << "comment : " << (iterator->first.type) << (iterator->first.parameter) << " " << (iterator->second) << endl;
      // If this is first advisor we need to initialize our final map
      float weight;
      //cout << "Agenda size :::::::::::::::::::::::::::::::::: " << beliefs->getAgenda().size() << endl;
      cout << "<" << advisor->get_name() << "," << iterator->first.type << "," << iterator->first.parameter << "> : " << iterator->second << endl;
      rswl_log << "<" << advisor->get_name() << "," << iterator->first.type << "," << iterator->first.parameter << "> : " << iterator->second * weight << endl;

      if(!(advisor->get_name() == "BaseLine" or advisor->get_name() == "BaseLineRotation")){
	if(beliefs->getAgenda().size() > 30)
	  weight = 0.05;
	else{
	  weight = advisor->get_weight();
	  std::size_t found = advisor->get_name().find("Rotation");
	  if(found != std::string::npos and weight < rotationBaseline){
	    weight = 0;
	  }
	  else if(weight < linearBaseline){
	    weight = 0;
	  }
	}
	cout << "Weight for this advisor : " << weight << endl;
	if( allComments.find(iterator->first) == allComments.end()){
	  allComments[iterator->first] =  iterator->second * weight;
	}
	else{
	  allComments[iterator->first] += iterator->second * weight;
	}
      } 
    }
  } 
  
  //Slavisa comment: Tier 3 has to reach decision, I don't see a reason for this variable
  decisionMade = true;
  
  
  }
}



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Aggregate tier 3 comments
//
//

bool Controller::aggregateTierThreeComments(){

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
  
  // Slavisa: again this if is not necessary Tier3 has to make some kind of decision
  if ( decisionMade )
  {
    //cout << "There are " << best_decisions.size() << " decisions that got the highest grade " << endl;
    if(best_decisions.size() == 0){
      executeDecision(FORRAction(PAUSE,0));
      return;
    }
    for(unsigned i = 0; i < best_decisions.size(); ++i)
      cout << "Action type: " << best_decisions.at(i).type << " parameter: " << best_decisions.at(i).parameter << endl;
    
    //generate random number using system clock as seed
    srand(time(NULL));
    int random_number = rand() % (best_decisions.size());
    
    decision = best_decisions.at(random_number);
    executeDecision(decision);
    return;
}



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Checks if an advisor is active
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



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// normalize_weights : Is a function which normalizes weights received from the desciptive manager 
// 
//
void Controller::
normalize_weights(){
  double maxR = -1000, minR = 1000, max = -1000, min = 1000;
  // collect the max and min
  for(int i = 0 ; i < tier3Advisors.size() ; i++){
    Tier3Advisor *advisor = tier3Advisors[i];
    if(advisor->is_active() == true){
      //cout << "isWeightStable >>" << advisor->get_name() << " " << advisor->get_weight() << endl;
      std::size_t found = advisor->get_name().find("Rotation");
      if(found != std::string::npos){
	//cout << advisor->get_name() << "R" << endl; 
	if(advisor->get_weight() > maxR)         maxR = advisor->get_weight();
	else if(advisor->get_weight() <= minR)   minR = advisor->get_weight();
      }
      else{
	//cout << advisor->get_name() << "M" << endl;
	if(advisor->get_weight() > max)          max = advisor->get_weight();
	else if(advisor->get_weight() <= min)    min = advisor->get_weight();
      }
    }
  }
  
  //cout << maxR << " " << minR << " " << min << " " << max << endl;

  double norm_factorR = (maxR - minR)/10;
  double norm_factor = (max - min) / 10;

  // normalize
  for(int i = 0 ; i < tier3Advisors.size() ; i++){
    Tier3Advisor *advisor = tier3Advisors[i];
    if(advisor->is_active() == true){
      std::size_t found = advisor->get_name().find("Rotation");
      if(found != std::string::npos){
	if(maxR != minR){
	  advisor->reset_weight((advisor->get_weight() - minR)/norm_factorR);
	  //cout << "isWeightStable>>" << (advisor->get_weight() - minR)/norm_factorR << endl;
	} 
      }
      else{
	if(max != min){
	  advisor->reset_weight((advisor->get_weight() - min)/norm_factor);
	  //cout << "isWeightStable R>>" << (advisor->get_weight() - min)/norm_factor << endl;
	}
      }
    }
  }
}

void Controller::wait_for_response(){
  while(received_reply == false){
    //cout <<" waiting for response" << endl;
    message_handler->check_inbox();
  }
}




Position Controller::get_position() {

  return beliefs->getCurrentPosition();
}

double Controller::estimate_cost(int x1, int y1, int x2, int y2) {
  return pow((pow((x1-x2),2) + pow((y1-y2),2)),0.5);
}


double Controller::estimate_cost(Position p1, Position p2) {
  return this->estimate_cost(p1.getX(), p1.getY(), p2.getX(), p2.getY());
}


/********************************************************************
 *
 *                          UTIL functions
 *
 *********************************************************************/

bool Controller::isDestinationReached(int epsilon) {
    Position cP = itl->getPosition();

    int dest_x = beliefs->getCurrentTask()->getX();
    int dest_y = beliefs->getCurrentTask()->getY(); 

    if(Utils::get_euclidian_distance(dest_x, dest_y, cP.getX(), cP.getY()) < epsilon) {
        return true;
    }
    return false;
}



//returns the mean (average) of a set of numbers stored in a vector. Takes a vector of double
double Controller::calculateMean(std::vector<double> &v){
  double return_value = 0;
  for(unsigned int i = 0; i < v.size(); i++){
    return_value += v[i];
  }
  return return_value/v.size();   
}



//calculates the variance given a mean and a list of the numbers
double Controller::calculateVariance(double mean, std::vector<double> &v){
  double sum = 0;
  for(unsigned int i = 0; i < v.size(); i++){
    sum = sum + pow((mean - v[i]),2);
  }
  //divide by n-1 to account for error (Bessell's correction)
  sum = sum / (v.size()-1);
  return sum;
}


//calculates the range (highest - lowest).  Passes as value not reference because sort mutates array
pair<double,double> Controller::calculateRange(std::vector<double> v){
  std::sort(v.begin(), v.end());
  return pair<double,double> (v[0],v[v.size()-1]);
}



double Controller::calculateMedian(std::vector<double> v){
  std::sort(v.begin(),v.end());
  int vectorsize = v.size();
  if(vectorsize%2 == 0){
    return (v[vectorsize/2] + v[vectorsize/2 + 1])/2;
  }
  else
    {
      return v[vectorsize/2];
    }


}


//maps an integer index of  wallDistanceVector to corresponding rotation move to reach that rotation
FORRAction Controller::mapIntegerToRotationAction(int index){
  // {0, .1548, -.1548, .3048, -.3048, .65, -.65, 1.3, -1.3, 3.39, -3.39}
  switch(index){
  case 0:
    return FORRAction(FORWARD, 5);
  case 1:
    return FORRAction(LEFT_TURN, 1);
  case 2:
    return FORRAction(RIGHT_TURN, 1);
  case 3:
    return FORRAction(LEFT_TURN, 2);
  case 4:
    return FORRAction(RIGHT_TURN, 2);
  case 5:
    return FORRAction(LEFT_TURN, 3);
  case 6:
    return FORRAction(RIGHT_TURN, 3);
  case 7:
    return FORRAction(LEFT_TURN, 4);
  case 8:
    return FORRAction(RIGHT_TURN, 4);
  case 9:
    return FORRAction(LEFT_TURN, 5);
  case 10:
    return FORRAction(RIGHT_TURN, 5);
  default:
    return FORRAction();
  }

}
