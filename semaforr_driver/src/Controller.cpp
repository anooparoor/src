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

// For 02-2012 experiment
#define COOLDOWN_TIME 5
#define REST_SECONDS  5
#define UNIT_MOVE 10 
#define STOP_SECONDS  1000

    
// Slavisa added 
double previousX;
double previousY;
double previousTheta;
bool auctionHappened = false;
long decisionCount = 0;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Initialize the controller and setup messaging to ROS
//
//
Controller::Controller(ros::NodeHandle &nh){
	    nh_ = nh;
	    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 100);
	    sub_laser_ = nh_.subscribe("laser_scan", 1000, updateLaserScan);
  	    sub_pose_ = nh_.subscribe("pose", 1000, updatePose);
  	    sub_crowd_pose_ = nh_.subscribe("crowd_pose", 1000, updateCrowdPose);
            destErrorThreshold = 7;

            // Initialize the agent's 'beliefs' of the world state with the map and nav
            // graph
            beliefs = new Beliefs();
	    beliefs->initVectors();
	    beliefs->positionHistory = new vector<Position>;
    	    beliefs->hasMoreTargets = true;
	    beliefs->square_size = 60;
	    cout << "before initializing create_locations() " << endl;
            beliefs->create_locations();
	    beliefs->set_moved_across_pipe(false);
	    beliefs->positionHistory = new vector<Position>;
	    beliefs->initializeTrace();
	    //beliefs->abstractMap.readCirclesFromFile("circles.conf");
            //cout << "after initalization hasMoreTarget " << beliefs->hasTargets() << endl;
            received_reply = false;
	    
	    cout << "after initializing beliefs" << endl;

            // Declare the sequence of Tier 1/2 advisors
            declareAdvisors();

            CONVEYORS = isAdvisorActive("WaypointFinderLinear");
            REGIONS = isAdvisorActive("ExitFinderLinear");
            TRAILS = true;

  	    //initialize waypoint grid
            beliefs->Waypoints.setGranularity(40);
            beliefs->Waypoints.setGrid(map_width, map_height);
  
           //initialize Explorer grid
           beliefs->set_visited_grid(map_width, map_height);
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
    		distances.push_back(distanceTravelled);
	}
      
      // Make decision for the current task
      FORRDecision();
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Read from the config file and intialize advisor and weights
//
//

void Controller::initialize_FORRAdvisors(string filename){

  // Populate the list of advisors. See docs in Controller.h.
    advisors.push_back(&Controller::advisorHResume);

    advisors.push_back(&Controller::advisorHHalt);

    advisors.push_back(&Controller::advisorVictory);
 
    //advisors.push_back(&Controller::advisor_avoid_walls);

    cout << advisors.size() << "Tier 1 advisors registered." << endl;


    string fileLine;

    cout << "Inside file in read_advisor_file " << endl;
    while(!file.eof()){
       getline(file, fileLine);
       if(fileLine[0] == '#')  // skip comment lines
          continue;
       else{
          d_manager->add_new_description(fileLine);
          cout << "Adding description " << fileLine << endl;
       }
     }
     // cannot send message because Controller was not yet created
     cout << "Message supposed to be sent to DM " << endl;  
     std::vector<std::string> args = msg.get_args();
     cout << "size of message: " << args.size() << endl; 
     bool advisor_active;
  
     // check if there is no more data 
     if(args.size() == 8){
     // last one is bool
     double auxiliary_array[4]; 
     auxiliary_array[0]= atof(args[4].c_str());
     auxiliary_array[1] = atof(args[5].c_str());
     auxiliary_array[2] = atof(args[6].c_str());
     auxiliary_array[3] = atof(args[7].c_str());
     
     std::string advisor_name = args[0]; 
    
     if(args[2] == "t")
       advisor_active = true;
     else
      advisor_active = false;
     cout << "In ROBOTCONTROLLERMESSAGE handler before calling add_advisor" << endl;;
     vector<double> advisorWeight;
     advisorWeight.push_back(atof(args[3].c_str()));
     controller->add_advisor(Tier3Advisor::makeAdvisor(beliefs, args[0], args[1], advisorWeight, auxiliary_array, advisor_active));
  
     cout << tier3Advisors.size() << " advisors registered." << endl;
     for(unsigned i = 0; i < tier3Advisors.size(); ++i)
      	cout << "Created advisor " << tier3Advisors[i]->get_name() << " with weight: " << tier3Advisors[i]->get_weight() << endl;
}




//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Pushes advisor into the advisor stack 
//
//
void Controller::add_advisor(Tier3Advisor* advisor){
  tier3Advisors.push_back(advisor);
}




//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Read from the config file and intialize robot parameters
//
//
void Controller::initialize_robotparameters(){
// robot intial position
// robot laser sensor range, span and increment
// robot action <-> semaFORR decision
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



/******************************************************************************
 * FORR/Ariadne advisors
 *
 * Tier-1 advisors can make a decision and then stop the other advisors
 * from overriding their decisions. All advisors return a bool to state if they
 * decided to take an action. If an advisor,
 * returns true, all the advisors following it will be skipped and the decision
 * loop will move on to the next iteration.
 * returns false, the next advisor will be permitted to run
 *****************************************************************************/


/*******************************
 // This advisor check if the robot's is in rotation mode and if the previous position is same as the current position then 
    ban all rotation moves which are not in the same direction as the rotation mode.  
 *******************************/
void Controller::advisorNotOpposite(Beliefs *b){
  cout << "COntroller::advisorNotOpposite > Entering function" << endl;
  vector<FORRAction> actions = b->getPreviousDecisions();
  int size = actions.size();
  if(actions.size() < 2){
    cout <<"actions list less than 2. Exiting not opposite" << endl;
    return;
  }
  FORRAction lastAction = actions[size - 1];
  FORRAction lastlastAction = actions[size - 2];
  cout << "Controller::advisorNotOpposite > " << lastAction.type << " " << lastAction.parameter << ", " << lastlastAction.type << " " << lastlastAction.parameter << endl; 
  if(lastlastAction.type == RIGHT_TURN or lastlastAction.type == LEFT_TURN){
    if(lastAction.type == PAUSE){
      cout << "Not opposite active "<< endl;
      if(lastlastAction.type == RIGHT_TURN)    for(int i = 1; i < 6 ; i++)   (b->vetoedActions)->insert(FORRAction(LEFT_TURN, i));
      else                                     for(int i = 1; i < 6 ; i++)   (b->vetoedActions)->insert(FORRAction(RIGHT_TURN, i));
    }
  }
  cout << "leaving notOpposite"<<endl;
  return;
}


// This advisor should ban all forward moves not in the direction of the exit unless all of the exits
// are already blocked by other robots
void Controller::advisorCircle(Beliefs *b){

  // Initialize variables
  vector<FORRCircle> circles = (b->abstractMap).getCircles();
  Position curr_pos = b->getCurrentPosition();
  Task *task = b->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  bool targetInCircle = false;
  bool currPosInCircleWithExit = false;
  int robotCircle=-1, targetCircle = -1;
  //cout << "Controller::advisorCircle >> Initializing variables " << endl;
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

  // if preconditions are met, veto forward moves in the direction of non exits
  if(targetInCircle == true and currPosInCircleWithExit == true and robotCircle != targetCircle){
    //cout << "Controller::advisorCircle >> Activating tier 1 get out of circle advisor" << endl;
    vector<FORRExit> exits = circles[robotCircle].getExits(); 
    bool facingExit = false;
    double forward_distance = beliefs->wallDistanceVector[0];
    for(int i = 0; i< exits.size(); i++){
      CartesianPoint exitPoint = exits[i].getExitPoint();
      double exitDistance = Utils::get_euclidian_distance(exitPoint.get_x(), exitPoint.get_y(), curr_pos.getX() , curr_pos.getY());
      if(!(beliefs->abstractMap).isExitToLeaf(exits[i]) || exits[i].getExitCircle() == targetCircle){
	if( ( isFacing(curr_pos, exitPoint, circles[robotCircle].getRadius()) and forward_distance - exitDistance > 20 ) or exitDistance < 40 ){
	  //cout << "Controller::advisorCircle >> Robot is facing exit " << exitPoint.get_x() << exitPoint.get_y() << endl;
	  facingExit = true;
	  break;
	}
      }
    }
    // Robot is not facing any of the exits ban all forward moves
    if(facingExit == false){
      //cout << "Controller::advisorCircle >> Vetoing all forward moves" << endl;
      for(int i = 1; i < 6; ++i){
	(b->vetoedActions)->insert(FORRAction(FORWARD, i));
      }
    }
  }
  return;
}

bool Controller::isFacing(Position robotPos , CartesianPoint point, double radius){
  bool isFacing = false;
  double robot_point_angle = atan2(point.get_y() - robotPos.getY(), point.get_x() - robotPos.getX());
  double angleDiff = robotPos.getTheta() - robot_point_angle;
  //cout << "In function isFacing " << endl;
  //cout << "Robot angle " << robotPos.getTheta() << endl;
  //cout << "Angle made by the robot and the position with x axis" << robot_point_angle << endl;
  double ratio = Utils::get_euclidian_distance(robotPos.getX(), robotPos.getY() ,point.get_x(), point.get_y())/(2*radius);
  double min_delta = 0.72; //30 degrees on each side
  double max_delta = 3.14/(2); //60 degrees on each side
  double delta = ratio*(min_delta) + (1-ratio)*(max_delta) ;

  if(abs(angleDiff) < delta){
    isFacing = true;
  }
  return isFacing;
}



bool Controller::advisorVictory(Beliefs *b) {
  cout << "Begin victory advisor" << endl;
  // if the robot is oriented towards the goal and the robot actions which are not vetoed allows the robot to reach the goal then take that action.
  bool decisionMade = false;
  set<FORRAction> *vetoedActions = b->vetoedActions;
  
  FORRAction max_forward_move = get_max_allowed_forward_move(*vetoedActions);
  Position curr_pos = b->getCurrentPosition();
  Task *task = b->getCurrentTask();
  
  // Check if the current heading intersects with any of the walls
  Map *map = b->getMap();
  int buffer = 30;
  bool targetNotInSight1 = map->isPathObstructed(curr_pos.getX() - buffer, curr_pos.getY(), task->getX() - buffer, task->getY());
  bool targetNotInSight2 = map->isPathObstructed(curr_pos.getX() + buffer, curr_pos.getY(), task->getX() + buffer, task->getY());
  bool targetNotInSight3 = map->isPathObstructed(curr_pos.getX(), curr_pos.getY() - buffer, task->getX(), task->getY() - buffer);
  bool targetNotInSight4 = map->isPathObstructed(curr_pos.getX(), curr_pos.getY() + buffer, task->getX(), task->getY() + buffer);
  //cout << targetNotInSight1 << targetNotInSight2 << targetNotInSight3 << targetNotInSight4 << endl;
  bool targetNotInSight = targetNotInSight1 || targetNotInSight2 || targetNotInSight3 || targetNotInSight4;
  //bool targetNotInSight = map->isPathObstructed(curr_pos.getX(), curr_pos.getY(), task->getX(), task->getY()) || map->isPathCrossesBuffer(curr_pos.getX(), curr_pos.getY(), task->getX(), task->getY());
  //cout << targetNotInSight << endl;
  
  if(targetNotInSight == true){
    cout << "Target not in sight , skipping victory advisor" << endl;
    
  }
  else if(beliefs->inCollisionMode == true){
    cout << "In collision mode , hence skipping victory advisor" << endl;
  }
  else{
    number_of_decisions_tier1_with_pauses++;
    number_of_decisions_tier1_no_pauses++;
    cout << "Target in sight , victory advisor active" << endl;
    double distance_from_target = Utils::get_euclidian_distance(curr_pos.getX(), curr_pos.getY(), task->getX(), task->getY());
    cout << "Distance from target : " << distance_from_target << endl;
    // compute the angular difference between the direction to the target and the current robot direction
    double robot_direction = curr_pos.getTheta();
    double goal_direction = atan2((task->getY() - curr_pos.getY()), (task->getX() - curr_pos.getX()));
    //if(task->getX() >= curr_pos.getX())
    //goal_direction = goal_direction + 3.1415;
    //goal_direction = goal_direction - 3.1415;
    double required_rotation = goal_direction - robot_direction;
    if(required_rotation > 3.39)
      required_rotation = required_rotation - 6.283;
    if(required_rotation < -3.39)
      required_rotation = required_rotation + 6.283;
    //cout << "Robot direction : " << robot_direction << ", Goal Direction : " << goal_direction << ", Required rotation : " << required_rotation << endl;
    // if the angular difference is greater than smallest turn possible 
    // pick the right turn to allign itself to the target
    
    if(fabs(required_rotation) > 0.3048){
      //cout << "Rotation move made " << endl;
      if( required_rotation > 0.1548 && required_rotation <= 0.3048)
	b->setDecision(FORRAction(LEFT_TURN, 1));
      else if( required_rotation > 0.3048 && required_rotation <= 0.65)
	b->setDecision(FORRAction(LEFT_TURN, 2));
      else if( required_rotation > 0.65 && required_rotation <= 1.3)
	b->setDecision(FORRAction(LEFT_TURN, 3));
      else if(required_rotation > 1.3 && required_rotation <= 3.39)
	b->setDecision(FORRAction(LEFT_TURN, 4));
      else if( required_rotation < -0.1548 && required_rotation >= -0.3048)
	b->setDecision(FORRAction(RIGHT_TURN, 1));
      else if( required_rotation < -0.3048 && required_rotation >= -0.65)
	b->setDecision(FORRAction(RIGHT_TURN, 2));
      else if( required_rotation < -0.65 && required_rotation >= -1.3)
	b->setDecision(FORRAction(RIGHT_TURN, 3));
      else if(required_rotation < -1.3 && required_rotation >= -3.39)
	b->setDecision(FORRAction(RIGHT_TURN, 4));
      decisionMade = true;
      decisionCount += 1;
    }
    else if(max_forward_move.parameter == 5 || distance_from_target < get_move_length(get_max_allowed_forward_move(*vetoedActions))){
      int intensity;
      //cout << "Forward move made" << endl;
      if(distance_from_target <= 3)
	intensity = 0;
      else if(distance_from_target <= 7 )
	intensity = 1;
      else if(distance_from_target <= 20)
	intensity = 2;
      else if(distance_from_target <= 25)
	intensity = 3;
      else if(distance_from_target <= 105)
	intensity = 4;
      else
	intensity = 5;
      b->setDecision(FORRAction(FORWARD,intensity));
      decisionMade = true;
      cout << "Intensity of the forward move : " << intensity << endl;
    }
  }
  return decisionMade;
}

FORRAction Controller::get_max_allowed_forward_move(set<FORRAction> vetoedActions){
  FORRAction max_forward(FORWARD, 5);
  cout << " Number of vetoed actions : " << vetoedActions.size() << endl;
  for(int intensity = 1; intensity <= 5 ; intensity++){
    if(vetoedActions.find(FORRAction(FORWARD,intensity)) != vetoedActions.end()){
      max_forward.type = FORWARD;
      max_forward.parameter = intensity - 1;
      break;
    }
  }
  return max_forward;
}


double Controller::get_move_length(FORRAction forward_move){
  switch(forward_move.parameter){
  case 1:
    return 3;
  case 2:
    return 7;
  case 3:
    return 20;
  case 4:
    return 25;
  case 5: 
    return 105;
  default:
    return 0;
  }
}



bool Controller::advisorHResume(Beliefs *b) {
    const string signature = "Controller::advisorHResume()> ";

    if(b->getDoResume()) {
        if(CTRL_DEBUG)
            cout << signature << "HRESUME received" << endl;

        b->setDoHalt(false);
        b->setDoWait(false);
        b->setStopUntilTime(0);

        b->setDoResume(false);

        return true;
    }
    return false;
}


// Slavisa added Jan. 2014
/*
 * This advisor has to do prevent all the actions that will result in robot hitting the wall.
 * Straight forward thing is to check for collsion in the orientation.
 * Slightly more complicated is to take into an account that robot takes up some space and 
 * it can hit the wall although there is no visible collision near (almost parallel to the wall).
 * Have to make it work and yet be simple computationally.
 */
bool Controller::advisor_avoid_walls(Beliefs *beliefs){
  const int THRESHOLD1 = 30;
  const int THRESHOLD2 = 45;
  //cout << "Avoid Walls called " << endl;
  message_handler->send_get_veto_forward_moves_advisor_data();
  wait_for_response();
  double wall_distance = beliefs->wall_distance;
  //cout << "avoid_walls_advisor:: Received distance from wall descriptive " << wall_distance << endl;
  
  //initialize vetoedActions
  beliefs->vetoedActions = new set<FORRAction>; 
  
  if(wall_distance - 3 < THRESHOLD1){
    (beliefs->vetoedActions)->insert(FORRAction(FORWARD, 1));
    //cout << "Vetoed Forward move with intensity 1 :" << endl;
  }
  if(wall_distance - 7 < THRESHOLD1){
    (beliefs->vetoedActions)->insert(FORRAction(FORWARD, 2));
    //cout << "Vetoed Forward move with intensity 2"<< endl;
  }
  if(wall_distance - 20 < THRESHOLD1){
    (beliefs->vetoedActions)->insert(FORRAction(FORWARD, 3));
    //cout << "Vetoed Forward move with intensity 3"<< endl;
  }
  if(wall_distance - 25 < THRESHOLD2){
    (beliefs->vetoedActions)->insert(FORRAction(FORWARD, 4));
    //cout << "Vetoed Forward move with intensity 4"<< endl;
  }
  if(wall_distance - 105 < THRESHOLD2){
    (beliefs->vetoedActions)->insert(FORRAction(FORWARD, 5));
    //cout << "Vetoed Forward move with intensity 5"<< endl;
  }
  for(int i = 1; i < 6; ++i)
    (beliefs->vetoedActions)->insert(FORRAction(BACKWARD, i));
  // because it is annoying I have to do this manually
  //vetoed_actions.insert(FORRAction(RIGHT_TURN, 5));
  //vetoed_actions.insert(FORRAction(LEFT_TURN, 5));
  //cout << " return from wall advisor" << endl;
  return false; 
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
