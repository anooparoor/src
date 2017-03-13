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

// This is not so intuitive. A halt/wait takes a paremeter that indicates
// how long the robot stop for. For the most part we can't predict how long
// that should be; for example, a collision avoidance could take 2 seconds
// or 10 seconds to resolve. For now we'll make this parameter large enough
// that it indicates a "stop forever", until some future resume command
// overrides it.
#define STOP_SECONDS  1000

    
// Slavisa added 
double previousX;
double previousY;
double previousTheta;
bool auctionHappened = false;
long decisionCount = 0;

//unsigned int Task::next_task_id = 0;

Controller::Controller(string player_host, int player_port, Map *map,
                       int rbt_size, string rbt_type, string robot_name, int prox,
                       RobotControllerMessageHandler *message_handler)
  : localMap(map), robot_size(rbt_size), robot_type(rbt_type), name(robot_name),
    message_handler(message_handler) {

        try {
            pCli = new PlayerClient(player_host, player_port);

            usingStage = false;
            if ( robot_type == "surveyor" ) {
                itl = new Surveyor(localMap);
                itl->setBlobFinderProxy(pCli);
                itl->setCameraProxy(pCli);
            }
            else if ( robot_type == "Blackfin" ) {
                itl = new Surveyor(localMap);
                //itl->setBlobFinderProxy(pCli);
                //itl->setCameraProxy(pCli);
            }
            else if ( robot_type == "stage" ){
                itl = new Surveyor(localMap);
                itl->setPosition2dProxy(pCli);
                itl->setLocalizeProxy(pCli);
                usingStage = true;
            }
            else if ( robot_type == "roomba" ) {
                //itl = new Roomba(localMap);
            }
            itl->setPosition2dProxy(pCli);

            itl->setPlayerClient(pCli);
            // parameters

            destErrorThreshold = 7;

            /***************************************************************************
             * Ariadne/FORR initialization
             **************************************************************************/
	    cout << "before initializing beliefs" << endl;
            // Initialize the agent's 'beliefs' of the world state with the map and nav
            // graph
            beliefs = new Beliefs(localMap);
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

	    srand(time(NULL));
            cycleCounter = 0;
        }
        catch (PlayerError) {
            cerr << "Failed to establish a connection to the Player Server.\n"
                << "Robot: " << name << ", type:" << robot_type  << "\n"
                << "Player Server hostname: " << player_host << "\n"
                << "Player Server port: " << player_port << endl;
            exit(1);
        }
}
void Controller::operator()() {
  //cout << "In operator () ()"<<endl;

//initialize counter variables  
  cumulative_delay_time_elapsed = 0;
  cumulative_decision_time_elapsed = 0;
  decisions_made_per_target = 0;
  bool was_waiting = false;
  bool LOG_INFO = true;
  function_duration = 0;
  total_distance = 0;
  learning_time_cumulative = 0;
  int map_height = beliefs->getMap()->getHeight();
  int map_width = beliefs->getMap()->getLength();
  number_of_decisions_tier1_with_pauses = 0;
  number_of_decisions_tier3_with_pauses = 0;
  number_of_decisions_tier1_no_pauses = 0;
  number_of_decisions_tier3_no_pauses = 0;
  
  
  GATES = isAdvisorActive("GateFinderLinear");
  CONVEYORS = isAdvisorActive("WaypointFinderLinear");
  REGIONS = isAdvisorActive("ExitFinderLinear");
  TRAILS = true;
 
  //initialize starting position
  //initialize gates

  //initialize waypoint grid
  beliefs->Waypoints.setGranularity(40);
  beliefs->Waypoints.setGrid(map_width, map_height);

  
  //initialize Explorer grid
  beliefs->set_visited_grid(map_width, map_height);
  added_parking_position = false;
  
   //open log file
  if(LOG_INFO){
    string filename_log = "SemaFORR.NUMBOTS.MAP."+get_robot_name()+".TRIALNUM.TARGETS.log";
    log_stream.open(filename_log.c_str());
      //    if(log_stream.is_open()){
      //	cout<<"Successfully opened time and distance log file: "<<filename_log<<endl;
	//      }
  }
  log_stream << "TARGET-TIME,PER-TASK-DECISION-TIME,TASK-DISTANCE,TASK-DELAY-TIME, NUM-DECISIONS, LEARNING-TIME"<<endl;
  // enter main loop
    while (true) {

        // Update the robot. This is the stage where it updates sensor input and
        // receive central server messages.
      //cout <<"In operator loop" << endl;
      if((beliefs->getAgenda().size()!=0) && (was_waiting == false)){
	
       gettimeofday(&tv,NULL);
       start_time = tv.tv_sec * 1000000 + tv.tv_usec;
	was_waiting = true;
	//before any cycles happen, record the starting position as the parking position
	parking_position = itl->getPosition();
	
      }
        cycleCounter++;

        pCli->ReadIfWaiting();
        itl->update();

        if ( usingStage )
	      itl->updateStagePosition();
	
        
	updateBehavior();
        message_handler->check_inbox();

        // Take a quick breath (don't hog the CPU!).
        usleep(1000);
    }
}

void Controller::updateBehavior() {
    currPos = itl->getPosition();
    
    move_completed = itl->isMoveCompleted();
    //cout << "In update behaviour loop" << endl;

    // Update the robot's current position in its Beliefs structure
    
    if(move_completed)
      previousPosition = beliefs->getCurrentPosition();
    
    beliefs->setCurrentPosition(currPos);
    gettimeofday(&clock_time,NULL);
    clock_sample_before_  = clock_time.tv_sec*1000000 + clock_time.tv_usec;

    // The new, FORR-based action-decision function
    // -ES 11/25
    
    //cout << currPos.getX() << " "  << currPos.getY() << " " << currPos.getTheta() << endl;

    FORRDecision();
    
    gettimeofday(&clock_time,NULL);
    clock_sample_after_ = clock_time.tv_sec * 1000000 + clock_time.tv_usec;
    function_duration = clock_sample_after_ - clock_sample_before_;
    if(move_completed && (beliefs->getAgenda().size()!=0)){
    cumulative_decision_time_elapsed += function_duration;
    per_decision_times.push_back(function_duration);
    }
}



/*******************************************************************************
 * Forr/Ariadne functions
 ******************************************************************************/
//read advisor from a config file and instantiate them with the weights
void Controller::initialize_FORRAdvisors(){
    bool advisor_active;
  
    // check if there is no more data 
    if(args.size() == 8){
    // instantiate advisor with this data
    // first two arguments are strings so no problem with them 
    // next is float in advisor weight then 4 floats for auxiliary array 
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
    
    vector<double> advisorWeight;
    advisorWeight.push_back(atof(args[3].c_str()));
    controller->add_advisor(Tier3Advisor::makeAdvisor(beliefs, args[0], args[1], advisorWeight, auxiliary_array, advisor_active)); // 
  
    cout << tier3Advisors.size() << " advisors registered." << endl;
    for(unsigned i = 0; i < tier3Advisors.size(); ++i)
    cout << "Created advisor " << tier3Advisors[i]->get_name() << " with weight: " << tier3Advisors[i]->get_weight() << endl;
}


void Controller::add_advisor(Tier3Advisor* advisor){
  tier3Advisors.push_back(advisor);
}

void Controller::declareAdvisors() {

    // Populate the list of advisors. See docs in Controller.h.
    advisors.push_back(&Controller::advisorHResume);

    advisors.push_back(&Controller::advisorHHalt);

    advisors.push_back(&Controller::advisorVictory);
 
    // Slavisa: this is postponed till I figure out how to do this as Tier1
    // advisor that is defined like this.
    //advisors.push_back(&Controller::advisor_avoid_walls);

    cout << advisors.size() << "Tier 1 advisors registered." << endl;
}

void Controller::executeDecision(FORRAction decision) {
  cout << "In execute decision" << endl;
    char param_char = '0' + decision.parameter;
    Beliefs* beliefs = getBeliefs();
    beliefs->addDecision(decision);
    
    switch ( decision.type ) {
        case NOOP:
            break;
        case FORWARD:
            itl->motion('F', param_char);
            break;
        case BACKWARD:
            itl->motion('B', param_char);
            break;
        case RIGHT_TURN:
            itl->motion('c',  param_char);
            break;
        case LEFT_TURN:
            itl->motion('a',  param_char);
            break;
        case WIDE_RIGHT_TURN:
            itl->motion('C',  param_char);
            break;
        case WIDE_LEFT_TURN:
            itl->motion('A', param_char);
            break;
        case PAUSE:
	    itl->motion('H', '0');
            break;
        case HALT:

            // Reset destination
            resetDestination();

            // Finally, stop the robot
            itl->motion('H', '0');

            break;
    }
}

void Controller::FORRDecision()
{
  //cout << "in FORRDecision" << endl;
  Position position;
  Position prev_position;
  CartesianPoint new_point, old_point;
  // ******************************wait for the robot to complete executing the previous action *********************
  if(itl->isMoveCompleted()){
    position = itl->getPosition();
    new_point = CartesianPoint(position.getX(), position.getY());
    // cout << "move completed. current position :" << position.getX() << "  " << position.getY() << " " << position.getTheta() << endl;
  
    //set in updateBehavior before FORRDecision is called
    old_point = CartesianPoint(previousPosition.getX(), previousPosition.getY());
    
    // keep track of the trace tempory hack need to work on a better structure later or may be not
    beliefs->update(old_point, new_point, previousPosition.getTheta());
  }   
  else
    return;
  // *********************************** end ***************************************************
  

  //************************************** initialization of variable and the log files **********************************************
  
  
  std::ofstream rswl_log;
  std::ofstream results_log;
  
  stringstream ss;
  ss << (int)message_handler->get_session_id();
  string robot_id_str = ss.str();
  const char *path = robot_id_str.c_str();
  rswl_log.open(path,std::ofstream::out | std::ofstream::app);
  //rswl_log.open(path, std::ofstream::out);
  // count the number of decisions made
  
  static long totalDecisionCount = 0;
  
  // time taken to complete a task
  static double timeTaken = 0;
  static double totalTimeTaken = 0;
  static std::clock_t start;
  
  // Distance travelled during the completion of a task
  static double distanceTravelled = 0;
  static double totalDistanceTravelled = 0;
  
  bool decisionMade = false;
  FORRAction decision;
  
  // ************************************************** end initialization ******************************************************
  

  // ***************** Get a new goal if there is none ***********************************/
  if(beliefs->getDoWait())
    return;
  
  list<Task*> agenda = beliefs->getAgenda();
  if(agenda.size() == 0 && auctionHappened == false){
    //cout << "nothing in the task list, waiting for an auction" << endl;
    return;
  }
  

  //if reached end of agenda, at the starting spot (the "parking spot") to the agenda and make that
  //the last point
  if((agenda.size() == 0) && !added_parking_position){
    Task * t = new Task();
    t->setAccessPoint(make_pair(parking_position.getX(), parking_position.getY()));
    beliefs->addTask(t);
    added_parking_position = true;
    gettimeofday(&tv,NULL);
    end_time = tv.tv_sec * 1000000 + tv.tv_usec;
    
    //agenda done.  calculate statistical measures
    pair<double, double> decision_range = calculateRange(decision_times);
    pair<double, double> distance_range = calculateRange(distances);
    pair<double, double> task_range = calculateRange(task_times);

    double decision_mean = calculateMean(decision_times);
    double distance_mean = calculateMean(distances);
    double task_mean = calculateMean(task_times);
    double per_decision_mean = calculateMean(per_decision_times);
    double learning_time_mean = calculateMean(learning_times);

    log_stream<<"TOTAL-RUN-TIME(S),"<<(end_time-start_time)/1000000.0<<endl<<endl;
    log_stream<<"TOTAL-DISTANCE,"<<total_distance<<endl;
    log_stream<<"DECISION-TIME-MEAN,"<<decision_mean<<endl;
    log_stream<<"DECISION-TIME-RANGE,["<<decision_range.first<<","<<decision_range.second<<"]"<<endl;
    log_stream<<"DECISION-TIME-STD-DEV,"<<sqrt(calculateVariance(decision_mean, decision_times))<<endl<<endl;
    log_stream<<"PER-DECISION-TIME-STD-DEV,"<<sqrt(calculateVariance(per_decision_mean, per_decision_times)) << endl; 
    log_stream<<"DISTANCE-MEAN,"<<distance_mean<<endl;
    log_stream<<"DISTANCE-RANGE,"<<distance_range.first<<","<<distance_range.second<<"]"<<endl;
    log_stream<<"DISTANCE-STD-DEV,"<<sqrt(calculateVariance(distance_mean, distances))<<endl<<endl;
    log_stream<<"TASK-TIME-MEAN,"<<task_mean<<endl;
    log_stream<<"TASK-TIME-RANGE,["<<task_range.first<<","<<task_range.second<<"]"<<endl;
    log_stream<<"TASK-TIME-STD-DEV,"<<sqrt(calculateVariance(task_mean, task_times));
    log_stream<<"LEARNING-TIME-STD-DEV,"<<(sqrt(calculateVariance(learning_time_mean, learning_times)))/1000000.0<<endl;
    log_stream<<"RATIO OF TIER 1 TO TOTAL DECISIONS WITH PAUSES: ,"<<((1.0 * number_of_decisions_tier1_with_pauses) / 
							  (number_of_decisions_tier1_with_pauses +
							   number_of_decisions_tier3_with_pauses));
     log_stream<<"RATIO OF TIER 1 TO TOTAL DECISIONS NO PAUSES: ,"<<((1.0 * number_of_decisions_tier1_no_pauses) / 
							  (number_of_decisions_tier1_no_pauses +
							   number_of_decisions_tier3_no_pauses));
    log_stream.close();

    return;
  }
  
 
  if((agenda.size() == 0) && added_parking_position){
    robot_parking_routine();
    exit(0);
  }
  else{
    auctionHappened = true;
    if(beliefs->getCurrentTask() == NULL){
      beliefs->setCurrentTask(beliefs->getNextTask());
      
      cout << "Begining a new task" << endl;
      start = std::clock();
      cout << "Initializing FORRAction to default halt, and saving the target point in the DM" << endl;
      
      beliefs->addDecision(FORRAction(HALT,0));
      CartesianPoint target_point = CartesianPoint(beliefs->getCurrentTask()->getX(), beliefs->getCurrentTask()->getY());
      cout << target_point.get_x() << " " << target_point.get_y() << endl;
      message_handler->send_set_current_target(target_point);
      //beliefs->current_target = target_point;
      //added 9/22/2014 for logging info
      gettimeofday(&task_timer, NULL);

      if(TRAILS){
	
	vector<double> wallDistance = beliefs->wallDistanceVector;
	  
	CartesianPoint currPoint = CartesianPoint(beliefs->getCurrentPosition().getX(), beliefs->getCurrentPosition().getY());
	Task *task = beliefs->getCurrentTask();
	CartesianPoint targetPoint = CartesianPoint(task->getX() , task->getY());
	beliefs->trail_vectors.findNearbyTrail(targetPoint, currPoint, wallDistance);
	
      }



      task_timer_before  = task_timer.tv_sec*1000000 + task_timer.tv_usec;




    }
  }
  
  //************************************** end *********************************************************
 
  
  // ********************************************* sense from DM or get local values ****************************************
  message_handler->send_get_distance_to_target();
  wait_for_response();
  message_handler->send_get_distance_from_walls();
  wait_for_response();
  message_handler->send_get_team_pose();
  wait_for_response();
  beliefs->positionHistory->push_back(position);
  //compute sum of distances from neighbours within some range after moving called robot_distance_vector

  beliefs->abstractMap.save(beliefs->wallDistanceVector, position);
  
  
  if(decisionCount > 0){
    distanceTravelled = distanceTravelled + sqrt((previousX - position.getX())*(previousX - position.getX()) + (previousY - position.getY())*(previousY - position.getY()));
  }
  
  if(distanceTravelled > 0){
    beliefs->update(CartesianPoint(position.getX(), position.getY()), CartesianPoint(previousX, previousY), position.getTheta());
  }

  if(previousX == position.getX() && previousY == position.getY() && previousTheta == position.getTheta() && beliefs->getPreviousDecisions().back().type != PAUSE){
    cout << "HELP!!!!!!!!!!!!!!! STUCK" << endl;
    ofstream wallCollision;
    if(beliefs->getPreviousDecisions().back().type == FORWARD){
      wallCollision.open("wallCollision.log",std::ofstream::out|std::ofstream::app);
      wallCollision << "previous position: " << previousX << "," << previousY << "  " << previousTheta << endl;
      wallCollision << "Current Position : " << position.getX() << " " << position.getY() << endl;
      wallCollision << "Action taken : " << beliefs->getPreviousDecisions().back().type << "," <<  beliefs->getPreviousDecisions().back().parameter << endl;
      wallCollision.close();
    }
  }
  
  //learn gates
  if(GATES){
  if((previousX != 0) && (previousY != 0)){
    gettimeofday(&learning_timer, NULL);
    learning_timer_before  = learning_timer.tv_sec*1000000 + learning_timer.tv_usec;
    beliefs->gates.learn_gate(previousX, previousY, position.getX(), position.getY(), beliefs->getMap()->getLength(), 
			      beliefs->getMap()->getHeight());
  

    gettimeofday(&learning_timer, NULL);
    learning_timer_after  = learning_timer.tv_sec*1000000 + learning_timer.tv_usec;
    learning_time_cumulative += (learning_timer_after - learning_timer_before);
  }
  }

 
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


  //********************************** record the position in the log*********************************

  ++decisionCount;
  rswl_log << "<decision>" << endl;
  rswl_log << "decisionId : " << decisionCount << endl;
  rswl_log << "decisionTier : 3" << endl;
  CartesianPoint targetPoint = CartesianPoint(beliefs->getCurrentTask()->getX(), beliefs->getCurrentTask()->getY());
  rswl_log << "targetX : " << targetPoint.get_x() << endl;
  rswl_log << "targetY : " << targetPoint.get_y() << endl;
  rswl_log << "RobotLocationX : " << previousX << endl;
  rswl_log << "RobotLocationY : " << previousY << endl;
  rswl_log << "RobotOrientation : " << previousTheta << endl;

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
  

  
  // current_path_wall_vectors.push_back(wallDistanceSegments);


  /*
  rswl_log << "unshortened distance vectors: "<< previousX << " " << previousY << " ";
  for(int i = 0; i < wallDistance.size(); i++){
    Vector line_from_rotation(previousX, previousY, previousTheta + rots[i], wallDistance[i]);
    CartesianPoint endpoint = line_from_rotation.get_endpoint();
    rswl_log << endpoint.get_x() << " ";
    rswl_log << endpoint.get_y() << " ";
  }

  rswl_log << endl;
  */
  //for each wall distance vector, calculate the endpoint using the Vector class
  
  wall_distance_vectors.push_back(wallDistanceSegments);
  wallDistanceSegments.clear();
  
  //rswl_log << " Th: " << previousTheta << endl;

  //******************************************** end logging ***************************************  
  

  if(decisionCount > 250){
    beliefs->finishTask();
    //rswl_log.open(path,std::ofstream::out | std::ofstream::trunc);
    rswl_log.close();
   
    wall_distance_vectors.clear();
    beliefs->positionHistory = new vector<Position>;
    decision_times.push_back(cumulative_decision_time_elapsed/1000000.0);
    gettimeofday(&task_timer, NULL);
    task_timer_after  = task_timer.tv_sec*1000000 + task_timer.tv_usec;
    
    log_stream<<((task_timer_after - task_timer_before)/1000000.0)<<","<<cumulative_decision_time_elapsed/1000000.0
	      <<","<<distanceTravelled<<","<<cumulative_delay_time_elapsed<<","<<decisionCount<<","<<
      learning_time_cumulative/1000000.0<<", FAILED"<<endl;
    
    decisionCount = 0;
    total_distance += distanceTravelled;
    distanceTravelled = 0;
    timeTaken = 0;
    cumulative_decision_time_elapsed = 0;
    learning_time_cumulative = 0;
    cout << "Time out !! giving up on the target point" << endl;
    beliefs->reset_visited_grid();
    return;
  }
  
  //***************************  goal test , logging and learing weights ********************************
  
  if (beliefs->getDistanceToTarget() < 7){
    
    beliefs->finishTask();
    beliefs->reset_visited_grid();
    beliefs->trail_vectors.resetChosenTrail();
    //beliefs->visited_points.clear();
    beliefs->positionHistory = new vector<Position>;
    cout << "target point reached" << endl;
    
    distances.push_back(distanceTravelled);
    // ***************************** weight learning ************
    rswl_log << "goalState : TRUE " << endl;
    rswl_log << "</decision>" << endl;
    //beliefs->create_locations();
    rswl_log.close();
    
    //copy from rswl log to a log with filename as robot name;
    ifstream in(path);
    ofstream out(get_robot_name().c_str());
    string str;
    while(getline(in, str)){
      out<<str;
      out<<endl;
    } 
    in.close();
    out.close();
      
    ofstream wall_distance_stream;
    wall_distance_stream.open("wallvectors.conf", std::ofstream::app);
    
    
    //output wall distance vectors to log file for path-cleaning
    for(int i = 0; i < wall_distance_vectors.size(); i++){
      for(int j = 0; j < wall_distance_vectors[i].size(); j++){
	wall_distance_stream << wall_distance_vectors[i][j] << " ";
      }
      wall_distance_stream << endl;
    }

    wall_distance_stream.close();
    wall_distance_vectors.clear();
    
    /*
    vector<double> wallDistance = beliefs->wallDistanceVector;
    CartesianPoint c = CartesianPoint(beliefs->getCurrentPosition().getX(), beliefs->getCurrentPosition().getY());
    pair< vector <double>, CartesianPoint> p = make_pair(wallDistance, c);
    beliefs->endDistanceVectors.push_back(p);
    */


    
    
    // send weights to the descriptive manager
    gettimeofday(&learning_timer, NULL);
    learning_timer_before  = learning_timer.tv_sec*1000000 + learning_timer.tv_usec;
    
    message_handler->send_get_advisor_weight();
    wait_for_response(); 

    gettimeofday(&learning_timer, NULL);
    learning_timer_after  = learning_timer.tv_sec*1000000 + learning_timer.tv_usec;
    learning_time_cumulative += (learning_timer_after - learning_timer_before);
  

    // update new weights and send it to the descriptive manager
    int counter = 0; 
    for(advisor3It it = tier3Advisors.begin(); it != tier3Advisors.end(); ++it){
      Tier3Advisor *advisor = *it;
      if(advisor->is_active() == true){
	advisor->set_weight((beliefs->advisorWeight).at(counter)); 
	counter++;
      }
    }

    normalize_weights();

    ofstream myfile;
    myfile.open(("weights_"+get_robot_name()+".conf").c_str(), ios::out | ios::app);
    
    myfile << "--------------------------" << endl;
    for (int i = 0; i < tier3Advisors.size(); i++){
      //weights[i] = weights[i]/commentCount[i];
      Tier3Advisor *advisor = tier3Advisors[i];
      myfile << advisor->get_weight() << " : ";
    }
    myfile << endl;
    myfile.close();
    
    // clear the rswl file --------
    rswl_log.open(path,std::ofstream::out | std::ofstream::trunc);
    rswl_log.close();
    // *********************** end of weight learning *********


    // *********************** start of learning spacial cognition *****************

    if(GATES || CONVEYORS || REGIONS){

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

      
      

      if(GATES){
	beliefs->clear_gates_in_circles();
	beliefs->clear_gates_far_from_paths("paths.conf");
      }
     
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
	if(GATES)
	  beliefs->gates.write_gates_to_file("gates_" + name + "_" + period  + ".conf");

	if(REGIONS)
	  (beliefs->abstractMap).saveCirclesIntoFile("regions_" + name + "_" + period  + ".conf");

	if(CONVEYORS)
	  beliefs->Waypoints.outputWaypoints("waypoints_" + name + "_" + period + ".conf");
      
	
	
	gettimeofday(&learning_timer, NULL);
	learning_timer_after  = learning_timer.tv_sec*1000000 + learning_timer.tv_usec;
	learning_time_cumulative += (learning_timer_after - learning_timer_before);
     
	learning_times.push_back(learning_time_cumulative);
     

	
      }
     
    }
    // **************************** end of spacial cognition code ******************************

    // log_stream << "TASK-DECISION-TIME-ELAPSED(S): "<<cumulative_decision_time_elapsed/1000000.0<<endl;
    if(!added_parking_position){
    decision_times.push_back(cumulative_decision_time_elapsed/1000000.0);
    gettimeofday(&task_timer, NULL);
    task_timer_after  = task_timer.tv_sec*1000000 + task_timer.tv_usec;
    //log_stream <<"TOTAL-TASK-TIME(S): "<< ((task_timer_after - task_timer_before)/1000000.0)<<endl;
    log_stream<<((task_timer_after - task_timer_before)/1000000.0)<<","<<cumulative_decision_time_elapsed/1000000.0
	      <<","<<distanceTravelled<<","<<cumulative_delay_time_elapsed<<","<<decisionCount<<","<<learning_time_cumulative/1000000.0
	      <<endl;
    task_times.push_back((task_timer_after - task_timer_before)/1000000.0);

    total_distance += distanceTravelled;
    
    //reset counters
    timeTaken = 0;
    decisionCount = 0;
    distanceTravelled = 0;
    cumulative_decision_time_elapsed = 0;
    learning_time_cumulative = 0;
    }
    //exit(0);
    return;
  }
  else
    rswl_log << "goalState : FALSE" << endl; 
  // *****************************************************  end goal test ************************************
  

  
  // ************************************************** decide and act **************************************************
  // Consult Tier 1 advisors which knows which moves are wrong and takes them out of consideration
  //cout << "Begin consulting tier 1 advisors : wall and robot avoid" << endl;
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
  //advisorCircle(beliefs);
  
  // End of tier 1 consultation ********************************

  cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Decision No : " << decisionCount << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << endl; 
  // Consult Tier 3 advisors in sequence***********************
  // This map will store comments of individual adviser
  
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
      number_of_decisions_tier3_no_pauses++;
      number_of_decisions_tier1_with_pauses++;
      executeDecision(FORRAction(PAUSE,0));
      return;
    }
    for(unsigned i = 0; i < best_decisions.size(); ++i)
      cout << "Action type: " << best_decisions.at(i).type << " parameter: " << best_decisions.at(i).parameter << endl;
    //generate random number using system clock as seed
    //srand(time(NULL));
    int random_number = rand() % (best_decisions.size());
    
    decision = best_decisions.at(random_number);
    rswl_log<< "ActionChosen : <" << decision.type << "," << decision.parameter << ">" << endl;
    cout << "Action Chosen: " << decision.type << "," << decision.parameter << endl; 
    //execute decision if not a pause
    //if(decision.type != PAUSE)
   
    //  cin.get();
    number_of_decisions_tier3_with_pauses++;
    number_of_decisions_tier3_no_pauses++;
    executeDecision(decision);

    rswl_log << "</decision>" << endl;
    rswl_log.close();
    return;
  }
  //*********************************************************** end of decide and act ***********************************************
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


bool Controller::advisorHHalt(Beliefs *b) {
    const string signature = "Controller::advisorHHalt()> ";

    if(!b->getDoHalt())
        return false;

    time_t now = time(NULL);

    // This is the first time we are aware that the robot has been halted
    if(b->getStopUntilTime() == 0) {
        if(CTRL_DEBUG)
            cout << signature << "HHALT received, stopping..." << endl;

        // Stop the robot
        stop();

        // Set the current position as the planner's source so that it can replan
        Position currentPosition = b->getCurrentPosition();

        // b->setStopUntilTime(now + STOP_SECONDS);
        if(b->getStopSeconds() > 0) {
            b->setStopUntilTime(now + b->getStopSeconds());
            // reset stopSeconds
            b->setStopSeconds(-1);
        }
        else {
            b->setStopUntilTime(INT_MAX);
        }
    }
    else if(b->getStopUntilTime() <= now) {
        if(CTRL_DEBUG)
            cout << signature << "Halt time expired, resuming..." << endl;

        Position currentPosition = b->getCurrentPosition();

        b->setStopUntilTime(0);
        b->setDoHalt(false);
    }

    return true;
}

bool Controller::advisorHWait(Beliefs *b) {
    const string signature = "Controller::advisorHWait()> ";

    if(!b->getDoWait())
        return false;

    time_t now = time(NULL);

    // This is the first time we are aware that the robot has been halted
    if(b->getStopUntilTime() == 0) {
        if(CTRL_DEBUG)
            cout << signature << "HWAIT received, pausing..." << endl;
        cout <<"HWAIT received pausing ... " << endl;
        // Stop the robot but do NOT kill the plan
        stop();

        //    b->setStopUntilTime(now + STOP_SECONDS);
        if(b->getStopSeconds() > 0) {
            b->setStopUntilTime(now + b->getStopSeconds());
            // reset stopSeconds
            b->setStopSeconds(-1);
        }
        else {
            b->setStopUntilTime(INT_MAX);
        }

        //    b->setDoWait(false);
    }
    else if(b->getStopUntilTime() <= now) {
        if(CTRL_DEBUG)
            cout << signature << "Wait time expired, resuming..." << endl;

        Position currentPosition = b->getCurrentPosition();

        b->setStopUntilTime(0);
        b->setDoWait(false);
    }

    return true;
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

Position Controller::get_final_position() {

    Position final_pos = beliefs->getCurrentPosition();

    list<Task*> agenda = beliefs->getAgenda();

    for (list<Task*>::iterator it = agenda.begin(); it != agenda.end(); ++it) {
        Task *task = *it;

        if(task != NULL) {
            final_pos = Position(task->getX(), task->getY(), 0);
        }
    }

    return final_pos;
}


double Controller::get_cumulative_cost() {

    Position start_pos, end_pos;
    double total_cost = 0;

    start_pos = beliefs->getCurrentPosition();

    list<Task*> agenda = beliefs->getAgenda();

    for (list<Task*>::iterator it = agenda.begin(); it != agenda.end(); ++it) {
        Task *task = *it;

        if(task != NULL) {
            end_pos = Position(task->getX(), task->getY(), 0);

            total_cost += estimate_cost(start_pos.getX(), start_pos.getY(),
                                                end_pos.getX(), end_pos.getY());

            start_pos = end_pos;
        }
    }

    return total_cost;
}

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



void Controller::robot_parking_routine(){
  Position p;
  double bound_low = -1.0;
  double bound_high = -0.7;
  Position a;
  vector<double> wallDistance = beliefs->wallDistanceVector;
  int min_index, min_value, i;
  min_value = 1000;
  itl->update();
  pCli->ReadIfWaiting();
  message_handler->check_inbox();

  for(i = 0; i < wallDistance.size(); i++){
    cout << "Wall distance: " <<wallDistance[i] << endl;
    if(wallDistance[i] < min_value){
      
      min_value = wallDistance[i];
      min_index = i;
    }  
  }
  cout << "Wall distance vector position: "<<min_index << endl;
  FORRAction action = mapIntegerToRotationAction(min_index);

  
  executeDecision(action);
  
  

  
  //turn robot until it's facing downward
  /*
  p = itl->getPosition();
  while((p.getTheta() < bound_low) || (p.getTheta() > bound_high)){
  
      itl->update();
      pCli->ReadIfWaiting();
      message_handler->check_inbox();
    
      executeDecision(FORRAction(RIGHT_TURN, 1));
      usleep(1000000);
    p = itl->getPosition();
    a = beliefs->getCurrentPosition();
    cout << "Theta: "<<p.getTheta()<<endl;
    cout << "Theta from belief: "<<a.getTheta()<<endl;
    
    
  }
  */
  usleep(2000000);

  //push robot into the wall
  if(action.type != FORWARD){
    executeDecision(FORRAction(FORWARD, 5));
  }
  usleep(2000000);
  beliefs->setCurrentPosition(itl->getPosition());


  pCli->ReadIfWaiting();
  itl->update();
  message_handler->check_inbox();



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
