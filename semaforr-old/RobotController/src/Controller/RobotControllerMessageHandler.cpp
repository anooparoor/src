#include "Message.h"
#include "MessageHandler.h"
#include "definitions.h"
#include "Position.h"
#include "RobotControllerMessageHandler.h"
#include "defines.h"
#include "FORRAction.h"
#include "Controller.h"
#include <sstream>
#include <unistd.h>


RobotControllerMessageHandler::RobotControllerMessageHandler(CommunicationManager *comm_mgr)
  : MessageHandler(comm_mgr), controller(NULL) {
  cout << "registering callbacks" << endl;
  register_callbacks();
}

void RobotControllerMessageHandler::set_controller(Controller *ctl) {
  this->controller = ctl;
  this->beliefs = controller->getBeliefs(); 
  this->localize = controller->getLocalization(); 
}
  
void RobotControllerMessageHandler::register_callbacks() {

  // ASKPOSE
  register_callback(CMD_ASKPOSE, static_cast<MessageCallback>(&RobotControllerMessageHandler::cmd_askpose_handler));

  // AUCTION_START
  register_callback(CMD_AUCTION_START, 
		    static_cast<MessageCallback>(&RobotControllerMessageHandler::cmd_auction_start_handler));

  // AUCTION_WON
  register_callback(CMD_AUCTION_WON, 
		    static_cast<MessageCallback>(&RobotControllerMessageHandler::cmd_auction_won_handler));

  /*
  // AUCTION_STATUS
  register_callback(CMD_AUCTION_STATUS, 
		    static_cast<MessageCallback>(&RobotControllerMessageHandler::cmd_auction_status_handler));
  */

  // EXECUTE_ROBOTS
  //register_callback(CMD_EXECUTE_ROBOTS, 
  //static_cast<MessageCallback>(&RobotControllerMessageHandler::cmd_execute_robots_handler));

  // UPDATE_TEAMMATE
  register_callback(CMD_UPDATE_TEAMMATE,
		    static_cast<MessageCallback>(&RobotControllerMessageHandler::cmd_update_teammate_handler));

  // PATH_ALTERATION_COST
  register_callback(CMD_TEAMMATE_PATH_ALTERATION_COST, 
		    static_cast<MessageCallback>(&RobotControllerMessageHandler::cmd_alteration_cost_handler));

  // WAITING_FOR_TEAMMATE
  register_callback(CMD_WAITING_FOR_TEAMMATE, 
		    static_cast<MessageCallback>(&RobotControllerMessageHandler::cmd_waiting_for_handler));

  // CAMPOSE
  register_callback(CMD_CAMPOSE, static_cast<MessageCallback>(&RobotControllerMessageHandler::cmd_campose_handler));
   
  // Gets a target from the descriptive manager 
  register_callback(CMD_TARGET_POINT_DESCRIPTIVE, static_cast<MessageCallback>(&RobotControllerMessageHandler::cmd_target_point_handler));
  // gets distance to target from descriptive manager
  register_callback(CMD_DISTANCE_TO_TARGET_DESCRIPTIVE, static_cast<MessageCallback>(&RobotControllerMessageHandler::cmd_distance_to_target_handler));
  // gets distance from obstacles from the descriptive manager
  register_callback(CMD_DISTANCE_FROM_OBSTACLES_DESCRIPTIVE, static_cast<MessageCallback>(&RobotControllerMessageHandler::cmd_distance_from_obstacles_handler));
  // gets previous action performed by the robot from the decriptive manager
  register_callback(CMD_LAST_ACTION_DESCRIPTIVE, static_cast<MessageCallback>(&RobotControllerMessageHandler::cmd_last_action_handler));
  // get new set of advisor weights
  register_callback(CMD_ADVISOR_WEIGHT_DESCRIPTIVE, static_cast<MessageCallback>(&RobotControllerMessageHandler::cmd_advisor_weight_handler));
  // get data to instantiate advisors
  register_callback(CMD_ADVISOR_DATA_DESCRIPTIVE, static_cast<MessageCallback>(&RobotControllerMessageHandler::cmd_advisor_data_handler));
  // get data for Tier1 advisor that vetes forward moves that would result in collision with walls or other obstacles
  register_callback(CMD_VETO_FORWARD_MOVES_DATA_DESCRIPTIVE, static_cast<MessageCallback>(&RobotControllerMessageHandler::cmd_veto_forward_moves_advisor_data_handler)); 
  // get new set of team position
  register_callback(CMD_TEAM_POSE_DESCRIPTIVE, static_cast<MessageCallback>(&RobotControllerMessageHandler::cmd_team_pose_handler));

  // GOTO
  register_callback(CMD_GOTO, 
		    static_cast<MessageCallback>(&RobotControllerMessageHandler::cmd_goto_handler));

  // HWAIT
  register_callback(CMD_HWAIT, 
		    static_cast<MessageCallback>(&RobotControllerMessageHandler::cmd_hwait_handler));

  // HRESUME
  register_callback(CMD_HRESUME, 
		    static_cast<MessageCallback>(&RobotControllerMessageHandler::cmd_hresume_handler));


  /* TASC functions */
  // EXECUTE_TASK
  register_callback(CMD_EXECUTE_TASK, 
		    static_cast<MessageCallback>(&RobotControllerMessageHandler::cmd_execute_task_handler));

  // CANCEL_TASK
  register_callback(CMD_CANCEL_TASK, 
		    static_cast<MessageCallback>(&RobotControllerMessageHandler::cmd_cancel_task_handler));

  // TASK_REACHED
  register_callback(CMD_TASK_REACHED, 
		    static_cast<MessageCallback>(&RobotControllerMessageHandler::cmd_task_reached_handler));

  // TASK_STARTED
  register_callback(CMD_TASK_STARTED, 
		    static_cast<MessageCallback>(&RobotControllerMessageHandler::cmd_task_started_handler));

  // TASK_COMPLETE
  register_callback(CMD_TASK_COMPLETE, 
		    static_cast<MessageCallback>(&RobotControllerMessageHandler::cmd_task_complete_handler));
}

void RobotControllerMessageHandler::cmd_askpose_handler(Message msg) {

  Position pos = beliefs->getCurrentPosition();
  
  Message pose_message;

  pose_message.set_command(CMD_POSE);

  pose_message.add_arg((int)comm_mgr->get_session_id());
  pose_message.add_arg(pos.getX());
  pose_message.add_arg(pos.getY());
  pose_message.add_arg(pos.getTheta());
  //cout << "received askpose .. about to send pose" << endl;
  this->send_message(pose_message);

  // Also send out an EXECUTE_ROBOTS UPDATE_TEAMMATE message
  /*Message exec_message;

  exec_message.set_command(CMD_EXECUTE_ROBOTS);
  exec_message.add_arg(CMD_UPDATE_TEAMMATE);
  exec_message.add_arg(-1);
  exec_message.add_arg((int)comm_mgr->get_session_id());
  exec_message.add_arg(controller->get_robot_size());
  exec_message.add_arg(pos.getX());
  exec_message.add_arg(pos.getY());
  exec_message.add_arg(pos.getTheta());

  this->send_message(exec_message); */
}


void RobotControllerMessageHandler::cmd_target_point_handler(Message msg) {

  std::cout << "Got a message " << msg.get_full_message() << endl;
  std::cout << "RC: received a target point descriptive" << std::endl;
  std::vector<std::string> args = msg.get_args();
  if(args[0] == "empty")
  {
    beliefs->hasMoreTargets = false;  
  }
  else
  {
    std::pair <double,double> point;
    point = std::make_pair(atof(args[0].c_str()), atof(args[1].c_str()));
    Task currentTask;
    currentTask.setAccessPoint(point);
    beliefs->setCurrentTask(&currentTask);
  }  
  controller->received_reply = true;
}

void RobotControllerMessageHandler::cmd_team_pose_handler(Message msg) {
  std::cout << "Got a message " << msg.get_full_message() << endl;
  std::cout << "RC: received a all_pose descriptive" << std::endl;
  std::vector<std::string> args = msg.get_args();
  (beliefs->teamPose).clear();
  Position robot_pose;
  for(unsigned i = 0 ; i <args.size();i++){
    int robot_id = i / 3; 
    robot_pose.setX(atof((args[i]).c_str()));
    i++;
    robot_pose.setY(atof((args[i]).c_str()));
    i++;
    robot_pose.setTheta(atof((args[i]).c_str()));
    (beliefs->teamPose).push_back(robot_pose);
  }
  controller->received_reply = true;
}


void RobotControllerMessageHandler::cmd_distance_to_target_handler(Message msg) {

  std::cout << "Got a message " << msg.get_full_message() << endl;
  std::cout << "RC: received a distance from target descriptive" << std::endl;
  std::vector<std::string> args = msg.get_args();

  for(unsigned i =0; i < args.size();i++){
    beliefs->targetDistanceVector[i] = atof((args[i]).c_str());              
  }
  controller->received_reply = true;
}

void RobotControllerMessageHandler::cmd_distance_from_obstacles_handler(Message msg) {

  std::cout << "Got a message " << msg.get_full_message() << endl;
  std::cout << "RC: received a distance from obstacle descriptive" << std::endl;
  std::vector<std::string> args = msg.get_args();
  for(int i =0; i < 12 ;i++){
    //double distance = beliefs->wallDistanceVector[i];
    //beliefs->previousWallDistanceVector[i] = distance;
    beliefs->wallDistanceVector[i] = atof((args[i]).c_str());              
  }
  controller->received_reply = true;
}

void RobotControllerMessageHandler::cmd_last_action_handler(Message msg) {

  std::cout << "Got a message " << msg.get_full_message() << endl;
  std::cout << "RC: received a last action" << std::endl;
  std::vector<std::string> args = msg.get_args();
  beliefs->lastAction.type = static_cast<FORRActionType>(atoi((args[0]).c_str()));
  beliefs->lastAction.parameter = atoi((args[1]).c_str());
  controller->received_reply = true;
  //cout << "returning from last action handler" << endl;
}

void RobotControllerMessageHandler::cmd_advisor_weight_handler(Message msg){
  //fprintf(stderr, "Got a '%s'\n", msg.get_full_message().c_str());
  std::cout << "Got a message " << msg.get_full_message() << endl;
  std::cout << "RC: received a advisor weight descriptive" << std::endl;
  std::vector<std::string> args = msg.get_args();
  //clear the old weights and push the new weights
  (beliefs->advisorWeight).clear();
  for(unsigned i =0; i < args.size() ;i++)
  {
      (beliefs->advisorWeight).push_back(atof((args[i]).c_str()));              
  }
  controller->received_reply = true;
}

void RobotControllerMessageHandler::cmd_advisor_data_handler(Message msg) {
  //fprintf(stderr, "Got a '%s'\n", msg.get_full_message().c_str());
  std::cout << "Got a message " << msg.get_full_message() << endl;
  std::cout << "RC: received a advisor data descriptive" << std::endl;
  std::vector<std::string> args = msg.get_args();
  cout << "size of message: " << args.size() << endl; 
  bool advisor_active;
  //vector <FORRActionType> actions;
  //actions.push_back(NOOP);
  //actions.push_back(WIDE_RIGHT_TURN);
  //actions.push_back(WIDE_LEFT_TURN);
  //actions.push_back(PAUSE);
  //actions.push_back(HALT);
  
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
    /*
    std::size_t found = advisor_name.find("Rotation");
    if(found!=std::string::npos){
      actions.push_back(RIGHT_TURN);
      actions.push_back(LEFT_TURN);
    }
    else{
      actions.push_back(FORWARD);
      //actions.push_back(BACKWARD);
    }
    */
    
    if(args[2] == "t")
      advisor_active = true;
    else
      advisor_active = false;
    cout << "In ROBOTCONTROLLERMESSAGE handler before calling add_advisor" << endl;;
    vector<double> advisorWeight;
    advisorWeight.push_back(atof(args[3].c_str()));
    controller->add_advisor(Tier3Advisor::makeAdvisor(beliefs, args[0], args[1], advisorWeight, auxiliary_array, advisor_active)); // this is vector of Tier3Advisors
    cout << "After calling add_advisor" << endl;
  }
  else{
    cout << "ERROR: advisor data received does not contain 8 words" << endl;
  } 
  controller->received_reply = true;
}


  
void RobotControllerMessageHandler::cmd_veto_forward_moves_advisor_data_handler(Message msg){
  std::cout << "Got a message " << msg.get_full_message() << endl;
  std::cout << "RC: received a veto forward moves data descriptive" << std::endl;
  std::vector<std::string> args = msg.get_args();
  
  cout << "RC Message Handler received " << args[0] << endl;
  beliefs->wall_distance = atof(args[0].c_str());
  controller->received_reply = true;
}

 
void RobotControllerMessageHandler::cmd_auction_start_handler(Message msg) {

    uint64_t coordinator_id;
    int auction_id, bundleplex_size, auction_type;
    double initial_cost, min_cost;
    std::vector<std::string> msg_args;
    Position current_pos, cost_from_pos;

    // Let's assume one big, flat bundle for now. Can change this later.
    bundle_t bundle;

    //fprintf(stderr, "Got AUCTION_START: %s\n", msg.get_full_message().c_str());
    //fprintf(stderr, "Sender: %d\n", (int)(msg.get_sender_id()) );

    msg_args = msg.get_args();
    arg_iterator arg_it = msg_args.begin();

    coordinator_id = msg.get_sender_id();

    // First get the auction id
    auction_id = Message::to_int(*arg_it);

    // Then the auction type
    auction_type = Message::to_int(*(++arg_it));

    // Then the size of the bundleplex
    bundleplex_size = Message::to_int(*(++arg_it));

    for (int i=0; i < bundleplex_size; i++) {
        int bundle_size;

        // Get the bundle size...
        bundle_size = Message::to_int(*(++arg_it));

        for (int j=0; j < bundle_size; j++) {
            int x, y;

            // Get X
            x = Message::to_int(*(++arg_it));

            // Get Y
            y = Message::to_int(*(++arg_it));

            point_t point(x, y);
            bundle.push_back(point);
        }
    }

    // Finished parsing points, now find costs

    current_pos = beliefs->getCurrentPosition();

    if ( auction_type != 3 ) { // Random, OSI or PSI

      if ( auction_type == 2 ) { // PSI
	cost_from_pos = current_pos;
	initial_cost = 0;
      }
      else {
	cost_from_pos = controller->get_final_position();
	initial_cost = controller->get_cumulative_cost();
	//fprintf(stderr, "Cumulative cost is now: %f\n", initial_cost);
      }

      // Pretty much always 1 for now
      for (int i=0; i < (int) bundle.size(); i++) {
        point_t cur_point = bundle[i];

        double current_cost = initial_cost +
	  controller->estimate_cost(cost_from_pos.getX(),
				cost_from_pos.getY(),
				cur_point.first,
				cur_point.second);

	// Construct the return message
	Message bid_msg;
	bid_msg.set_command(CMD_AUCTION_BID);

	bid_msg.set_message_type(DEST_SINGLE);
	bid_msg.set_destination_id(Message::to_string((int)coordinator_id));

	bid_msg.add_arg(auction_id);
	bid_msg.add_arg(i);
	bid_msg.add_arg(current_cost);

	send_message(bid_msg);
      }
    }
    // It's SSI
    else {
      cost_from_pos = controller->get_final_position();
      initial_cost = controller->get_cumulative_cost();
      //fprintf(stderr, "Cumulative cost is now: %f\n", initial_cost);

      int min_bundle_index = 0;
      for (unsigned int i=0; i < bundle.size(); i++) {
        point_t cur_point = bundle[i];
	
        // For PSI
        double current_cost = initial_cost +
	  controller->estimate_cost(cost_from_pos.getX(),
				cost_from_pos.getY(),
				cur_point.first,
				cur_point.second);
	
        if ( i == 0 || current_cost < min_cost ) {
	  min_cost = current_cost;
	  min_bundle_index = i;
        }
      }
      
      // Construct the return message
      Message bid_msg;
      bid_msg.set_command(CMD_AUCTION_BID);
      
      bid_msg.set_message_type(DEST_SINGLE);
      bid_msg.set_destination_id(Message::to_string((int)coordinator_id));
      
      bid_msg.add_arg(auction_id);
      bid_msg.add_arg(min_bundle_index);
      bid_msg.add_arg(min_cost);
      
      send_message(bid_msg);
    }
}


void RobotControllerMessageHandler::cmd_auction_won_handler(Message msg) {

  int coordinator_id, task_id, bundle_id, x, y;
  std::vector<std::string> msg_args = msg.get_args();
  
  coordinator_id = msg.get_sender_id();
  task_id = Message::to_int(msg_args[0]);
  bundle_id = Message::to_int(msg_args[1]);
  x = Message::to_int(msg_args[2]);
  y = Message::to_int(msg_args[3]);

  //fprintf(stderr, "Won task %d\n", task_id);

  std::vector<int> empty_vector;
  Task *new_task = new Task(coordinator_id, task_id, SENSOR_SWEEP, 1, empty_vector, 0, x, y);

  beliefs->addTask(new_task);
}


void RobotControllerMessageHandler::cmd_update_teammate_handler(Message msg) {
  std::vector<std::string> args = msg.get_args();

  unsigned int other_id = Message::to_int(args[1]);
  int other_size = Message::to_int(args[2]);
  double other_x = Message::to_double(args[3]);
  double other_y = Message::to_double(args[4]);
  double other_theta = Message::to_double(args[5]);

  // Don't check for collisions with ourself!
  if ( other_id == comm_mgr->get_session_id() ) {
    return;
  }

  if(args.size() != 6) {
    cerr << "Malformed UPDATE_TEAMMATE message." << endl;
    return;
  }

  // update the teammate info
  beliefs->updateTeammate(other_id, other_x, other_y, other_theta, other_size);

  // call collisionmanager to check for collision for teammate
  //collisionMgr->updateCollisionInfo(other_id); 
}


void RobotControllerMessageHandler::cmd_alteration_cost_handler(Message msg) {

  unsigned int other_id;
  double cost;

  std::vector<std::string> args = msg.get_args();

  other_id = Message::to_int(args[1]);
  cost = Message::to_double(args[2]);

  int other_size = Message::to_int(args[3]);
  double other_x = Message::to_double(args[4]);
  double other_y = Message::to_double(args[5]);
  double other_theta = Message::to_double(args[6]);

  if ( other_id != comm_mgr->get_session_id() ) {
    beliefs->updateTeammate(other_id, other_x, other_y, other_theta, other_size);
    //collisionMgr->updateAlterationCost(other_id, cost); 
  }
}


void RobotControllerMessageHandler::cmd_waiting_for_handler(Message msg) {

  //  std::vector<std::string> args = msg.get_args();
  //  int other_id = Message::to_int(args[1]);
}

void RobotControllerMessageHandler::cmd_campose_handler(Message msg) {
    std::vector<std::string> args = msg.get_args();

    Position pos(Message::to_int(args[1]), Message::to_int(args[2]),
                 Message::to_double(args[3]));

    if(controller->getLocalization()->isSimulatingGPS()) {
        controller->getLocalization()->setLatestPosition(pos);
    }
    else {
        CameraObservation cobs;
        cobs.setPose(pos);
        controller->getLocalization()->updateCameraObservations(cobs);
    }
}


void RobotControllerMessageHandler::cmd_execute_task_handler(Message msg) {

  int coord_id = msg.get_sender_id();

  std::vector<std::string> args = msg.get_args();

  int task_id = Message::to_int(args[0]);
  TASK_TYPE type = static_cast<TASK_TYPE>(Message::to_int(args[1]));
  int num_required = Message::to_int(args[2]); 
  int num_assigned = Message::to_int(args[3]); 

  cout << "RobotControllerMessageHandler::cmd_execute_task_handler> num_required: " 
       << num_required << " & num_assigned: " << num_assigned << endl; 

  vector<int> assigned_robots; 
  int i; 
  for(i = 4; i < num_assigned + 4; i++) {
    assigned_robots.push_back(Message::to_int(args[i])); 
  }
  
  int duration = Message::to_int(args[i++]);
  int x = Message::to_int(args[i++]); 
  int y = Message::to_int(args[i++]);
  
  Task* t = beliefs->getTask(task_id);
  if(t->get_id() == -1) { 
    beliefs->addTask(new Task(coord_id,
			      task_id,
			      type,
			      num_required,
			      assigned_robots,
			      duration,
			      x,
			      y));
  }
  else {
    // task already exists update the assigned robots bit 
    vector<int>::iterator iter;
    for(iter = assigned_robots.begin(); iter != assigned_robots.end(); iter++) {
      beliefs->robotAssignedToTask(task_id, (*iter));      
    }
    
    // resend the task reached message since there are new robots assigned to the task
    beliefs->setReachedMsgSent(false); 

    // update the duration and the assigned access point info in case it has been changed
    std::pair<int,int> aP = std::make_pair(x,y); 
    beliefs->updateTaskResponsibility(task_id, duration, aP); 
  }
}


void RobotControllerMessageHandler::cmd_cancel_task_handler(Message msg) {

  std::vector<std::string> args = msg.get_args();  

  int arg1 = Message::to_int(args[0]); 

  beliefs->removeTask(arg1);
}



void RobotControllerMessageHandler::cmd_hwait_handler(Message msg) {

  // If we need to check the authority of the HWAIT, do it here
  //std::vector<std::string> args = msg.get_args();  

  beliefs->setDoWait(true);

}

void RobotControllerMessageHandler::cmd_hresume_handler(Message msg) {

  // If we need to check the authority of the HRESUME, do it here
  //std::vector<std::string> args = msg.get_args();  

  beliefs->setDoWait(false);

}

void RobotControllerMessageHandler::cmd_goto_handler(Message msg) {
  
  std::vector<std::string> args = msg.get_args();  

  int arg1 = Message::to_int(args[0]);
  int arg2 = Message::to_int(args[1]);

  localize->resetDestinationInfo();

  Position p = localize->getPosition();
  
  //Node s(1, p.getX(), p.getY()); 

  //planner->setSource(s); 

  //Node t(1, arg1, arg2); 

  //planner->setTarget(t); 

}


void RobotControllerMessageHandler::cmd_task_reached_handler(Message msg) {

  std::vector<std::string> args = msg.get_args();  

  int robot_id = Message::to_int(args[0]); 
  int task_id = Message::to_int(args[1]); 

  cout << "RCMsgHandler::task_reached_handler> " << robot_id << " " << task_id << endl; 

  // update task status. if the task is not on our agenda the returned task will have an invalid id of -1
  // this will ensure that the robot will only keep track of status of tasks in its own agenda
  Task * tsk = beliefs->getTask(task_id);
  if(tsk->get_id() != -1) {
    tsk->robotArrived(robot_id);
  } 
  else {
    cout << "Not assigned to task " << task_id << ", ignoring message" << endl;
  }

}


void RobotControllerMessageHandler::cmd_task_started_handler(Message msg) {

  std::vector<std::string> args = msg.get_args();  

  int robot_id = Message::to_int(args[0]); 
  int task_id = Message::to_int(args[1]); 

  cout << "RCMsgHandler::task_started_handler> " << robot_id << " " << task_id << " doing nothing with this message" << endl; 
}


void RobotControllerMessageHandler::cmd_task_complete_handler(Message msg) {

  std::vector<std::string> args = msg.get_args();  

  int robot_id = Message::to_int(args[0]); 
  int task_id = Message::to_int(args[1]); 

  cout << "RCMsgHandler::task_complete_handler> " << robot_id << " " << task_id << " doing nothing with this message" << endl; 
}


// void RobotControllerMessageHandler::cmd_yield_reason_handler(Message msg) {

//   std::vector<std::string> args = msg.get_args();  

//   int robot_id = Message::to_int(args[0]); 
//   std::string reason = args[1]; 

//   //collisionMgr->setTeammateYieldReason(robot_id, reason); 
// }

/*outgoing messages */

// messages to descriptive manager

void RobotControllerMessageHandler::send_set_last_action(FORRAction action){ 
    stringstream ss1,ss2;
    ss1 << action.type;
    ss2 << action.parameter;
    string actionstring = ss1.str() + " " + ss2.str();
    cout << "sending last action descriptive to save the last action: " <<actionstring << endl;
    send_message("BROADCAST_TYPE descriptive_manager LAST_ACTION_DESCRIPTIVE "+actionstring);
}

void RobotControllerMessageHandler::send_set_current_target(CartesianPoint current_target){ 
    stringstream ss1,ss2;
    ss1 << current_target.get_x();
    ss2 << current_target.get_y();
    string targetstring = ss1.str() + " " + ss2.str();
    cout << "sending current target descriptive to save the current pursuing target: " <<targetstring << endl;
    send_message("BROADCAST_TYPE descriptive_manager CURRENT_TARGET_DESCRIPTIVE " + targetstring);
}

void RobotControllerMessageHandler::send_get_next_target(){
  cout << "sending ASK_TARGET_POINT_DESCRIPTIVE" << endl;
  send_message("BROADCAST_TYPE descriptive_manager ASK_TARGET_POINT_DESCRIPTIVE");
  controller->received_reply = false;
}

void RobotControllerMessageHandler::send_get_team_pose(){
  cout << "sending ASK_TEAM_POSE_DESCRIPTIVE" << endl;
  send_message("BROADCAST_TYPE descriptive_manager ASK_TEAM_POSE_DESCRIPTIVE");
  controller->received_reply = false;
}


void RobotControllerMessageHandler::send_get_distance_to_target(){ 
   cout << "sending ASK_DISTANCE_TO_TARGET_DESCRIPTIVE" << endl;
   send_message("BROADCAST_TYPE descriptive_manager ASK_DISTANCE_TO_TARGET_DESCRIPTIVE");
   controller->received_reply = false;
}

void RobotControllerMessageHandler::send_get_distance_from_walls(){
   cout << "sending ASK_DISTANCE_FROM_OBSTACLES_DESCRIPTIVE" << endl;
   send_message("BROADCAST_TYPE descriptive_manager ASK_DISTANCE_FROM_OBSTACLES_DESCRIPTIVE");
   controller->received_reply = false;
}

void RobotControllerMessageHandler::send_get_last_action(){
     // send message to the 
   cout << "sending ASK_LAST_ACTION_DESCRITPIVE" << endl;
   send_message("BROADCAST_TYPE descriptive_manager ASK_LAST_ACTION_DESCRIPTIVE");
   controller->received_reply = false;
}

void RobotControllerMessageHandler::send_get_advisor_weight(){
   cout << "sending ASK_ADVISOR_WEIGHT_DESCRIPTIVE" << endl;
   send_message("BROADCAST_TYPE descriptive_manager ASK_ADVISOR_WEIGHT_DESCRIPTIVE");
   controller->received_reply = false;
}

void RobotControllerMessageHandler::send_get_advisor_data(){
  cout << "sending ASK_ADVISOR_DATA_DESCRIPTIVE" << endl;
  this->send_message("BROADCAST_TYPE descriptive_manager ASK_ADVISOR_DATA_DESCRIPTIVE");
  controller->received_reply = false;
}

void RobotControllerMessageHandler::send_get_veto_forward_moves_advisor_data(){
  cout << "sending ASK_VETO_FORWARD_MOVES_DATA_DESCRIPTIVE" << endl;
  this->send_message("BROADCAST_TYPE descriptive_manager ASK_VETO_FORWARD_MOVES_DATA_DESCRIPTIVE");

  controller->received_reply = false;
}

// messages to auction manager

void RobotControllerMessageHandler::send_auction_complete(int task_id, int coordinator_id) {

    Message msg;

    msg.set_command(CMD_AUCTION_COMPLETE);

    msg.set_message_type(DEST_SINGLE);
    msg.set_destination_id(Message::to_string(coordinator_id));

    msg.add_arg((int)comm_mgr->get_session_id());
    msg.add_arg(task_id);
    msg.add_arg(coordinator_id);

    send_message(msg);
}

void RobotControllerMessageHandler::send_auction_finished(int coordinator_id) {

    Message msg;

    msg.set_message_type(DEST_SINGLE);
    msg.set_destination_id(Message::to_string(coordinator_id));

    msg.set_command(CMD_AUCTION_FINISHED);
    msg.add_arg(coordinator_id);

    send_message(msg);
}

/*
void RobotControllerMessageHandler::send_waypoints() {

  Message msg;
  msg.set_command(CMD_WAYPOINTS);

  list<int> waypoints = planner->getPath(); 
  Node target = planner->getTarget();

  int num_points = waypoints.size(); 
  bool is_target_excluded = false; 

    // add target to num points if the path is empty
  if(num_points == 0) {
    num_points++; 
    is_target_excluded = true;
  }
  else {
    Node last = planner->getGraph()->getNode(waypoints.back());
    
    // add x, y of the target if the last waypoint is not the final destination 
    if(last.getID() != Node::invalid_node_index) {
      if(last.getX() != target.getX() || last.getY() != target.getY()) {
	is_target_excluded = true; 
	num_points ++; 
      }
    }
  }

  // check if there are enough points in the path to report 
  if(num_points == 0) {
    return;
  }

  msg.add_arg((int)comm_mgr->get_session_id());
  msg.add_arg(num_points); 

  list<int>::iterator it; 
  for(it = waypoints.begin(); it != waypoints.end(); it++) {
    Node n = planner->getGraph()->getNode(*it);
    msg.add_arg(n.getX());
    msg.add_arg(n.getY());
  }
    
  // add the x, y of the target if it wasn't in the plan
  if(is_target_excluded) {
    msg.add_arg(target.getX()); 
    msg.add_arg(target.getY()); 
  }
 
  send_message(msg);
}

*/

void RobotControllerMessageHandler::send_alteration_cost(uint64_t teammate_id, double cost) {

  Position pos = beliefs->getCurrentPosition(); 

  Message cost_msg;
  
  cost_msg.set_command(CMD_EXECUTE_ROBOTS);
  
  cost_msg.add_arg(CMD_TEAMMATE_PATH_ALTERATION_COST);
  
  cost_msg.add_arg((int)teammate_id);
  cost_msg.add_arg((int)(comm_mgr->get_session_id()));
  cost_msg.add_arg(cost);
  cost_msg.add_arg(controller->get_robot_size());
  cost_msg.add_arg(pos.getX());
  cost_msg.add_arg(pos.getY());
  cost_msg.add_arg(pos.getTheta());
  
  send_message(cost_msg);
}


/* This function can be called on two occasions: 
   1. this robot is initiating the message in that case the tMate will be the robotsProp object 
   that this robot is waiting on and id will be its session id
   2. relaying a message for a higher priority sender in that case tMate will still be the 
   robotsProp object that this robot is waiting on but the id will be the senders id
   */
void RobotControllerMessageHandler::send_waiting_for(uint64_t teammate_id, uint64_t id)
{
    Message waiting_msg;

    waiting_msg.set_command(CMD_EXECUTE_ROBOTS);
    waiting_msg.add_arg(CMD_WAITING_FOR_TEAMMATE);

    waiting_msg.add_arg((int)teammate_id);
    waiting_msg.add_arg((int)id);

    send_message(waiting_msg);
}


void RobotControllerMessageHandler::send_auction_pause(uint64_t coordinator_id,
						       int auction_id) {

  Message pause_msg;

  pause_msg.set_command(CMD_AUCTION_STATUS);

  pause_msg.add_arg((int)(comm_mgr->get_session_id()));
  pause_msg.add_arg((int)coordinator_id);
  pause_msg.add_arg(TASK_PAUSE);
  pause_msg.add_arg(auction_id);
  pause_msg.add_arg(TASK_SWEEP);

  send_message(pause_msg);
}

void RobotControllerMessageHandler::send_auction_resume(uint64_t coordinator_id,
							int auction_id) {

  Message resume_msg;

  resume_msg.set_command(CMD_AUCTION_STATUS);

  resume_msg.add_arg((int)(comm_mgr->get_session_id()));
  resume_msg.add_arg((int)coordinator_id);
  resume_msg.add_arg(TASK_RESUME);
  resume_msg.add_arg(auction_id);
  resume_msg.add_arg(TASK_SWEEP);

  send_message(resume_msg);
}

/* TASC messages */

void RobotControllerMessageHandler::send_task_reached(uint64_t coordinator_id, int task_id) {

  Message task_reached_msg; 

  task_reached_msg.set_message_type(DEST_BROADCAST_ALL);

  task_reached_msg.add_arg(CMD_TASK_REACHED);
  task_reached_msg.add_arg((int)(comm_mgr->get_session_id()));
  task_reached_msg.add_arg((int) task_id);

  send_message(task_reached_msg);
  
  beliefs->setReachedMsgSent(true); 
}


void RobotControllerMessageHandler::send_task_started(uint64_t coordinator_id, int task_id) {

  Message task_started_msg; 

  task_started_msg.set_message_type(DEST_BROADCAST_ALL);

  task_started_msg.add_arg(CMD_TASK_STARTED);
  task_started_msg.add_arg((int)(comm_mgr->get_session_id()));
  task_started_msg.add_arg((int) task_id);

  send_message(task_started_msg);

  beliefs->setStartedMsgSent(true); 
}


void RobotControllerMessageHandler::send_task_complete(uint64_t coordinator_id, int task_id) {

  Message task_complete_msg; 

  task_complete_msg.set_message_type(DEST_BROADCAST_ALL);

  task_complete_msg.add_arg(CMD_TASK_COMPLETE);
  task_complete_msg.add_arg((int)(comm_mgr->get_session_id()));
  task_complete_msg.add_arg((int) task_id);
  
  send_message(task_complete_msg);

  beliefs->setReachedMsgSent(true); 
}



void RobotControllerMessageHandler::send_delayed(uint64_t coordinator_id) {
  // send task started message to the coordinator
  Message msg;
  
  msg.set_message_type(DEST_SINGLE);

  msg.set_destination_id(Message::to_string((int)coordinator_id));

  msg.set_command(CMD_DELAYED);

  msg.add_arg((int)(comm_mgr->get_session_id()));
  
  send_message(msg);
}


void RobotControllerMessageHandler::send_moving(uint64_t coordinator_id) {
  // send task started message to the coordinator
  Message msg;
  
  msg.set_message_type(DEST_SINGLE);

  msg.set_destination_id(Message::to_string((int)coordinator_id));

  msg.set_command(CMD_MOVING);

  msg.add_arg((int)(comm_mgr->get_session_id()));
  
  send_message(msg);
} 



// void RobotControllerMessageHandler::send_yield_reason(uint64_t teammate_id, YieldReason reason) {
//   Message msg; 
  
//   msg.set_message_type(DEST_SINGLE); 
  
//   msg.set_destination_id(Message::to_string((int)teammate_id)); 

//   msg.set_command(CMD_YIELD_REASON); 

//   msg.add_arg((int)(comm_mgr->get_session_id()));
  
//   //msg.add_arg(Collision::getYRString(reason)); 

//   send_message(msg);
// }
