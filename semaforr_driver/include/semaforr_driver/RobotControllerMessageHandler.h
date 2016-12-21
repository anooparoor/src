#ifndef ROBOT_CONTROLLER_MESSAGE_HANDLER
#define ROBOT_CONTROLLER_MESSAGE_HANDLER

#include "MessageHandler.h"
#include <deque>
#include <list>
#include "FORRAction.h"
#include "FORRGeometry.h"
#include "Beliefs.h"

typedef std::vector<std::string>::iterator arg_iterator;

class Controller;
class Localization;
class Beliefs; 
class Tier3Advisor;

class RobotControllerMessageHandler : public MessageHandler {
 public:
  RobotControllerMessageHandler(CommunicationManager*);

  void set_controller(Controller *ctl);

  void send_waypoints(std::list<int> waypoints);

  void send_auction_complete(int auction_id, int coordinator_id);

  void send_auction_finished(int coordinator_id);

  void send_waypoints();

  void send_alteration_cost(uint64_t teammate_id, double cost);

  void send_waiting_for(uint64_t teammate_id, uint64_t id);

  void send_auction_pause(uint64_t coordinator_id = 0, int task_id = 0);

  void send_auction_resume(uint64_t coordinator_id = 0, int task_id = 0);

  void send_set_last_action(FORRAction action);
  
  void send_set_current_target(CartesianPoint current_target);

  void send_get_next_target();

  void send_get_team_pose();

  void send_get_distance_to_target();

  void send_get_distance_from_walls();

  void send_get_last_action();
 
  void send_get_advisor_weight();

  void send_get_advisor_data();

  void send_get_veto_forward_moves_advisor_data();

  /* TASC messages */

  //! sent when the robot reaches the task's access point
  void send_task_reached(uint64_t coordinator_id = 0, int task_id = 0); 

  //! sent when all robots assigned to the task and start executing the task
  void send_task_started(uint64_t coordinator_id = 0, int task_id = 0); 

  //! sent when the task execution is finished
  void send_task_complete(uint64_t coordinator_id = 0, int task_id = 0); 

  //! sent when the robot waits for another during obstacle avoidance
  void send_delayed(uint64_t coordinator_id = 0);
  
  //! sent when the robot is on the move
  void send_moving(uint64_t coordinator_id = 0);

 private:

  Beliefs * beliefs; 

  Localization * localize;
  /* end object ptrs */ 

  Controller * controller;
  
  void register_callbacks();

  void cmd_askpose_handler(Message msg);

  void cmd_auction_start_handler(Message msg);

  // Tuna: this doesn't work anymore on account of removal of Controller::add_task function
  void cmd_auction_won_handler(Message msg);

  //void cmd_execute_robots_handler(Message msg);

  void cmd_update_teammate_handler(Message msg);

  /*
  void cmd_auction_status_handler(Message msg);
  */

  void cmd_alteration_cost_handler(Message msg);

  void cmd_waiting_for_handler(Message msg);

  void cmd_campose_handler(Message msg);
  
  void cmd_distance_to_target_handler(Message msg);

  void cmd_distance_from_obstacles_handler(Message msg);

  void cmd_last_action_handler(Message msg);

  void cmd_advisor_weight_handler(Message msg);
 
  void cmd_advisor_data_handler(Message msg);

  void cmd_target_point_handler(Message msg);

  void cmd_team_pose_handler(Message msg);

  void cmd_veto_forward_moves_advisor_data_handler(Message msg); 

  void cmd_goto_handler(Message msg);

  void cmd_add_obstacle_handler(Message msg);

  void cmd_remove_obstacle_handler(Message msg);

  void cmd_hwait_handler(Message msg);

  void cmd_hresume_handler(Message msg);

  /* TASC messages */ 
  
  void cmd_execute_task_handler(Message msg); 

  void cmd_cancel_task_handler(Message msg);

  void cmd_task_reached_handler(Message msg); 

  void cmd_task_started_handler(Message msg); 

  void cmd_task_complete_handler(Message msg);
			 
};


#endif
