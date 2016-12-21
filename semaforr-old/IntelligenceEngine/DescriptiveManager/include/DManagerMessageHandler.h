/* DManagerMessageHandler.h
 *
 * Copyright 2013 HRTeam.
 * License:
 * Created: Dec 5, 2013
 * Original authors: Anoop Aroor <anoop.a.rao@gmail.com>
 */

#ifndef DMANAGER_MESSAGE_HANDLER_H
#define DMANAGER_MESSAGE_HANDLER_H

#include "MessageHandler.h"
#include "DescriptiveManager.h"

#include <string>

class DescriptiveManager;
class DManagerMessageHandler : public MessageHandler {

public:

  DManagerMessageHandler(CommunicationManager *comm_mgr, DescriptiveManager *desc_mgr);
  
private:

  DescriptiveManager *desc_mgr;
  void register_callbacks();
  
  void cmd_pose_handler(Message msg);
  void cmd_askpose_handler(Message msg);
  void cmd_last_action_handler(Message msg); 
  void cmd_current_target_handler(Message msg);

  void cmd_ask_team_pose_handler(Message msg);  
  void cmd_ask_target_point_handler(Message msg);
  void cmd_ask_distance_to_target_handler(Message msg);
  void cmd_ask_distance_from_obstacles_handler(Message msg);
  void cmd_ask_last_action_handler(Message msg);
  void cmd_ask_advisor_weight_handler(Message msg);
  void cmd_ask_advisor_parameters_handler(Message msg);
  void cmd_ask_advisor_data_handler(Message msg);
  void cmd_veto_forward_moves_advisor_data_handler(Message msg);

};

#endif  // INTELLIGENCE_ENGINE_DESCRIPTIVE_MANAGER_INCLUDE_DMANAGER_MESSAGE_HANDLER_H_
