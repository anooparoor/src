/* MessageHandler.h
 *
 * Copyright 2013 HRTeam.
 * License:
 * Created: May 30, 2013
 * Original authors: Eric Schneider <nitsuga@pobox.com>
 */

#ifndef MESSAGE_HANDLER_H
#define MESSAGE_HANDLER_H

#include <map>
#include <string>

//#include "Client.h"
#include "CommunicationManager.h"
#include "Message.h"
#include "definitions.h"

class MessageHandler;

typedef void (MessageHandler::*MessageCallback)( Message );

class MessageHandler {

protected:

  CommunicationManager *comm_mgr;

  std::map<std::string, MessageCallback> callbacks;

  void process_message(Message message);

  void register_callback(std::string command, MessageCallback cb);
  void register_callback(const char command[], MessageCallback cb);

  // Subclasses register their own callbacks here
  virtual void register_callbacks();

  // Default handlers for all clients
  void cmd_ping_handler(Message msg);
  void cmd_pong_handler(Message msg);

  // Some basic handlers for nop and debug situations
  void cmd_nop_handler(Message msg);
  void cmd_print_handler(Message msg);

  void print_time();
public:

  explicit MessageHandler(CommunicationManager *comm_mgr);

  // If there are any incoming messages, pass them to ProcessMessage
  void check_inbox();

  // Put a message in comm_mgr_'s output queue
  void send_message(std::string msg);

  // Overload for sending a Message object
  // author: Ken Sinclair
  void send_message(Message msg);

  // Returns comm_mgr->get_session_id()
  uint64_t get_session_id();
};

#endif  // UTILS_METROCOMMUNICATION_INCLUDE_MESSAGE_HANDLER_H_
