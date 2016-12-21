/* communication_manager.h
 *
 * Copyright 2013 HRTeam.
 * License:
 * Created: May 30, 2013
 * Original authors: Eric Schneider <nitsuga@pobox.com>
 *
 * Quick-start example:
 *
 *   string my_type = "Blackfin";
 *   string my_name = "blackfin-10";
 *   string cs_host = "skygrid.hrteam.org"; // or an ip like "127.0.0.1"
 *   int port = 6667;
 *
 *   metrobotics::CommunicationManager *comm_mgr = 
 *     new CommunicationManager(my_type, my_name, cs_host, cs_port);
 *
 *   // Connect to the central server
 *   comm_mgr->Connect();
 *
 *   // Start a thread for the communication manager
 *   boost::thread *comm_mgr_thread = new boost::thread(boost::ref(*comm_mgr));
 *
 *   // Your subclass of MessageHandler handles messages specific to
 *   // your component
 *   MyMessageHandler *msg_handler = new MyMessageHandler(comm_mgr);
 *
 *   // At this point it's assumed that msg_handler->CheckInbox() will be
 *   // called periodically and frequently inside some kind of controller or
 *   // event loop. CheckInbox() invokes msg_handler->ProcessMessage() if
 *   // any new messages have been received since the last call to CheckInbox().
 */

#ifndef UTILS_METROCOMMUNICATION_INCLUDE_COMMUNICATION_MANAGER_H
#define UTILS_METROCOMMUNICATION_INCLUDE_COMMUNICATION_MANAGER_H

#include "metrocommunication.h"
#include "definitions.h"
#include "PosixTimer.h"

#include <stdint.h>
#include <unistd.h>

#include <cstdlib>
#include <deque>
#include <string>
#include <vector>

#include <boost/asio.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <sys/time.h>


// The length of a message should fit in this type.
// The plan is to change this from one byte to four.
typedef unsigned char cmd_len_t;

class CommunicationManager {

public:
  CommunicationManager(std::string type_id, std::string name_id,
                       std::string cs_host, uint32_t cs_port);
  
  ~CommunicationManager();
  
  // Called by boost when starting a thread
  void operator()();

  // State management.
  int get_state() const {
    return current_state;
  }
  
  // Have we connected and been assigned a session id?
  bool is_registered() const {
    return session_id > 0;
  }

  // Connect to the central server.
  bool connect();
  void disconnect();
  bool is_comm_alive() const;

  // Updates and maintains the internal state machine.
  void update();

  // Is anything buffered in the socket, waiting to be read?
  bool message_waiting() const;

  // Place a message on the out queue
  void send_message(std::string msg);

  // Grab the next message from the in queue, if any
  std::string next_message();

  // Get this session's id
  uint64_t get_session_id();

  // Are both the input and output queues empty?
  bool queues_are_empty();

private:
  std::deque<std::string> in_queue;        // Incoming message queue
  std::deque<std::string> out_queue;       // Outgoing message queue

  boost::mutex in_queue_mutex;             // Mutexes for the queues
  boost::mutex out_queue_mutex;

  // CommunicationManager properties.
  std::string type_id;                     // e.g., "Blackfin"
  std::string name_id;                     // e.g., "blackfin-10"
  std::string cs_host;                     // central server hostname
  uint32_t cs_port;
  std::vector<std::string> provides_list;  // e.g. "position2d"

  std::string front;                       // front of queue

  // Boost ASIO (for sockets)
  boost::asio::io_service io_service;
  boost::asio::ip::tcp::socket socket;

  // State properties.
  int current_state;
  uint64_t session_id;

  // Internal timers.
  metrobotics::PosixTimer silence_timer;
  metrobotics::PosixTimer state_timer;

  // Internal buffers.
  std::string string_buffer;

  // Internal functions.
  virtual void init_state();                 // Clients that need to
  virtual void init_provides();              // should override these

  // Read and Write to the com socket
  std::string read();
  bool write(const std::stringstream & ss);

  // State actions.
  void do_state_change(int state);
  void do_state_action_init();
  void do_state_action_ack();
  void do_state_action_ping_send();
  void do_state_action_pong_read();
  void do_state_action_pong_send();
  void do_state_action_listen();

  void print_time();
};

#endif  // UTILS_METROCOMMUNICATION_INCLUDE_COMMUNICATION_MANAGER_H_
