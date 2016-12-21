/* message_handler.cpp
 *
 * Copyright 2013 HRTeam.
 * License:
 * Created: June 17, 2013
 * Original authors: Eric Schneider <nitsuga@pobox.com>
 */

#include <stdio.h>
#include <boost/bind.hpp>
#include <boost/algorithm/string/join.hpp>
#include <boost/thread/mutex.hpp>

#include "MessageHandler.h"

/**
 * MessageHandler constructor
 *
 */
MessageHandler::MessageHandler(CommunicationManager *comm_mgr)
    : comm_mgr(comm_mgr) {

  // Default handlers for any client (PING, PONG handlers, etc.)
  register_callbacks();
}

/**
 * Register a handler callback for a (string) command
 *
 */
void MessageHandler::register_callback(std::string command, MessageCallback cb) {
  callbacks[command] = cb;
}

/**
 * An alias for the above with a char[] constant instead of a std::string
 *
 */
void MessageHandler::register_callback(const char command[], MessageCallback cb) {
  register_callback(std::string(command), cb);
}

void MessageHandler::register_callbacks() {

  register_callback(CMD_PING, static_cast<MessageCallback>(&MessageHandler::cmd_ping_handler));
  register_callback(CMD_PONG, static_cast<MessageCallback>(&MessageHandler::cmd_pong_handler));

}

void MessageHandler::cmd_ping_handler(Message msg) {
  // PING --> PONG
  send_message(CMD_PONG);
}

void MessageHandler::cmd_pong_handler(Message msg) {
  // PONG --> /dev/null
}

void MessageHandler::cmd_nop_handler(Message msg) {}

void MessageHandler::cmd_print_handler(Message msg) {
    fprintf(stderr, "Sender %lu sent message %s\n",
            msg.get_sender_id(), msg.get_full_message().c_str());
    return;
}

void MessageHandler::check_inbox() {

  // Make sure we're connected to the central server.
  if ( ! comm_mgr || ! comm_mgr->is_registered() ) {
    fprintf(stderr, "MessageHandler::check_inbox() - Not connected!\n");
    return;
  }

  std::string raw_message_in;

  // Pop any message(s) off of the in queue and process it
  while (true) {
    raw_message_in = comm_mgr->next_message();

    if (raw_message_in.empty())
      break;

    //    print_time();
    //    fprintf(stderr, "Raw message '%s'\n", raw_message_in.c_str());

    Message message_in(raw_message_in);

    process_message(message_in);
  }
}

void MessageHandler::process_message(Message msg) {

  //  fprintf(stderr, "Processing a '%s' message\n", msg.get_command().c_str());

  if ( callbacks.find(msg.get_command()) == callbacks.end() ) {
    fprintf(stderr, "No handler for message type '%s'!\n", msg.get_command().c_str());
  }  else {
    MessageCallback callback = callbacks[msg.get_command()];
    (this->*callback)(msg);
  }

}

void MessageHandler::send_message(std::string msg) {

  // We're not connected to the central server
  if ( ! comm_mgr || ! comm_mgr->is_registered() )
    return;

  comm_mgr->send_message(msg);
}

/**
 * send_message()
 *
 * overloaded function taking a Message as an argument
 *
 */
void MessageHandler::send_message(Message msg) {
  std::string args = boost::algorithm::join( msg.get_args(), " " );
  std::string msg_string = "";

  if (! msg.get_message_type().empty() ) {
    msg_string += msg.get_message_type() + " " + msg.get_destination_id() + " ";
  }

  msg_string += msg.get_command() + " " + args;

  //  fprintf(stderr, "Sending: %s\n", msg_string.c_str());

  send_message(msg_string);
} // end of send_message()

/**
 * get_session_id()
 *
 * Returns the id for this session (initialized in comm_mgr)
 */
uint64_t MessageHandler::get_session_id() {
 return comm_mgr->get_session_id();
}

void MessageHandler::print_time() {
  time_t rawtime;
  struct tm *tm;
  time ( &rawtime );
  tm = localtime ( &rawtime );
  // get only the milliseconds
  struct timeval tv;
  gettimeofday( &tv, NULL );
  char time_buffer[] = "00:00:00:000:000";
  sprintf( time_buffer, "%02d:%02d:%02d:%03d:%03d", tm->tm_hour, tm->tm_min, tm->tm_sec, (int)tv.tv_usec/1000, (int)tv.tv_usec%1000 );

  fprintf(stderr, "%s", time_buffer);
}
