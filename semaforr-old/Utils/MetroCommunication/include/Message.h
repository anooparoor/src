/* Message.h 
 *
 * Copyright 2013 HRTeam.
 * License:
 * Created: June 17, 2013
 * Original authors: Eric Schneider <nitsuga@pobox.com>
 */

#ifndef UTILS_METROCOMMUNICATION_INCLUDE_MESSAGE_H_
#define UTILS_METROCOMMUNICATION_INCLUDE_MESSAGE_H_

#include "metrocommunication.h"

#include <stdint.h>
#include <unistd.h>

#include <cstdlib>
#include <string>
#include <vector>
#include <sstream>

class Message {

public:
  Message();
  Message(std::string raw_message);

  ~Message();

  void set_args(std::string args0);
  void set_args(std::vector<std::string> args0);
  void set_destination_id(std::string id);
  void set_message_type(std::string msg_type);

  std::string get_full_message();
  std::string get_command();
  void set_command(std::string command);
  std::vector<std::string> & get_args();
  uint64_t get_sender_id();
  std::string get_destination_id();
  std::string get_message_type();

  // This needs to be implemented here because of how the linker works
  template <class T>
  void add_arg(T arg) {
      std::stringstream ss;
      ss << arg;
      args.push_back(ss.str());
  }

  // Convenience functions
  static std::string to_string(double arg);
  static std::string to_string(int arg);

  static int to_int(std::string arg);
  static double to_double(std::string arg);

private:
  void parse_raw_message(std::string raw_message);

  // Session id of the message's sender
  uint64_t sender_id;

  // Session id(s) of the destination of the mssage
  // Used when creating a message to be sent by the
  // MessageHandler - Ken
  std::string destination_id;

  // Was this message send as part of a broadcast?
  bool broadcast;

  // part of the header
  std::string message_type;

  // The entire message stripped of its header, if any
  std::string full_message;

  // The message's command (first token) and remaining arguments
  std::string command;
  std::vector<std::string> args;

  // These header constants are defined in message.cpp
  static const std::string SOURCE_SINGLE;
  static const std::string SOURCE_MULTI;
};


#endif
