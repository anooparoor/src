/* Message.cpp
 *
 * Copyright 2013 HRTeam.
 * License:
 * Created: June 17, 2013
 * Original authors: Eric Schneider <nitsuga@pobox.com>
 */

#include "Message.h"

#include <stdio.h>
#include <string>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>

const std::string Message::SOURCE_SINGLE = "SOURCE_SINGLE";
const std::string Message::SOURCE_MULTI = "SOURCE_MULTI";

Message::Message()
  : sender_id(-1),
    destination_id("-1"),
    broadcast(false) {
}

Message::Message(std::string raw_message)
  : sender_id(-1),
    destination_id("-1"),
    broadcast(false) {
  parse_raw_message(raw_message);
}

Message::~Message() {
}

void Message::parse_raw_message(std::string raw_message) {

  std::string signature = "Message::parse_raw_message()";

  // TODO This is a test
  // Sanitize the message so it's ascii only
  for(int i = 0; i < (int)raw_message.size(); i++) {
      // ASCII defined between ' ' and '~'
      if(raw_message[i] != '\n' && 
         (raw_message[i] < ' ' || raw_message[i] > '~')) { 
          raw_message.erase(raw_message.begin() + i);
      }
  }

  std::stringstream raw_stream(raw_message);
  std::string token, remainder;

  // Read the first token
  if ( !(raw_stream >> token) ) {
    fprintf(stderr, "%s: malformed source header\n", signature.c_str());
    return;
  }

  // This message has the new-style header
  if (token == SOURCE_SINGLE || token == SOURCE_MULTI) {

    if ( !(raw_stream >> sender_id) ) {
      fprintf(stderr, "%s: malformed source header\n", signature.c_str());
      return;
    }

    getline(raw_stream, full_message);

    raw_stream.clear();
    raw_stream.str(full_message);

    raw_stream >> command;

    while ( (raw_stream >> token) )
      args.push_back(token);

  } else {  // This message uses the old headerless style

    std::stringstream ss;

    ss << token;

    command = token;

    while ( raw_stream >> token ) {
         args.push_back(token);
         ss << " " << token;
    }

    full_message = ss.str();
  }

}  //  ParseMessage()

void Message::set_args(std::string args0) {
  std::vector<std::string> args_vector;

  boost::split( args_vector, args0, boost::is_any_of(" ") );

  args = args_vector;

} // end of set_args() taking string argument

void Message::set_args(std::vector<std::string> args0) {
  args = args0;

} // end of set_args() taking vector<string> argument

void Message::set_destination_id(std::string id) {
  destination_id = id;
}

void Message::set_message_type(std::string msg_type) {
  message_type = msg_type;

} // set_message_type()

std::string Message::get_full_message() {
  return full_message;
}

std::string Message::get_command() {
  return command;
}

void Message::set_command(std::string command0) {
  this->command = command0;
}

std::vector<std::string> & Message::get_args() {
  return args;
}

uint64_t Message::get_sender_id() {
  return sender_id;
} // end of get_sender_id()

std::string Message::get_destination_id() {
  return destination_id;

} // end of get_destination_id()

std::string Message::get_message_type() {
  return message_type;
} // end of get_message_type()


std::string Message::to_string(double arg) {
  std::stringstream convert;
  std::string ret_string;

  convert << arg;
  convert >> ret_string;

  return ret_string;
}

std::string Message::to_string(int arg) {
  std::stringstream convert;
  std::string ret_string;

  convert << arg;
  convert >> ret_string;

  return ret_string;
}

int Message::to_int(std::string arg) {
  std::stringstream convert;
  int ret_int;

  convert << arg;
  convert >> ret_int;

  return ret_int;
}

double Message::to_double(std::string arg) {
  std::stringstream convert;
  double ret_double;

  convert << arg;
  convert >> ret_double;

  return ret_double;
}
