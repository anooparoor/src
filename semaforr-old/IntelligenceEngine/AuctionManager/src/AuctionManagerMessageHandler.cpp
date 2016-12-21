/* AuctionManagerMessageHandler.cpp
 *
 * Handle messages coming in to the auction manager.
 *
 * Copyright 2013 HRTeam.
 * License:
 * Created: Sep 30, 2013
 * Original authors: Eric Schneider <eric.schneider@liverpool.ac.uk>
 */

#include <string>

#include "AuctionManagerMessageHandler.h"
#include "definitions.h"
#include "manager.h"

// TODO: don't hardcode this
#define ROBOT_NAME_STRING "blackfin"

AuctionManagerMessageHandler::AuctionManagerMessageHandler(CommunicationManager *comm_mgr)
  : MessageHandler(comm_mgr), manager_(NULL) {
  register_callbacks();
}

void AuctionManagerMessageHandler::set_manager(manager *mgr) {
  this->manager_ = mgr;
}

void AuctionManagerMessageHandler::register_callbacks() {

  register_callback("-1", static_cast<MessageCallback>(&AuctionManagerMessageHandler::cmd_print_handler));
  register_callback("DELAYED", static_cast<MessageCallback>(&AuctionManagerMessageHandler::cmd_nop_handler));
  register_callback("MOVING", static_cast<MessageCallback>(&AuctionManagerMessageHandler::cmd_nop_handler));
  register_callback("AUCTION_STATUS", static_cast<MessageCallback>(&AuctionManagerMessageHandler::cmd_nop_handler));
  register_callback(CMD_IDENT, static_cast<MessageCallback>(&AuctionManagerMessageHandler::cmd_ident_handler));
  register_callback(CMD_AUCTION_BID, static_cast<MessageCallback>(&AuctionManagerMessageHandler::cmd_auction_bid_handler));
  register_callback(CMD_AUCTION_COMPLETE, static_cast<MessageCallback>(&AuctionManagerMessageHandler::cmd_auction_complete_handler));
  register_callback(CMD_AUCTION_FINISHED, static_cast<MessageCallback>(&AuctionManagerMessageHandler::cmd_auction_finished_handler));
}

void AuctionManagerMessageHandler::cmd_ident_handler(Message msg) {

  std::vector<std::string> ids;
  size_t pos = 0;

  std::string ident_message = msg.get_full_message();

  if ( ident_message.empty() ) {
    return;
  }

  while(true) {
    pos = ident_message.find(ROBOT_NAME_STRING, pos);
    if(pos == std::string::npos)
      break;
    ids.push_back(ident_message.substr(ident_message.rfind(' ', pos-2)+1, 10));
    pos++;
  }

  manager_->set_robots(ids.size()); // Redundant, get rid of
  manager_->set_robot_ids(ids);
}


void AuctionManagerMessageHandler::cmd_auction_bid_handler(Message msg) {

  // AUCTION_BID message format:
  // AUCTION_BID <robot_id> <auction_id> <bid> <auctioneer_id>

  sstream convert;
  int robot_id, auction_id, bundle_index;
  double bid;

  robot_id = (int)msg.get_sender_id();

  std::vector<std::string> args = msg.get_args();

  auction_id = Message::to_int(args[0]);
  bundle_index = Message::to_int(args[1]); // Is this necessary?
  bid = Message::to_double(args[2]);

  fprintf(stderr, "BID: robot %d bids %f on task %d\n", robot_id, bid, auction_id);

  // Check for auc_id, and check for winner in that ID
  auction_t *auc = manager_->get_auction_by_id(auction_id);

  if(auc == NULL) {
    fprintf(stderr, "Error: No such id\n");
    return;
  }

  if(auc->isresolved()) {
    fprintf(stderr, "Error: Bid for resolved auction?\n");
    return;
  }

  //  man->add_bid(bidstream.str(), auc_id);
  manager_->add_bid(robot_id, auction_id, bundle_index, bid);

  std::vector<string> winners;
  winners = manager_->check_auc(auction_id);

  if(!winners.empty()) {
    auc->set_resolved();
    for(int i = 0; i < (int)winners.size(); i++) {
      std::string win_message = winners[i];

      fprintf(stderr, "WINNER: %s\n", win_message.c_str());

      send_message(win_message);
    }
  }
}

void AuctionManagerMessageHandler::cmd_auction_complete_handler(Message msg) {
  manager_->auction_completed();
}

void AuctionManagerMessageHandler::cmd_auction_finished_handler(Message msg) {
  // Nothing for now.
}
