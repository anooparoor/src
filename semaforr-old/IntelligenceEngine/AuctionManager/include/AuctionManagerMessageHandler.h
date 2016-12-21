/* AuctionManagerMessageHandler.h
 *
 * Copyright 2013 HRTeam.
 * License:
 * Created: Sep 30, 2013
 * Original authors: Eric Schneider <eric.schneider@liverpool.ac.uk>
 */

#ifndef AUCTION_MANAGER_MESSAGE_HANDLER_H
#define AUCTION_MANAGER_MESSAGE_HANDLER_H

//#include "manager.h"
#include "MessageHandler.h"

class manager;

class AuctionManagerMessageHandler : public MessageHandler {

public:

  AuctionManagerMessageHandler(CommunicationManager *comm_mgr);

  void set_manager(manager *mgr);

private:

  // This can't be named 'manager'. That's the name of the class.
  manager *manager_;

  void register_callbacks();

  void cmd_ident_handler(Message msg);

  void cmd_auction_bid_handler(Message msg);

  void cmd_auction_complete_handler(Message msg);

  void cmd_auction_finished_handler(Message msg);
};

#endif  // AUCTION_MANAGER_MESSAGE_HANDLER_H
