#include <sstream>

#include "manager.h"

unsigned int manager::auctions_completed = 0;

manager::manager(int robots, int type, AuctionManagerMessageHandler *message_handler)
  : robots(robots), random_auc_id(0), message_handler(message_handler)
{
    this->auctions.clear();
    switch(type)
    {
        case 0:
            this->auc_type = AUT_RANDOM;
            this->single_auction = false;
            break;
        case 1:
            this->auc_type = AUT_OSI;
            this->single_auction = true;
            break;
        case 2:
            this->auc_type = AUT_PSI;
            this->single_auction = false;
            break;
        default:
            this->auc_type = AUT_SSI;
            this->single_auction = true;
            break;
    }
}

manager::~manager()
{
}

auction_t manager::pbi_rebundler(auction_t oldauc, bundleplex_t alloced)
{
    bundleplex_t ret;
    for(int i = 0; i < oldauc.bundle_size(); i++)
    {
        bool exists = false;
        for(int j = 0; j < (int)alloced.size(); j++)
        {
            if(alloced[j] == oldauc.get_bundle(i))
            {
                exists = true;
                break;
            }
        }
        if(!exists)
            ret.push_back(oldauc.get_bundle(i));
    }
    return auction_t(ret, this->auc_type);
}

void manager::ssi_reauction(const auction_t *oldauc, string windex)
{
    sstream ss(windex);
    string throwaway;
    int eindex;
    //    ss >> throwaway;
    //    ss >> eindex >> eindex >> eindex; // get rid of mess type, auc_id, rob_id and get index
    ss >> throwaway >> eindex >> throwaway >> eindex >> eindex;
    bundleplex_t bund = oldauc->get_bundleplex();
    if(bund.size() == 1)
        return;
    bund.erase(bund.begin()+eindex);
    this->auctions.push_back(auction_t(bund, this->auc_type));
}

bool manager::is_single()
{
    return this->single_auction;
}

int manager::get_robots()
{
    return this->robots;
}

void manager::set_robots(int robots) {
    if(robots < 0)
        return;
    this->robots = robots;

}

int manager::auctions_size()
{
    return this->auctions.size();
}

int manager::ongoing_auctions() {
    int count = 0;
    for(size_t i = 0; i < this->auctions.size(); i++)
        if(!auctions[i].isresolved())
            count++;
    return count;
}

auction_t *manager::get_auction(int index)
{
    return &*(auctions.begin() + index);
}

auction_t *manager::get_auction_by_id(int auc_id)
{
    for(int i = 0; i < (int)this->auctions.size(); i++)
    {
        if(this->auctions[i].get_id() == auc_id)
            return &*(auctions.begin() + i);
    }
    return NULL;
}

AucType manager::get_auc_type()
{
    return this->auc_type;
}

void manager::add_bid(int robot_id, int auction_id, int bundle_index, double bid)
{
  bid_t b;
  b.bid = bid;
  b.rob_id = robot_id;
  b.index = bundle_index;

  bool exists = false;
  for(int i = 0; i < (int)this->auctions.size(); i++) {
    if(auctions[i].get_id() == auction_id) {
      auctions[i].add_bid(b);
      exists = true;
    }
  }
  if(!exists)
    fprintf(stderr, "ERROR: No such auction!\n");
}

//
std::vector<string> manager::check_auc(int auc_id)
{
    auction_t *auc = this->get_auction_by_id(auc_id);
    std::vector<string> ret;
    if(auc == NULL)
        return ret;

    fprintf(stderr, "bids_size: %d, robots: %d\n", auc->bids_size(), this->robots);
    if( auc->bids_size() >= this->robots )
    {
        ret = auc->get_winners();
        switch(auc->get_type())
        {
	case AUT_SSI:
	    auc->set_resolved();
	    this->ssi_reauction(auc, ret[0]);
	  break;
	default:
	  break;
        }
    }

    return ret;
}


void manager::do_auctioneer(bundle_t const &points)
{
    switch(this->auc_type)
    {
        case AUT_SSI:
            this->ssi_auctioneer(points);
            break;
        case AUT_PSI:
        case AUT_OSI:
        default:
            this->psi_auctioneer(points);
            break;
    }

    return;
}

void manager::pbi_auctioneer(const bundle_t &points)
{
    bundleplex_t bplex; bplex.clear();
    for(size_t i = 0; i < points.size(); i++)
    {
        bundle_t bund;
        bund.push_back(points[i]);
        bplex.push_back(bund);
    }
    auction_t ret(bplex, this->auc_type);
    this->auctions.push_back(ret);
}

void manager::psi_auctioneer(const bundle_t &points)
{
    for(size_t i = 0; i < points.size(); i++)
    {
        auction_t ret(points[i], this->auc_type);
        this->auctions.push_back(ret);
    }
}

void manager::ssi_auctioneer(const bundle_t &points)
{
    bundleplex_t bund;
    for(size_t i = 0; i < points.size(); i++)
    {
        bundle_t b;
        b.push_back(points[i]);
        bund.push_back(b);
    }
    auction_t ret(bund, this->auc_type);
    this->auctions.push_back(ret);
}

void manager::update() {
  message_handler->check_inbox();
}

void manager::send_ident() {
  if (message_handler == NULL) {
    return;
  }

  Message ident_message(CMD_IDENT);

  message_handler->send_message(ident_message);
}

std::vector<std::string> manager::get_robot_ids() {
  return robot_ids;
}

void manager::set_robot_ids(std::vector<std::string> ids) {
  this->robot_ids = ids;
}

void manager::send_cmd_auction_won(string winner_id, int x, int y) {

  Message won_message;
  std::vector<std::string> args;
  std::ostringstream ostr;

  won_message.set_message_type("SINGLE");
  won_message.set_destination_id(winner_id);

  won_message.set_command(CMD_AUCTION_WON);

  won_message.add_arg(random_auc_id);
  random_auc_id++;
  won_message.add_arg(0);
  won_message.add_arg(x);
  won_message.add_arg(y);

  message_handler->send_message(won_message);
}

void manager::send_message(std::string msg) {
  message_handler->send_message(msg);
}

void manager::auction_completed() {
  auctions_completed++;
}

unsigned int manager::num_auctions_completed() {
  return auctions_completed;
}

void manager::broadcast_wait() {

  Message wait_message;

  wait_message.set_message_type(DEST_BROADCAST_TYPE);
  wait_message.set_destination_id(SKYGRID_ROBOT_TYPE);

  wait_message.set_command(CMD_HWAIT);

  message_handler->send_message(wait_message);
}

void manager::broadcast_resume() {

  Message wait_message;

  wait_message.set_message_type(DEST_BROADCAST_TYPE);
  wait_message.set_destination_id(SKYGRID_ROBOT_TYPE);

  wait_message.set_command(CMD_HRESUME);

  message_handler->send_message(wait_message);
}
