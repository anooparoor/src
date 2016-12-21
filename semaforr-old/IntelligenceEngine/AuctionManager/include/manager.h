/* Auction Manager object:
 * The manager object is in charge of the auction process, it will take in
 * a bundle of points, bundle them into an auction item, and then broadcast such
 * an auction to the robots, it will then choose a winner for some points in the
 * auction, rebundle the auction, and rebroadcast until there are no unallocated
 * points in the auction.
 *
 * Author: Ofear Balas <ofear.balas@gmail.com>
 */

#ifndef MANAGER_H
#define MANAGER_H

#include <string>
#include <vector>

#include "defines.h"
#include "definitions.h"
#include "auction_t.h"
#include "AuctionManagerMessageHandler.h"

//class AuctionManagerMessageHandler;

class manager {
    private:

        auctionplex_t auctions;
        AucType auc_type;
        unsigned int robots;
        bool single_auction;
        std::vector<std::string> robot_ids;

	int random_auc_id;

        AuctionManagerMessageHandler *message_handler;

        // Private funcs
        //

        // Reauction: takes an auction and a list of bundles to remove from said
        // auction, returns a new auction with the same bundleplex sans the
        // auctioned off points
        auction_t pbi_rebundler(auction_t, bundleplex_t);
        void ssi_reauction(const auction_t*, string);

	static unsigned int auctions_completed;

    public:
        manager(int, int, AuctionManagerMessageHandler*);
        ~manager();

        bool is_single();
        int get_robots();
        void set_robots(int);
        int auctions_size();
        int ongoing_auctions(); // Return number of uncompleted auctions
        auction_t *get_auction(int);
        auction_t *get_auction_by_id(int);
        AucType get_auc_type();
        void add_bid(int, int, int, double);
        std::vector<string> check_auc(int);

        // Auctioneers: Add new auctions by bundling points with these
        // auctioneers
        void do_auctioneer(const bundle_t&); // Chooses the right auctioneer based on auc_type
        void pbi_auctioneer(const bundle_t&);
        void psi_auctioneer(const bundle_t&); // PSI and OSI: adds an auction for each point
        void ssi_auctioneer(const bundle_t&);

        // Do some work. At least check out inbox.
        void update();

        std::vector<std::string> get_robot_ids();
        void set_robot_ids(std::vector<std::string>);

        void send_ident();
        void send_cmd_auction_won(string id, int x, int y);

        // TODO: This is a hack for man_loop() in main.cpp. Make it cleaner.
        void send_message(std::string msg);

	void auction_completed();
	unsigned int num_auctions_completed();

	// Pause the bots during the auction phase. Resume when
	// the auction is finished.
	void broadcast_wait();
	void broadcast_resume();
};

#endif
