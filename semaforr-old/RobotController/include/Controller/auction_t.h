#ifndef AUCTION_T_H
#define AUCTION_T_H

#include "defines.h"
#include "bid_t.h"

class auction_t {
    private:
        static int next_id;
        int id;
        // We use a bundle_t to represent winners since it's just a pair of ints
        // which is what we need to represent the robot and the bundle index
        bundle_t winners;
        bundleplex_t bundle;
        bidplex_t bids;
        AucStatus status;
        AucType type;

        //timespec timeout;
    public:
        auction_t(const bundleplex_t&, AucType&); // Multi-winner auction
        auction_t(const bundle_t&, AucType&);    // Single bundle auction
        auction_t(const point_t&, AucType&);    // Single point auction
        auction_t(string);       // Construct auction from AUCTION_START
        ~auction_t();

        int get_id();
        int bundle_size();
        bundle_t get_bundle(int);
        bundleplex_t get_bundleplex() const;
        int bids_size();
        bid_t get_bid(int);
        AucStatus get_status();
        AucType get_type();

        //int increment_status();
        void set_waiting();
        void set_resolved();
        bool isnew();
        bool iswaiting();
        bool isresolved();

        int winners_size();
        std::vector<string> get_winners();

        void add_bid(bid_t);
        void construct_winner();
        void construct_winners();

        string construct_message();
};

#endif
