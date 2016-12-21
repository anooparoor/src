#include "auction_t.h"
#include <iostream>

int auction_t::next_id = 0;

auction_t::auction_t(const bundleplex_t &bundle, AucType& type)
{
    this->id = this->next_id;
    this->next_id++;
    this->winners.clear();
    this->bundle = bundle;
    this->bids.clear();
    this->status = AUC_NEW;
    this->type = type;
}

auction_t::auction_t(const bundle_t &bundle, AucType& type)
{
    this->id = this->next_id;
    this->next_id++;
    bundleplex_t bund;
    bund.push_back(bundle);
    this->bundle = bund;
    this->status = AUC_NEW;
    this->type = type;
}

auction_t::auction_t(const point_t &p, AucType& type)
{
    this->id = this->next_id;
    this->next_id++;
    bundleplex_t bund;
    bundle_t b;
    b.push_back(p);
    bund.push_back(b);
    this->bundle = bund;
    this->status = AUC_NEW;
    this->type = type;
}

auction_t::auction_t(string mess)
{
    sstream stream(mess);
    // Throw away AUCTION_START
    stream >> *(new string);
    // Get id
    stream >> this->id;
    // Number of bundles
    int n;
    stream >> n;
    for(int i = 0; i < n; i++)
    {
        // Number of points in bundles
        int m;
        bundle_t b;
        stream >> m;
        for(int j = 0; j < m; j++)
        {
            point_t p;
            stream >> p.first >> p.second;
            b.push_back(p);
        }
        this->bundle.push_back(b);
    }
}

auction_t::~auction_t()
{
}

int auction_t::get_id()
{
    return id;
}

int auction_t::bundle_size()
{
    return this->bundle.size();
}

bundle_t auction_t::get_bundle(int index)
{
    return this->bundle[index];
}

bundleplex_t auction_t::get_bundleplex() const
{
    return this->bundle;
}

int auction_t::bids_size()
{
    return this->bids.size();
}

bid_t auction_t::get_bid(int index)
{
    return this->bids[index];
}

AucStatus auction_t::get_status()
{
    return this->status;
}

AucType auction_t::get_type()
{
    return this->type;
}

/*int auction_t::increment_status()
{
    if(this->status == AUC_NEW)
        this->status = AUC_WAIT;
    else if(this->status == AUC_WAIT)
        this->status = AUC_RESOLVED;
    else
    {
        fprintf(stderr, "Error: Attempt to increment status on resolved auction! Auction ID: %d\n", this->id);
        return -1;
    }
    return 1;
}
*/

void auction_t::set_waiting()
{
    this->status = AUC_WAIT;
}

void auction_t::set_resolved()
{
    this->status = AUC_RESOLVED;
}

bool auction_t::isnew()
{
    return this->status == AUC_NEW;
}

bool auction_t::iswaiting()
{
    return this->status == AUC_WAIT;
}

bool auction_t::isresolved()
{
    return this->status == AUC_RESOLVED;
}

int auction_t::winners_size()
{
    return this->winners.size();
}

std::vector<string> auction_t::get_winners()
{
    std::vector<string> ret;
    switch(this->type)
    {
        case AUT_SSI:
            this->construct_winner();
            break;
        case AUT_OSI:
        case AUT_PSI:
        default:
            this->construct_winners();
    }

    for(int i = 0; i < (int)this->winners.size(); i++)
    {
        // winners holds two-tuple: robot_id, index
        bundle_t *bundle_won = &bundle[winners[i].second];
        for(bundle_t::iterator j=bundle_won->begin(); j != bundle_won->end(); j++) {

            sstream ss;
            ss << "SINGLE " << winners[i].first << " AUCTION_WIN "
                << j->first << " " << j->second;

            fprintf(stderr, "In get_winners(): %s\n", ss.str().c_str());
            ret.push_back(ss.str());
        }
    }
    return ret;
}

void auction_t::add_bid(bid_t bid)
{
    this->bids.push_back(bid);
}

// Gets the single lowest bid (for SSI)
void auction_t::construct_winner()
{
    this->winners.clear();
    double biddex = 0;
    for(int i = 1; i < (int)this->bids.size(); i++)
    {
        if(bids[i].bid < bids[biddex].bid)
        {
            biddex = i;
        }
    }
    this->winners.push_back(point_t(bids[biddex].rob_id, bids[biddex].index));
}

// Gets the minimum bids for any indexes that were bid on
void auction_t::construct_winners()
{
    this->winners.clear();
    std::vector<int> found_winners;
    for(int i = 0; i < (int)this->bids.size(); i++)
    {
        // Don't do anything for indexes we've checked already
        bool found = false;
        for(int j = 0; j < (int)found_winners.size(); j++)
        {
            if(found_winners[j] == bids[i].index)
            {
                found = true;
                break;
            }
        }
        if(found)
            continue;
        found_winners.push_back(bids[i].index);

        int biddex = i;
        for(int j = i+1; j < (int)this->bids.size(); j++)
        {
            if(bids[j].index == bids[i].index)
            {
                if(bids[j].bid < bids[biddex].bid)
                {
                    biddex = j;
                }
            }
        }
        // winners holds two-tuple: robot_id, index
        this->winners.push_back(point_t(bids[biddex].rob_id, bids[biddex].index));
    }
}

string auction_t::construct_message()
{
  // E.g.,
  // BROADCAST_TYPE stage AUCTION_START 1380729193 0 3 1 55 55 1 82 73 1 91 147

    sstream ss;
    // New Auction
    ss << "BROADCAST_TYPE " << SKYGRID_ROBOT_TYPE << " AUCTION_START ";
    // Auction ID
    ss << this->id << " ";
    // Bundleplex size

    ss << this->bundle.size() << " ";
    // Possible bundles to auction for, a robot will return a bid on ONE bundle
    for(bundleplex_t::iterator i = this->bundle.begin(); i != this->bundle.end(); i++)
    {
        // Bundle size
        ss << i->size() << " ";
        for(bundle_t::iterator j = i->begin(); j != i->end(); j++)
            ss << j->first << " " << j->second << " ";
    }
    return ss.str();
}

