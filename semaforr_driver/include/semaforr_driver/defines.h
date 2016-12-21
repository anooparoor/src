#ifndef DEFINES_H
#define DEFINES_H

#include <utility>
#include <vector>
#include <string>
#include <sstream>
#include <ctime>
#include <stdio.h>
#include <cmath>

#define SKYGRID_ROBOT_TYPE "stage"
class auction_t; // For our typedef
class bid_t;

typedef std::pair<int, int> point_t;
typedef std::vector<point_t> bundle_t;
typedef std::vector<bundle_t> bundleplex_t;
typedef std::vector<auction_t> auctionplex_t;
typedef std::vector<bid_t> bidplex_t;
typedef std::string string;
typedef std::stringstream sstream;

enum AucStatus
{
    AUC_NEW, AUC_WAIT, AUC_RESOLVED
};

enum AucType
{
    AUT_RANDOM, AUT_OSI, AUT_PSI, AUT_SSI
};

#endif
