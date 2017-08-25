/*!
 * FORRActionStats.h
 *
 * Class that collects data about a decision
 *
 * \author Raj Korpan <rkorpan@gradcenter.cuny.edu>
 */
#ifndef FORRACTIONSTATS_H
#define FORRACTIONSTATS_H

#include <string>
#include <iostream>
#include <set>

class FORRActionStats {

  public:
    int decisionTier;
    std::string vetoedActions;
    std::string advisors;
    std::string advisorComments;
    std::string advisorInfluence;

    FORRActionStats(int decTier, std::string vActions, std::string adv, std::string advComments, std::string advInfluence) : decisionTier(decTier), vetoedActions(vActions), advisors(adv), advisorComments(advComments), advisorInfluence(advInfluence) {};
    FORRActionStats(): decisionTier(0), vetoedActions(" "), advisors(" "), advisorComments(" "), advisorInfluence(" ") {};

};


#endif
