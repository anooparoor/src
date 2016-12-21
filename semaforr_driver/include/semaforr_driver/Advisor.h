/*!
 * Advisor.h
 *
 * \brief Abstract base class of all Ariadne/FORR advisors.
 *
 * \author Eric Schneider <esch@hunter.cuny.edu>, edited by Anoop
 */
#ifndef ADVISOR_H
#define ADVISOR_H

#include "FORRComment.h"
#include "Beliefs.h"

#include <string>
#include <vector>

class Advisor
{
public:

  /** Advisor constructor
   *
   */
  Advisor(string name, string description);

  /** Advisor::generateAdvise
   *
   *  The main function of an advisor is to look into the beliefs about the world and itself and
   *  produce advise . Advise could be of three types:
   *  1. Tier 1: Known correct action
   *  2. Tier 1.1: Veto moves knows to be inactive
   *  3. Tier 2: Plan
   *  4. Tier 3: Set of comments
   */ 
  void generateAdvise(Beliefs *beliefs);

  /** Advisor::explain
   *
   *  Explain 
   */
  string explain();

protected:

  // The advisor's name
  string name;

  // Short description of the advisor's purpose and function
  string description;

  // Time in seconds allowed per decision
  double time_limit;

  // "List of constant values". Type should be a well-defined, not (void*)
  std::vector<void *> aux_constants;

  bool do_reset;

  // Number of times consulted in a run
  int times_consulted;

  // Number of times produced comments in a run
  int times_commented;

  // Nmber of times resources were exhausted (e.g., time limit reached)
  // in a run
  int times_exhausted;

  // Number of times resources were exhausted while commenting
  int times_exhausted_and_commented;

  // Total time consumed in a run, in seconds
  double time_consumed;

};

#endif
