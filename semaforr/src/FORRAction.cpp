/*
 * Implementation of FORRAction class
 * It got complicated because this class is the key
 * for the map I am using for Tier3 advisers so there
 * is whole lot of thing that we need to define.
 *
 * Slavisa Djukic <sdjukic@hunter.cuny.edu>
 * July 22, 2013 
 */

#include "FORRAction.h"

// I think we need this copy constructor because adviser's
// function is returning a map and for datatypes that
// are returned from function we have to define copy constructor
// because they can be placed on the right side of assignment 
// operator
FORRAction::FORRAction(const FORRAction& action){
  type = action.type;
  parameter = action.parameter;
  decisionTier = action.decisionTier;
  vetoedActions = action.vetoedActions;
  advisors = action.advisors;
  advisorComments = action.advisorComments;
  }

// We need to overload this operator because this class is 
// used as a key to the map. In C++ if data type is used
// as a key in the map it has to have this operator defined.
bool FORRAction::operator < (const FORRAction &action) const{
  if(type < action.type)
    return true;
  else if (type == action.type)
    if(parameter < action.parameter)
      return true;
    else 
      return false;
  else
    return false;
}
