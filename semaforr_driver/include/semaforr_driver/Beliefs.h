/*!
 * Beliefs.h
 *
 * \brief Contains spatial representations of the world (e.g. Map, Graph),
 *        the agent's curren state (an AgentState), the agent's knowledge
 *        about other agents and whatever else may be useful.
 *
 * \author Anoop Aroor 
 *
 */
#ifndef BELIEFS_H
#define BELIEFS_H

#include "AgentState.h"
#include "TeamState.h"
#include "FORRAction.h"
#include "Map.h"
#include "SpatialModel.h"

#include <time.h>
#include <list>
#include <map>
#include <string>
#include <set>
#include <fstream>
#include <sstream>
#include <vector>

using std::vector;
using std::set;


class Beliefs
{
public:
    /*! \brief Belief constructor
     *
     * \param m A line segment map
     */

    Beliefs(){
	agentState = new AgentState();
	teamState = new TeamState();
	spatialModel = new SpatialModel();
    }

    Beliefs(Map *m) : map(m) {
        agentState = new AgentState();
	teamState = new TeamState();
	spatialModel = new SpatialModel();
    }

        
 private:

    /** \brief A line-segment representation of the world, in situation when the map is already known */
    Map *map;
    
    /** \brief The agent's current position, its task status and immediate sensor values*/
    AgentState *agentState;
    
    /*! \brief Manages the state of the agent's teammates */
    TeamState *teamState; 

    /*! \brief Manages the environment model learned by the agent */ 
    SpatialModel *spatialModel;
};

#endif

