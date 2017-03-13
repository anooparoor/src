/*
 * This is header file for tier-1 advisors. 
 *
 * Date Created: Jan. 7, 2014.
 * Last Edited: Jan. 7, 2014.
 * Created by: Slavisa Djukic <sdjukic@hunter.cuny.edu>
 */

#include <string>
#include <vector>
#include "FORRAction.h"
#include "Beliefs.h"

// A t1 advisor class contains a list of functions each of which returns a single action or vetoes a set of actions
class Tier1Advisor{
public:

        //void advisorNotOpposite(Beliefs *beliefs);
	//void advisorCircle(Beliefs *beliefs);
	bool advisorAvoidWalls(Beliefs *beliefs);

	bool advisorVictory(FORRAction *decision, Beliefs *beliefs);
	
};
