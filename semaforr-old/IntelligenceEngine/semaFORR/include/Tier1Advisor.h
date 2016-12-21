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

/*
 * At this point there is only one tier1 advisor which will veto all the
 * actions that would result in colliding with wall.
 * Later we can take out actions that would collide with other robots, but
 * I think there are better ways to solve collision avoidance between robots.
 *
 * When I thought of it I think this file is not necessary, right now.
 * Because it is not clear what should be common functionality of tier1
 * advisors. What I will do is create one tier1 advisor inside RC.
 */
