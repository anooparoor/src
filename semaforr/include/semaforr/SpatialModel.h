/*!
 * SpatialModel.h
 *
 * \brief Represents the current spatial model of the environment: Regions, exits, conveyors, etc
 *
 * \author Anoop Aroor
 * \date 11/11/2016 Created
 */

#include <FORRAbstractMap.h>
#include <FORRTrace.h>
#include <FORRTrails.h>
#include <FORRWaypoints.h>

class SpatialModel{

public:
	SpatialModel(){
		abstract_map = new FORRAbstractMap();
		trace = new FORRTrace();
		trails = new FORRTrails();
		waypoints = new FORRWaypoints();
	};

        FORRAbstractMap* getAbstractMap(){return abstract_map;}
	FORRTrace* getTrace(){return trace;}
	FORRTrails* getTrails(){return trails;}
	FORRWaypoints* getWaypoints(){return waypoints;}

private:
	FORRAbstractMap *abstract_map;
        FORRTrace *trace;
	FORRTrails *trails;
	FORRWaypoints *waypoints;
};
