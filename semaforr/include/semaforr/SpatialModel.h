/*!
 * SpatialModel.h
 *
 * \brief Represents the current spatial model of the environment: Regions, exits, conveyors, etc
 *
 * \author Anoop Aroor
 * \date 11/11/2016 Created
 */

#include <FORRAbstractMap.h>
#include <FORRTrails.h>
#include <FORRWaypoints.h>
#include <FORRDoors.h>

class SpatialModel{

public:
	SpatialModel(double width, double height, double granularity){
		abstract_map = new FORRAbstractMap();
		//trace = new FORRTrace();
		trails = new FORRTrails();
		waypoints = new FORRWaypoints(width, height, granularity);
		doors = new FORRDoors();
	};

        FORRAbstractMap* getAbstractMap(){return abstract_map;}
	//FORRTrace* getTrace(){return trace;}
	FORRTrails* getTrails(){return trails;}
	FORRWaypoints* getWaypoints(){return waypoints;}
	FORRDoors* getDoors(){return doors;}

private:
	FORRAbstractMap *abstract_map;
        //FORRTrace *trace;
	FORRTrails *trails;
	FORRWaypoints *waypoints;
	FORRDoors *doors;
};
