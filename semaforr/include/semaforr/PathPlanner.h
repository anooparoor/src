/*! \file PathPlanner.h

  \addtogroup PathPlanner
  @{
*/

#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include "astar.h"
#include <semaforr/CrowdModel.h>
#include <math.h>

/*! 
  \brief PathPlanner class in PathPlanner module

  This class manages the major navigational waypoints that construct a path from a source to a target. 
 */

class PathPlanner {
private: 
  Graph * navGraph; 
  Map map;
  semaforr::CrowdModel crowdModel;
  Node source, target; 
  list<int> path; 
  double pathCost;

  //list<int>::iterator head;
  Node waypoint; 
  bool objectiveSet;
  bool pathCompleted;
  bool pathCalculated;

  void smoothPath(list<int>&, Node, Node);
  double computeCrowdFlow(Node s, Node d);
  double projection(double angle, double length, double xs, double ys, double xd, double yd);

public: 
  /*! \brief C'tor (only version) 
    \param Graph navigation graph 
    \param Node starting point (source)
    \param Node destination point (target)
  */
 PathPlanner(Graph * g, Map& m, Node s, Node t): navGraph(g), map(m), source(s), target(t), pathCalculated(false){}

  int calcPath(bool cautious = false); 

  /*! \return list of node indexes of waypoints */
  list<int> getPath(){ return path; }

  void resetPath() { 
    path.clear();
    pathCompleted = true; 
    pathCalculated = false;
  }

  void setCrowdModel(semaforr::CrowdModel c){ 
	crowdModel = c;
  }
  semaforr::CrowdModel getCrowdModel(){ return crowdModel;}

  void updateNavGraph();
  double computeNewEdgeCost(Node s, Node d, bool direction, double oldcost);

  Graph* getGraph(){ return navGraph; }

  Map* getMap() { return &map;}

  Node getSource(){ return source; }

  void setSource(Node s){ source = s; } 

  Node getClosestNode(Node n, Node ref);

  Node getTarget(){ return target; }

  double cellCost(int sx, int sy, int buffer);

  void setTarget(Node t){ target = t; }

  bool isAccessible(Node s, Node t);


  /*! \brief returns true if the line segment between \f$(x1,y1)\f$ and \f$(x2,y2)\f$ 
   *         is not obstructed by known obstacles represented in the map.
   */
  bool isInLineOfSight(int x1, int y1, int x2, int y2) { return !map.isPathObstructed(x1, y1, x2, y2); }


  //! Checks if a given \f$(x,y)\f$ is inside the buffer zones of the walls. 
  bool isPointCloseToWalls(int x, int y) { return map.isPointInBuffer(x, y); }


  /*! \brief returns a list of waypoints between arbitrary points, from source \f$(x1,y1)\f$ ending at target \f$(x2,y2)\f$.
   *         this function will not change the path that the robot is going to follow. it is used for checking if 
   *         paths exist between any two points, generally required for reasoning. Works a bit different than calcPath()
   *        
   *
   *  \param (x1,y1) represents the starting point or the source
   *  \param (x2,y2) represents the end point or the target
   *
   *  \return a list of \f$(x,y)\f$ pairs starting from the source ending with target
   *
   *  Initially if the \f$(x1,y1)\f$ and/or \f$(x2, y2)\f$ are not valid graph nodes (most of the cases are in this category)
   *  the function will call getClosestNode() which will return the closest valid node to the point, if one exists 
   *  within reasonable distance from it. If this fails the function will return the list containing a single pair
   *  <em> (source_error, target_error) </em>, which the values will be -1 if this is the case. 
   *  e.g. \f$(-1,0)\f$ means a valid source node was not found, while \f$(-1,-1)\f$ means neither a valid source or a target
   *       was found
   *
   *  If both source and target are valid but there is no path from one to the other the returned list will be empty.
   * 
   *  If a path is found it will be smoothed (first waypoint and last waypoint will be tested for relevance) 
   *
   *  \sa calcPath()
   *  \sa getClosestNode()
   *  \sa smoothPath()
   */
  list<pair<int,int> > getPathXYBetween(int, int, int, int);

  /*! \brief return the length of a given path in pairs of (x,y)
   */
  int getPathLength(list<pair<int,int> > path);

  /*! \brief this is used for checking if the source and target points on the map are within borders & not on a wall
   */
  bool isNodeValid(Node n){
    return (map.isWithinBorders(n.getX(), n.getY()) && map.isAccessible(n.getX(), n.getY()));
  }

  bool pathEmpty() { return path.empty(); }

  double getPathCost() { return pathCost; }

  double getRemainingPathLength(double x, double y);  
    
  double calcPathCost(list<int>);

  double estimateCost(Node, Node, int); 

  double estimateCost(int x1, int y1, int x2, int y2);
  
  //! returns the first waypoint in the path
  Node getWaypoint() { 
    Node wp ; 
    ( !pathEmpty() ) ? wp = navGraph->getNode(path.front()) : wp = target; 
    return wp; 
  } 


  //! returns the second waypoint in the path
  Node getNextWaypoint() {
    Node wp; 
    if ( !pathEmpty() ) {
      list<int>::iterator iter=path.begin();
      iter++; 
      ( iter != path.end() ) ? wp = navGraph->getNode((*iter)) : wp = target; 
    }
    return wp;
  }

  bool isObjectiveSet() { return objectiveSet; }
 
  void waypointReached() {
    if ( !pathEmpty() )
      path.pop_front(); 
    objectiveSet = false;
  }
  
  // Double check: The change in Graph::addNode(Node&) returns a reference to the actual
  // node. This may cause it to be consumed when robot follows the actual path
  void waypointSet() {
    objectiveSet = true; 
    ( !pathEmpty() ) ? waypoint = navGraph->getNode(path.front()) : waypoint = target;
  }

  bool isPathCompleted() { 
    return pathCompleted;
  }

  bool isPathCalculated() { 
    return pathCalculated; 
  }

  /* Wrapper functions for adding 'hard' obstacles. These change the usability of edges */ 
  void clearGraph() { 
    navGraph->clearGraph(); 
  }
  
  void printPath();

  void printPath(list<int>);

  void printPath(list<pair<int,int> > p); 

  bool allWaypointsValid();
};

#endif

/*! @} */
