/*
 * Declaration of FORRWall. Construct we use to represent walls in our maze.
 *
 * Last Modified: March 31, 2014.
 * Created by: Slavisa Djukic <sdjukic@hunter.cuny.edu>
 */
#ifndef FORRWALL_H
#define FORRWALL_H

#include <utility>          // for pair
#include "FORRGeometry.h"

using std::pair;

class FORRWall: public LineSegment{
 public:
  /***************************************************************************
                             Constructors
  ****************************************************************************/
  FORRWall(double x1, double y1, double x2, double y2);
  FORRWall(CartesianPoint p1, CartesianPoint p2);
  FORRWall(){};
  ~FORRWall();
  /***************************************************************************
                              Methods
  ****************************************************************************/

  // function will add buffers to the end point of the wall
  // the size of buffer will depend on the robot size
  // function will change wall size directly
  void buffer_wall(double size);

  // function will return endpoints of the wall
  pair<CartesianPoint, CartesianPoint> get_endpoints();
  
};

#endif
