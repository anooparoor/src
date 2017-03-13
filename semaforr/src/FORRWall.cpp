/* 
 * Implementation of FORRWall class.
 *
 * Last Changed: March 31, 2014.
 * Created by: Slavisa Djukic <sdjukic@hunter.cuny.edu>
 */

#include "FORRWall.h"
#include <iostream>
#include <math.h>

using std::cout;
using std::endl;

/***************************************************************************
                       Constructors
***************************************************************************/
FORRWall::FORRWall(double x1, double y1, double x2, double y2): LineSegment(x1, y1, x2, y2) {}


FORRWall::FORRWall(CartesianPoint p1, CartesianPoint p2): LineSegment(p1, p2) {}


FORRWall::~FORRWall() {}
/***************************************************************************
                         Methods
****************************************************************************/
void FORRWall::buffer_wall(double size){
  double slope = Line(end_point_1, end_point_2).get_slope();
  double angle = atan(slope);

  CartesianPoint p1, p2, p3, p4;
  /*
   * We are interested in two of this points whose distance from each other is largest
   */
  p1 = CartesianPoint(end_point_1.get_x() + size*cos(angle), end_point_1.get_y() + size*sin(angle));
  p2 = CartesianPoint(end_point_1.get_x() - size*cos(angle), end_point_1.get_y() - size*sin(angle));

  p3 = CartesianPoint(end_point_2.get_x() + size*cos(angle), end_point_2.get_y() + size*sin(angle));
  p4 = CartesianPoint(end_point_2.get_x() - size*cos(angle), end_point_2.get_y() - size*sin(angle));

  /*
   * And decided that this is the easiest way to check it.
   * The newly created point which is further away from other end point of the wall
   * is the end point of the buffered wall.
   */
  if(distance(p1, end_point_2) > distance(p2, end_point_2))
    end_point_1 = p1;
  else
    end_point_1 = p2;

  if(distance(p3, end_point_1) > distance(p4, end_point_1))
    end_point_2 = p3;
  else
    end_point_2 = p4;
	  
}


pair<CartesianPoint, CartesianPoint> FORRWall::get_endpoints(){

  return std::make_pair(end_point_1, end_point_2);
}


