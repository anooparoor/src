/************************************************
FORRExit.h 
This file contains the class FORRExit which is intended to be a data structure which abstracts the notion of an exit

Written by Anoop Aroor, 2014
**********************************************/

#ifndef FORREXIT_H
#define FORREXIT_H

#include <iostream>
#include <FORRGeometry.h>
#include <FORRCircle.h>

class FORRExit{
 public:
  FORRExit(){};
  FORRExit(CartesianPoint point, int circle_id ){
    exitPoint = point;
    exitCircle = circle_id;
  }

  void print(){
    cout << exitPoint.get_x() << " " << exitPoint.get_y() << " " << exitCircle << endl;
  }
    
  double distance(CartesianPoint point1, CartesianPoint point2){
    double dy = point1.get_y() - point2.get_y();
    double dx = point1.get_x() - point2.get_x();
    return sqrt((dx*dx) + (dy*dy));
  }

  CartesianPoint getExitPoint(){ return exitPoint;}
  void setExitPoint(CartesianPoint point) { exitPoint = point;}

  void setExitCircle(int circle_id) { exitCircle = circle_id;}
  int getExitCircle(){ return exitCircle; }

 private:
  CartesianPoint exitPoint;
  int exitCircle;
};


#endif
