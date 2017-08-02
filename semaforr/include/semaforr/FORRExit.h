/************************************************
FORRExit.h 
This file contains the class FORRExit which is intended to be a data structure which abstracts the notion of an exit

Written by Anoop Aroor, 2014
**********************************************/

#ifndef FORREXIT_H
#define FORREXIT_H

#include <iostream>
#include <FORRGeometry.h>
#include <FORRRegion.h>

class FORRExit{
 public:
  FORRExit(){};
  FORRExit(CartesianPoint point, int region_id ){
    exitPoint = point;
    exitRegion = region_id;
  }

  void print(){
    cout << exitPoint.get_x() << " " << exitPoint.get_y() << " " << exitRegion << endl;
  }
    
  double distance(CartesianPoint point1, CartesianPoint point2){
    double dy = point1.get_y() - point2.get_y();
    double dx = point1.get_x() - point2.get_x();
    return sqrt((dx*dx) + (dy*dy));
  }

  CartesianPoint getExitPoint(){ return exitPoint;}
  void setExitPoint(CartesianPoint point) { exitPoint = point;}

  void setExitRegion(int region_id) { exitRegion = region_id;}
  int getExitRegion(){ return exitRegion; }

  bool operator < (const FORRExit &exit) const{
    return false;
  }

 private:
  CartesianPoint exitPoint;
  int exitRegion;
};


#endif
