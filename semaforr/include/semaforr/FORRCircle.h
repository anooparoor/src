/************************************************
FORRCircle.h 
This file contains the class FORRCircle which is intended to be a data structure which abstracts the notion of 
open spaces and doors in the maps as circles

Written by Anoop Aroor, 2014
**********************************************/

#ifndef FORRCIRCLE_H
#define FORRCIRCLE_H

#include <iostream>
#include "FORRGeometry.h"
#include "FORRExit.h"
#include <algorithm>
#include <vector>

class FORRCircle{
 public:
  FORRCircle(){};
  FORRCircle(CartesianPoint point, double r){
    center = point;
    this->setRadius(r);
  }

  bool doIntersect(FORRCircle test){
    int buffer = 0;
    if((test.getRadius() + this->getRadius() - distance(test.getCenter(), center)) > buffer)
      return true;
    else 
      return false;
  }

  bool operator==(FORRCircle b){
    bool equal = false;
    if(radius == b.getRadius() && center.get_x() == b.getCenter().get_x() && center.get_y() == b.getCenter().get_y()){
      equal = true;
    }
    return equal;
  }
  
  void clearExits(){
    exits.clear();
  }

  void print(){
    cout << center.get_x() << " " << center.get_y() << " " << this->getRadius();
    for(int i = 0; i < exits.size() ; i++){
      cout << " " << exits[i].getExitPoint().get_x() << " "  << exits[i].getExitPoint().get_y() << " "  << exits[i].getExitCircle();
    }
    cout << endl;
  }

  bool equals(FORRCircle test) { 
    if(test.getCenter().get_x() == center.get_x() && test.getCenter().get_y() == center.get_y() && test.getRadius() == this->getRadius())
      return true;
    else 
      return false;
  }

  bool isExitAlreadyPresent(FORRExit exit){
    for(int i = 0; i < exits.size() ; i++){
      if(exit.getExitPoint().get_x() == exits[i].getExitPoint().get_x() && exit.getExitPoint().get_y() == exits[i].getExitPoint().get_y() 
	 && exit.getExitCircle() == exits[i].getExitCircle())
	return true;
    }
    return false;
  }

  bool inCircle(double x, double y){ return (distance(CartesianPoint(x,y),center) < this->getRadius());}

  bool inCircle(CartesianPoint p){ return (distance(p,center) < this->getRadius());}
    
  double distance(CartesianPoint point1, CartesianPoint point2){
    double dy = point1.get_y() - point2.get_y();
    double dx = point1.get_x() - point2.get_x();
    return sqrt((dx*dx) + (dy*dy));
  }

  CartesianPoint getCenter(){ return center;}
  void setCenter(CartesianPoint point) { center = point;}
  
  double getRadius(){ 
    return radius;
  }
  void setRadius(double r){ 
    radius = r;
  }

  vector<FORRExit> getExtExits() { return ext_exits; }

  vector<FORRExit> getExits() { return exits;}
  void setExits(vector<FORRExit> exit_points) { exits = exit_points;}
  void addExit(FORRExit exit) {
    exits.push_back(exit);
    ext_exits.push_back(FORRExit(CartesianPoint(2*(exit.getExitPoint().get_x()) - center.get_x(), 2*(exit.getExitPoint().get_y()) - center.get_y()), exit.getExitCircle()));
  }

  void setIsLeaf(bool leaf){isLeaf = leaf;}
  bool getIsLeaf() {return isLeaf;}

 private:
  CartesianPoint center;
  double radius;
  bool isLeaf;
  // each value in the list denotes a possible exit from the circle
  vector<FORRExit> exits;
  vector<FORRExit> ext_exits;
};


#endif
