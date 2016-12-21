/********************************
FORRGates.h
This is the FORRGate class, which stores the gates that takes the robot 
from one quadrant to another, facilitating more open movement in the map.

Written by Matthew Evanusa, October 2014
*********************************/

#ifndef FORRGATES_H
#define FORRGATES_H


#include <iostream>
#include <FORRGeometry.h>
#include <vector>


using namespace std;



struct Gate{
    CartesianPoint point_from, point_to;
    int quad_from, quad_to; //the two quadrants it connects to
  Gate(CartesianPoint p_f, CartesianPoint p_t, int q_f, int q_t) : point_from(p_f), point_to(p_t), quad_from(q_f),
      quad_to(q_t) {}
  
};



class FORRGates{
 public:
  /*
  struct Gate{
    CartesianPoint point_from, point_to;
    int quad_from, quad_to; //the two quadrants it connects to
  Gate(CartesianPoint p_f, CartesianPoint p_t, int q_f, int q_t) : point_from(p_f), point_to(p_t), quad_from(q_f),
      quad_to(q_t) {}
  
   };
  */

  
  FORRGates(vector<Gate> &v){
    gates = v;
  }

  FORRGates(){}
  
  void add_gate(CartesianPoint point_from, CartesianPoint point_to, int quad_from, int quad_to);
  void read_gates_from_file();
  void learn_gate(int previous_x, int previous_y, int curr_x,
		  int curr_y, int map_x, int map_y);
  void load_gates_from_file(string filename);
  int getSize(){ return gates.size();}
  Gate getGate(int index){return gates[index];} 
 
  int region(int x, int y, int max_x, int max_y);
  void remove_gate(int index);
  void write_gates_to_file(string filename);
  void setGates(vector<Gate> &v){ gates = v;}
  std::vector<Gate> getGates(){return gates;}


 private:
 std::vector<Gate> gates;



};


#endif
