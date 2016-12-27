/**!
  * Task.h
  * 
  * /author: Tuna Ozgelen
  *
  *          Replaced with Task.h written by Eric Schneider for AuctionManager. 
  *          The definition in here is for TASC experiments.
  */

#ifndef TASK_H
#define TASK_H

#include "definitions.h"
#include <vector>
#include <map>
#include <algorithm>

class Task {
  
 public:
  
  Task(int x, int y)  
    {
      target.first = x; 
      access_point_.second = y; 
      
      status_ = ENROUTE; 
    }

  int getX() { return access_point_.first; }
  
  int getY() { return access_point_.second; }
  
  void setAccessPoint(std::pair<int,int> p) { access_point_ = p; }

  TASK_TYPE getType() { return type_; }
  
  TASK_STATUS getStatus() { return status_; }
  
  void setStatus(TASK_STATUS stat) { status_ = stat; }
 
  int getDuration() { return duration_; }

  void setDuration(int d) { duration_ = d; }
 

 private:
  
  //<! expected task execution time in seconds 
  float time_taken; 

  //<! The point in the map, that the robot needs to go in order to execute this task 
  Position target; 
  
  //<! States the execution status of the task
  TASK_STATUS status; 
  
  
};

#endif
