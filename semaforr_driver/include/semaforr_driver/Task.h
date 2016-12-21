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
  
  Task(): task_id_(-1) {}

  Task(int coordinator_id, int task_id, TASK_TYPE t, int num_req, std::vector<int> asgn_rbts, int dur, int x, int y) 
    : coordinator_id_(coordinator_id), 
    task_id_(task_id), 
    type_(t), 
    num_robots_required_(num_req), 
    assigned_robots_(asgn_rbts), 
    duration_(dur) 
    {
      access_point_.first = x; 
      access_point_.second = y; 
      
      status_ = ENROUTE; 

      std::vector<int>::iterator iter; 
      for(iter = assigned_robots_.begin(); iter != assigned_robots_.end(); iter++) {
	robot_arrival_status_[*iter] = false;
      }
    }

  int get_id() { return task_id_; }
  
  int get_coordinator_id() { return coordinator_id_; }
  
  void set_coordinator_id(int c_id) { coordinator_id_ = c_id; }
  
  int getX() { return access_point_.first; }
  
  int getY() { return access_point_.second; }
  
  void setAccessPoint(std::pair<int,int> p) { access_point_ = p; }

  TASK_TYPE getType() { return type_; }
  
  TASK_STATUS getStatus() { return status_; }
  
  void setStatus(TASK_STATUS stat) { status_ = stat; }
 
  int getDuration() { return duration_; }

  void setDuration(int d) { duration_ = d; }
 
  
  void robotArrived(int robot_id) {
    cout << "Robot " << robot_id << " has arrived to task " << task_id_ << endl; 
    robot_arrival_status_[robot_id] = true; 
    if(isReadyForExecution())
      status_ = READY; 
  }

  void robotAssigned(int robot_id) {
    cout << "Robot " << robot_id << " assigned to task : " << task_id_ << endl; 
    if(std::find(assigned_robots_.begin(), assigned_robots_.end(), robot_id) == assigned_robots_.end()) {
      cout << "this is a new robot adding to assigned robots," 
	   << " setting its arrival status to false and setting the task status to ENROUTE" << endl;
      assigned_robots_.push_back(robot_id); 
      robot_arrival_status_[robot_id] = false;
      status_ = ENROUTE;
    }
    else {
      std::cout << "Task.h> robot " << robot_id << " is already assigned to task, skipping!" << std::endl; 
    }
  }
  
  void robotUnassigned(int robot_id) {
    assigned_robots_.erase(std::find(assigned_robots_.begin(), assigned_robots_.end(), robot_id));
    robot_arrival_status_.erase(robot_arrival_status_.find(robot_id));
    status_ = ENROUTE;
  }

  bool isRobotAssigned(int robot_id) { 
    return (std::find(assigned_robots_.begin(), assigned_robots_.end(), robot_id) != 
	    assigned_robots_.end());
  }

 private:
  
  int coordinator_id_;
  
  int task_id_;
  
  //<! type of the task as defined in definitions.h
  TASK_TYPE type_;

  //<! number of robots required for the task
  int num_robots_required_; 

  //<! session id's of all assigned robots to the task including this robot. the size may not be equal to num_robots_required_ 
  std::vector<int> assigned_robots_; 

  //<! expected task execution time in seconds 
  int duration_; 

  //<! The point in the map, that the robot needs to go in order to execute this task 
  std::pair<int,int> access_point_; 
  
  //<! States the execution status of the task
  TASK_STATUS status_; 
  
  //<! maps the robot id's to a bool flag that represents if the robot is arrived to its access_point_ and ready for execution
  std::map<int,bool> robot_arrival_status_; 
  
  bool isReadyForExecution() {
    // if the assignment is not complete yet
    if (static_cast<int>(assigned_robots_.size()) != num_robots_required_) 
      return false;
    
    // if the assignment is complete but not all the assigned robots arrived at their assigned destinations
    std::map<int,bool>::iterator iter; 
    for(iter = robot_arrival_status_.begin(); iter != robot_arrival_status_.end(); iter++) {
      if(!iter->second) 
	return false; 
    }
    
    // otherwise
    return true; 
  }
  
};

#endif
