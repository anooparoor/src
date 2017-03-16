/*!
 * AgentState.h
 *
 * \brief Represents the current state of an agent: its current position,
 *        its plans, etc.).
 *
 * \author Eric Schneider <esch@hunter.cuny.edu>
 * \date 11/11/2011 Created
 */
#ifndef AGENTSTATE_H
#define AGENTSTATE_H

#include "Task.h"
#include "FORRAction.h"
#include "Position.h"
#include "Utils.h"
#include "FORRGeometry.h"

#include <time.h>
#include <unistd.h>

#include <vector>
#include <list>
#include <deque>
#include <set>

#include <sensor_msgs/LaserScan.h>

using namespace std;

class AgentState
{
 public:
  /** \brief AgentState constructor */
  AgentState() : currentTask(NULL) {
	vetoedActions = new set<FORRAction>();
        pos_hist = new vector<Position>();
        action_set = new set<FORRAction>();

    	for(int i = 1; i < 6; i++){
      		action_set->insert(FORRAction(LEFT_TURN, i));
      		action_set->insert(FORRAction(RIGHT_TURN, i));
    	}
    	for(int i = 1; i < 6; i++){
      		action_set->insert(FORRAction(FORWARD, i));
    	}
    	action_set->insert(FORRAction(PAUSE,0));
  }
  
  // Use the laser scan value to check if the target is in sight
  bool isTargetInSight();
  // Best possible move towards the target
  FORRAction singleMoveTowardsTarget();

  set<FORRAction> *getActionSet(){return action_set;}

  vector<Position> *getPositionHistory(){return pos_hist;}

  void clearPositionHistory(){pos_hist->clear();}

  void savePosition(){
     Position pos = pos_hist->back();
     if(!(pos == currentPosition)) 
	pos_hist->push_back(currentPosition);
  }

  double getDistanceToTarget(double x, double y){ 
    double dx = x - currentTask->getX();
    double dy = y - currentTask->getY();
    return sqrt((dx*dx) + (dy*dy));
  }
  
  bool isDestinationReached(int epsilon){
    if(getDistanceToTarget() < epsilon) {
        return true;
    }
    return false;
  }

  Position getCurrentPosition() { return currentPosition; }
  void setCurrentPosition(Position p) { currentPosition = p; }
  
  
  Task *getCurrentTask() { return currentTask; }
  void setCurrentTask(Task *task) { 
    currentTask = task; 
  }


  set<FORRAction> *getVetoedActions() { return vetoedActions;}
  
  void addTask(float x, float y) {
    Task *task = new Task(x,y);
    agenda.push_back(task); 
  }
  
  Task *getNextTask() { return agenda.front(); }
  
  list<Task*>& getAgenda() { return agenda; }
  
  void finishTask() {
    if (currentTask != NULL)
      agenda.remove(currentTask);

    currentTask = NULL;
  }

  void skipTask() {
    if (currentTask != NULL)
      agenda.remove(currentTask);

    currentTask = NULL;
  }

  sensor_msgs::LaserScan getCurrentLaserScan(){return currentLaserScan;}
  void setCurrentLaserScan(sensor_msgs::LaserScan scan){ currentLaserScan = scan;}
    
  //return the distance to the forward obstacle using the current laser range scan
  double getForwardObstacleDistance();
  vector<double> get_angle_difference_vector();
  double normalize_angle(double angle);

  Position getExpectedPositionAfterAction(FORRAction action);
  Position getExpectedPositionAfterActions(vector<FORRAction> actions);

  double getDistanceToObstacle(FORRAction action);
  double getDistanceToObstacle();
  double getDistanceToTarget();

  // Can a robot see a segment or a point using its laser scan data?
  bool canSeeSegment(CartesianPoint point1, CartesianPoint point2);
  bool canSeeSegment(vector<CartesianPoint> givenLaserEndpoints, CartesianPoint laserPos, CartesianPoint point1, CartesianPoint point2);
  bool canSeePoint(vector<CartesianPoint> givenLaserEndpoints, CartesianPoint laserPos, CartesianPoint point, double epsilon);
  bool canSeePoint(CartesianPoint point, double epsilon);
  
  //Converts current laser range scanner to endpoints
  void transformToEndpoints();

 private:

  FORRAction get_max_allowed_forward_move();
  double get_move_length(FORRAction forward_move);

  // Current position of the agent x, y, theta 
  Position currentPosition;

  // Position History : Set of all unique positions the robot has been in , while pursuing the target
  vector<Position> *pos_hist;

  // set of vetoed actions that the robot cant execute in its current state
  set<FORRAction> *vetoedActions;

  // Set of all actions that the robot has in its action set
  set<FORRAction> *action_set;
  
  // aggregate decision making statistics of the agent
  // Total travel time
  double total_travel_time;

  // Total travel distance
  double total_travel_distance;

  // Total tier 1 decisions 
  int total_t1_decisions;

  // Total tier 2 decisions
  int total_t2_decisions;

  // Total tier 3 decisions
  int total_t3_decisions;
  
  /** \brief The agent's list of tasks */
  list<Task*> agenda;
  
  // Current task in progress
  Task *currentTask;

  // Currrent laser scan reading at the current position
  sensor_msgs::LaserScan currentLaserScan;

  // Current laser scan data as endpoints in the x-y coordinate frame
  vector<CartesianPoint> laserEndpoints;

};

#endif
