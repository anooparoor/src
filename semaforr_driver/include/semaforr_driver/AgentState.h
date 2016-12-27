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

#include "Position.h"
#include "Task.h"
#include "FORRAction.h"

#include <time.h>
#include <unistd.h>


#include <list>
#include <deque>
#include <set>

class AgentState
{
 public:
  /** \brief AgentState constructor */
 AgentState() : is_paused(false), halt_until_time(0), wait_until_time(0),
    resume_time(0), rest_until_time(0), stop_until_time(0),
    doHalt(false), doWait(false), doResume(false), stopSeconds(-1),
    currentTask(NULL) {}

  Position getExpectedPositionAfterActions(Position initialPosition, vector<FORRAction> actionList);
  Position getExpectedPositionAfterAction(Position initialPosition, FORRAction action);
  Position afterLinearMove(Position initialPosition, double intensity);
  Position afterAngularMove(Position initialPosition, double intensity);

  double getDistanceToTarget(){
    return getDistanceToTarget(currentPosition.getX(), currentPosition.getY());
  }
  
  double getDistanceToTarget(double x, double y){ 
    double dx = x - currentTask->getX();
    double dy = y - currentTask->getY();
    return sqrt((dx*dx) + (dy*dy));
  }
  
  Position getCurrentPosition() { return currentPosition; }
  void setCurrentPosition(Position p) { currentPosition = p; }
  
  list<Position> getCurrentPlan() { return currentPlan; }
  void setCurrentPlan(list<Position> _plan) { currentPlan = _plan; }
  
  
  Task *getCurrentTask() { return currentTask; }
  void setCurrentTask(Task *task) { 
    currentTask = task; 
    resetCurrTaskFlags();
  }
  
  void addTask(Task *task) {
    list<Task*>::iterator itr; 
    for(itr = agenda.begin(); itr != agenda.end(); itr++) {
      if(task->get_id() == (*itr)->get_id()) { 
	return;
      }
    }
    agenda.push_back(task); 
  }
  
  
  
  void removeTask(int task_id) { 
    list<Task*>::iterator iter;
    for(iter = agenda.begin(); iter != agenda.end(); iter++)
      if((*iter)->get_id() == task_id) {
	agenda.remove(*iter); 
	break;
      }
  }    
  
  Task* getTask(int task_id) {
    list<Task*>::iterator iter;
    for(iter = agenda.begin(); iter != agenda.end(); iter++)
      if((*iter)->get_id() == task_id) {
	return (*iter); 
      }
    
    // else return invalid task id = -1
    Task * t = new Task(); 
    return t; 
  }
  
  Task *getNextTask() { return agenda.front(); }
  
  list<Task*>& getAgenda() { return agenda; }
  
  //  void finishTask() { agenda.pop_front(); currentTask = NULL; }
  void finishTask() {
    if (currentTask != NULL)
      agenda.remove(currentTask);

    currentTask = NULL;
  }
  
  
 private:

  /** \brief The current position of the agent 
   */
  Position currentPosition;
  
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
  
  Task *currentTask;
};

#endif
