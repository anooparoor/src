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
  
  Position getNextPosition() { return nextPosition; }
  void setNextPosition(Position p) { nextPosition = p; }

  list<Position> getCurrentPlan() { return currentPlan; }
  void setCurrentPlan(list<Position> _plan) { currentPlan = _plan; }
  
  bool getIsPaused() { return is_paused; }
  void setIsPaused(bool _is_paused) { is_paused = _is_paused; }
  
  time_t getRestUntilTime() { return rest_until_time; }
  void setRestUntilTime(time_t t) { rest_until_time = t; }
  
  time_t getStopUntilTime() { return stop_until_time; }
  void setStopUntilTime(time_t t) { stop_until_time = t; }
  
  bool getDoHalt() { return doHalt; }
  void setDoHalt(bool b) { doHalt = b; }
  
  bool getDoWait() { return doWait; }
  void setDoWait(bool b) { doWait = b; }
  
  bool getDoResume() { return doResume; }
  void setDoResume(bool b) { doResume = b; }
  
  int getStopSeconds() { return stopSeconds; }
  void setStopSeconds(int s) { stopSeconds = s; }
  
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
  
  void robotAssignedToTask(int task_id, int robot_id) { getTask(task_id)->robotAssigned(robot_id); }
  
  void updateTaskResponsibility(int task_id, int dur, std::pair<int,int> aP) {
    cout << "Task responsibility is updated" << endl;
    Task* t = getTask(task_id);
    t->setDuration(dur); 
    t->setAccessPoint(aP);
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
  
  /* TASC functions */ 
  bool isReachedMsgSent() { return reached_msg_sent; }
  void setReachedMsgSent(bool b) { reached_msg_sent = b; }
  
  bool isStartedMsgSent() { return started_msg_sent; }
  void setStartedMsgSent(bool b) { started_msg_sent = b; }
  
  bool isCompletedMsgSent() { return completed_msg_sent; }
  void setCompletedMsgSent(bool b) { completed_msg_sent = b; }
  /* end TASC functions */
  
 private:

  
  /** \brief The current position of the agent 
   */
  Position currentPosition;
  
  /** \brief The agent's next position, assigned by some advisor 
   */
  Position nextPosition;
  
  /** \brief Is the agent paused? (due to e.g., collision avoidance) 
   */
  bool is_paused;
  
  /** \brief Halt until this time (and then resume)
   */
  int halt_until_time;
  
  /** \brief Wait until this time (and then resume)
   */
  int wait_until_time; 
  
  /** \brief Resume motion at this time
   */
  int resume_time;
  
  /** \brief For 02/2012 experiments. There should really be only one
   *   "paused/stopped until" time.
   */
  time_t rest_until_time;
  
  /** \brief This should subsume all of [rest|wait|halt]_until_times
   *
   */
  time_t stop_until_time;
  
  /** These should be set by commands coming in from the Human Interface,
   *  CollisionAvoider, etc.
   */
  bool doHalt;
  bool doWait;
  bool doResume;
  
  // The number of seconds to stop when a HALT or WAIT is issued.
  // The default (-1) means stop indefinitely (until time() == INT_MAX)
  int stopSeconds;
  
  /** \brief A list of waypoints to the current interest point
   *
   * \note The PlathPlanner actually stores its plan as a list<int>
   *       of Node ids, not Positions. This is currently unused, but will
   *       be used when RobotController/PathPlanner is refactored.
   */
  list<Position> currentPlan;
  
  /** \brief The agent's list of tasks */
  list<Task*> agenda;
  
  Task *currentTask;
  
  /* TASC flags */
  bool reached_msg_sent; 
  bool started_msg_sent; 
  bool completed_msg_sent; 
  
  void resetCurrTaskFlags() {
    reached_msg_sent = false; 
    started_msg_sent = false; 
    completed_msg_sent = false; 
  }
};

#endif
