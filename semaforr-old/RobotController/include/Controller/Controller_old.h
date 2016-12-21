#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <map>
#include <set>
#include <string>
#include <time.h>
#include <limits.h>
#include <cstdlib>
#include <time.h>
#include <math.h>
#include <vector>
#include <utility>

#include "definitions.h"
#include "CommunicationManager.h"
#include "Localization.h"
#include "Surveyor.h"
#include "libplayerc++/playerc++.h"
#include "metrotimer.h"
#include "metrocommunication.h"
#include "Timer.h"
#include "PosixTimer.h"

// SemaFORR
#include "Beliefs.h"
#include "FORRComment.h"
#include "FORRAction.h"
#include "FORRGeometry.h"

#include "Advisor.h"
#include "Tier3Advisor.h"
#include "AvoidRobots.h"

#include "rswl.h"
#include <fstream>

#include "RobotControllerMessageHandler.h"


using namespace PlayerCc;

/*******************************************************************************
 * BEGIN Ariadne/FORR typedefs
 ******************************************************************************/

// Forward-declare Controller so the typedef below can reference it
class Controller;
// AdvisorFunc is a pointer to an Ariadne/FORR Tier 1 advisor function. This
// typedef is a stopgap until a full Advisor class hierarchy is defined.
//
// A sequence of AdvisorFuncs is declared in Controller::declareAdvisors().
// AdvisorFuncs are consulted sequentially in Controller::FORRDecision(). The
// first AdvisorFunc to return true breaks the consultation loop and is
// considered to have "made" the decision for the current time step.
typedef bool (Controller::*AdvisorFunc)(Beliefs *beliefs);

// An iterator over a vector of AdvisorFuncs. typedef'd here for brevity.
typedef std::vector<AdvisorFunc>::iterator advisorFuncIt;
typedef std::vector<Advisor*>::iterator advisorIt;
typedef std::vector<Tier3Advisor*>::iterator advisor3It;

/*******************************************************************************
 * END Ariadne/FORR typedefs
 ******************************************************************************/

class Controller {
public:
  Controller(string player_host, int player_port, Map *map,
	     int robot_size, string robot_type, string robot_name, int proximity, RobotControllerMessageHandler *message_handler);
  ~Controller(){ 
    stop(); 
  }

  void stop(){ 
    resetDestination();
    itl->setSpeed(0,0,0); 
  }

  void resetDestination() {
    Position nullPosition(0,0,0);
    beliefs->setNextPosition(nullPosition);
    itl->resetDestinationInfo(); 
  }


  Localization * getLocalization() { return itl; }

  Beliefs * getBeliefs() { return beliefs; }
  
  //void setParkingPoint( Beliefs *beliefs);

  void wait_for_response();
  bool received_reply;  

  void operator()();

  double get_cumulative_cost();
  // added 6th march ******
  Position get_position();
 
  Position get_position(int,int,int,int);
  
  Position get_position(Position, Position);

  double estimate_cost();

  double estimate_cost(int, int, int, int);

  double estimate_cost(Position, Position);
  // **********************
  Position get_final_position();

  double get_move_length(FORRAction forward_move);

  std::string get_robot_type() { return robot_type; }
  void set_robot_type(std::string robot_type) { this->robot_type = robot_type; }

  int get_robot_size() { return robot_size; }

  void set_robot_size(int robot_size) { this->robot_size = robot_size; }

  /*****************************************************************************
   * BEGIN Ariadne/FORR functions and advisors
   ****************************************************************************/

  void executeDecision( FORRAction );

  /** Controller::FORRDecision */
  /*!
   *  \brief THE Ariadne/FORR decision-making process.
   *
   *  Tier 1 advisors are consulted in order of priority until one returns true
   *  or the sequence is exhausted (i.e., they all return false).
   *
   *  Tier 2 is not yet implemented, although the PathPlanner class may be
   *  considered a kind of Tier 2 advisor. PathPlanner uses a navigation graph
   *  to generate a plan -- a list of intermediate waypoints between the robot's
   *  current position and a given target position. A Tier 1 advisor (an
   *  "enforcer") is responsible for moving to these waypoints if such a plan
   *  has been generated.
   *
   *  Tier 3 is not yet implemented. It will entail consulting all Tier 3
   *  advisors in parallel subject to resource (e.g., time) limits, with
   *  weighted vote tabulation. 
   *
   *  1/9/2012 TODO: Advisors should return a pointer to an Action object (yet
   *                 to be defined) or NULL. An Action will be of type 'MOVE',
   *                 'SENSE', etc., with a relevant parameter (e.g., a move-to
   *                 position). Once an action is produced, This function will
   *                 execute it.
   */
  void FORRDecision();
  
  /** Controller::declareAdvisors
   *
   *  Adds AdvisorFuncs to the Controller::advisors vector, esablishing the
   *  (priority) order in which they will be consulted by
   *  Controller::FORRDecision().
   *
   *  1/9/2012 TODO: In FORR, multiple Tier-1 advisors can have the same
   *                 priority level. In this case, they must be consulted in
   *                 parallel rather than in sequence. Is this capability
   *                 required here?
   */
  void declareAdvisors();

  /** Tier 1(/2?) advisor functions *******************************************/
  
  /** Controller::advisorHHalt */
  /*!  
   *  \brief A command from a human operator stops the robot and discards the current plan
   */

  bool advisorVictory(Beliefs *beliefs);

  FORRAction get_max_allowed_forward_move(set<FORRAction> vetoedActions);
  int get_forward_move_intensity(double distance);

  bool advisorHHalt(Beliefs *beliefs);
   
  /** Controller::advisorHWait */
  /*!
   *  \brief A command from a human operator pauses (stops) the robot but preserves the current plan
   */
  bool advisorHWait(Beliefs *beliefs);


  /** Controller::advisorHResume */
  /*!
   *  \brief A command from a human operator countermands a prior halt/wait and allows
   *         the robot to move again.
   */
  bool advisorHResume(Beliefs *beliefs);



  bool advisorHResume(Beliefs *beliefs, int resume_time);

  void advisorCircle(Beliefs *beliefs);

  void advisorNotOpposite(Beliefs *beliefs);
  
  void normalize_weights();

  void learnCircle(Beliefs *b);

  void learnExits(Beliefs *b);
  
  bool isFacing(Position robotPos, CartesianPoint point, double radius);

  bool advisor_avoid_walls(Beliefs *beliefs);
  
  void add_advisor(Tier3Advisor* advisor);

  /*****************************************************************************
   * END Ariadne/FORR functions
   ****************************************************************************/
  // since we have to pass messages between DescriptiveManager and Controller
  // can't do this in the Controller constructor   
  void initialize_FORRAdvisors();

private:
    //added 9/4/2014 for logging and testing information
  //----------------------------------------------------
  
  //9/9/2014
  //Note: So far, there are two separate fields in the log file, time and distance, 
  //separated by a new line
  
  
  //stores the number of actual decisions made (not just enters loop and exits)
  int decisions_made_per_target;

  //stores the saved wall distance vectors for each move towards one target point
  std::vector< std::vector<double> > wall_distance_vectors;

  bool move_completed;
 
  std::vector<double> decision_times;
  std::vector<double> per_decision_times;
  std::vector<double> distances;
  std::vector<double> task_times;
  std::vector<double> learning_times;
  struct timeval learning_timer;
  unsigned long learning_timer_before, learning_timer_after;
  double learning_time_cumulative;


  //Gate learning rolling sum
  double gate_rolling_sum;

  //counter for number of decisions made in Tier 1 and Tier 3
  int number_of_decisions_tier1_with_pauses;
  int number_of_decisions_tier3_with_pauses;
  int number_of_decisions_tier1_no_pauses;
  int number_of_decisions_tier3_no_pauses;

    
  //TASK TIMES
  struct timeval task_timer;
  unsigned long task_timer_before, task_timer_after;

  //SUM OF THE FUNCTION CALLS OVER A TASK
  unsigned long cumulative_decision_time_elapsed;
  
  //SUM OF THE WAIT (DELAY) TIME
  double cumulative_delay_time_elapsed;

  //SUM OF THE TOTAL TASK TIME
  unsigned long cumulative_task_time_elapsed;

  //TESTING OUT CLOCK_SAMPLE WITH MICROSECOND RESOLUTION 
  struct timeval clock_time;
  unsigned long clock_sample_before_;
  unsigned long clock_sample_after_;

  //USED FOR THE FINAL TIMESTAMP, START TIME OF THREAD, FINAL END TIME, STORED IN MICROSECONDS
  struct timeval tv;
  unsigned long start_time;
  unsigned long end_time;

  Position previousPosition;

  //stores a time sample from clock() for delay time
  clock_t delay_sample_before;
  clock_t delay_sample_after;
  
  //time it took to run target function
  double function_duration;

  //stream used to write to file for all log information
  ofstream log_stream;  

  double total_distance;

  
  double calculateMean(std::vector<double> &v);
  double calculateVariance(double mean, std::vector<double> &v); 
  pair<double,double> calculateRange(std::vector<double> v); //passes as value not reference 
  double calculateMedian(std::vector<double> v);
   
  //-------------------------------------------------------


  //Used for parking information, stores the starting position as its ending position 
  Position parking_position;
  
  //flag to say that the parking position has been added
  bool added_parking_position;

  void robot_parking_routine();

  FORRAction mapIntegerToRotationAction(int index);


  //flags to turn off/on specific spatial cognition learning
  bool GATES;
  bool REGIONS;
  bool CONVEYORS;
  bool TRAILS;



  PlayerClient * pCli; 
  CommunicationManager * cMan;
  Localization * itl ;
  Map * localMap;
  string name;

  void updateBehavior();
  bool isTargetSet();
  //bool isTargetReached();
  bool isDestinationReached(int);

  // behavior vars
  Position prevPos, currPos, startPos, checkPos; 

  bool usingStage;

  double destErrorThreshold; 

  // Robot properties
  int robot_size; 
  std::string robot_type;
  string get_robot_name() {return name;}
  // Robot's message handler
  RobotControllerMessageHandler *message_handler;

  // DEBUGGING
  int cycleCounter; 

  // Checks if a given advisor is active
  bool isAdvisorActive(string advisorName);

  
  /*****************************************************************************
   * BEGIN Ariadne/FORR data members
   ****************************************************************************/
  // The robot's beliefs about its environment; its "world view". The basis of
  // Ariadne/FORR decisions.
  Beliefs *beliefs;

  // An ordered list of advisors that are consulted by Controller::FORRDecision
  std::vector<AdvisorFunc> advisors;
  std::vector<AdvisorFunc> tier1Advisors;
  std::vector<Tier3Advisor*> tier3Advisors;
  
  /*****************************************************************************
   * END Ariadne/FORR data members
   ****************************************************************************/
};
  
#endif /* CONTROLLER_H */
