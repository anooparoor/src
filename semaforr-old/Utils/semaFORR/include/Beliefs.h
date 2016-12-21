/*!
 * Beliefs.h
 *
 * \brief Contains spatial representations of the world (e.g. Map, Graph),
 *        the agent's curren state (an AgentState), the agent's knowledge
 *        about other agents and whatever else may be useful.
 *
 * \author Eric Schneider <esch@hunter.cuny.edu>
 *         A. Tuna Ozgelen 
 *
 * \date 11/11/2011 Created
 * \date 11/15/2011 Renamed "AgentWorldView" -> "Beliefs"
 *
 * \date 12/14/2013 Added TASC related functions 
 * \date 1/14/2014 Added TeamState functions  
 */
#ifndef BELIEFS_H
#define BELIEFS_H

#include "AgentState.h"
#include "TeamState.h"
#include "FORRAction.h"

// These happen to work when building RobotController because that component
// will use its own versions. Instead creating another set of these for
// Ariadne/FORR, there should be a global set, probably in Utils
#include "Map.h"
#include "Observation.h"
#include "Position.h"
#include "Task.h"
#include "FORRCircle.h"
#include "FORRGeometry.h"
#include "FORRGates.h"
#include "FORRTrails.h"


#include <time.h>
#include <list>
#include <map>
#include <string>
#include <set>
#include <fstream>
#include <sstream>
#include <vector>
#include <FORRWall.h>
#include "FORRGates.h"
#include "FORRAbstractMap.h"
#include "FORRWaypoints.h"
#include "FORRTrace.h"


using std::vector;
using std::set;


class Beliefs
{
public:
    /*! \brief Belief constructor
     *
     * \param m A line segment map
     */

    Beliefs(Map *m) : map(m) {
        cout << "In belief constructor" << endl;
        agentState = new AgentState();
	teamState = new TeamState();
	cout << "End of belief constructor" << endl;
    }

    Map * getMap() { return map; }
    
    Position getExpectedPositionAfterAction(FORRAction action){return agentState->getExpectedPositionAfterAction(getCurrentPosition(), action);}
    Position getExpectedPositionAfterActions(vector<FORRAction> actionList){
      //cout << "Beliefs::getExpectedPositionAfterAction >> " <<endl;
      return agentState->getExpectedPositionAfterActions(getCurrentPosition(), actionList);
    }
    Position getCurrentPosition() { return agentState->getCurrentPosition(); }
    void setCurrentPosition(Position p) { agentState->setCurrentPosition(p); }

    Position getNextPosition() { return agentState->getNextPosition(); }
    void setNextPosition(Position p) { agentState->setNextPosition(p); }

    double getDistanceToTarget() { return agentState->getDistanceToTarget(); } 

    bool getIsPaused() { return agentState->getIsPaused(); }
    void setIsPaused(bool _is_paused) { agentState->setIsPaused(_is_paused); }

    time_t getRestUntilTime() { return agentState->getRestUntilTime(); }
    void setRestUntilTime(time_t t) { agentState->setRestUntilTime(t); }

    time_t getStopUntilTime() { return agentState->getStopUntilTime(); }
    void setStopUntilTime(time_t t) { agentState->setStopUntilTime(t); }

    bool getDoHalt() { return agentState->getDoHalt(); }
    void setDoHalt(bool b) { agentState->setDoHalt(b); }

    bool isInParkingMode() { return inParkingMode;}
    void setInParkingMode(bool parkingMode) {inParkingMode = parkingMode ;}

    bool getDoWait() { return agentState->getDoWait(); }
    void setDoWait(bool b) { agentState->setDoWait(b); }

    bool getDoResume() { return agentState->getDoResume(); }
    void setDoResume(bool b) { agentState->setDoResume(b); }

    int getStopSeconds() { return agentState->getStopSeconds(); }
    void setStopSeconds(int s) { agentState->setStopSeconds(s); }
    
    Task *getCurrentTask() { return agentState->getCurrentTask(); }
    void setCurrentTask(Task *task) { agentState->setCurrentTask(task); }

    FORRAction getDecision() { return decision; }
    void setDecision(FORRAction actionDecided) {decision = actionDecided ; } 
    
    void addTask(Task *task) { agentState->addTask(task); }
    void removeTask(int task_id) { agentState->removeTask(task_id); }
    Task *getTask(int task_id) { return agentState->getTask(task_id); }
    
    void robotAssignedToTask(int task_id, int robot_id) { agentState->robotAssignedToTask(task_id, robot_id); }
    
    void updateTaskResponsibility(int task_id, int dur, std::pair<int,int> aP) { agentState->updateTaskResponsibility(task_id, dur, aP); }
    
    Task *getNextTask() { return agentState->getNextTask(); }
    
    void finishTask() { agentState->finishTask(); }

    list<Task*>& getAgenda() { return agentState->getAgenda(); }

   /* TASC functions */ 
    bool isReachedMsgSent() { return agentState->isReachedMsgSent(); }
    void setReachedMsgSent(bool b) { agentState->setReachedMsgSent(b); }

    bool isStartedMsgSent() { return agentState->isStartedMsgSent(); }
    void setStartedMsgSent(bool b) { agentState->setStartedMsgSent(b); }

    bool isCompletedMsgSent() { return agentState->isCompletedMsgSent(); }
    void setCompletedMsgSent(bool b) { agentState->setCompletedMsgSent(b); }
    /* end TASC functions */


    /* TeamState functions */
    void updateTeammate(long id, double x, double y, double theta, int size) { 
      teamState->updateTeammate(id, x, y, theta, size); 
      teamState->updateCollisionStatus(id, agentState->getCurrentPosition());
    }

    bool removeTeammate(long id) { return teamState->removeTeammate(id); }

    bool isRobotTeammate(long id) { return teamState->isRobotTeammate(id); }

    bool isTeammateInCollisionCourse(long id) { return teamState->isTeammateInCollisionCourse(id); }

    bool isTeammateInCollisionZone(long id) { return teamState->isTeammateInCollisionZone(id); }

    bool isTeammateInBufferZone(long id) { return teamState->isTeammateInBufferZone(id); }

    bool isTeammateBehind(long id) { return teamState->isTeammateBehind(id); }

    Position getTeammatePosition(long id) const { return teamState->getPosition(id); }

    Position getTeammatePreviousPosition(long id) const { return teamState->getPreviousPosition(id); }

    //double getSumOfTeammateDistances(Position p, double range) {return teamState->getSumOfDistances(p ,range); }

    //computes the sum of the distances of the teammates from a given position (x,y) within a given RANGE
    double getSumOfTeammateDistances(Position p, double range){
      double distance = 0;
      for(int i = 1; i < teamPose.size(); i++){
	Position teammatePosition = teamPose[i];
	double teammateDistance = p.getDistance(teammatePosition);
	if(teammateDistance < range)
	  distance += teammateDistance;
      }
      return distance;
    }

    double alignmentDiff(Position robotPos, CartesianPoint point){
      double robot_point_angle = atan2(point.get_y() - robotPos.getY(), point.get_x() - robotPos.getX());
      double angleDiff = robotPos.getTheta() - robot_point_angle;
      return angleDiff;
    }
    
    bool isFacing(Position robotPos , CartesianPoint point){
      bool isFacing = false;
      if(abs(alignmentDiff(robotPos, point)) < 0.52){
	isFacing = true;
      }
      return isFacing;
    }
    
    double getDistanceToTarget(Position position){return agentState->getDistanceToTarget(position.getX(), position.getY()); }

    int getSize(long id) const { return teamState->getSize(id); }

    std::vector<long> getTeammateIDs() { return teamState->getTeammateIDs(); }
    /* end TeamState functions */


    // things from fdesc which needs to be integrated into the belief
    //dummy default constructor
    void initVectors();
    
    // this function will update the descriptives
    void update(CartesianPoint, CartesianPoint, double);
    
    // Jan 13. added function that will normalize the angle to -PI : PI range
    
    double normalize_angle(double angle);
    
    vector<double> get_angle_difference_vector();
    
    // functions that will manipulate list of targets
    bool isCurrentTargetReached(){ return reached_current_target;}
    void reachedTarget(){ reached_current_target = true;}
    
    // add decision made before current stores last 2 decisions, with last decision at the end of the vector
    void addDecision(FORRAction action){ 
      actionsMade.push_back(action); 
      if(actionsMade.size() > 2)   actionsMade.erase(actionsMade.begin());
    }
    vector<FORRAction> getPreviousDecisions() { return actionsMade;}
    
    // various accessors
    bool hasTargets() { return hasMoreTargets; }
    FORRAction getLastAction(){ return lastAction; }
    
    // Jan. 2014 added construct for Explorer advisor
    int times_at_location(CartesianPoint location);
    void create_locations();
    void location_lookup();
    //static int square_size; // used for explorer to initialize table 
    
    // Jan. 2014 this is helper for GoAround advisor
    /* 
     * Function will do the preprocessing of the distances to make
     * decision to which side the robot should turn.
     * The decision is as follows. I set up threshold as 120px so
     * if robot is further away from the obstacle it will not turn, because
     * it considers that it is far enough from the target (like ObstacleAvoid) 
     * If robot is under the treshold, get the average value of distances
     * on the right and left side. And we want to turn to one with bigger
     * average value. If there is tie we chose left(my arbitrary decision)
     * Decision is stored in the member value side and values can be
     * n - don't turn, you are far enough from the obstacle
     * r - turn right
     * l - turn left.
     */
    // Note that this function is triggered only by the obstacle distance
    // right in front of the robot, not concerned about sides.
    void turn_to_side();

    // Message to Tier1 version of NotOpposite advisor
    FORRAction not_opposite_data();
    
    // helpers that can be deleted after debugging  
    void printTargets();
    void printWallDistances();
    void printTargetDistances();
    void display_visited_map();
    
    
    // this vector stores all walls in our map
    std::vector<FORRWall> maze;
    
    bool reached_current_target;
    bool hasMoreTargets;
    bool inCollisionMode;
    
    std::vector<Position> teamPose;
    std::vector<CartesianPoint> listOfTargets;
    // this is array that will store how many times robot visited location
    // it will be updated in update method
    double **visited_locations;
    int locations_size; // since I am dealing with array get some bounds checking


    //an upgrade to the previous visited_locations grid using vectors for easier use and 
    //for bug fixing
    vector< vector<int> > visited_grid;

    
    //new version of explorer, saves the visited points and votes based on distance from previous points 
    vector<CartesianPoint> visited_points;
    
    //initializes the grid to have width/box_size width and height/box_size height
    void set_visited_grid(int width, int height);

    int boxes_height, boxes_width;


    //used for the FORRTrails to know if the target is near a previous trail
    //The outside vector contains all the endDistanceVectors as endpoints, where
    //the first pair of numbers is the x,y coordinate of the robot
    //thus, lines can be created from the starting point to all the endpoints
    //The format is the same as the vectors used for cleaning the path
    vector< vector< vector<double> > > endDistanceVectors;
    
    //increments the grid position where the robot is
    //to be used each time a move is completed
    //this is now done in update()


    //resets values to 0 after successful target reached
    void reset_visited_grid();

    // this array store distance to the obstacle in all possible directions in
    // which robot can move. Calculated by Jeffrey's function.
    // This array will become decisionVector for some advisors (ObstacleAvoid and BigStep).
    
    vector<double> wallDistanceVector;

    double wall_distance;

    vector<double> targetDistanceVector;
    
    // need to calculate how many different distances there are for the size of this vector
    // 5 going forward, 5 going backward and 5 rotations to left and 5 rotations 
    // to the right so it is 20
    // this vector will become decisionVector for some advisors (Greedy and CloseIn
    
    vector<double> advisorWeight;
    
    // List of previous actions made..
    vector<FORRAction> actionsMade;
    
    // since I have problem with list I will just use this variable
    // to store the last action
    FORRAction lastAction;
    
    // use this variable to control the opposite advisors
    bool my_opposite;
    // variable that will tell robot to which side it should turn
    char m_side;
    
    double wallDistance;
    
    double square_size;
    
    vector<Position> *positionHistory;

    set<FORRAction> *allActions;
  
    set<FORRAction> *vetoedActions;

    FORRAbstractMap abstractMap;

    FORRGates gates;

    FORRWaypoints Waypoints;
    
    FORRTrails trail_vectors;
    
    //gets a gate that connects to the region that the target point is in
    CartesianPoint get_target_gate(double wall_distance);
    
    int size_of_gate_cluster(Position p, int radius);

    void clear_gates_in_circles();
    
    void clear_gates_far_from_paths(string filename);
    
    bool just_moved_across_pipe;
    void set_moved_across_pipe(bool moved){ just_moved_across_pipe = moved;};
    bool get_moved_across_pipe(){return just_moved_across_pipe;}
    //if the robot is within 20 pixels of a gate, returns the point of the opposing gate.  otherwise, returns null 
    CartesianPoint get_nearby_gate_exit();

    double get_visited_locations_value(Position expectedPosition);

    int get_maximum_intensity_following_linear_move(FORRAction action);
    FORRTrace* getRunTrace() { return runTrace; };
    void initializeTrace() { runTrace = new FORRTrace;}
    
 private:

    /** \brief A line-segment representation of the world */
    Map *map;
    
    /** \brief The agent's current state in the world */
    AgentState *agentState;
    
    /*! \brief Manages the state of the agent's teammates */
    TeamState *teamState; 

    FORRTrace *runTrace;

    /** \brief A list of observations the agent has made */
    list<Observation*> observations;

    FORRAction decision;

    bool inParkingMode;

};

#endif

