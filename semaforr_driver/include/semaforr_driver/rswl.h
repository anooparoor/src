//Anoop Aroor
/*Module Name: Relative support weight learning
Objective: refer documentation
Author: Anoop Aroor
Date : 09-20-2013
Files : main.cpp, rswl.cpp and rswl.h */

#include <iostream>
#include <algorithm>
#include <list>
#include <string>
#include <map>
#include <time.h>
#include "FORRGeometry.h"


using namespace std;

// hack 
const int globalActionCount = 45;

class Location{
 public:
  int xCoordinate;
  int yCoordinate;
};

/*
//object of this class contains information about a single comment made by one advisor on one action

class Comment{
 private:
  FORRAction action;
  string advisor;
  float commentStrength;
  //action rank : 1 indicates the advisor has given the action the highest preference, which is inverse of comment strength.  
  int actionRank;
 public:
  FORRAction getAction(){return action;}
  string getAdvisor(){return advisor;}
  float getCommentStrength(){return commentStrength;}
  int gotActionRank(){return actionRank;}
  void setCommentStrength(float commentStrength){commentStrength = comment;}
  void setAction(FORRAction action){action = action;}
  void setAdvisor(string advisor){advisor = advisor;}
  void setActionRank(int actionRank){actionRank = actionRank;}
}
*/

class RobotDecision
{
public:
  void setArray(int advisorCount,int actionCount){
  	advisorComments = new float*[advisorCount]; //creates a new array of pointers to int objects
	for(int i=0; i<advisorCount; ++i){
               advisorComments[i]=new float[actionCount];
        }
        for(int i=0; i <advisorCount; ++i){
              for(int j =0; j <actionCount; ++j){
                     advisorComments[i][j] = 1000;
              //       cout << advisorComments[i][j] << endl;
              }
        }
        advisorPreference = new float*[advisorCount]; //creates a new array of pointers to int objects
        for(int i=0; i<advisorCount; ++i){
               advisorPreference[i]=new float[actionCount];
        }
        for(int i=0; i <advisorCount; ++i){
               for(int j =0; j <actionCount; ++j){
                     advisorPreference[i][j] = 0;
                //     cout << advisorPreference[i][j] << endl;
               }
        }
        cout << "allocating memory: " << endl;
        cout << advisorPreference[advisorCount-1][actionCount -1] << " " << advisorComments[advisorCount-1][actionCount -1];
  }

  /* More object oriented design for future work
  void computeActionRank(list<Comment> comments){

  }
  list<FORRAction> actionsInvolved(list<Comment> comments){

  }
  list<string> advisorInvolved(list<Comment> comments){

  }
  */
  
  
  
  int decisionId;
  float distanceFromTarget;	
  Location robotLocation;
  double robotOrientation;
  int maxForwardMove;

  int actionTypeId;
  int intensityId;

  Location targetLocation;
  int robotId;
  //map<string, string> infoUsed;
  float ** advisorComments;
  //list<Comment> comments;
  float ** advisorPreference;
  int actionChoice;
  int tier;
  bool goalState;	
  bool was_pruned; // output for learning to state whether node was redirected from correct_path()
  vector<CartesianPoint> wallVectorEndpoints;
};


class Digression
{
public:
  RobotDecision start;
  RobotDecision end;
};


class RSWL
{	
 public: 
  RSWL(string advisor_conf_file, string rswl_conf_file);
  RSWL(){};
  int advisorCount;
  int actionTypeCount;
  int intensityCount;
  int actionCount;
  int robotCount;
  float closenessThreshold;
  
  // stores the initial default weight and the status of the advisors from the config file advisor1.conf
  vector<std::string> advisorList;
  vector<std::string> advisorStatus;
  vector<float> advisorInitWeights;

  //first value/point is the origin (robot) x,y; subsequent are the endpoints of the rays
  vector< vector<CartesianPoint> > wall_distance_segments;
  void parsePathFile();
  list<RobotDecision> correct_path(list<RobotDecision> uncorrected_path);
  list<RobotDecision> current_path; // stores the current path for the robot decision; 
  
  // stores the current weight of advisors
  vector<double> activeAdvisorWeights; 
  vector<double> activeAdvisorCommentCount;
  map<string,int> activeAdvisorList;
  
  int learnWeights(std::string);
  int printPathList(list< list<RobotDecision> >);
  int printDecisionList(list<RobotDecision>);
  int printDecision(RobotDecision);
  list<RobotDecision> computeDistanceFromTarget(list<RobotDecision>);
  
  void readConfigFile(string advisor_conf_file, string rswl_conf_file);
  list<Digression> removeDuplicateDigression(list<Digression>);
  list<RobotDecision> parseLog(std::string);
  RobotDecision insertDetail(string, string, RobotDecision);
  //list< list<RobotDecision> > extractSuccessfulPaths(list<RobotDecision>);
  pair< float*, float*> computeWeights(list<RobotDecision>);
  Digression findDigression(list<RobotDecision>);
  list<Digression> extractDigression(list<RobotDecision>);	
  int addPenalty(Digression,float[],float[]);
  list<RobotDecision> removeDigression(list<RobotDecision>, list<Digression>);
  RobotDecision findInPath(list<RobotDecision>, int);
  int addReward(float[], RobotDecision, float[]);
  float* computeRelativeSupport(float**, int);
  bool checkEqual(RobotDecision, RobotDecision);
  bool isAdvisorInvolved(int, float**, int);
  float getAverage(float []);
  float getDistance(Location, Location);
  float getDistance(int x1,int y1,int x2,int y2);
  float* distinct(float[]);
  float* sort(float[]);
  RobotDecision computeAdvisorPreference(RobotDecision);
  void computeCorrectActions(list<RobotDecision> *);
  int computeAction(RobotDecision, int, int);
  int getDistanceAfterMove(int, int, int, double, int, int);
};


