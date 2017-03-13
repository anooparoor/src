//Anoop Aroor
/*Module Name: Relative support weight learning
Objective: refer documentation
Author: Anoop Aroor
Date : 09-20-2013
Files : RSWL.cpp and RSWL.h */

#include <iostream>
#include <list>
#include <string>
#include <map>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <regex>
#include "rswl.h"
#include <utility>
#include <time.h>

using namespace std;

float RSWL::getDistance(int x1, int y1, int x2, int y2){
  return sqrt(pow(x1-x2,2) + pow(y1-y2,2));
}



// The main function which reads logs file , computes weights and prints them. It uses all other functions in this class.
RSWL::RSWL(string advisor_conf, string rswl_conf)
{
  //cout << "RSWL::RSWL>>read config files" << endl;
  readConfigFile(advisor_conf, rswl_conf);
  //cout << "RSWL::RSWL>>finished reading config files" << endl;
  int activeAdvisorCount = 0;
  for(int k = 0; k < advisorList.size(); k++)
    {
      if(advisorStatus[k] == "t")
	{
	  activeAdvisorWeights.push_back(advisorInitWeights[k]);
          activeAdvisorCommentCount.push_back(1);
          activeAdvisorCount = activeAdvisorCount + 1;
          //cout << "active advisor ID: " << activeAdvisorCount << " advisor name: " << advisorList[k] << endl;
	  activeAdvisorList.insert(std::pair<string,int>(advisorList[k],activeAdvisorCount));
	}
    }
  advisorCount = activeAdvisorCount;
  actionCount = actionTypeCount * intensityCount;
  //cout << "RSWL::RSWL>>advisor count:" << advisorCount << endl;
}
         
int RSWL::learnWeights(string log_name)
{
  //cout << "Parsing the robotcontroller log" << endl;    
  list<RobotDecision> decisionList = parseLog(log_name);
  //cout << "Printing the raw decision list parse from the log" << endl;
  //printDecisionList(rawDecision);      
  //**** not needed for online learning: list< list<RobotDecision> > successfulPathList = extractSuccessfulPaths(rawDecision);
  //cout<<  "Before path correction."<<endl;
  //parseDistanceVectorFile();
  // cout << "---------------------------------"<<endl;
  //cout << "Path length: " << decisionList.size() << endl;
  //cout << "---------------------------------"<<endl;
  list<RobotDecision> corrected_path = correct_path(decisionList);
  //cout<< "After path correction." << endl;
  //decisionList = computeDistanceFromTarget(decisionList);
  // log results
  //std::ofstream results_log;
  //results_log.open("results_log.txt", std::ofstream::out | std::ofstream::app);
  //results_log <<  iter->distanceFromTarget << endl;  
  //results_log.close();
  // end of log results;
  
  //cout << "Printing the path :: " << endl;
  //printDecisionList(corrected_path);
  /*
  cout << "computeWeights() start \n";
  float *weights;
  float *commentCount;
  pair< float*, float*> result;
  result = computeWeights(corrected_path);
  weights = result.first;
  commentCount = result.second;     
  for(int i = 0 ; i < advisorCount; i ++)
  {
     cout << "Weights before learning : Advisor "<< i+1 << " : " << activeAdvisorWeights.at(i) << endl;
     cout << "After learning from task : Advisor " << i+1 << " : " << weights[i] << endl;
     cout << "CommentCount before learning : Advisor "<< i+1 << " : " << activeAdvisorCommentCount.at(i) << endl;
     cout << "CommentCount after learning from task : Advisor " << i+1 << " : " << commentCount[i] << endl;
  }
  for(int i = 0 ; i <advisorCount; i ++)
    {
      cout << "Final weights of advisor after the previous task" << i+1 << " : " << (weights[i]);
    }
  activeAdvisorWeights.clear();
  activeAdvisorCommentCount.clear();
  for(int i = 0 ; i < advisorCount; i ++)
  {
     activeAdvisorWeights.push_back(weights[i]);
     activeAdvisorCommentCount.push_back(commentCount[i]);
  } 
  */
  //writeIntoConfigFile
  return 0;
}


//given a list of paths, where each path is a list of robotdecision which took the robot from its starting point
//to its target, this function computes at every decision point, the distance from that point to the target point
//along the path.
void RSWL::readConfigFile(string advisor_conf_file, string rswl_conf_file)
{
  std::ifstream rswl_conf;
  rswl_conf.open(rswl_conf_file); 
  string fileLine,buffer;
    
  while(!rswl_conf.eof()){
    getline(rswl_conf, fileLine);
    stringstream ss(fileLine);
    ss << fileLine;
    string buffer;
    while(ss>>buffer)
      {
	if(buffer == "#"){
	  break;
	}
	else if(buffer == "actionTypeCount"){
	  ss >> buffer;       
	  actionTypeCount = atoi(buffer.c_str());
          //cout << "action type count = " << actionTypeCount << endl;
	}
	else if(buffer == "robotCount"){
	  ss >> buffer;       
	  robotCount = atoi(buffer.c_str());
          //cout << "robot count = " << robotCount << endl;
	}
	else if(buffer == "intensityCount"){
	  ss >> buffer;       
	  intensityCount = atoi(buffer.c_str());
          //cout << "intensity count = " << intensityCount << endl;
	}
	else if(buffer == "closenessThreshold"){
	  ss >> buffer;       
	  closenessThreshold = atof(buffer.c_str());
          //cout << "closeness Threshold = " << closenessThreshold << endl;
	}
	else
	  cout << "error while reading config file, unrecognized command : " << buffer << endl;
      }
  }
  std::ifstream advisor_conf;
  advisor_conf.open(advisor_conf_file);                                                                      
  //cout << "opening advisor1.conf file " << endl;   
  while(!advisor_conf.eof()){
    getline(advisor_conf, fileLine);
    stringstream ss(fileLine);
    //cout << "new line : " << fileLine<< endl;
    //ss << fileLine;
    string buffer;
    while(ss>>buffer)
      {
	if(buffer == "#"){
	  break;
	}
	else{
          std::transform(buffer.begin(), buffer.end(), buffer.begin(), ::tolower);
          advisorList.push_back(buffer);
          //cout << "adding into advisor list : " << advisorList.back() << endl;
	  ss >> buffer;
	  ss >> buffer;
	  advisorStatus.push_back(buffer);
          //cout << "adding into advisor status : " << advisorStatus.back() <<  endl;
	  ss >> buffer;
	  advisorInitWeights.push_back(atof(buffer.c_str()));
          //cout << "adding into advisor init weights : " <<advisorInitWeights.back() << endl;
	}      
      }
  }
}


list<RobotDecision> RSWL::computeDistanceFromTarget(list<RobotDecision> path)
{
  list<RobotDecision>::reverse_iterator decisioniterator;
  //traverse the path in the reverse direction from the target to the orginal position
  RobotDecision previous;
  previous.setArray(advisorCount,actionCount);
  for(decisioniterator = path.rbegin(); decisioniterator != path.rend(); decisioniterator++)
    {
      //when the decision is the final decision , decision from target = 0
      if(decisioniterator == path.rbegin())	
	{
	  (*decisioniterator).distanceFromTarget = 0;	
	}
      // otherwise distance from target = distance from this position to the previous position + distance to
      // target of the previous decision 
      else
	{
	  (*decisioniterator).distanceFromTarget = previous.distanceFromTarget + getDistance((*decisioniterator).robotLocation, previous.robotLocation);
	  //cout << "previous to target :" << previous.distanceFromTarget << endl;
	  //cout << " distance to previous :" << getDistance((*decisioniterator).robotLocation, previous.robotLocation) << endl;
	}
      //cout << (*decisioniterator).distanceFromTarget << endl;
      //cout << "current decision id : " << (*decisioniterator).decisionId <<endl;
      //cout << "previous decision id : " << previous.decisionId << endl;
      previous = (*decisioniterator);
    }
  return path;
}


// Prints the given list of paths
int RSWL::printPathList(list< list<RobotDecision> > pathList)
{	
  list< list<RobotDecision> >::iterator pathiterator;
  for(pathiterator = pathList.begin(); pathiterator != pathList.end(); pathiterator++)
    {
      cout << "new path==========================================================\n";
      printDecisionList(*pathiterator);
    }
  return 0;
}


// Prints a given list of decision or a single path
int RSWL::printDecisionList(list<RobotDecision> decisionList)
{	
  list<RobotDecision>::iterator decisioniterator;
  for(decisioniterator = decisionList.begin(); decisioniterator != decisionList.end(); decisioniterator++)
    {
      printDecision(*decisioniterator);
    }
  return 0;
}


// Prints a single decision and the details
int RSWL::printDecision(RobotDecision decision)
{
  cout << "new decision ---> " << endl;
  cout << "Robot ID - " << decision.robotId <<endl;		
  cout << "Decision ID - " << decision.decisionId <<endl;
  cout << "Decision Tier - " << decision.tier << endl;
  cout << "Goal state - " << decision.goalState <<endl;
  cout << "Action chosen - " << decision.actionChoice <<endl;
  cout << "Target (x,y)- (" << decision.targetLocation.xCoordinate <<","<< decision.targetLocation.yCoordinate << ")"<< endl;
  cout << "Robot Location (x,y) - (" <<decision.robotLocation.xCoordinate<<","<<decision.robotLocation.yCoordinate<< ","<< decision.robotOrientation << ")" << endl;
  cout << "Distance from target - " << decision.distanceFromTarget << endl;
  int i,j;		
  for(i = 0 ; i < advisorCount; i++)
    {
      //cout << "Advisor " << to_string(i+1) << " comments are:" <<endl;
      for(j = 0; j<actionCount; j++)
	{
	  //cout << "   Action " << j+1 << " comment strength is - " << to_string(decision.advisorComments[i][j]) <<endl;
	  //cout<<"   Action " <<j+1<< " preference strength is - " <<to_string(decision.advisorPreference[i][j]) <<endl;
	}
    }
  return 0;
}


list<RobotDecision> RSWL::correct_path(list<RobotDecision> uncorrected_path){
  //read from file
  //parseDistanceVectorFile();
  list<RobotDecision>::iterator pathIterator;
  list<RobotDecision>::reverse_iterator reverse_pathIterator; 
  list<RobotDecision> new_path;
  vector<RobotDecision> uncorrected_path_vector;
  string timestamp;
  //keep the same timestamp for the uncleaned and cleaned paths
  ofstream wallvectors;
  wallvectors.open("corrected_wallvectors.conf", std::ofstream::out);
  

 
  time_t t1 = time(NULL);
  timestamp = "uncorrected_path-" + to_string((int)t1);
  ofstream uncorrected_path_stream_collected;
  //ofstream uncorrected_path_single;
  uncorrected_path_stream_collected.open("paths_uncorrected.conf", std::ofstream::out | std::ofstream::app);
  //uncorrected_path_single.open(timestamp);
  // list<RobotDecision>::iterator pathIterator;
  for(pathIterator = uncorrected_path.begin(); pathIterator != uncorrected_path.end(); pathIterator++){
    RobotDecision uncorr_path_point = *pathIterator;
    if(uncorr_path_point.goalState == false){
      uncorrected_path_stream_collected << uncorr_path_point.robotLocation.xCoordinate << " " << uncorr_path_point.robotLocation.yCoordinate <<" ";
      // uncorrected_path_single << uncorr_path_point.robotLocation.xCoordinate << " " << uncorr_path_point.robotLocation.yCoordinate <<" ";
      // uncorrected_path_single << endl;
    }
  }
  uncorrected_path_stream_collected << endl;
  uncorrected_path_stream_collected.close();
  // uncorrected_path_single.close();
  


 
  cout << "RSWL::correct_path : uncorrected_path length: "<<uncorrected_path.size()<<endl;

  //transfer it to a vector because they're easier to work with than iterators
  for(pathIterator = uncorrected_path.begin(); pathIterator != uncorrected_path.end(); pathIterator++){
    RobotDecision decision = *pathIterator;
    if(decision.goalState == false){
      uncorrected_path_vector.push_back(*pathIterator);
      RobotDecision path_point = *pathIterator;
    }
  }
  
  

   cout<<"After moving to vector."<<endl;
 
  //note i++ is absent, don't want to increment every time, only when there's no path to skip to
  for(unsigned int i = 0; i < uncorrected_path_vector.size();){
    cout << "i = "<<i<<endl;
    RobotDecision path_point = uncorrected_path_vector[i];
     
     //add the current point to the new path
     new_path.push_back(path_point);
     
     //output corrected wallvectors, for the time being until we merge RSWL into RobotController
     //for(int z = 0; z < uncorrected_path_vector[i].wallVectorEndpoints.size(); z++){
     //  wallvectors << uncorrected_path_vector[i].wallVectorEndpoints[z].get_x() << " "
     //		   << uncorrected_path_vector[i].wallVectorEndpoints[z].get_y() << " ";
     //}
     

     bool found_path = false;
     //path_point is the current point along the path that we're checking,
     //test_point is the current point along the reverse path that we're looking for a shorter
     //connection to the path_point

       cout << "Current position from line distance segment: ("<<uncorrected_path_vector[i].wallVectorEndpoints[0].get_x()<<","<<
	     uncorrected_path_vector[i].wallVectorEndpoints[0].get_y()<<")"<<endl;
	   cout << "Current position from path: ("<<path_point.robotLocation.xCoordinate<<","<<
	     path_point.robotLocation.yCoordinate<<")"<<endl;
	   

   
     for(unsigned int j = uncorrected_path_vector.size()-1; j > i; j--){
       cout << "j = "<<j << endl;
       RobotDecision test_point = uncorrected_path_vector[j];
       double distance_to_point = getDistance(test_point.robotLocation.xCoordinate,test_point.robotLocation.yCoordinate,
						       path_point.robotLocation.xCoordinate,path_point.robotLocation.yCoordinate);
       
       cout << "distance from test point to current point: " << distance_to_point << endl;
       //if point is reachable in 1 step
       //Future work: allow for multiple steps
       
          cout << "Test position from line distance segment: ("<<uncorrected_path_vector[j].wallVectorEndpoints[0].get_x()<<","<<
	     uncorrected_path_vector[j].wallVectorEndpoints[0].get_y()<<")"<<endl;
	   cout << "Test position from path: ("<<test_point.robotLocation.xCoordinate<<","<<
	     test_point.robotLocation.yCoordinate<<")"<<endl;
	   

       if(distance_to_point < 105){
	 cout << "Distance < 105" << endl;
	 LineSegment line_from_distance_vector;
	 
	 //consider the path point a "circle" to account for the epsilon 
	 double distance_to_circle;
	 //check the wall distance vectors for the particular point and see if there is a direct line to it
	 //first element of wall_distance_segments is the robot's current position
	 CartesianPoint circle_center;
	 cout << "Wall distance vector endpoints size: " << uncorrected_path_vector[j].wallVectorEndpoints.size() << endl;
	 //starts at 1 because the first point is the robot's position
	 for(unsigned int k = 1; k < uncorrected_path_vector[j].wallVectorEndpoints.size(); k++){
	   cout << "k = "<<k<<", ";
	   line_from_distance_vector = LineSegment(uncorrected_path_vector[j].wallVectorEndpoints[0], 
						   uncorrected_path_vector[j].wallVectorEndpoints[k]);
	   
	   
	   //going to set the radius of the circle to 15 units for now
	   circle_center = CartesianPoint(path_point.robotLocation.xCoordinate, path_point.robotLocation.yCoordinate);
	   
	   distance_to_circle = distance(circle_center, line_from_distance_vector);
	   cout << "Distance to circle: "<<distance_to_circle<<endl;
	   //distance < 30 means that the line is within 30 pixels of intersecting, so it is good
	   if(distance_to_circle < 15){
	     cout << "Distance to circle < 15, breaking" << endl;
	     i = j; //move i up to j
	     found_path = true;
	     break; //don't need to keep checking because we are moving backwards along the path, so 
	            //we are guaranteed to have a better path
	   }
	 }
       
	 if(found_path){ break; cout<<"Breaking from j loop"<<endl;}
       }
    }
     //else, the closest path point is the next in the list
     if(!found_path) i++;
  }


  timestamp = "corrected_path-" + to_string((int)t1);
  //ofstream path_stream;
  ofstream collected_path_stream;
  ofstream single_trail;
  //ofstream corrected_path_single;
  //path_stream.open(timestamp.c_str());
  collected_path_stream.open("paths.conf", std::ofstream::out | std::ofstream::app);
  single_trail.open("trail.conf", std::ofstream::out);
  //corrected_path_single.open(timestamp);
  
  // list<RobotDecision>::iterator pathIterator;
  for(pathIterator = new_path.begin(); pathIterator != new_path.end(); pathIterator++){
    RobotDecision path_point = *pathIterator;
    // corrected_path_single << path_point.robotLocation.xCoordinate << " " << path_point.robotLocation.yCoordinate <<" " << endl;
    collected_path_stream << path_point.robotLocation.xCoordinate << " " << path_point.robotLocation.yCoordinate <<" ";
    single_trail << path_point.robotLocation.xCoordinate << " " << path_point.robotLocation.yCoordinate <<" ";
    for(int z = 0; z < path_point.wallVectorEndpoints.size(); z++){
       wallvectors << path_point.wallVectorEndpoints[z].get_x() << " "
     		   << path_point.wallVectorEndpoints[z].get_y() << " ";
     }
    wallvectors << endl;
    

}
  //path_stream.close();
  collected_path_stream << endl;
  collected_path_stream.close();
  //corrected_path_single.close();
  //wallvectors << endl;
  wallvectors.close();
  
  single_trail << endl;
  single_trail.close();
  

  return new_path;
}







// This is the function which reads the log files and stores the data as Objects of RobotDecision class
list<RobotDecision> RSWL::parseLog(string log_name)
{
  //cout << "in function parselog"  << endl;
  list<RobotDecision> rawDecision; 
  // Each robot controller is expected to have a seperate log file which documents its decisions and movements
  // robot session Id is used to name the log files.		
  string line;
  ifstream myfile(log_name);
  // Open the file, read every line and fill in the object of the class RobotDecision
  if(myfile.is_open())
    {
      cout << "opening new file: " << log_name << "\n";
      RobotDecision decision;
      decision.setArray(advisorCount,actionCount);
      
      while(getline(myfile,line))
	{
	  // cout << "reading log file::" << line << endl;
	  string tag = line.substr(0,15);
	  // cout << "Tag: " <<tag<<endl;
	  //was messing up the distance vectors creating a giant string of numbers
	  if(tag != "distancevectors"){
	  line.erase(remove(line.begin(), line.end(),' '), line.end());
	  transform(line.begin(),line.end(), line.begin(), ::tolower);
	  }
	  else cout << line.substr(0,line.find(":")) << endl;
	  //			cout << line << "\n";
	  //every decision in the log has a ID number and also the current goal that it is pursuing. 
	  //decisions are written using xml like tags, with <decision> indicating a new decision , and 
	  //</decision> indicating end of information related to the decision.
	  if(line == "<decision>")
	    {
	      //reinitialize decision
	      RobotDecision newDecision;
	      newDecision.setArray(advisorCount,actionCount);
              //cout << "Printing new decision before inserting values from the logs" << endl;
              //printDecision(newDecision);
	      decision = newDecision;
	      continue;	
	    }
	  else if(line == "</decision>")
	    { 
	      //cout << "decision inserted into the rawDecision list \n";
	      //printDecision(decision);
	      // Every time a new decision object is created, to normalize the comment strenghts from 
	      // differnt advisors we use computeAdvisorPreference function
              //cout << "calling compute advisor preference" << endl;
	      decision = computeAdvisorPreference(decision);
	      //printDecision(decision);
	      rawDecision.push_back(decision);					
	    }
	  // insertdetail() function is called to insert a specific information about the decision into 
	  // the corresponding fields of the RobotDecision object
	  else
	    {
	      //every informaiton is the log is written as a name value pair , separated by :
	      unsigned delimPos = line.find(":");
	      string name = line.substr(0,delimPos);
	      string value = line.substr(delimPos + 1);	
	      // cout << "calling function insertDetail()\n";
	      if(tag == "distancevectors"){ 
		cout << "name = " << name << endl;
		name = tag;
	      }
		//cout << "	with name = " << name << endl;
	      //	cout << "	with value = " << value << endl;
	      decision = insertDetail(name, value, decision);
	    }
	}
      myfile.close();
    }

  cout << "end of parselog"  << endl;
  return rawDecision;	
}


void RSWL::parsePathFile(){
  cout << "Entered parse distance vector. " << endl;
  current_path.clear();
  wall_distance_segments.clear();
  std::ifstream vector_stream;
  vector_stream.open("wallvectors.conf"); 
  vector<CartesianPoint> one_position;
  string fileLine;
  while(!vector_stream.eof()){
    getline(vector_stream, fileLine);
    stringstream ss(fileLine);
    //ss << fileLine;
    string first, second;
    int x,y;
    while(ss>>first){
      ss>>second;
      x = atoi(first.c_str());
      y = atoi(second.c_str());
      one_position.push_back(CartesianPoint(x,y));
    }
    wall_distance_segments.push_back(one_position);
    one_position.clear();
  }
  vector_stream.close();
  cout << "Finished loading in distance vector." << endl;
  //load in the first elements to the path list
  /*

  for(int i = 0; i < wall_distance_segments.size(); i++){
    cout<< "Entered path loop with i =" << i << endl;
    RobotDecision decision;
    decision.robotLocation.xCoordinate = wall_distance_segments[i][0].get_x();
    decision.robotLocation.yCoordinate = wall_distance_segments[i][0].get_y();
    current_path.push_back(decision); //push back only the location of the robot for the path
  }
  */
  cout << "End of parse distance vector." << endl;
}


RobotDecision RSWL::computeAdvisorPreference(RobotDecision decision){
  //cout << "start compute advisor preference function" << endl;
  std::set<float> distinctComments;
  for(int i = 0; i< advisorCount; i++){
    distinctComments.clear();
    for(int k = 0 ; k < actionCount; k++){
      if(decision.advisorComments[i][k] != 1000)
	distinctComments.insert(decision.advisorComments[i][k]);
    }
    for(int j = 0; j < actionCount; j++){
      float test = decision.advisorComments[i][j];
      //cout << "Comment value is : " << decision.advisorComments[i][j] << endl;
      if(test == 1000){
	decision.advisorPreference[i][j] = decision.advisorComments[i][j];;
      }
      else{
        float rank = 1;
        set<float>::iterator iter; 
	for(iter = distinctComments.begin(); iter != distinctComments.end(); iter++){
          //cout << "distinct list: " << (*iter) << endl;
	  if(test < (*iter)){
	    rank++;
	  }
	}
        decision.advisorPreference[i][j] = rank;
        //cout << "Rank: " << rank << endl;
      }
    }
  }
  return decision;
}


// this function is used by the parse log, to identify the type of the detail and insert into the 
// RobotDecision object accordingly
RobotDecision RSWL::insertDetail(string name, string value, RobotDecision decision)
{
  //cout << "RSWL::insertDetail >> " << endl;
  regex rx ("(<)(.*)");
  //if(name == "robotid")
    //decision.robotId = 1; 
  if(name == "decisionid")
    decision.decisionId = stoi(value);
  else if(name == "targetx")
    decision.targetLocation.xCoordinate = stoi(value);		
  else if(name == "targety")
    decision.targetLocation.yCoordinate = stoi(value);	
  else if(name == "tier")
    decision.tier = stoi(value);
  else if(name == "goalstate")
    {		
      if(value == "true")
	decision.goalState = true;		
      else
	decision.goalState = false;		
    }		
  else if(name == "actionchosen")
    {
      int actionTypeId = stoi(value.substr((value.find("<") +1),(value.find(",") -1)));
      int intensityId = stoi(value.substr((value.find(",") +1),(value.find(">") -1)));
      decision.actionTypeId = actionTypeId;
      decision.intensityId = intensityId;
      decision.actionChoice = (actionTypeId * intensityCount) + intensityId;
    }		
  else if(name == "robotlocationx")
    decision.robotLocation.xCoordinate = stoi(value);
  else if(name == "robotlocationy")
    decision.robotLocation.yCoordinate = stoi(value);
  else if(name == "robotorientation")
    decision.robotOrientation = stof(value);
  else if(name == "maxforwardmove")
    decision.maxForwardMove = stoi(value);
  else if(name == "distancevectors"){
    cout << "Distance vector recieved. Value is: "<<value<<endl;
    stringstream ss(value);
    //ss << value;
    string first, second;
    double x,y;
    while(ss>>first){
      ss>>second;
      x = atoi(first.c_str()) * 1.0;
      y = atoi(second.c_str()) * 1.0;
      decision.wallVectorEndpoints.push_back(CartesianPoint(x,y)); 
    }
  }
  else if(regex_match (name,rx))
    {
      string advisorName = name.substr((name.find("<") + 1),(name.find(",") -1));
      int actionTypeId = stoi(name.substr((name.find(",") +1),(name.find_last_of(",") -1)));
      int intensityId = stoi(name.substr((name.find_last_of(",") +1),(name.find(">") -1)));
      //cout << "action type id :" << actionTypeId<< endl;
      //cout << "intensity id :" << intensityId<< endl;
      //cout << "advisor name :" << advisorName << endl;
      int actionId = (actionTypeId * intensityCount) + intensityId;
      //cout << "action id : " << actionId << endl;
      int advisorId;
      //cout << "before getting iterator test" << endl;
      std::map<string,int>::iterator it = activeAdvisorList.find(advisorName);
        advisorId = it->second;
        //cout << "Advisor Name : " << advisorName << " AdvisorId : " << advisorId << endl;
    
      if(value == "notcommenting")
	{
	  int j = 0;
	  //cout << advisorId << endl;
	  for(j = 0; j < actionCount; j++)
	    {
	      decision.advisorComments[advisorId - 1][j] = 1000;
	      //cout << advisorId << j+1 << " " << decision.advisorComments[advisorId-1][j] <<endl;
	    }
	}
      else
	{
          //cout << "saving comments" << endl;
	  decision.advisorComments[advisorId - 1][actionId - 1] = stof(value);
	  //cout << advisorId << actionId << " " << decision.advisorComments[advisorId-1][actionId-1] <<endl;
	}			
    }
  //cout << "after inserting details" << endl;
  // remove after getting the right log
  decision.tier = 3;
  decision.robotId = 1;
  return decision;	
}
 

// Once the raw decision list is created and advisorPreference ranks are generated , this function splits a single list 
// into a list of list of decisions or a list of paths, with each path indicating a robot meeting a particular objective 
/*list< list<RobotDecision> > RSWL::extractSuccessfulPaths(list<RobotDecision> rawDecisions)
{ 
  list< list<RobotDecision> > successfulPathList;
  list<RobotDecision>::iterator decisioniterator;
  list<RobotDecision> robotArray[robotCount];
  // add every decision into a new path until the decision has goal state = true, and then add the path into the path list.
  for(decisioniterator = rawDecisions.begin(); decisioniterator != rawDecisions.end(); decisioniterator++)
    {	
      RobotDecision decision = *decisioniterator; 
      //cout << "tier: "<< decision.decisionId << endl;
      if(decision.tier == 3)
	{
	  robotArray[(decision.robotId)-1].push_back(decision);
	  if(decision.goalState == true)
	    {
	      list<RobotDecision> path = robotArray[(decision.robotId)-1]; 		
	      robotArray[(decision.robotId)-1].clear();
	      //printDecisionList(path);
	      successfulPathList.push_back(path);
	    }
	}
    }
  return successfulPathList;
}			
*/


void RSWL::computeCorrectActions(list<RobotDecision> *path){
  list<RobotDecision>::iterator itr1, itr2;
  for(itr1 = path->begin(); itr1 != path->end(); itr1++){
    RobotDecision current = *itr1;
    Location currentLoc = current.robotLocation;
    Location next;
    bool isNext = false;
    cout << "RSWL::computeCorrectActions>>" << endl;
    printDecision(current);
    for(itr2 = itr1, itr2++; itr2 != path->end() ; itr2++){
      next = (*itr2).robotLocation;
      //cout << "Before correcting ----------------------------------------------------- :" << endl;
      //printDecision(*itr2);
      if(next.xCoordinate != currentLoc.xCoordinate || next.yCoordinate != currentLoc.yCoordinate){
	isNext = true; 
	break;
      }
    }
    if(isNext == true){
      cout << "Next point to reach is " << next.xCoordinate << " " << next.yCoordinate << endl;
      itr1->actionChoice = computeAction(*itr1, next.xCoordinate, next.yCoordinate);
    }
    else{
      itr1->actionChoice = -1;
      cout << "Did not find the right path , hence setting actionChoice to -1 " << endl;
    }
    cout << "after correcting --------------------------->" << endl;
    printDecision(*itr1);
    cout << "----------------------------------------------" << endl;
  } 
  cout << "RSWL::computeCorrectActions>> : Exit() " << endl;
}


int RSWL::computeAction(RobotDecision decision, int x, int y){
  printDecision(decision);
  int actionChoice = -1;
  double robot_direction = decision.robotOrientation;
  int curX = decision.robotLocation.xCoordinate;
  int curY = decision.robotLocation.yCoordinate;
  
  double distance_from_target = getDistance(curX, curY, x, y);
  
  cout << "Distance from target : " << distance_from_target << endl;
  // compute the angular difference between the direction to the target and the current robot direction
  
  // when in rotate mode decision id = odd => rotation.
  if(decision.decisionId % 2 == 1){
    double goal_direction = atan2((y - curY), (x - curY));
    double required_rotation = goal_direction - robot_direction;
    
    if(required_rotation >  3.39)   required_rotation -= 6.283;
    if(required_rotation < -3.39)   required_rotation += 6.283;
    
    cout << "Robot direction : " << robot_direction << ", Goal Direction : " << goal_direction << ", Required rotation : " << required_rotation << endl;
    // if the angular difference is greater than smallest turn possible 
    // pick the right turn to allign itself to the target
    if( required_rotation > 0.1548 && required_rotation <= 0.3048)
      actionChoice = 3*intensityCount + 1; // turn left with intensity 1
    else if( required_rotation > 0.3048 && required_rotation <= 0.65)
      actionChoice = 3*intensityCount + 2; // turn left, i = 2
    else if( required_rotation > 0.65 && required_rotation <= 1.3)
      actionChoice = 3*intensityCount + 3; // turn left, i = 3
    else if(required_rotation > 1.3 && required_rotation <= 3.39)
      actionChoice = 3*intensityCount + 4; // turnleft , i = 4
    else if( required_rotation < -0.1548 && required_rotation >= -0.3048)
      actionChoice = 4*intensityCount + 1; // turn right , i = 1
    else if( required_rotation < -0.3048 && required_rotation >= -0.65)
      actionChoice = 4*intensityCount + 2; // turn right , i = 2
    else if( required_rotation < -0.65 && required_rotation >= -1.3)
      actionChoice = 4*intensityCount + 3; // turn right , i = 3
    else if(required_rotation < -1.3 && required_rotation >= -3.39)
      actionChoice = 4*intensityCount + 4; // turn right , i = 4
    cout << "Correct RotationAction : " << actionChoice << endl;
  }
  else{
    double min = 1000;
    int intensity = 0;
    for(int i = 0 ; i < 6 ; i++){
      int d = getDistanceAfterMove(i, curX, curY, decision.robotOrientation, x, y);
      if(d < min){
	intensity = i;
	min = d;
      }
    }
    actionChoice = 1*intensityCount + intensity;
    cout << "Correct ForwardAction before adjusting for vetoed moves: " << actionChoice << endl;
    // pick the action choice which has not been vetoed
    if(actionChoice > decision.maxForwardMove)  
      actionChoice = 1*intensityCount + decision.maxForwardMove;

    cout << "Correct ForwardAction : " << actionChoice << endl;
  }
  return actionChoice;
}


int RSWL::getDistanceAfterMove(int intensity, int curX, int curY, double curT, int targetX, int targetY){
  int distance = 0;
  if (intensity == 0) distance = 0;
  if (intensity == 1) distance = 3;
  if (intensity == 2) distance = 7;
  if (intensity == 3) distance = 20;
  if (intensity == 4) distance = 40;
  if (intensity == 5) distance = 105;

  double newX = curX + distance * cos(curT);
  double newY = curY + distance * sin(curT);
  
  return getDistance(newX, newY, targetX, targetY);
} 

						
// returns a vector with pair of total weight and the number of comments made for each advisor
pair < float* , float * > RSWL::computeWeights(list<RobotDecision> path)
{
  //------------- Declare and initialize weights and comment count---
  std::pair <float* , float*> result;
  float* weights = new float[advisorCount];
  float* commentCount = new float[advisorCount];
  for(int k = 0; k < advisorCount; k++){
    commentCount[k] = activeAdvisorCommentCount[k];
    weights[k] = activeAdvisorWeights[k];
  }

  // computes the right action choice based on the next location in the corrected path
  cout << "before computing correct actions " << endl;
  computeCorrectActions(&path);
  cout << "after computing correct actions " << endl;

  //--------------- Extraction digressions ------------------
  //list<Digression> totalDigression = extractDigression(path);
  
  //--------------- Add penalty for every digression -------
  //list<Digression>::iterator digressioniterator;			
  //for(digressioniterator = totalDigression.begin(); digressioniterator != totalDigression.end(); digressioniterator++)
  //{
  //  cout << "calling penalty function for decisionId :" << (*digressioniterator).start.decisionId << endl; 		
  //  addPenalty((*digressioniterator),weights,commentCount);
  //  cout << "Weights after adding penalty: \n";
      //for(int i = 0 ; i < advisorCount; i ++)
      //	{
      //	  cout << "Advisor " << i+1 << " : " << weights[i] << endl;
      //	}
  //}
  
  //-------------- Remove Digression to get positive decision list ----
  //list<RobotDecision> positiveDecisionList = path;
  //if(totalDigression.size() >= 1)
  //{
  //  			cout << "calling remove digression" << endl;
  //  positiveDecisionList = removeDigression(path,totalDigression);
  //}
  list<RobotDecision>::iterator decisioniterator1;
  cout << "Positive decision list:   ---------" << endl;
  printDecisionList(path);
  cout << "End of positive Decision list :  ------- "<< endl;
  
  //-------------- Add reward for every positive decision --------------
  for(decisioniterator1 = path.begin(); decisioniterator1 != path.end(); decisioniterator1++)
    {
      RobotDecision decision = *decisioniterator1;
      //cout << "calling reward function for decisionId :" << decision.decisionId << endl;
      if(decision.goalState == false){
	addReward(weights,decision,commentCount);
	//cout << "Weights ADDING REWARD: \n";
	//for(int i = 0 ; i < advisorCount; i ++)
	// {
	//   cout << "Advisor " << i+1 << " : " << weights[i] << endl;
	//  }
      }
    }
  // --------------- compute final weights by dividing p-weight by total comment count---------
  //for(int k = 0; k < advisorCount; k++)
  //{
  //  cout << "comment count of advisor " << k+1 << " : " << commentCount[k] << endl;
  //  if(commentCount[k] != 0)
  //	{
  //	  weights[k] = weights[k]/commentCount[k];
  //	}
  // }
  result.first = weights;
  result.second = commentCount;
  return result;
}  

// Finds and returns digressions in a given path
list<Digression> RSWL::extractDigression(list<RobotDecision> path)	
{
  list<Digression> totalDigression;
  list<RobotDecision> coveredPath; 
  list<RobotDecision>::iterator decisioniterator;
  // for every decision in the path
  for(decisioniterator = path.begin(); decisioniterator != path.end(); decisioniterator++)
    {
      RobotDecision decision = *decisioniterator;
      coveredPath.push_back(decision);
      if(coveredPath.size() > 1)
	{
	  // cout << "Calling function findDigression" << endl;
	  // check if the decision is same as the other previous decisions already covered by calling the 
	  // function findDigression()
	  Digression digression = findDigression(coveredPath);
	  if(digression.start.decisionId != 0 || digression.end.decisionId != 0)				
	    {	
	      totalDigression.push_front(digression);
	      cout << "new digression found: "<< digression.start.decisionId <<" to " << digression.end.decisionId <<endl;
	    }
	}		
    }
  // because we use an epsilon margin to check if two decisions are same, this leads to duplicate digressions
  totalDigression = removeDuplicateDigression(totalDigression);
  return totalDigression;
}

// this function removes the duplicate digressions created by the epsilon
list<Digression> RSWL::removeDuplicateDigression(list<Digression> digressionList)
{
  list<Digression> tempDigressionList; 
  list<Digression>::iterator digressioniterator;
  Digression current,previous;
  // for every digression
  for(digressioniterator = digressionList.begin(); digressioniterator != digressionList.end(); digressioniterator++)
    {
      if(digressioniterator == digressionList.begin())
	{ 
	  previous = *digressioniterator;
	  tempDigressionList.push_back(previous);
	} 
      else
	{
	  current = *digressioniterator;
	  if(current.start.decisionId == previous.start.decisionId)
	    {
	      tempDigressionList.pop_back();
	    }
	  tempDigressionList.push_back(current);
	  previous = current;
	}
    }
  return tempDigressionList;
}

// returns a single digression 
Digression RSWL::findDigression(list<RobotDecision> path)
{
  Digression digression;
  digression.start.decisionId = 0;
  digression.end.decisionId = 0;
  // the last decision is the decision , with which all other earlier decisions will be tested to check for digression
  RobotDecision currentDecision = path.back();
  path.pop_back();
  list<RobotDecision>::reverse_iterator decisioniterator;
  // wasOutOfRange is used to make sure that there is atleast one decision point outside of the epsilon to get rid of some 
  // duplicate digressions created by introducing digressions
  bool wasOutOfRange = false;
  for(decisioniterator = path.rbegin(); decisioniterator != path.rend(); decisioniterator++)
    {
      RobotDecision decision = *decisioniterator;
      if(checkEqual(decision,currentDecision) == false)
	{
	  wasOutOfRange = true;
	}
      if((checkEqual(decision,currentDecision) == true) && (wasOutOfRange == true))
	{
	  digression.start = decision;
	  digression.end = currentDecision;
	  //return digression; // important dont remove
	}
    }
  return digression;		 
}


// given a digression , this function computes the penalty using the decision which caused digression
int RSWL::addPenalty(Digression digression,float* weights,float commentCount[])
{
  // use the decision which caused the digression, i.e start of the digression
  RobotDecision wrongDecision = digression.start;
  //cout << "negative decision:-------------- "<< endl;
  //printDecision(wrongDecision);
  //cout << "End:-------------- "<< endl;
  // compute relative support which tells how much a particular action is supported by each of the advisor which is interested
  // in the action.
  float* relativeSupport = computeRelativeSupport(wrongDecision.advisorPreference, wrongDecision.actionChoice);
  // because the action resulted in the bad decision , which lead to the digression, each advisor which supported a bad
  // decision is penalized
  for(int advisor = 1; advisor<= advisorCount; advisor++)
    {
      if(isAdvisorInvolved(advisor, wrongDecision.advisorPreference,  wrongDecision.actionChoice))
	{
	  //commentCount[advisor-1] = commentCount[advisor-1] + 1;
	  //		cout << "advisor " << advisor-1 << "is involved in decision " << wrongDecision.decisionId << endl;
	  // We consider an advisor supporting an action , if its relative support is > 0
	  if(relativeSupport[advisor - 1] > 0)
	    {	
	      // p-weight = p-weight - RS/Difficulty(distance covered)
	      weights[advisor - 1] = weights[advisor - 1] - relativeSupport[advisor - 1];
	      //    /wrongDecision.distanceFromTarget);
	      //commentCount[advisor-1] = commentCount[advisor-1] + 1;
	    }
	  else
	    {
	      cout << "advisor does not support this decision" << endl;
	    }		
	}
      else
	{	
	  //cout << "advisor " << advisor << " is not involved in this decision" << endl;
	}
    }
  return 0;
}
        

// removes the digression from the path to leave behind only good or correct decisions which lead to the robot reaching the goal
list<RobotDecision> RSWL::removeDigression(list<RobotDecision> path, list<Digression> totalDigression)
{
  /*
  string timestamp;
  //keep the same timestamp for the uncleaned and cleaned paths
  time_t t1 = time(NULL);
  timestamp = "uncorrected_path-" + to_string((int)t1);
  ofstream uncorrected_path_stream;
  uncorrected_path_stream.open(timestamp.c_str());
  list<RobotDecision>::iterator pathIterator;
  for(pathIterator = path.begin(); pathIterator != path.end(); pathIterator++){
    RobotDecision path_point = *pathIterator;
    uncorrected_path_stream << path_point.robotLocation.xCoordinate << " " << path_point.robotLocation.yCoordinate << endl;
  }
  uncorrected_path_stream.close();
  */

  
  //printDecisionList(path);
  list<RobotDecision>::iterator decisioniterator;	
  list<Digression>::iterator digressioniterator;
  // for every digression
  for(digressioniterator = totalDigression.begin(); digressioniterator != totalDigression.end(); digressioniterator++)
    {
      Digression digression = *digressioniterator;
      list<RobotDecision> newPath;
      // for every decision in the path
      for(decisioniterator = path.begin(); decisioniterator != path.end(); decisioniterator++)
	{
	  RobotDecision decision =  *decisioniterator;
	  //cout << "Decision id : " << decision.decisionId << endl;
	  //cout << "Digre start : " << digression.startDecisionId << endl;		
	  //cout << "Digre end : " << digression.endDecisionId << endl;
	  // if the decision is not a digression, save the decision in a new path
	  if(decision.decisionId < digression.start.decisionId || decision.decisionId >= digression.end.decisionId) 
	    {
	      newPath.push_back(decision);
	    }
	}
      path = newPath;
    }
  //cout << "after removing digression !!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
  //printDecisionList(path);

  //print path to file with timestamp name for now
  
  //time_t t1 = time(NULL);
  /*
  timestamp = "corrected_path-" + to_string((int)t1);
  ofstream path_stream;
  path_stream.open(timestamp.c_str());
  // list<RobotDecision>::iterator pathIterator;
  for(pathIterator = path.begin(); pathIterator != path.end(); pathIterator++){
    RobotDecision path_point = *pathIterator;
    path_stream << path_point.robotLocation.xCoordinate << " " << path_point.robotLocation.yCoordinate << endl;
  }
  path_stream.close();
  */

  return path;
}



// similar to add penalty , it adds penalty based on the relative support , and updates the comment count
int RSWL::addReward(float* weights, RobotDecision decision, float commentCount[])
{
  cout << "RSWL::addReward >> " << endl;
  float* relativeSupport = computeRelativeSupport(decision.advisorPreference,decision.actionChoice);
  int advisor;		
  for(advisor = 1; advisor <= advisorCount; advisor ++)
    {	
      if(isAdvisorInvolved(advisor, decision.advisorPreference, decision.actionChoice))
	{
	  commentCount[advisor - 1] = commentCount[advisor - 1] + 1;
	  if(relativeSupport[advisor - 1] > 0)
	    {
	      cout << "relative support of advisor " << advisor << ":" << relativeSupport[advisor - 1] << endl;
              //int decisionImpact = 1;
              //if(decision.actionChoice >= 6 && decision.actionChoice <= 10){
	      //decisionImpact = decision.actionChoice - 5;
	      //}
	      weights[advisor - 1] = weights[advisor - 1] + relativeSupport[advisor - 1]; // *decisionImpact;
	      //commentCount[advisor-1] = commentCount[advisor-1] + 1;
	    }
	  else
	    {
	      cout << "Advisor " << advisor << " is not supporting the decision" << endl;	
	    }
	}
      else
	{
	  cout << "Advisor " << advisor << " is not involved in the decision making" << endl;
	}
    }
  return 0;
}


// computes relative support for an action using the formula RS = commentstrength - avg of all of its comments
float* RSWL::computeRelativeSupport(float** advisorPreference, int action)
{
  //cout << "action choosen :" << action << endl;
  float* relativeSupport = new float[advisorCount];
  for(int advisor = 1; advisor <= advisorCount; advisor++)
    {
      //cout << "action: " << action << " Comment : " << advisorPreference[advisor-1][action - 1] << endl;
      float *comments = advisorPreference[advisor - 1];
      // cout << "action comment strength: " << comments[action - 1] << endl;
      float average = getAverage(comments);
      //cout << "average comment strength: " << average << endl;
      if(average == 0  || comments[action - 1] == 1000){
	relativeSupport[advisor - 1] = 0;
      }
      else{
	relativeSupport[advisor - 1] = (average - comments[action - 1])/average;
      }
      //cout << "average support: " << average << endl;
      //cout << "relatve support: " << relativeSupport[advisor - 1] << endl;
    }
  return relativeSupport;
}


// computes the average of an array except for the 1000..
float RSWL::getAverage(float comments[])
{
  float sum = 0;
  float count = 0;
  bool novalidComments = true;
  for(int i = 0; i < actionCount ; i++)
    {      
      if(comments[i] != 1000){
	sum = sum + comments[i];
        count++;
        novalidComments = false;
      }
    }
  if(novalidComments == false){
    return (sum/count);
  }
  else{
    return 0;
  }
}	


//Checks if the two decision states are same.. In this case robot state is just the robot location
bool RSWL::checkEqual(RobotDecision decision, RobotDecision currentDecision)
{
  //cout << " in function checkequal" << endl;
  if(getDistance(decision.robotLocation,currentDecision.robotLocation) < closenessThreshold)
    {
      //	cout << "two decision are equal" << endl;
      return true;
    }
  else
    {
      //	cout << "two decision are not equal" << endl;
      return false;
    }
}


// computes euclidean distance betweem two points
float RSWL::getDistance(Location location1, Location location2)
{
  float distance;
  distance = sqrt(pow(location1.xCoordinate-location2.xCoordinate,2) + pow(location1.yCoordinate-location2.yCoordinate,2));
  return distance;
}



// Checks if the advisor is involved in decision making or not
bool RSWL::isAdvisorInvolved(int advisor, float ** advisorPreference, int choosenAction )
{
  float* comments = advisorPreference[advisor-1];
  //cout << "advisorComment on action " << choosenAction << " is " << comments[choosenAction-1] << endl;
  // if the advisor has not commented on the chosen action, we have no way of knowing it correctness
  if(comments[choosenAction-1] == 1000){
    return false;
  }
  for(int i = 1; i <= actionCount ; i++)
    {
      //cout << "advisor Comment : " << comments[i-1] << endl;
      if(comments[i-1] != 1000)
	{
	  return true;
	}
    }
  return false;
}


