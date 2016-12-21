/***********************************************************************
The class FORRTrace.h contains datastructure and functions related to
recording, cleaning, maintainig the previous History of the robot/s
location in the run

Author : Anoop Aroor
Data   : 26th Dec 2014
***********************************************************************/

#include <iostream>
#include <vector>
#include <FORRGeometry.h>

class FORRTrace{  
 public:
  // reads the last line from paths.conf which is the cleaned path and updates the trace ds
  void read_trace_from_file(){
    ifstream pathstream;
    pathstream.open("paths_uncorrected.conf");
    string line, lastLine;
    while(!pathstream.eof()){
      lastLine = line;
      getline(pathstream, line);
      //cout << "line" << line << endl;
    }
    //cout << "lastline :" << lastLine << endl;
    istringstream iss(lastLine);
    vector<CartesianPoint> trace;
    do{
      string sub1, sub2;
      iss >> sub1;
      iss >> sub2;
      if(sub1 != ""){
	//cout << sub1 << " " << sub2 << endl;
	CartesianPoint p( atof(sub1.c_str()), atof(sub2.c_str()) );
	trace.push_back(p);
      }
    }
    while(iss);
    run_trace.push_back(trace);
    //cout << "run trace size : " << run_trace.size() << endl;
  };
  
  // returns trace of the last completed task
  vector<CartesianPoint> getLastTrace(){
    return run_trace[run_trace.size() - 1];
  };

  vector< vector<CartesianPoint> > getAllTrace(){  return run_trace; };

 private:
  vector< vector<CartesianPoint> > run_trace;
};
