/*!
 * main.cpp
 *
 * \brief 
 * This is the main function which spawns the descriptive manager and the communication manager thread
 * \author Anoop Aroor <anoop.a.rao@gmail.com>
 */

#include <cstdlib>
#include <fstream>
#include <iostream>
#include "Map.h"
#include "FORRWall.h"

#include <boost/thread.hpp>

#include "CommunicationManager.h"
#include "DManagerMessageHandler.h"

#include "DescriptiveManager.h"

//#define DEFAULT_CS_HOST	"192.168.1.200"
#define DEFAULT_CS_HOST "127.0.0.1"
#define DEFAULT_CS_PORT	6667


using namespace std;
// buffer size is used for increasing the cost of the edges near the walls
const int DEFAULT_BUFFER_SIZE = 30;

// function that reads advisors description
void read_advisor_file(ifstream& file, DescriptiveManager* d_manager);

// code used to read from the config files
Map readMapFile(string filename); 
void parseLine(string line, vector<CartesianPoint> *target);

void createMap_defaultField(int rsize, Map *myMap) {
  myMap = new Map(500, 400, rsize);
  cout << "creating default map!!! " << endl;
  // outer walls
  myMap->addWall("wall1", 0, 0, 0, 400); 
  myMap->addWall("wall2", 0, 0, 500, 0); 
  myMap->addWall("wall3", 500, 0, 500, 400); 
  myMap->addWall("wall4", 0, 400, 500, 400); 
}


Map readMapFile(string filename) {
   Map *myMap; 
   ifstream mFile(filename); 
   if( !mFile ) {
	  cout << "Can't open map configuration file: " << optarg << ". Aborted" << endl; 
	  exit(1);
   }
  
  string  cmd, label, tmp, line;
  int x1, y1, x2, y2, lx, ly, rx, ry; 
  bool first = true ;
  int bufferSize = DEFAULT_BUFFER_SIZE;

  //cout << "inside readMapFile: " <<endl; //mFile.rdbuf() << endl;
  
  while(getline(mFile, line)) {
    cmd = "",label = ""; x1 = 0 ; y1 = 0 ; x2 = 0 ; y2 = 0 ; 
    //cout << line << endl;
    stringstream ss(line); 
    ss >> cmd; 
    if (! (( cmd[0] == '/' && cmd[1] == '/' )|| cmd =="") ) { 
      if ( first ){
	first = false;
	if ( cmd == "size" ) {
          //cout << "size command" << endl;
	  ss >> x1 >> y1 >> bufferSize; 
	  myMap = new Map(x1, y1, bufferSize);
	  continue;
	}
	else
	  createMap_defaultField(bufferSize, myMap);
      }

      // process the command
      if ( cmd == "marker" ) { 
        //cout << "marker command" << endl;
	ss >> label >> x1 >> y1 >> lx >> ly >> rx >> ry ; 
	myMap->addMarker(MapMarker(label, x1, y1, lx, ly, rx, ry));	
      }
      else if ( cmd == "wall" ) {
        //cout << "wall command" << endl;
	ss >> label >> x1 >> y1 >> x2 >> y2 ; 
	myMap->addWall(label, x1, y1, x2, y2); 
      }
      else if ( cmd == "virtualWall" ) {
        //cout << "virtualwall command" << endl;
	ss >> label >> x1 >> y1 >> x2 >> y2 ; 
	myMap->addVirtualWall(label, x1, y1, x2, y2);
      }
      else {
	if ( cmd == "size" )
	  ;//cout << "size command has to be the first command in map config file. command ignored." << endl;
	else
	  cout << "Unknown map config command: " << cmd << endl; 
      }
    }
  } 
  //cout << "map generated Hopefully" << (mapwall[6]).getX0() << endl;
  return *myMap;
}


// code to read from the configuration file the target points
/*
vector<CartesianPoint> readTargetFile(char *filename){
    vector<CartesianPoint> target;
    cout << "configuration file containing targets: " << endl;
    ifstream configFile(filename);
    if( !configFile ) 
    {
	  cout << "Can't open configuration file: " << filename << ". Aborted" << endl; 
	  exit(1);
    }
  string fileLine;

  while(getline(configFile, fileLine)){
    parseLine(fileLine, &target);
  }
  //cout << "before returning: " << (target[0]).get_x() <<endl;
  return target;
}
*/

/*
void parseLine(string line, vector<CartesianPoint> *target){
  // this is parsing line by tokens that are separated by whitespace from SO
  stringstream ss(line);
  string buffer;
  // since I am reading line by tokens I need counter to tell me at which position
  // am I so to know which 
  // for the price of less intuitive code function is more general and configuration
  // files can have comment lines
  int counter = 0;

  try{
    while(ss >> buffer){
      // is this a comment line
      if(buffer == "#")
      {
	//cout << "found comment" << endl;
	  return;
      }
      double x, y;
      if(counter == 0)
      {
   	  x = strtod(buffer.c_str(), NULL);
      }
      else
      {
	  y = strtod(buffer.c_str(), NULL);
	  //cout << "In target points, read " << x << ' ' << y << endl;
	  target->push_back(CartesianPoint(x, y));
	  counter = -1;
      }
      counter++;
    }
  }
  catch(int e){
    cout << e << " exception raised " << endl;
  }
}
*/

// Main function!!!!!!!!!!!!
// unsigned int Task::next_task_id = 0;

int main(int argc, char** argv) {

  string cs_host = DEFAULT_CS_HOST;
  unsigned short cs_port = DEFAULT_CS_PORT;
  DescriptiveManager *d_manager;

  CommunicationManager *comm_mgr;
  DManagerMessageHandler *msg_handler;

  boost::thread *comm_manager_thread, *d_manager_thread;
  
  string my_name = "descriptive_manager";
  
  comm_mgr = new CommunicationManager(SID_DESCRIPTIVE_MANAGER,
                                      my_name, cs_host, cs_port);
  
  // Connect to the central server
  comm_mgr->connect();
  
  // Start a thread for the communication manager
  comm_manager_thread = new boost::thread(boost::ref(*comm_mgr));

  // usage: <exec> [-h central_server_host] [-p central_server_port]
  // To add more flags add it to the end of string followed
  // by a : or :: if the flag doesn't require an argument
  const char* optflags = "h:p:m:a:r:"; 
  int ch;
  ifstream configFile;
  Map map;
  string advisor_conf_file,rswl_conf_file,map_conf_file;
  vector<MapWall> mapwall;
 
  // Read command line options
  while ( -1 != ( ch = getopt( argc, argv, optflags ))) {
    switch (ch) {
    case 'h':
      cs_host = optarg;
      break;
    case 'p':
      cs_port = atoi(optarg);
      break;
    case 'm':
      map_conf_file = optarg;
      break;
    case 'a':
      advisor_conf_file = optarg;
      break;
    case 'r':
      rswl_conf_file = optarg;
      break;
    }
  }

  map = readMapFile(map_conf_file);
  mapwall = map.getWalls();
  
  // create a new decriptive manager message
  d_manager = new DescriptiveManager(map, advisor_conf_file, rswl_conf_file);
  
  configFile.open(advisor_conf_file, ios::in);
  if(!configFile){
    cout << "Can't open configuration file: " << optarg << ". Aborted" << endl; 
    exit(1);
  }
  read_advisor_file(configFile, d_manager);
  configFile.close();
  
  // Create a new message handler and Allow Dmanager message handler to access d_manager and comm_mgr
  msg_handler = new DManagerMessageHandler(comm_mgr, d_manager);

  // Allow the Discriptive manager to access the message handler
  d_manager->set_message_handler(msg_handler);
  
  // Create a new thread for the descriptive manager
  d_manager_thread = new boost::thread(boost::ref(*d_manager));
  cout << "Before creating threads multiple threads" << endl;
  cout << "something" << endl;
  // Handle messages until done
  d_manager_thread->join();
  cout << "created dmanager thread" << endl;
  comm_manager_thread->join();
  
  cout << "out of the main function after creating multiple threads" << endl;
  return 0;
}

void read_advisor_file(ifstream& file, DescriptiveManager* d_manager){
  string fileLine;

  cout << "Inside file in read_advisor_file " << endl;
  while(!file.eof()){
    getline(file, fileLine);
    if(fileLine[0] == '#')  // skip comment lines
      continue;
    else{
      d_manager->add_new_description(fileLine);
      cout << "Adding description " << fileLine << endl;
    }
  }
}
