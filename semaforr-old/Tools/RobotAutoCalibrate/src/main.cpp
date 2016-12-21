/*! \mainpage RobotController Documentation
 * \brief RobotController governs low-level actions of a robot.
 *
 * \authors A. Tuna Ozgelen (contact author), Samuel Sanchez, George Rabanca, Mark Manashirov
 *
 * \version 1.0
 *
 * \date 2010 - 2011
 *
 * \b Introduction:
 *      This program is a collection of low-level control functions of a robot. Each robot 
 *  	posess different type of sensors and hardware, therefore the some of these behaviors
 *  	may not be available and/or may execute differently depending on the platform.
 *  	Low level behaviors implemented: 
 *  	- Motion: vision-based version development is in progress. 
 *  	- PathPlanner: given a map builds a navigation graph and finds the shortest path.  
 *  	- Obstacle avoidance: Not implemented yet. 
 *  	- Mapping: Not implemented yet.
 *  	- Perception: Vision-based version is in progress.
 *  	- Controller: basic behavior control
 *     	Also, to for debugging a visual debugger tool is available. 
 *
 * \b Compiling:
 *      - Run 'make' to build the libraries and the .controller executable
 *    	- Run 'make clean' to delete object files and the controller (doesn't rm libraries)
 *  	- Run 'make purge' to delete all *.o and *.a files from lib directories. 
 * 	- Run 'make docs' to install the documentation files into the /doc directory.
 *
 * \b Usage:
 *      Run ./controller with:
 *	   - -d option (optional) to run with the visual debugger
 *	   - -f option to specify the location of the configuration file. This file contains
 *	       - ip and port information of the central server
 *	       - map filename and path
 *	       - an entry for each robot containing:
 *	            - ip and port information of the player server
 *		    - robot's label
 *		    - robot's type (aibo, surveyor, scribbler, nxt)
 *		 
 * \todo implement ranger-based localization
 * \todo implement VFH for obstacle avoidance both with vision and range sensors
 * \todo implement mapping functionality
 * \todo implement ranger-based perception 
 *
 * \warning Still in development, contact author for questions. 
 *
 * \file main.cpp
 *
 * Parses the configuration files to connect to central server and player for the 
 * robot. Also parses the map file to use it in localization and initiates the visual 
 * debugger if -d option passed as an argument 
 *
 * Usage : ./controller [-d] -f <config-file>
 *
 * \defgroup Controller
 * \defgroup PathPlanner
 * \defgroup Motion
 * \defgroup VisualDebugger
 * \defgroup Perception
 */
#include "ConfigManager.h"
#include "definitions.h"
#include "CommunicationManager.h"
#include "Controller.h"
#include "Utils.h"
#include "boost/asio.hpp"
#include "boost/shared_ptr.hpp"
#include <fstream>
#include "libplayerc++/playerc++.h"
#include <time.h>
using namespace PlayerCc;

#include <cstdlib>
#include <iostream>
#include <sstream>
#include <unistd.h>
#include <cstring>
using namespace std;

Controller * ct; 

/*! \brief Displays the usage of the program. 

  Called when the arguments cannot be processed.
 */ 
void displayUsage(int argc, char** argv){
  cout << "USAGE: " << argv[0] << " [ options ]" << endl << endl;
  cout << "Where [ options ] can be: " << endl ;
  cout << "\t-d \t(for running optional visual debugger)" << endl;
  cout << "\t-f <config_filename>" << endl;
  cout << "\t-c to Calibrate Robot" << endl;
  cout << "\t-m to Build a Motion Model" << endl << endl;
  
  exit(1);
}

/*! \brief Parses the configuration file passed as an argument

  \param input file stream for the config file
  \param string ref Central server hostname
  \param string ref Central server port
  \param string ref Player hostname (for the robot)
  \param string ref Player port (for the robot)
  \param string ref robot label
  \param string ref robot type
  
  This function parses and populates the information for the program to connect to the other components of the system.
 */
void readConfigFile(ifstream& cfFile, string& csHost, int& csPort, string& pHost, int& pPort, string& lab, string& ty, string &srobotconf){
  string cmd, tmp;
  string cmd2;
  string mainConfig; 

  // parse configuration file + attempt to connect the player and central servers
  while( !cfFile.eof() ) 
  {
   	cmd = ""; 
    	cfFile >> cmd ; 
    
    // if the line is commented skip it otherwise process.
   	if (! (( cmd[0] == '/' && cmd[1] == '/' ) || ( cmd == "")) )
		{ 
			if( cmd == "main_config")
	      {
	      	cfFile >> mainConfig;
	      	cout << endl << "Reading Main Config File: " << mainConfig;
	      	ifstream mcFile(mainConfig.c_str());
	      	
	      	while( !mcFile.eof() ) 
  				{
	      		cmd2 = ""; 
    				mcFile >> cmd2 ;

		      	if (! (( cmd2[0] == '/' && cmd2[1] == '/' ) || ( cmd2 == "")) )
					{ 
		      		if ( cmd2 == "central_server" ) 
				     	{
						  	mcFile >> csHost >> csPort ;
						  	cout << "\nHost: " << csHost;
						  	cout << "\nPort: " << csPort;
				      } 
		      		else if ( cmd2 == "map" )
				      {
						  	getline(mcFile, tmp); 
						  	cout << "\nMap: " << tmp;
				      }   
		      		else if ( cmd2 == "botId" )
				      {
						  	//mcFile >> botNames[i] >> botBraille[i];
						  	getline(mcFile, tmp); 
						  	//cout << "\nRobot " << i << ": " <<  botNames[i] << " " << botBraille[i];
						  	//i++;
				      }    	
		      	}
		      	else 
				   {
				   	getline(mcFile, tmp); 
				   }
	      	}
	      	mcFile.close();
	      	cout << "\nReading Main Config File Complete";
	      }      
	      else if ( cmd == "map" )
	      {
				getline(cfFile, tmp);
	      }
	      else if ( cmd == "RobotConfig" )
	      {
	      	cfFile.seekg (1, ios::cur);
	      	getline(cfFile, srobotconf);	
	      }
	      else if ( cmd == "robot" )
	      { 
				cfFile >> lab >> ty >> pHost >> pPort ; 
	      }
	      else if ( cmd == "camera" )
	      {
				getline(cfFile, tmp);
	      }
	      else 
	      {
				cout << "Unknown config command: " << cmd << endl;
				getline(cfFile, tmp); 
	      }
		}
   	else 
   	{
      	getline(cfFile, tmp); 
		}
	} 
  
  cout << "File Read in Successfully\n\n";
}
















/*! \brief Entry point to the program
  
  Creates a connection to the Central Server and player for the robot. It also generates map and other objects required by the controller. If ran with a -d option, displays visual debugger windows.  

  It generates a Controller object ct with the parameters read from the config file.

  \callgraph
 */
int main(int argc, char **argv)
{
  Utils::initRandom();          // srand(time(NULL))

  // usage: <exec> [-d] [-f <config-file>]. to add more flags add it to the end of string followed
  // by a : or :: if the flag doesn't require an argument
  const char* optflags = "f:c::m::"; 
  //bool calibrate = false;
  //bool motionModel = false;
  int ch;

  ifstream configFile;
  string central_server_hostname = "127.0.0.1"; 
  int central_server_port = 6667;
  string  player_hostname = "127.0.0.1"; 
  int player_port = 6665;
  string label = "" , type = ""; 
  string srobotconf;
  
  if ( argc == 1 ) {
    cout << "no config file arguments provided" << endl;
    displayUsage(argc, argv);
    exit(1); 
  }
  else {
    while ( -1 != ( ch = getopt( argc, argv, optflags ))){
      switch ( ch ) {
      case 'c':	
	cout << "Calibration Mode On\n\n";
	//calibrate = true;
	break;
      case 'm':	
	cout << "Motion Model Mode On\n\n";
	//motionModel = true;
	break;
      case 'f': {		
	cout << "Reading configuration file: " << optarg << endl;
	configFile.open(optarg, ios::in); 
	if( !configFile ) {
	  cout << "Can't open configuration file: " << optarg << ". Aborted" << endl; 
	  exit(1);
	}
	readConfigFile(configFile, central_server_hostname, central_server_port, player_hostname, player_port, label, type, srobotconf);
	configFile.close();
	break;
      }
      default:
	displayUsage(argc, argv);
	break;
      }
    }
  }
  
	BlackfinConfig * bc;
  

  	if(srobotconf.c_str() == NULL)
  	{
  		cout << "Blackfin Config file missing from" << optarg << endl << endl;
  		return 0;
  	}
  	else
  	{
  		bc = new BlackfinConfig(srobotconf.c_str());
  	}
  			
  	ct = new Controller(player_hostname, player_port, central_server_hostname, central_server_port, label, type);
	
	
	time_t start, end;
	start = time(NULL);
	end = time(NULL);
	
	const char * starttime = asctime(localtime(&start));
	cout << "\nStart time: " << starttime << endl;
	


	cout << "Calibrating rotational motion..." << endl; 
  	ct->calibrateRotationalMotion(bc); 
  		
  	const char * endtime = asctime(localtime(&end));

	cout << "Start time: " << starttime << endl;
	cout << "End time: " << endtime << endl << endl;

  	cout << "Calibrating linear motion..." << endl; 
  	ct->calibrateLinearMotion(bc);
  		

  

  return 0;
}
