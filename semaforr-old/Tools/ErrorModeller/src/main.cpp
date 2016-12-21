/*! 
 * \mainpage ErrorModeller Documentation
 * \brief ErrorModeller uses overhead camera to sample motion errors of a particular robot. 
 *
 * \author A. Tuna Ozgelen 
 *
 * \date Jan 2012
 *
 * \b Introduction: Program is composed of 3 stages: 
 *              1. Moves a robot with a series of translation and rotation commands. 
 *                 After completion of a move it records the intended move's (x, y, theta), 
 *                 start position (x,y,theta) and end position (x,y,theta) to a temporary file 
 *              2. Calculates the mean and variance for : 
 *                 translation_error
 *                 drift_error (orientation change during translation)
 *                 rotation_left_error
 *                 rotation_right_error
 *              3. Writes or updates the values calculated in stage (2) to <robot_label>.conf file 
 *                 used by the RobotController for dead reckoning
 *
 * \b Compiling: Run makefile from the components root dir
 *
 * \b Usage: ./ErrorModeller -f <config_filename> -s <setup_config_filename>
 *           alternatively use/modify <HRTeam_DIR>/scripts/error_modeller.sh
 *
 * \file main.cpp
 *
 */

#include "definitions.h"
#include "CommunicationManager.h"
#include "Controller.h"
#include "MessageHandler.h"
#include "Utils.h"
#include "boost/asio.hpp"
#include "boost/shared_ptr.hpp"
#include <fstream>
#include "libplayerc++/playerc++.h"
using namespace PlayerCc;

#include <cstdlib>
#include <iostream>
#include <sstream>
#include <unistd.h>
#include <cstring>
using namespace std;

Controller * ct; 
CommunicationManager * cMan; 
Motion * mot; 
PlayerClient * pCli;
MessageHandler * msgH;

/*! \brief Displays the usage of the program. 

  Called when the arguments cannot be processed.
 */ 
void displayUsage(int argc, char** argv){
  cout << "USAGE: " << argv[0] << " -f <config_filename>" << " -s <setup_config_filename>" << endl << endl;
  exit(1);
}

/*! \brief Parses the configuration file passed as an argument

  \param input file stream for the config file
  \param string ref Player hostname (for the robot)
  \param string ref Player port (for the robot)
  \param string ref robot label
  \param string ref robot type
  
  This function parses and populates the information for the program to connect to the other components of the system.
 */
void readConfigFile(ifstream& cfFile, string& pHost, int& pPort, string& lab, string& ty) {
  string cmd, tmp;

  // parse configuration file + attempt to connect the player and central servers
  while( !cfFile.eof() ) {
    cmd = ""; 
    cfFile >> cmd ; 
    
    // if the line is commented skip it otherwise process.
    if (! (( cmd[0] == '/' && cmd[1] == '/' ) || ( cmd == "")) ){ 
      if ( cmd == "map" ){
	getline(cfFile, tmp);
      }
      else if ( cmd == "robot" ){ 
	cfFile >> lab >> ty >> pHost >> pPort ; 
      }
      else if ( cmd == "camera" ){
	getline(cfFile, tmp);
      }
      else {
	cout << "Unknown config command: " << cmd << endl;
	getline(cfFile, tmp); 
      }
    }
    else {
      // ignore the rest of the line.
      getline(cfFile, tmp); 
    }
  } 
}

/*! \brief Parses the server configuration file passed as an argument

  \param input file stream for the config file
  \param string ref Central server hostname
  \param string ref Central server port
  
  This function parses and populates the information for the program to connect to the other components of the system.
 */
void readSetupConfigFile(ifstream& cfFile, string& csHost, int& csPort) {
  string cmd, tmp;

  // parse configuration file + attempt to connect the player and central servers
  while( !cfFile.eof() ) {
    cmd = ""; 
    cfFile >> cmd ; 
    
    // if the line is commented skip it otherwise process.
    if (! (( cmd[0] == '/' && cmd[1] == '/' ) || ( cmd == "")) ){ 
      if ( cmd == "central_server" ) {
	cfFile >> csHost >> csPort ;
      }      
      else if ( cmd == "map" ){
	getline(cfFile, tmp);
      }
      else {
	cout << "Unknown setup configuration command: " << cmd << endl;
	getline(cfFile, tmp); 
      }
    }
    else {
      // ignore the rest of the line.
      getline(cfFile, tmp); 
    }
  } 
}

/*! \brief Entry point to the program
  
  Creates a connection to the Central Server and player for the robot. It also generates map and other objects required by the controller. If ran with a -d option, displays visual debugger windows.  

  It generates a Controller object ct with the parameters read from the config file.

  \callgraph
*/
int main(int argc, char **argv) {
  Utils::initRandom();          // srand(time(NULL))

  // usage: <exec> [-d] [-f <config-file>]. to add more flags add it to the end of string followed
  // by a : or :: if the flag doesn't require an argument
  const char* optflags = "f:s:"; 
  int ch;

  string configFilename, setupFilename; 
  ifstream configFile, setupFile;
  string central_server_hostname = "127.0.0.1"; 
  int central_server_port = 6667;
  string  player_hostname = "127.0.0.1"; 
  int player_port = 6665;
  string label = "" , type = ""; 
  
  if ( argc == 1 ) {
    cout << "no config file arguments provided" << endl;
    displayUsage(argc, argv);
    exit(1); 
  }
  else {
    while ( -1 != ( ch = getopt( argc, argv, optflags ))){
      switch ( ch ) {
      case 'f': {
	configFilename = optarg;
	cout << "configuration file: " << configFilename << endl;
	configFile.open(configFilename.c_str(), ios::in); 
	if( !configFile ) {
	  cout << "Can't open configuration file: " << configFilename << ". Aborted" << endl; 
	  exit(1);
	}
	readConfigFile(configFile, player_hostname, player_port, label, type);
	configFile.close();
	break;
      }
      case 's': {
	setupFilename = optarg;
	cout << "setup configuration file: " << setupFilename << endl;
	setupFile.open(setupFilename.c_str(), ios::in); 
	if( !setupFile ) {
	  cout << "Can't open setup configuration file: " << setupFilename << ". Aborted" << endl; 
	  exit(1);
	}
	readSetupConfigFile(setupFile, central_server_hostname, central_server_port);
	setupFile.close();
	break;
      }
      default:
	displayUsage(argc, argv);
	break;
      }
    }
  }

  ct = new Controller(player_hostname, player_port, central_server_hostname, central_server_port, label, type);
  string setupFileDir = setupFilename.substr(0, setupFilename.substr(0,setupFilename.find_last_of("/")).find_last_of("/") + 1);
  ct->setFileDir(setupFileDir + "MotionSamples/"); 

  cMan = ct->getCommunicationManager(); 
  msgH = new MessageHandler(cMan); 
  boost::thread * msgHThread = new boost::thread(*msgH); 

  char c; 
  do {
  cout << "Place the robot close to the center of the overhead camera view then press 's' to start> " ; 
  cin >> c;
  } while ( c != 's' ); 
  ct->setCenterXY();

  cout << "Starting linear motion sampling..." << endl; 
  ct->sampleLinearMotion();

  cout << "Starting rotational motion sampling..." << endl; 
  ct->sampleRotationalMotion(); 

  cout << "Calculating pdf of translational, rotational and drift error" << endl;
  // read the sample file created by the Controller and distribute the motions into different data structures 
  string fname = ct->getFilename(); 
  ifstream infile(fname.c_str(), ios::in); 
  if ( !infile ) {
    cerr << "Could not open the file: " << fname << endl;
    exit(1);    
  }
  
  // post-process sample file. this is the same sample file minus the entries regardes as noise
  string post_fname = fname.substr(0, fname.find_last_of("-")) + "-post-samples.dat" ; 
  cout << "post_fname: " << post_fname << endl;
  ofstream outfile(post_fname.c_str(), ios::out); 
  if ( !outfile ) {
    cerr << "Could not open the file: " << post_fname << endl; 
    exit(1); 
  }

  vector<double> translation, rotation_left, rotation_right, drift ;
  while ( !infile.eof() ){
    double mx = 0, my = 0, mt = 0, sx = 0, sy = 0, st = 0, ex = 0, ey = 0, et = 0;   // move(x,y,theta), start(x,y,theta), end(x,y,theta)
    
    infile >> mx >> my >> mt >> sx >> sy >> st >> ex >> ey >> et ;
    if ( mx > 0 ) {
      // if noisy theta value due to bad campose, discard 
      if ( abs(et-st) < Utils::toRadians(45) ){
	outfile << mx << " " << my << " " << mt << " " << sx << " " << sy << " " << st << " " << ex << " " << ey << " " << et << endl; 
	translation.push_back(Utils::get_euclidian_distance(sx, sy, ex, ey)/mx);
	drift.push_back(et-st); 
      }
      else { 
	cout << "discarding translation move record due to noisy orientation - start: " << Utils::toDegrees(st) << "\t end: " << Utils::toDegrees(et) << endl ; 
      }
    }
    else if ( mt > 0 ) {
      outfile << mx << " " << my << " " << mt << " " << sx << " " << sy << " " << st << " " << ex << " " << ey << " " << et << endl; 
      rotation_left.push_back(et-st/mt); 
    }
    else if ( mt < 0 ) {
      outfile << mx << " " << my << " " << mt << " " << sx << " " << sy << " " << st << " " << ex << " " << ey << " " << et << endl; 
      rotation_right.push_back(et-st/mt); 
    }
  }
  infile.close();
  outfile.close();

  // calculate the $\sigma_{rot}, \sigma_{trans}, \sigma_{drift}$
  double trans_mean = Utils::getMean(translation); 
  double trans_var = Utils::getVariance(translation, trans_mean); 

  double rot_l_mean = Utils::getMean(rotation_left);
  double rot_l_var = Utils::getVariance(rotation_left, rot_l_mean);

  double rot_r_mean = Utils::getMean(rotation_right);
  double rot_r_var = Utils::getVariance(rotation_right, rot_r_mean);

  double drift_mean = Utils::getMean(drift); 
  double drift_var = Utils::getVariance(drift, drift_mean);

  // write the values into .conf file. 
  ifstream inconf(configFilename.c_str(), ios::in);
  if ( !inconf ) {
    cerr << "Could not open the file: " << configFilename << endl;
    exit(1);    
  }

  string cfname = configFilename.substr(0, configFilename.find_last_of('.'));
  string newConfFile = cfname + "-updated.conf";
  ofstream outconf(newConfFile.c_str(), ios::out); 
  if ( !outconf ) {
    cerr << "Could not open the file: " << newConfFile << endl;
    exit(1);    
  }

  bool update_trans = false, update_drift = false, update_l_rot = false, update_r_rot = false;
  while ( !inconf.eof() ) {
    string cmd = ""; 
    inconf >> cmd; 
    if ( cmd == "translation" ){
      outconf << "translation " << trans_mean << " " << trans_var << endl;       
      update_trans = true; 
      string tmp; 
      getline(inconf, tmp); 
    }
    else if ( cmd == "rotation_left" ){
      outconf << "rotation_left " << rot_l_mean << " " << rot_l_var << endl;  
      update_l_rot = true; 
      string tmp; 
      getline(inconf, tmp); 
    }
    else if ( cmd == "rotation_right" ){
      outconf << "rotation_right " << rot_r_mean << " " << rot_r_var << endl; 
      update_r_rot = true; 
      string tmp; 
      getline(inconf, tmp); 
    }
    else if ( cmd == "drift" ) {
      outconf << "drift " << drift_mean << " " << drift_var << endl; 
      update_drift = true; 
      string tmp; 
      getline(inconf, tmp); 
    }
    else {
      string line; 
      getline(inconf, line); 
      outconf << cmd << line << endl ; 
    }
  }
  if ( !update_trans ) 
    outconf << "translation " << trans_mean << " " << trans_var << endl; 
  if ( !update_drift )
    outconf << "drift " << drift_mean << " " << drift_var << endl; 
  if ( !update_l_rot )
    outconf << "rotation_left " << rot_l_mean << " " << rot_l_var << endl;  
  if ( !update_r_rot )
    outconf << "rotation_right " << rot_r_mean << " " << rot_r_var << endl; 
  
  inconf.close();
  outconf.close();

  // copy contents of robot*-updated.conf to the original robot*.conf file
  ifstream updated(newConfFile.c_str(), fstream::binary);
  ofstream old(configFilename.c_str(), fstream::trunc|fstream::binary);
  old << updated.rdbuf(); 
  updated.close(); 
  old.close(); 

  // remove the temporary robot*-update.conf
  if ( remove(newConfFile.c_str()) != 0 ) {
    cout << "Failed to remove : "  << newConfFile << endl; 
  }

  cout << "DONE" << endl;
  return 0;
  
  msgHThread->join(); 
}
