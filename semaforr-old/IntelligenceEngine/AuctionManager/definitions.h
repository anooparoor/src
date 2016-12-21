#ifndef _DEFINITIONS_H
#define _DEFINITIONS_H

// Command Preamble Type
typedef unsigned char cmd_len_t;


// Client/Server Commands
#define CMD_ERROR	 "ERROR"   // <--> ERROR <message>
#define CMD_INIT	 "INIT"    // ---> INIT <type> <name> <num-provides> [<provides-list>]
#define CMD_ACK		 "ACK"     // <--- ACK <id>
#define CMD_PING	 "PING"    // <--> PING
#define CMD_PONG	 "PONG"    // <--> PONG
#define CMD_WAIT	 "WAIT"    // <--> WAIT (maybe not used -- use ERROR instead?)
#define CMD_QUIT     "QUIT"    // <--> QUIT
#define CMD_MOVE     "MOVE"    // <--- MOVE <id> <x-vel> <y-vel> <a-vel>
#define CMD_MOVING   "MOVING"  // ---> MOVING
#define CMD_GOTO     "GOTO"    // <--- GOTO <id> <xpos> <ypos>
#define CMD_BROADCAST "BROADCAST" //
#define CMD_STATE    "STATE"   // <--- STATE <id> (currently replaced by ASKPOSE?)
#define CMD_ASKPOSE  "ASKPOSE" // <--- ASKPOSE <id>
#define CMD_POSE     "POSE"    // ---> POSE <x-pos> <y-pos> <a-pos> <confidence>
#define CMD_LOCK     "LOCK"    // <--- LOCK
#define CMD_UNLOCK   "UNLOCK"  // <--- UNLOCK
#define CMD_SNAP     "SNAP"    // <--- SNAP <id> (request an image)
#define CMD_IMAGE    "IMAGE"   // ---> IMAGE <image-data>
#define CMD_IDENT    "IDENT"   // IDENT <num-robots> [ <robot_id> <name> <type> <num-provides> <provides>]
#define CMD_REGISTER "REGISTER" //
#define CMD_UNREGISTER "REGISTER" //
#define CMD_ASK_POSE "ASKPOSE" //
#define CMD_GET_POSE "POSE" //
#define CMD_ASK_PLAYER "ASKPLAYER" //
#define CMD_GET_PLAYER "PLAYER" //
#define CMD_CAMPOSE "CAMPOSE"   // CAMPOSE <id> <xpos> <ypos> <orientation> 

#define CMD_AUCTION_START 		"AUCTION_START" // AUCTION_START <auc_id><type of task> <x-coord> <y-coord> <sensor> 
#define CMD_AUCTION_BID  		"AUCTION_BID" // AUCTION_BID <auc_id> <bid>
#define CMD_AUCTION_ALLBID		"AUCTION_ALLBID" //Gets the id off all the robots
#define CMD_AUCTION_ASKBID     		"AUCTION_ASKBID"	// Gets all the bids skygrid has 
#define CMD_AUCTION_WON			"AUCTION_WON"	//Send the robot to an area to perform a task ---> WON <id> <Auc_id>
#define CMD_AUCTION_SEARCH 		"AUCTION_SEARCH"  //Gives robots the OK to beginning searching
#define CMD_AUCTION_COMPLETE   		"AUCTION_COMPLETE"	//Robot replies after every point finished
#define CMD_AUCTION_FINISHED            "AUCTION_FINISHED"      //Robot replies after their auctions are completed
#define CMD_AUCTION_ASKSTATUS  		"AUCTION_ASKSTATUS"  //Return searching if the robot is still alive or dead otherwise
#define CMD_AUCTION_STATUS		"AUCTION_STATUS"  //Same as robot_finished 
#define CMD_AUCTION_RESUME		"AUCTION_RESUME"  //Resume all work
#define CMD_AUCTION_PAUSE		"AUCTION_PAUSE"   //Pause all work
#define CMD_AUCTION_STOP 		"AUCTION_STOP"    //Stop all work
#define CMD_AUCTION_PRINT               "AUCTION_PRINT"   //Print the list of points to be searched

//Receive message to keep information in the log file -> For the paper in October
//#define CMD_TASK_PAUSE			"TASK_PAUSE"	//This is broadcast everytime the robot is pause
//#define CMD_TASK_RESUME			"TASK_RESUME"	//This is broadcast everytime the robot is resume

//Generic broadcast
#define CMD_EXECUTE_ENTITIES            "EXECUTE_ENTITIES"  //Broadcast to all entities including non robots
#define CMD_EXECUTE_ROBOTS		"EXECUTE_ROBOTS"    //Boradcast only to robots    
//format CMD_EXECUTE_ROBOTS CMD_...(ANYCOMMAND) -1 ..... (ANY MESSAGE)  //-1 broadcast, or id to communicate with that robot

// Human interface 'halt' command
#define CMD_HHALT   "HHALT"     // HHALT <id>

// Human interface 'wait' command
#define CMD_HWAIT   "HWAIT"     // HWAIT <id>

// Human interface 'resume' command
#define CMD_HRESUME "HRESUME"   // HRESUME <id>

// A robot comunicates the waypoints in its current plan
#define CMD_WAYPOINTS   "WAYPOINTS" // WAYPOINTS <id> n x_0 y_0 x_1 y_1 ... x_n y_n

#define CMD_MOVE_START	"MOVE_START"

// Client/Server States
#define STATE_INIT      0
#define STATE_ACK       1
#define STATE_PING      2
#define STATE_PONG      3
#define STATE_QUIT      4
#define STATE_ERROR     5
#define STATE_BAD_CMD   6
#define STATE_MOVING    7
#define STATE_GUI_WAIT  8

#define STATE_IDLE	9
#define STATE_IDENT	10
#define STATE_LOCK	11
#define STATE_UNLOCK	12
#define STATE_REGISTER	13
#define STATE_UNREGISTER 14

#define STATE_PROC_CMD	15
#define STATE_MOVE	16

#define STATE_ASK_POSE	17
#define STATE_GET_POSE	18

#define STATE_ASK_PLAYER	19
#define STATE_GET_PLAYER	20
#define STATE_BROADCAST 21
#define STATE_GOTO      22

//AUCTION
#define STATE_AUCTION_INIT		23
#define STATE_AUCTION_BID		24
#define STATE_AUCTION_COLLECT_BID  	25
#define STATE_AUCTION_WON 		26
#define STATE_AUCTION_SEARCH   		27
#define STATE_AUCTION_COMPLETE 		28
#define STATE_AUCTION_FINISHED		29
#define STATE_AUCTION_ASKSTATUS  	30
#define STATE_AUCTION_STATUS 		31
#define STATE_AUCTION_RESUME	    32
#define STATE_AUCTION_PAUSE	    33
#define STATE_AUCTION_STOP 	    34
#define STATE_AUCTION_PRINT	    35

//CAMPOSE
#define STATE_CAMPOSE   36

//BROADCAST
#define STATE_EXECUTE_ENTITIES		37
#define STATE_EXECUTE_ROBOTS		38

#define STATE_HHALT     39
#define STATE_HWAIT     40
#define STATE_HRESUME   41

#define STATE_WAYPOINTS 42

#define STATE_MOVE_START 43

// Unique Client Names
#define UID_AIBO_GROWL   "growl"
#define UID_AIBO_BETSY   "betsy"
#define UID_GUI_PABLO    "pablo"
#define UID_OHCAMERA     "overcamera"
#define UID_COORDINATOR   "auctioneer"

// Client Species Types
#define SID_AIBO         "aibo"
#define SID_SURVEYOR     "surveyor"
#define SID_SCRIBBLER    "scribbler"
#define SID_NXT          "nxt"
#define SID_GUI          "gui"
#define SID_OHCAMERA     "ohcamera"
#define SID_COORDINATOR   "coordinator"


// Client Capabilities (Provides)
#define CAPS_POSITION2D  "position2d"
#define CAPS_CAMERA      "camera"
#define CAPS_METROCAM    "metrocam"
#define CAPS_BLOBFINDER  "blobfinder"
#define CAPS_RANGER      "ranger"
#define CAPS_SONAR       "sonar"

#endif
