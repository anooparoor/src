#ifndef _DEFINITIONS_H
#define _DEFINITIONS_H

// Command Preamble Type
typedef unsigned char cmd_len_t;


// Client/Server Commands
#define CMD_ERROR	   "ERROR"       // <--> ERROR <message>
#define CMD_WAIT	   "WAIT"        // <--> WAIT (maybe not used -- use ERROR instead?)
#define CMD_QUIT       "QUIT"        // <--> QUIT
#define CMD_MOVE       "MOVE"        // <--- MOVE <id> <x-vel> <y-vel> <a-vel>
#define CMD_MOVING     "MOVING"      // ---> MOVING
#define CMD_FOUND      "BROADCAST"   // ---> BROADCAST FOUND <color>
#define CMD_STATE      "STATE"       // <--- STATE <id> (currently replaced by ASKPOSE?)
#define CMD_ASKPOSE    "ASKPOSE"     // <--- ASKPOSE <id>
#define CMD_POSE       "POSE"        // ---> POSE <x-pos> <y-pos> <a-pos> <confidence>
#define CMD_ASKPLAYER  "ASKPLAYER"   // <--- ASKPLAYER <id>
#define CMD_PLAYER     "PLAYER"      // ---> PLAYER <port> <ip>
#define CMD_LOCK       "LOCK"        // <--- LOCK
#define CMD_UNLOCK     "UNLOCK"      // <--- UNLOCK
#define CMD_SNAP       "SNAP"        // <--- SNAP <id> (request an image)
#define CMD_IMAGE      "IMAGE"       // ---> IMAGE <image-data>
#define CMD_IDENT      "IDENT"       // IDENT <num-robots> [<robot_id> <name> <type> <num-provides> <provides>]
#define CMD_GOTO       "GOTO"        // GOTO Map-X Map-Y
#define CMD_CAMPOSE    "CAMPOSE"     // CAMPOSE <robot_id> <map-x> <map-y> <orientation>
#define CMD_AUCTION_START 		"AUCTION_START" // AUCTION_START <auc_id><type of task> <x-coord> <y-coord> <sensor> 
#define CMD_AUCTION_BID  		"AUCTION_BID" // AUCTION_BID <auc_id> <bid>
#define CMD_AUCTION_ASKBID     		"AUCTION_ASKBID"	// Gets all the bids skygrid has 
#define CMD_AUCTION_WON			"AUCTION_WON"	//Send the robot to an area to perform a task ---> WON <id> <Auc_id>
#define CMD_AUCTION_SEARCH 		"AUCTION_SEARCH"  //Gives robots the OK to beginning searching
#define CMD_AUCTION_COMPLETE   		"AUCTION_COMPLETE"	//Robot sends back this command when a point has been finished
#define CMD_AUCTION_FINISHED		"AUCTION_FINISHED"      //Robot sends back this command when the whole arena has been finished
#define CMD_AUCTION_ASKSTATUS  		"AUCTION_ASKSTATUS"  //Return searching if the robot is still alive or dead otherwise
#define CMD_AUCTION_STATUS		"AUCTION_STATUS"  //Same as robot_finished 
#define CMD_AUCTION_RESUME		"AUCTION_RESUME"  //Resume all work
#define CMD_AUCTION_PAUSE		"AUCTION_PAUSE"   //Pause all work
#define CMD_AUCTION_STOP 		"AUCTION_STOP"    //Stop all work
#define CMD_AUCTION_PRINT               "AUCTION_PRINT"   //Print the list of points to be searched

//Generic broadcast
#define CMD_EXECUTE_ENTITIES            "EXECUTE_ENTITIES"  //Broadcast to all entities including non robots
#define CMD_EXECUTE_ROBOTS		"EXECUTE_ROBOTS"    //Boradcast only to robots    
//format CMD_EXECUTE_ROBOTS CMD_...(ANYCOMMAND) -1 ..... (ANY MESSAGE)  //-1 broadcast, or id to communicate with that robot
#define CMD_UPDATE_TEAMMATE		"UPDATE_TEAMMATE" //Update neigbors information to other robots
#define CMD_TEAMMATE_PATH_ALTERATION_COST    "PATH_ALTERATION_COST"
#define CMD_WAITING_FOR_TEAMMATE             "WAITING_FOR_TEAMMATE" // WAITING_FOR_TEAMMATE r1 r2    ( r2 is waiting on r1 directly or indirectly )
//format for remaining path info
#define CMD_REMAINING_PATH              "WAYPOINTS" // WAYPOINTS robot_session_id num_waypoints waypoint_1 .. waypoint_n target

// FORR commands
#define CMD_HHALT   "HHALT"
#define CMD_HWAIT   "HWAIT"
#define CMD_HRESUME "HRESUME"

#define CMD_MOVE_START  "MOVE_START"

//FUNCTIONS TO BROADCAST EVENTS ->FOR OCTOBER PAPER
//The robot is pause everytime there's a robot inside my inner circle
//The robot is resume evertyime there are no robots in my outter circle
//There's a buffer between the inner circle and the outter circle to prevent multiple stops for the robot
//This is broadcast to keep it in the log file of the Central Server to analyze the data
//#define CMD_TASK_PAUSE			"TASK_PAUSE"	//This is broadcast everytime the robot is pause
#define TASK_RESUME			"RESUME"	//This is broadcast everytime the robot is resume

// Sumon's stuff
#define CMD_ASK_TASK_ORDER	"ASK_TASK_ORDER"
#define CMD_TASK_ORDER 		"TASK_ORDER"
#define CMD_SET_TASK_ORDER 	"SET_TASK_ORDER"
#define CMD_CANCEL_TASK		"CANCEL_TASK"
#define CMD_BEGIN_TASKS		"BEGIN_TASKS"
#define CMD_ADD_OBSTACLE	"ADD_OBSTACLE"
#define CMD_REMOVE_OBSTACLE	"REMOVE_OBSTACLE"
#define CMD_STUCK		"STUCK"


//Tasks 
#define TASK_SEARCHING			"SEARCHING"
#define TASK_PAUSE			"PAUSE"
#define TASK_IDLE			"IDLE"
#define TASK_SWEEP			"SWEEP"

// Client/Server States
//#define STATE_INIT           0
//#define STATE_ACK            1
//#define STATE_IDLE           2
//#define STATE_CMD_PROC       3
//#define STATE_CMD_BAD        4
//#define STATE_PING_SEND      5
//#define STATE_PING_READ      6
//#define STATE_PONG_SEND      7
//#define STATE_PONG_READ      8
//#define STATE_QUIT           9
#define STATE_ERROR         10
#define STATE_MOVING        11
#define STATE_POSE          12
#define STATE_PLAYER        13
#define STATE_GUI_WAIT      14
#define STATE_FOUND         15
#define STATE_GOTO          16
//AUCTION
#define STATE_AUCTION_BID	    17
#define STATE_AUCTION_WON   	    18
#define STATE_AUCTION_SEARCH        19
#define STATE_AUCTION_COMPLETE	    20
#define STATE_AUCTION_FINISHED      21
#define STATE_AUCTION_STATUS        22
#define STATE_AUCTION_RESUME	    23
#define STATE_AUCTION_PAUSE	    24
#define STATE_AUCTION_STOP 	    25
#define STATE_AUCTION_PRINT	    26
//OVERHEAD CAMERA
#define STATE_CAMPOSE       	    27
//BROADCAST
#define STATE_EXECUTE_ENTITIES		28
#define STATE_EXECUTE_ROBOTS		29
#define STATE_UPDATE_TEAMMATE		30
#define STATE_TEAMMATE_AUCTION_FINISHED	31
#define STATE_TEAMMATE_ALTERATION_COST  32

// NEW 
#define STATE_TEAMMATE_WAITING_FOR      33

// FORR command states
#define STATE_HHALT     34
#define STATE_HWAIT     35
#define STATE_HRESUME   36
// For sending plan updates
#define STATE_PLAN_UPDATED  37

#define STATE_MOVE_START  38

// Sumon's stuff
#define STATE_TASK_ORDER	39
#define STATE_SET_TASK_ORDER	40
#define STATE_CANCEL_TASK	41
#define STATE_BEGIN_TASKS	42
#define STATE_ADD_OBSTACLE	43
#define STATE_REMOVE_OBSTACLE	44
#define STATE_STUCK		45

// Unique Client Names
#define UID_SURVEYOR_SRV10    "srv10"
#define UID_AIBO_GROWL        "growl"
#define UID_AIBO_BETSY        "betsy"
#define UID_GUI_PABLO         "pablo"


// Client Species Types
#define SID_AIBO         "aibo"
#define SID_SURVEYOR     "surveyor"
#define SID_SCRIBBLER    "scribbler"
#define SID_NXT          "nxt"
#define SID_GUI          "gui"
#define SID_COORDINATOR  "coordinator"

// Client Capabilities (Provides)
#define CAPS_POSITION2D  "position2d"
#define CAPS_CAMERA      "camera"
#define CAPS_METROCAM    "metrocam"
#define CAPS_BLOBFINDER  "blobfinder"
#define CAPS_RANGER      "ranger"
#define CAPS_SONAR       "sonar"

// Controller States
#define CTRL_HALT 0
#define CTRL_READY 1
enum Mode{ MANUAL, MIXED_INIT, AUTO };  // Modes of operation: manual, mixed initiative and auto control
#define KEY_CTRL_STEP 0          // sets the keyboard control commands the robot to move in discrete steps 
#define KEY_CTRL_CONT 1          // sets the keyboard control commands the robot to move in continuous mode


// Message source types
//#define SOURCE_SINGLE       "SOURCE_SINGLE"     // Message was sent to a single recipient
//#define SOURCE_MULTI        "SOURCE_MULTI"      // Message was sent to multiple recipients

// Message destination types
#define DEST_SINGLE         "SINGLE"            // "SINGLE <id>"
#define DEST_MULTI          "MULTI"             // "MULTI <n> <id_1> <id_2> .. <id_n>"
#define DEST_BROADCAST_ALL  "BROADCAST_ALL"     // "BROADCAST"
#define DEST_BROADCAST_TYPE "BROADCAST_TYPE"    // "BROADCAST_TYPE <type>"
#define DEST_BROADCAST_SUBSCRIBERS  "BROADCAST_SUBSCRIBERS" // BROADCAST_SUBSCRIBERS


#endif
