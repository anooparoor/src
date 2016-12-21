/*
	These definitions are 
 */

#ifndef _DEFINITIONS_H
#define _DEFINITIONS_H

// Command Preamble Type
typedef unsigned char cmd_len_t;


// Client/Server Commands

/*
#define CMD_ERROR	   "ERROR"       // <--> ERROR <message>
#define CMD_INIT	   "INIT"        // ---> INIT <type> <name> <num-provides> [<provides-list>]
#define CMD_ACK		   "ACK"         // <--- ACK <id>
#define CMD_PING	   "PING"        // <--> PING
#define CMD_PONG	   "PONG"        // <--> PONG
*/
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
//format for remaining path info
#define CMD_WAYPOINTS              "WAYPOINTS" // WAYPOINTS robot_session_id num_waypoints waypoint_1 .. waypoint_n target
#define CMD_ASK_WAYPOINTS				"ASK_WAYPOINTS" // ASK_WAYPOINTS <id>

//messages requesting for descriptivse

#define CMD_ASK_TARGET_POINT_DESCRIPTIVE            "ASK_TARGET_POINT_DESCRIPTIVE"             //Request message asking for all target points
#define CMD_ASK_DISTANCE_TO_TARGET_DESCRIPTIVE      "ASK_DISTANCE_TO_TARGET_DESCRIPTIVE"       //asking for distance from its immediate future position to the target point in pursuit
#define CMD_ASK_DISTANCE_FROM_OBSTACLES_DESCRIPTIVE "ASK_DISTANCE_FROM_OBSTACLES_DESCRIPTIVE"  //ask for distance from the obstacles currently walls with buffer
#define CMD_ASK_LAST_ACTION_DESCRIPTIVE             "ASK_LAST_ACTION_DESCRIPTIVE"              //ask for the last action performed by the robot
#define CMD_ASK_ADVISOR_WEIGHT_DESCRIPTIVE          "ASK_ADVISOR_WEIGHT_DESCRIPTIVE"           // learned weights of the advisors
#define CMD_ASK_TEAM_POSE_DESCRIPTIVE               "ASK_TEAM_POSE_DESCRIPTIVE"                // position of all robots
#define CMD_ASK_ADVISOR_DATA_DESCRIPTIVE            "ASK_ADVISOR_DATA_DESCRIPTIVE"             // data needed to instantiate advisors
#define CMD_ASK_VETO_FORWARD_MOVES_DATA_DESCRIPTIVE "ASK_VETO_FORWARD_MOVES_DATA_DESCRIPTIVE"  // ask for data needed to veto forward moves

//messages containing descriptives
#define CMD_CURRENT_TARGET_DESCRIPTIVE        "CURRENT_TARGET_DESCRIPTIVE"            // message containing current target that the robot is pursuing
#define CMD_TARGET_POINT_DESCRIPTIVE            "TARGET_POINT_DESCRIPTIVE"            //message containing all target points
#define CMD_DISTANCE_TO_TARGET_DESCRIPTIVE      "DISTANCE_TO_TARGET_DESCRIPTIVE"      //distance from its immediate future position to the target point in pursuit
#define CMD_DISTANCE_FROM_OBSTACLES_DESCRIPTIVE     "DISTANCE_FROM_OBSTACLES_DESCRIPTIVE"     //distance from the obstacles currently walls with buffer
#define CMD_LAST_ACTION_DESCRIPTIVE                 "LAST_ACTION_DESCRIPTIVE"                 // the last action performed by the robot
#define CMD_ADVISOR_WEIGHT_DESCRIPTIVE              "ADVISOR_WEIGHT_DESCRIPTIVE"              // learned weights of the advisors
#define CMD_TEAM_POSE_DESCRIPTIVE                   "TEAM_POSE_DESCRIPTIVE"                   // position of all robots
#define CMD_ADVISOR_DATA_DESCRIPTIVE                "ADVISOR_DATA_DESCRIPTIVE"                // data to instantiate advisors
#define CMD_VETO_FORWARD_MOVES_DATA_DESCRIPTIVE "VETO_FORWARD_MOVES_DATA_DESCRIPTIVE" // data to veto forward moves


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



//Tasks 
#define TASK_SEARCHING			"SEARCHING"
#define TASK_PAUSE			"PAUSE"
#define TASK_IDLE			"IDLE"
#define TASK_SWEEP			"SWEEP"

/*
// Client/Server States
#define STATE_INIT           0
#define STATE_ACK            1
#define STATE_IDLE           2
#define STATE_CMD_PROC       3
#define STATE_CMD_BAD        4
#define STATE_PING_SEND      5
#define STATE_PING_READ      6
#define STATE_PONG_SEND      7
#define STATE_PONG_READ      8
#define STATE_QUIT           9
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
// FORR command states
#define STATE_HHALT     33
#define STATE_HWAIT     34
#define STATE_HRESUME   35
// For sending plan updates
#define STATE_PLAN_UPDATED  36

#define STATE_MOVE_START  37

#define STATE_SEND_IDENT	38
#define STATE_PROC_IDENT	39

#define STATE_SEND_PENDING	40

#define STATE_PROC_WAYPOINTS	41
*/

// Unique Client Names
#define UID_SURVEYOR_SRV10    "srv10"
#define UID_AIBO_GROWL        "growl"
#define UID_AIBO_BETSY        "betsy"
#define UID_GUI_PABLO         "pablo"

#define UID_DMANAGER          "descriptive_manager"

// Client Species Types
#define SID_AIBO         "aibo"
#define SID_SURVEYOR     "surveyor"
#define SID_BLACKFIN     "Blackfin"
#define SID_SCRIBBLER    "scribbler"
#define SID_NXT          "nxt"
#define SID_GUI          "gui"
#define SID_COORDINATOR  "coordinator"
#define SID_OHCAMERA     "ohcamera"
#define SID_DESCRIPTIVE_MANAGER     "descriptive_manager"

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

#endif
