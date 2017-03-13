#ifndef _DEFINITIONS_H
#define _DEFINITIONS_H

// For sanity's sake
#define BOOST_SIGNALS_NO_DEPRECATION_WARNING

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
#define CMD_WAYPOINTS	"WAYPOINTS"  // Synonym for CMD_REMAINING_PATH

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
#define CMD_CANCEL_TASK		"CANCEL_TASK"     // format CANCEL_TASK <task_id> 
#define CMD_BEGIN_TASKS		"BEGIN_TASKS"
#define CMD_ADD_OBSTACLE	"ADD_OBSTACLE"
#define CMD_REMOVE_OBSTACLE	"REMOVE_OBSTACLE"
#define CMD_STUCK		"STUCK"

// Tuna's stuff
#define CMD_EXECUTE_TASK        "EXECUTE_TASK"    // format: EXECUTE_TASK <task_id> <task_type> <num_robots_required> <num_assigned> 
                                                  //         [ robot_1, ... robot_n ] <duration> <x> <y>
#define CMD_TASK_REACHED        "TASK_REACHED"    // format: TASK_REACHED <robot_id> <task_id>
#define CMD_TASK_STARTED        "TASK_STARTED"    // format: TASK_STARTED <robot_id> <task_id>
#define CMD_TASK_COMPLETE       "TASK_COMPLETE"   // format: TASK_COMPLETE <robot_id> <task_id>
#define CMD_DELAYED             "DELAYED"         // format: DELAYED <robot_id>, sent when robot waits due to obstacle avoidance
#define CMD_YIELD_REASON        "YIELD_REASON"    // format: YIELD_REASON <robot_id> <reason_for_yield>

// End Tuna's stuff


//Tasks 
#define TASK_SEARCHING			"SEARCHING"
#define TASK_PAUSE			"PAUSE"
#define TASK_IDLE			"IDLE"
#define TASK_SWEEP			"SWEEP"

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

// Message destination types
#define DEST_SINGLE         "SINGLE"            // "SINGLE <id>"
#define DEST_MULTI          "MULTI"             // "MULTI <n> <id_1> <id_2> .. <id_n>"
#define DEST_BROADCAST_ALL  "BROADCAST_ALL"     // "BROADCAST"
#define DEST_BROADCAST_TYPE "BROADCAST_TYPE"    // "BROADCAST_TYPE <type>"
#define DEST_BROADCAST_SUBSCRIBERS  "BROADCAST_SUBSCRIBERS" // BROADCAST_SUBSCRIBERS

//messages requesting for descriptives
#define CMD_ASK_TARGET_POINT_DESCRIPTIVE            "ASK_TARGET_POINT_DESCRIPTIVE"            // Request message asking for all target points
#define CMD_ASK_DISTANCE_TO_TARGET_DESCRIPTIVE      "ASK_DISTANCE_TO_TARGET_DESCRIPTIVE"      // asking for distance from its immediate future position to the target point in pursuit
#define CMD_ASK_DISTANCE_FROM_OBSTACLES_DESCRIPTIVE "ASK_DISTANCE_FROM_OBSTACLES_DESCRIPTIVE" // ask for distance from the obstacles currently walls with buffer
#define CMD_ASK_LAST_ACTION_DESCRIPTIVE             "ASK_LAST_ACTION_DESCRIPTIVE"             // ask for the last action performed by the robot
#define CMD_ASK_ADVISOR_WEIGHT_DESCRIPTIVE          "ASK_ADVISOR_WEIGHT_DESCRIPTIVE"          // ask for advisor weights
#define CMD_ASK_TEAM_POSE_DESCRIPTIVE               "ASK_TEAM_POSE_DESCRIPITVE"               // ask for team pose
#define CMD_ASK_ADVISOR_DATA_DESCRIPTIVE            "ASK_ADVISOR_DATA_DESCRIPTIVE"            // ask for data to instantiate advisors
#define CMD_ASK_VETO_FORWARD_MOVES_DATA_DESCRIPTIVE "ASK_VETO_FORWARD_MOVES_DATA_DESCRIPTIVE" // ask for data to veto forward moves

//messages containing descriptives
#define CMD_TARGET_POINT_DESCRIPTIVE            "TARGET_POINT_DESCRIPTIVE"            // message containing all target points
#define CMD_DISTANCE_TO_TARGET_DESCRIPTIVE      "DISTANCE_TO_TARGET_DESCRIPTIVE"      // distance from its immediate future position to the target point in pursuit
#define CMD_DISTANCE_FROM_OBSTACLES_DESCRIPTIVE "DISTANCE_FROM_OBSTACLES_DESCRIPTIVE" // distance from the obstacles currently walls with buffer
#define CMD_LAST_ACTION_DESCRIPTIVE             "LAST_ACTION_DESCRIPTIVE"             // the last action performed by the robot

#define CMD_LAST_ACTION                         "LAST_ACTION"
#define CMD_ADVISOR_WEIGHT_DESCRIPTIVE          "ADVISOR_WEIGHT_DESCRIPTIVE"          // ask for advisor weights
#define CMD_TEAM_POSE_DESCRIPTIVE               "TEAM_POSE_DESCRIPTIVE"               // gets team pose message
#define CMD_ADVISOR_DATA_DESCRIPTIVE            "ADVISOR_DATA_DESCRIPTIVE"            // message containing data for advisor instantiation 
#define CMD_VETO_FORWARD_MOVES_DATA_DESCRIPTIVE "VETO_FORWARD_MOVES_DATA_DESCRIPTIVE" // data to veto forward moves

enum TASK_TYPE { SENSOR_SWEEP, FIRE, DEBRIS };
enum TASK_STATUS { ENROUTE, READY, IN_PROGRESS, COMPLETE };

#endif
