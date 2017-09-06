/* \mainpage ROSiffied semaforr Documentation
 * \brief RobotDriver governs low-level actions of a robot.
 *
 * \author Anoop Aroor.
 *
 * \version SEMAFORR ROS 1.0
 *
 *
 */

#include <iostream>
#include <stdlib.h>
#include <cmath>
#include <sys/time.h>

#include "Controller.h"
#include "FORRAction.h"
#include "Visualizer.h"

#include <ros/package.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h> 
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_datatypes.h>
#include <semaforr/CrowdModel.h>


using namespace std;

// Main interface between SemaFORR and ROS, all ROS related information should be in RobotDriver
class RobotDriver
{
private:
  //! The node handle we'll be using
  ros::NodeHandle nh_;
  //! We will be publishing to the "cmd_vel" topic to issue commands
  ros::Publisher cmd_vel_pub_;
  //! We will be listening to \pose, \laserscan and \crowd_model topics
  ros::Subscriber sub_pose_;
  ros::Subscriber sub_laser_;
  ros::Subscriber sub_crowd_model_;
  // Current position and previous stopping position of the robot
  Position current, previous;
  // Current and previous laser scan
  sensor_msgs::LaserScan laserscan;
  // Current crowd_model
  semaforr::CrowdModel crowdModel;
  // Controller
  Controller *controller;
  // Pos received
  bool init_pos_received;
  // Visualization 
  Visualizer *viz_;
public:
  //! ROS node initialization
  RobotDriver(ros::NodeHandle &nh, Controller *con)
  {
    nh_ = nh;
    //set up the publisher for the cmd_vel topic
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    sub_pose_ = nh_.subscribe("pose", 1000, &RobotDriver::updatePose, this);
    sub_laser_ = nh_.subscribe("base_scan", 1000, &RobotDriver::updateLaserScan, this);
    sub_crowd_model_ = nh_.subscribe("crowd_model", 1000, &RobotDriver::updateCrowdModel, this);
    //declare and create a controller with task, action and advisor configuration
    controller = con;
    init_pos_received = false;
    current.setX(0);current.setY(0);current.setTheta(0);
    previous.setX(0);previous.setY(0);previous.setTheta(0);
    viz_ = new Visualizer(&nh_, con);
  }

  // Callback function for pose message
  void updateCrowdModel(const semaforr::CrowdModel & crowd_model){
	ROS_DEBUG("Inside callback for crowd model");
        cout << crowd_model.height << " " << crowd_model.width << endl;
	//update the crowd model of the belief
	controller->getPlanner()->setCrowdModel(crowd_model);
  }

  // Callback function for pose message
  void updatePose(const geometry_msgs::PoseStamped & pose){
	//ROS_DEBUG("Inside callback for position message");
        double x = pose.pose.position.x;
  	double y = pose.pose.position.y;
	tf::Quaternion q(pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
        Position currentPose(x,y,yaw);
	if(init_pos_received == false){
		//ROS_DEBUG("First Pose message");
		init_pos_received = true;
		previous = currentPose;
	}
        current = currentPose;
	//ROS_INFO_STREAM("Recieved pose message from menge: " << x << " " << y << " " << yaw << endl);
  }

   // Callback function for laser_scan message
   void updateLaserScan(const sensor_msgs::LaserScan & scan){ 
	//ROS_DEBUG("Inside callback for base_scan message");
	laserscan = scan; 
	//ROS_INFO_STREAM("Recieved base_scan message from menge ");
   }
 
  //Collect initial sensor data from robot
  void initialize(){
	ros::spinOnce();
        previous = current;
  }

  //Call semaforr and execute decisions, until mission is successful
  void run(){ 
     //ROS_DEBUG("main::run()");  	
     //Declares the message to be sent
     geometry_msgs::Twist base_cmd;

     ros::Rate rate(30.0);
     double epsilon_move = 0.5; //Meters
     double epsilon_turn = 0.1; //Radians
     bool action_complete = true;
     bool mission_complete = false;
     FORRAction semaforr_action;
     double overallTimeSec=0.0, computationTimeSec=0.0;
     timeval tv, cv;
     double start_time, start_timecv;
     double end_time, end_timecv;
     gettimeofday(&tv,NULL);
     start_time = tv.tv_sec + (tv.tv_usec/1000000.0);

     // Run the loop , the input sensing and the output beaming is asynchrounous
     while(nh_.ok()) {
	  // If pos value is not received from menge wait
	  while(init_pos_received == false){
		ROS_DEBUG("Waiting for first message");
		//wait for some time
          	rate.sleep();
          	// Sense input 
          	ros::spinOnce();	
	  }
	  gettimeofday(&tv,NULL);
          end_time = tv.tv_sec + (tv.tv_usec/1000000.0);
          overallTimeSec = (end_time-start_time);
          //Sense the input and the current target to run the advisors and generate a decision
          if(action_complete){
		ROS_INFO_STREAM("Action completed. Save sensor info, Current position: " << current.getX() << " " << current.getY() << " " << current.getTheta());
		viz_->publishLog(semaforr_action, overallTimeSec, computationTimeSec);
                gettimeofday(&cv,NULL);
                start_timecv = cv.tv_sec + (cv.tv_usec/1000000.0);
		controller->updateState(current, laserscan);
		viz_->publish();
		previous = current;
		ROS_DEBUG("Check if mission is complete");
		mission_complete = controller->isMissionComplete();
		if(mission_complete){
			ROS_INFO("Mission completed");
			gettimeofday(&cv,NULL);
                        end_timecv = cv.tv_sec + (cv.tv_usec/1000000.0);
                        computationTimeSec = (end_timecv-start_timecv);
			viz_->publishLog(semaforr_action, overallTimeSec, computationTimeSec);
			break;
		}
		else{
			ROS_INFO("Mission still in progress, invoke semaforr");
       			semaforr_action = controller->decide();
			ROS_INFO_STREAM("SemaFORRdecision is " << semaforr_action.type << " " << semaforr_action.parameter); 
  			base_cmd = convert_to_vel(semaforr_action);
			action_complete = false;
		}
		gettimeofday(&cv,NULL);
                end_timecv = cv.tv_sec + (cv.tv_usec/1000000.0);
                computationTimeSec = (end_timecv-start_timecv);
          }
          //send the drive command 
          cmd_vel_pub_.publish(base_cmd);

	  //wait for some time
          rate.sleep();
          // Sense input 
          ros::spinOnce();

	  // Check if the action is complete
       	  action_complete = testActionCompletion(semaforr_action, current, previous, epsilon_move, epsilon_turn);
     }
  }


  //! Drive the robot according to the semaforr action, episilon = 0 to 1 indicating percent of task completion
  // Need to improve this
  bool testActionCompletion(FORRAction action, Position current, Position previous, double epsilon_move, double epsilon_rotate)
  {
	  //ROS_DEBUG("Testing if action has been completed by the robot");
	  //ROS_DEBUG_STREAM("Current position " << current.getX() << " " << current.getY() << " " << current.getTheta()); 
	  //ROS_DEBUG_STREAM("Previous position " << previous.getX() << " " << previous.getY() << " " << previous.getTheta());
	  bool actionComplete = false;
	  //ROS_DEBUG_STREAM("Position expected " << expected.getX() << " " << expected.getY() << " " << expected.getTheta());
	  if (action.type == FORWARD){
 	  	double distance_travelled = previous.getDistance(current);
		double expected_travel = controller->getBeliefs()->getAgentState()->getMovement(action.parameter);
	  	if(abs(distance_travelled - expected_travel) < epsilon_move){
			 actionComplete = true;
		}
	  	else{  
			actionComplete = false;  
		}
	  }
	  else if(action.type == RIGHT_TURN or action.type == LEFT_TURN){
		double turn_completed = current.getTheta() - previous.getTheta();
		
		if(turn_completed > 3.1416)
      			turn_completed = turn_completed - 6.283;
    		if(turn_completed < -3.1416)
      			turn_completed = turn_completed + 6.283;

		double turn_expected = controller->getBeliefs()->getAgentState()->getRotation(action.parameter);
		if(action.type == RIGHT_TURN) 
			turn_expected = (-1)*turn_expected;
		
		if(abs(turn_completed - turn_expected) < epsilon_rotate) 
			actionComplete = true;
		else                                   
			actionComplete = false;
	  }
   	  else if(action.type == PAUSE){
		actionComplete = true;
	  }
	  //ROS_DEBUG_STREAM(" Action Completed ? : " << actionComplete);
  	  return actionComplete;
  }

  
  // Function that converts FORRAction into ROS compatible base_cmd

  geometry_msgs::Twist convert_to_vel(FORRAction action){

    geometry_msgs::Twist base_cmd;

    base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0; 
    if(action.type == FORWARD){
    	base_cmd.linear.x = 0.5;
    }   
    else if(action.type == RIGHT_TURN){
 	base_cmd.linear.x = 0.01;
	base_cmd.angular.z = -0.05;
    }
    else if(action.type == LEFT_TURN){
	base_cmd.linear.x = 0.01;
        base_cmd.angular.z = 0.05;
    }
    else if(action.type == PAUSE){
        base_cmd.linear.x = 0;
    }
    return base_cmd;
  }

};






// Main file : Load configuration files and create a controller, Initialize and run robot driver! 
// Stops when the mission is complete or when a mission aborts
int main(int argc, char **argv) {

     //init the ROS node
     ros::init(argc, argv, "semaforr_controller");
     if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
   	ros::console::notifyLoggerLevelsChanged();
     }
     ros::NodeHandle nh;
	
     
     string path = ros::package::getPath("semaforr");

     //string path = "/home/anooparoor/catkin_ws/src/semaforr";

     //bool ok = ros::param::get("/semaforr_controller/semaforr_package_path",path);
     //if(!ok){
	//ROS_DEBUG("Could not get semaforr_package_path");
	//exit(1);
     //}
     
     string advisor_config = path + "/config/advisors.conf";
     string action_config = path + "/config/action.conf";
     string tasks_config = path + "/config/target.conf";
     // currently not implemented
     string planner_config = path;
     ROS_INFO("Controller Initialized");
     Controller *controller = new Controller(advisor_config,tasks_config,action_config,planner_config); 
     
     
     RobotDriver driver(nh, controller);
     driver.initialize();
     ROS_INFO("Robot Driver Initialized");
     driver.run();
     
     ROS_INFO("Mission Accomplished!"); 

     return 0;
}



