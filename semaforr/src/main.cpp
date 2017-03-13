/* \mainpage ROSiffied RobotController Documentation
 * \brief RobotController governs low-level actions of a robot.
 *
 * \authors A. Tuna Ozgelen (contact author), Eric Schneider, Ofear Balas, Anoop Aroor, Matthew.
 *
 * \version SEMAFORR Learning experiments
 *
 *
 */


#include <ros/ros.h>
#include <geometry_msgs/Twist.h> 
#include <stdlib.h>
#include "Controller.h"

using namespace std;


// Function that converts FORRAction into ROS compatible base_cmd

geometry_msgs::Twist convert_to_base(FORRAction action){

    geometry_msgs::Twist base_cmd;
    return base_cmd;

}


/*! \brief Entry point to the controller
  
  It generates a Controller object with the parameters read from the config file.

  \callgraph
 */
int main(int argc, char **argv) {
     //Initializes ROS, and sets up a node
     ros::init(argc, argv, "semaforr");
     ros::NodeHandle nh;

     //Ceates the publisher, and tells it to publish
     //to the menge/cmd_vel topic, with a queue size of 100
     ros::Publisher pub=nh.advertise<geometry_msgs::Twist>("cmd_vel", 100);

     string advisor_config = "advisor.conf";
     string robot_config = "robot.conf";
     string tasks_config = "tasks.conf";

     //Sets up the random number generator
     srand(time(0));

     //Sets the loop to publish at a rate of 10Hz
     ros::Rate rate(10);

     //declare and create a controller and pass pub, crowd_sub, pos_sub, laser_sub (map service can be incorporated as needed)
     Controller * controller = new Controller(nh,advisor_config,robot_config,tasks_config);

     //Declares the message to be sent
     geometry_msgs::Twist base_cmd;
     // Run the loop , the input sensing the output beaming is asynchrounous
     while(ros::ok()) {
          //Sense the input and the current target to run the advisors and generate a decision
	  base_cmd = convert_to_base(controller->decide());
          // publish the decision to menge 
          pub.publish(base_cmd);
          rate.sleep();
     }
     return 0;
}



