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

/*! \brief Entry point to the controller
  
  It generates a Controller object with the parameters read from the config file.

  \callgraph
 */
int main(int argc, char **argv) {
     //Initializes ROS, and sets up a node
     ros::init(argc, argv, "semaforr");
     ros::NodeHandle nh;

     //Sets up the random number generator
     srand(time(0));

     //Sets the loop to publish at a rate of 10Hz
     ros::Rate rate(10);

     //declare and create a controller and pass pub, crowd_sub, pos_sub, laser_sub (map service can be incorporated as needed)
     Controller * controller = new Controller();
     // intialize advisors, robot laser parameters and action parameters, targetset from config files
     controller->initialize_FORRAdvisors();
     controller->initialize_robotparameters();
     controller->initialize_targetparameters();

     //Declares the message to be sent
     geometry_msgs::Twist base_cmd;
     // Run the loop
     while(ros::ok()) {
          //publish the assembled command
	  controller->sense();
	  controller->decide();
	  base_cmd = controller->getAction();
          pub.publish(base_cmd);
          rate.sleep();
     }
  return 0;
}


