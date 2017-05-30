/* \mainpage crowd estimator module Documentation
 * \brief crowd estimator returns a desity and flow map of the crowd, using partial observed data.
 * Keep this independent of semaforr specific data structure!! Please
 * Input : Robot pos, Partially observed crowd data, Given map and preprocessed features
 * Output : Real time density and Flowmap estimates of the crowd in the entire map
 * \author Anoop Aroor. 
 *
 *
 *
 */

#include <iostream>
#include <stdlib.h>
#include <cmath> 

#include <ros/package.h>
#include <ros/ros.h>
#include <ros/console.h>
#include "crowd_estimator.h"

using namespace std;

// Main file : Load configuration files and import map and other preprocesses features and run crowd estimator! 
// Is run forever
int main(int argc, char **argv) {

     //init the ROS node
     ros::init(argc, argv, "crowd_estimator");
     if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
   	ros::console::notifyLoggerLevelsChanged();
     }
     ros::NodeHandle nh;

     ROS_INFO("Crowd estimator");

     //initialize the crowd estimator using the robot observations
     CrowdEstimator crowd(nh);
     ROS_INFO("Crowd Estimator Initialized");
     crowd.run();
     
     ROS_INFO("Mission Accomplished!"); 

     return 0;
}

