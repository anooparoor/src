// Publish robot status into Rviz for visualization and debugging


#include <iostream>
#include <stdlib.h>
#include "Beliefs.h"

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h> 
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_datatypes.h>


using namespace std;


class Visualizer
{
private:
  //! We will be publishing to the "target_point" topic to display target point on rviz
  ros::Publisher target_pub_;
  Beliefs *beliefs;
  ros::NodeHandle *nh_;
  
public:
  //! ROS node initialization
  Visualizer(ros::NodeHandle *nh, Beliefs *b)
  {
    nh_ = nh;
    //set up the publisher for the cmd_vel topic
    target_pub_ = nh_->advertise<geometry_msgs::PointStamped>("target_point", 1);
    //declare and create a controller with task, action and advisor configuration
    beliefs = b;
  }

  void publish(){
	if(beliefs->getAgentState()->getCurrentTask() != NULL){
		publish_target();
	}
  }

  void publish_target(){
	ROS_DEBUG("Inside visualization tool!!");
	geometry_msgs::PointStamped target;
	target.header.frame_id = "map";
	target.header.stamp = ros::Time::now();
	target.point.x = beliefs->getAgentState()->getCurrentTask()->getX();
	target.point.y = beliefs->getAgentState()->getCurrentTask()->getY();
	target.point.z = 0;
	target_pub_.publish(target);
  }
};

