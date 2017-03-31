// Publish robot status into Rviz for visualization and debugging


#include <iostream>
#include <stdlib.h>
#include "Beliefs.h"

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h> 
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;


class Visualizer
{
private:
  //! We will be publishing to the "target_point" topic to display target point on rviz
  ros::Publisher target_pub_;
  ros::Publisher region_pub_;
  ros::Publisher conveyor_pub_;
  Beliefs *beliefs;
  ros::NodeHandle *nh_;
  
public:
  //! ROS node initialization
  Visualizer(ros::NodeHandle *nh, Beliefs *b)
  {
    nh_ = nh;
    //set up the publisher for the cmd_vel topic
    target_pub_ = nh_->advertise<geometry_msgs::PointStamped>("target_point", 1);
    conveyor_pub_ = nh_->advertise<nav_msgs::OccupancyGrid>("conveyor", 1);
    region_pub_ = nh_->advertise<visualization_msgs::MarkerArray>("region", 1);
    //declare and create a controller with task, action and advisor configuration
    beliefs = b;
  }

  void publish(){
	if(beliefs->getAgentState()->getCurrentTask() != NULL){
		publish_target();
	}
	publish_conveyor();
	publish_region();
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

  void publish_conveyor(){
	ROS_DEBUG("Inside publish conveyor");
	nav_msgs::OccupancyGrid grid;

	grid.header.frame_id = "map";
	grid.header.stamp = ros::Time::now();
	grid.info.map_load_time = ros::Time::now();

	grid.info.origin.orientation.w = 0;
	grid.info.resolution = beliefs->getSpatialModel()->getWaypoints()->getGranularity();
	grid.info.width = beliefs->getSpatialModel()->getWaypoints()->getBoxWidth();
	grid.info.height = beliefs->getSpatialModel()->getWaypoints()->getBoxHeight();
	
	vector< vector<int> > waypoints = beliefs->getSpatialModel()->getWaypoints()->getWaypoints();
	for(int j = 0; j < grid.info.height; j++){
	    for(int i = 0; i < grid.info.width; i++){
      		grid.data.push_back(waypoints[i][j]);
    	    }
  	}
	conveyor_pub_.publish(grid);	
  }

  void publish_region(){
	ROS_DEBUG("Inside publish regions");

	visualization_msgs::MarkerArray markerArray;
	vector<FORRCircle> circles = beliefs->getSpatialModel()->getAbstractMap()->getCircles();
	cout << "There are currently " << circles.size() << " circles" << endl;
	for(int i = 0 ; i < circles.size(); i++){	
		//circles[i].print();
		visualization_msgs::Marker marker;	
		marker.header.frame_id = "map";
    		marker.header.stamp = ros::Time::now();
		marker.ns = "basic_shapes";
    		marker.id = i;
		marker.type = visualization_msgs::Marker::CYLINDER;
    	
		marker.pose.position.x = circles[i].getCenter().get_x();
    		marker.pose.position.y = circles[i].getCenter().get_y();
    		marker.pose.position.z = 0;
    		marker.pose.orientation.x = 0.0;
    		marker.pose.orientation.y = 0.0;
    		marker.pose.orientation.z = 0.0;
    		marker.pose.orientation.w = 1.0;

    		// Set the scale of the marker -- 1x1x1 here means 1m on a side
    		marker.scale.x = marker.scale.y = circles[i].getRadius()*2;
    		marker.scale.z = 1.0;

    		// Set the color -- be sure to set alpha to something non-zero!
    		marker.color.r = 0.0f;
    		marker.color.g = 1.0f;
    		marker.color.b = 0.0f;
    		marker.color.a = 0.5;

    		marker.lifetime = ros::Duration();
		markerArray.markers.push_back(marker);		

	}
	region_pub_.publish(markerArray);
  }
};

