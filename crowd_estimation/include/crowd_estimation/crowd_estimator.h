/* \crowd_estimator.h
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
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_datatypes.h>


using namespace std;

// Generates a real time estimate of crowd density and flows in the environment using robot observations
class CrowdEstimator
{
private:
  //! The node handle we'll be using
  ros::NodeHandle _nh;
  //! We will be publishing to the "flow_map" topic 
  //ros::Publisher _flow_map_pub;
  //! We will be publishing to the "density_map" topic 
  //ros::Publisher _density_map_pub;
  //! We will be listening to \crowd_pose topic
  ros::Subscriber _sub_crowd_pose;
  //! We will be listening to \pose topic
  ros::Subscriber _sub_pose;
  // Current robot position
  geometry_msgs::PoseStamped current_pose;
  // Vector of crowd positions
  geometry_msgs::PoseArray crowd_pose;

public:
  //! ROS node initialization
  CrowdEstimator(ros::NodeHandle &nh)
  {
    _nh = nh;
    _sub_crowd_pose = _nh.subscribe("crowd_pose",1000, &CrowdEstimator::getCrowdPose, this);
    _sub_pose = _nh.subscribe("pose", 1000, &CrowdEstimator::getPose, this);
  }

  
  // Callback function for pose message
  void getPose(const geometry_msgs::PoseStamped & pose){
	//ROS_DEBUG("Inside callback for position message");
        double x = pose.pose.position.x;
  	double y = pose.pose.position.y;
	tf::Quaternion q(pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
        current_pose = pose;
	ROS_INFO_STREAM("Recieved pose message from menge: " << x << " " << y << " " << yaw << endl);
  }


  // Callback function for pose message
  void getCrowdPose(const geometry_msgs::PoseArray & crowdPose){
	//ROS_DEBUG("Inside callback for crowd positions message");
	int crowd_size = crowdPose.poses.size();
	for(int i = 0; i < crowd_size; i++){
		geometry_msgs::Pose pose = crowdPose.poses[i];
        	double x = pose.position.x;
  		double y = pose.position.y;
		tf::Quaternion q(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w);
		tf::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);
		ROS_INFO_STREAM("Recieved agent message from menge: " << x << " " << y << " " << yaw << endl);
	}
	crowd_pose = crowdPose;
	ROS_INFO_STREAM("Recieved crowd pose message from menge: " << crowd_size << endl);
	generateCrowdEstimate();
  }
  // Main loop of the crowd state estimator 
  void run(){
	ros::Rate rate(30.0);
	// Run the loop , the input sensing and the output beaming is asynchrounous
     	while(_nh.ok()) {
	  //wait for some time
          rate.sleep();
          // Sense input 
          ros::spinOnce();
	}
  }


  // Generates crowd estimate across the map
  void generateCrowdEstimate(){
	// Convert posearray into observation z
        // 
	




  }

  
  


};
