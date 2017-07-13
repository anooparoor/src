/* \mainpage SemaFORR Explanation
 * \brief Explains low-level actions of a robot.
 *
 * \author Raj Korpan.
 *
 * \version SEMAFORR Explanation 1.0
 *
 *
 */

#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <string>

#include <ros/package.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>

using namespace std;

class Explanation
{
private:
	//! The node handle we'll be using
	ros::NodeHandle nh_;
	//! We will be publishing to the "explanations" topic
	ros::Publisher explanations_pub_;
	//! We will be listening to \decision_log topic
	ros::Subscriber sub_decisionLog_;
	// Current log
	string current_log;
	// Message received
	bool init_message_received;
public:
	//! ROS node initialization
	Explanation(ros::NodeHandle &nh)
	{
		nh_ = nh;
		//set up the publisher for the explanations topic
		explanations_pub_ = nh_.advertise<std_msgs::String>("explanations", 1);
		sub_decisionLog_ = nh.subscribe("decision_log", 1000, &Explanation::updateLog, this);
		init_message_received = false;
	}

	void updateLog(const std_msgs::String & log){
		if(init_message_received == false){
			init_message_received = true;
			current_log = log.data;
		}
		else {
			current_log = log.data;
		}
		//ROS_INFO_STREAM("Recieved log data: " << current_log << endl);
	}

	void initialize(string text_config){
		string fileLine;
		std::ifstream file(text_config.c_str());
		ROS_DEBUG_STREAM("Reading text_config_file:" << text_config);
		if(!file.is_open()){
			ROS_DEBUG("Unable to locate or read text config file!");
		}

		while(getline(file, fileLine)){
			//cout << "Inside while in tasks" << endl;
			if(fileLine[0] == '#')  // skip comment lines
				continue;
			else{
				std::vector<std::string> vstrings = parseText(fileLine);
				//ROS_DEBUG_STREAM("File text:" << vstrings[0]);
			}
		}


		ros::spinOnce();
	}
	
	void run(){
		std_msgs::String explanationString;
		ros::Rate rate(30.0);
		while(nh_.ok()) {
			while(init_message_received == false){
				ROS_DEBUG("Waiting for first message");
				//wait for some time
				rate.sleep();
				// Sense input 
				ros::spinOnce();
			}
			explanationString.data = parseText(current_log)[10];
			//send the explanation
			explanations_pub_.publish(explanationString);
			//wait for some time
			rate.sleep();
			// Sense input 
			ros::spinOnce();
		}
	}

	std::vector<std::string> parseText(string text){
		std::stringstream ss(text);
		std::istream_iterator<std::string> begin(ss);
		std::istream_iterator<std::string> end;
		std::vector<std::string> vstrings(begin, end);
		//ROS_DEBUG_STREAM("Log text:" << vstrings[0]);
		return vstrings;
	}
};

// Main file : Load configuration file

int main(int argc, char **argv) {

	//init the ROS node
	ros::init(argc, argv, "semaforr_explanation");
	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
		ros::console::notifyLoggerLevelsChanged();
	}
	ros::NodeHandle nh;

	string path = ros::package::getPath("semaforr_explanation");
	string text_config = path + "/config/text.conf";
	Explanation explain(nh);
	explain.initialize(text_config);
	ROS_INFO("Explanation Initialized");
	explain.run();

	return 0;
}
