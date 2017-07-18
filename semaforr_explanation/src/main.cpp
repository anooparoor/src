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
#include <vector>
#include <sstream>
#include <iterator>
#include <map>

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
	// Actions with their associated phrases
	std::map <std::string, std::string> actionText;
	// t-score intervals with their associated phrases
	std::vector <double> tScoreThreshold;
	std::vector <std::string> tScorePhrase;
	// Advisors with their associated rationales
	std::map <std::string, std::string> advSupportRationales;
	std::map <std::string, std::string> advOpposeRationales;
	// Stats on tier 3
	std::set<std::string> advisors;
	std::map <std::string, double> advisorTotal;
	std::map <std::string, double> advisorCount;
	std::map <std::string, double> advisorMean;
	std::map <std::string, double> advisorStandardDeviation;
	std::map <std::string, double> advisorTScore;
	std::set<std::string> actions;
	std::map <std::string, double> actionTotal;
	std::map <std::string, double> actionCount;
	std::map <std::string, double> actionMean;
	std::map <std::string, double> actionStandardDeviation;

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
		init_message_received = true;
		current_log = log.data;
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
			else if (fileLine.find("actiontext") != std::string::npos){
				std::vector<std::string> vstrings = parseText(fileLine);
				for(int i=1; i < vstrings.size(); i+=2){
					actionText.insert( std::pair<std::string,std::string>(vstrings[i],vstrings[i+1]));
					ROS_DEBUG_STREAM("File text:" << vstrings[i] << " " << vstrings[i+1] << endl);
				}
				//ROS_DEBUG_STREAM("File text:" << vstrings[0]);
			}
			else if (fileLine.find("tscorephrase") != std::string::npos){
				std::vector<std::string> vstrings = parseText(fileLine);
				for(int i=1; i < vstrings.size(); i+=2){
					tScoreThreshold.push_back(atof(vstrings[i].c_str()));
					tScorePhrase.push_back(vstrings[i+1]);
					ROS_DEBUG_STREAM("File text:" << vstrings[i+1] << " " << vstrings[i] << endl);
				}
				//ROS_DEBUG_STREAM("File text:" << vstrings[0]);
			}
			else if (fileLine.find("advsupportrationales") != std::string::npos){
				std::vector<std::string> vstrings = parseText(fileLine);
				for(int i=1; i < vstrings.size(); i+=2){
					advSupportRationales.insert( std::pair<std::string,std::string>(vstrings[i],vstrings[i+1]));
					ROS_DEBUG_STREAM("File text:" << vstrings[i] << " " << vstrings[i+1] << endl);
				}
			}
			else if (fileLine.find("advopposerationales") != std::string::npos){
				std::vector<std::string> vstrings = parseText(fileLine);
				for(int i=1; i < vstrings.size(); i+=2){
					advOpposeRationales.insert( std::pair<std::string,std::string>(vstrings[i],vstrings[i+1]));
					ROS_DEBUG_STREAM("File text:" << vstrings[i] << " " << vstrings[i+1] << endl);
				}
			}
		}


		ros::spinOnce();
	}
	
	void run(){
		std_msgs::String explanationString;
		string vetoedActions, chosenAction, advisorComments;
		int decisionTier;
		ros::Rate rate(30.0);
		while(nh_.ok()) {
			while(init_message_received == false){
				ROS_DEBUG("Waiting for first message");
				//wait for some time
				rate.sleep();
				// Sense input 
				ros::spinOnce();
			}
			//target = "(" + parseText(current_log)[4] + ", " + parseText(current_log)[5] + ")";
			decisionTier = atoi(parseText(current_log)[10].c_str());
        		vetoedActions = parseText(current_log)[11];
        		chosenAction = parseText(current_log)[12]+parseText(current_log)[13];
			advisorComments = parseText(current_log)[15];
			//ROS_INFO_STREAM(decisionTier << " " << vetoedActions << " " << chosenAction << " " << advisorComments << endl << endl);

			if (decisionTier == 1){
				explanationString.data = "I could see our target and " + actionText[chosenAction] + " would get us closer to it.";
			} else if (vetoedActions == "0 1;0 2;0 3;0 4;0 5;") {
				//ROS_DEBUG(vetoedActions << endl);
				explanationString.data = "I decided to " + actionText[chosenAction] + " because there's not enough room to move forward.";
			} else {
				parseTier3Comments(advisorComments);
				computeTier3TScores(advisorComments, chosenAction);
				explanationString.data = tier3Explanation(chosenAction);
			}
			//explanationString.data = parseText(current_log)[10];
			//send the explanation
			explanations_pub_.publish(explanationString);
			init_message_received = false;
			clearStats();
			//wait for some time
			rate.sleep();
			// Sense input 
			ros::spinOnce();
		}
	}

	void parseTier3Comments(string advisorComments) {
		std::stringstream ss1;
		ss1.str(advisorComments);
		std::string item, val;
		char delim = ';';
		while (std::getline(ss1, item, delim)) {
			std::stringstream subss;
			subss.str(item);
			char delim2 = ' ';
			std::vector<std::string> vstrings;
			while (std::getline(subss, val, delim2)) {
				vstrings.push_back(val);
			}
			std::string action = (vstrings[1]+vstrings[2]);
			if (advisorTotal.find(vstrings[0]) == advisorTotal.end()) {
				advisorTotal[vstrings[0]] = atof(vstrings[3].c_str());
			} else {
				advisorTotal[vstrings[0]] = advisorTotal[vstrings[0]] + atof(vstrings[3].c_str());
			}
			if (advisorCount.find(vstrings[0]) == advisorCount.end()) {
				advisorCount[vstrings[0]] = 1;
			} else {
				advisorCount[vstrings[0]] = advisorCount[vstrings[0]] + 1;
			}
			advisors.insert(vstrings[0]);
			if (actionTotal.find(action) == actionTotal.end()) {
				actionTotal[action] = atof(vstrings[3].c_str());
			} else {
				actionTotal[action] = actionTotal[action] + atof(vstrings[3].c_str());
			}
			if (actionCount.find(action) == actionCount.end()) {
				actionCount[action] = 1;
			} else {
				actionCount[action] = actionCount[action] + 1;
			}
			actions.insert(action);
			//ROS_INFO_STREAM(vstrings[0] << " " << action << " " << vstrings[3] << ";");
			vstrings.clear();
		}
		std::set<std::string>::iterator adv, act;
		for (adv = advisors.begin(); adv != advisors.end(); adv++) {
			advisorMean[*adv] = (advisorTotal[*adv] / advisorCount[*adv]);
		}
		for (act = actions.begin(); act != actions.end(); act++) {
			actionMean[*act] = (actionTotal[*act] / actionCount[*act]);
		}

		std::stringstream ss2;
		ss2.str(advisorComments);
		while (std::getline(ss2, item, delim)) {
			std::stringstream subss;
			subss.str(item);
			char delim2 = ' ';
			std::vector<std::string> vstrings;
			while (std::getline(subss, val, delim2)) {
				vstrings.push_back(val);
			}
			std::string action = (vstrings[1]+vstrings[2]);
			if (advisorStandardDeviation.find(vstrings[0]) == advisorStandardDeviation.end()) {
				advisorStandardDeviation[vstrings[0]] = pow((atof(vstrings[3].c_str()) - advisorMean[vstrings[0]]), 2);
			} else {
				advisorStandardDeviation[vstrings[0]] = advisorStandardDeviation[vstrings[0]] + pow((atof(vstrings[3].c_str()) - advisorMean[vstrings[0]]), 2);
			}
			
			if (actionStandardDeviation.find(action) == actionStandardDeviation.end()) {
				actionStandardDeviation[action] = pow((atof(vstrings[3].c_str()) - actionMean[action]), 2);
			} else {
				actionStandardDeviation[action] = actionStandardDeviation[action] + pow((atof(vstrings[3].c_str()) - actionMean[action]), 2);
			}
			vstrings.clear();
		}
		for (adv = advisors.begin(); adv != advisors.end(); adv++) {
			advisorStandardDeviation[*adv] = sqrt(advisorStandardDeviation[*adv] / advisorCount[*adv]);
		}
		for (act = actions.begin(); act != actions.end(); act++) {
			actionStandardDeviation[*act] = sqrt(actionStandardDeviation[*act] / actionCount[*act]);
		}
	}

	void computeTier3TScores(string advisorComments, string chosenAction) {
		std::stringstream ss1;
		ss1.str(advisorComments);
		std::string item, val;
		char delim = ';';
		while (std::getline(ss1, item, delim)) {
			std::stringstream subss;
			subss.str(item);
			char delim2 = ' ';
			std::vector<std::string> vstrings;
			while (std::getline(subss, val, delim2)) {
				vstrings.push_back(val);
			}
			std::string action = (vstrings[1]+vstrings[2]);
			if (action == chosenAction) {
				if (advisorStandardDeviation[vstrings[0]] != 0) {
					advisorTScore[vstrings[0]] = ((atof(vstrings[3].c_str()) - advisorMean[vstrings[0]]) / advisorStandardDeviation[vstrings[0]]);
					//ROS_INFO_STREAM(vstrings[0] << " " << action << ": " << vstrings[3] << ", mean: " << advisorMean[vstrings[0]] << ", stdev: " << advisorStandardDeviation[vstrings[0]] << ", t score: " << (atof(vstrings[3].c_str()) - advisorMean[vstrings[0]]) / advisorStandardDeviation[vstrings[0]]);
				} else {
					advisorTScore[vstrings[0]] = 0;
				}
			}
			vstrings.clear();
		}
	}

	std::string tier3TScoretoPhrase(double tscore) {
		std::string phrase;
		for (int i = tScoreThreshold.size()-1; i >= 0; --i) {
			if (tscore <= tScoreThreshold[i]) {
				phrase = tScorePhrase[i];
			}
		}
		//ROS_INFO_STREAM(tscore << " " << phrase);
		return phrase;
	}

	std::string tier3Explanation(string chosenAction) {
		std::string explanation, supportConcat, opposeConcat;
		std::vector<std::string> supportPhrases, slightSupportPhrases;
		std::vector<std::string> opposePhrases, slightOpposePhrases;
		
		std::map <std::string, double>::iterator itr;
		//ROS_INFO_STREAM(advisorTScore.size());
		for (itr = advisorTScore.begin(); itr != advisorTScore.end(); itr++) {
			if (itr->second > (0.75)) {
				supportPhrases.push_back("I " + tier3TScoretoPhrase(itr->second) + " to " + advSupportRationales[itr->first]);
				//ROS_INFO_STREAM(itr->first << ": " << itr->second);
				//ROS_INFO_STREAM("I " + tier3TScoretoPhrase(itr->second) + " to " + advSupportRationales[itr->first]);
			}
			else if (itr->second > (0)) {
				slightSupportPhrases.push_back("I " + tier3TScoretoPhrase(itr->second) + " to " + advSupportRationales[itr->first]);
				//ROS_INFO_STREAM(itr->first << ": " << itr->second);
				//ROS_INFO_STREAM("I " + tier3TScoretoPhrase(itr->second) + " to " + advSupportRationales[itr->first]);
			}
			else if (itr->second > (-0.75)) {
				slightOpposePhrases.push_back("I " + tier3TScoretoPhrase(itr->second) + " to " + advOpposeRationales[itr->first]);
				//ROS_INFO_STREAM(itr->first << ": " << itr->second);
				//ROS_INFO_STREAM("I " + tier3TScoretoPhrase(itr->second) + " to " + advOpposeRationales[itr->first]);
			}
			else {
				opposePhrases.push_back("I " + tier3TScoretoPhrase(itr->second) + " to " + advOpposeRationales[itr->first]);
				//ROS_INFO_STREAM(itr->first << ": " << itr->second);
				//ROS_INFO_STREAM("I " + tier3TScoretoPhrase(itr->second) + " to " + advOpposeRationales[itr->first]);
			}
		}
		
		if (supportPhrases.size() > 2) {
			for (int i = 0; i < supportPhrases.size()-1; i++) {
				supportConcat = supportConcat + supportPhrases[i] + ", ";
			}
			supportConcat = supportConcat + "and " + supportPhrases[supportPhrases.size()-1];
			//ROS_INFO_STREAM("Greater than 2: " << supportConcat);
		}
		else if (supportPhrases.size() == 2) {
			supportConcat = supportPhrases[0] + " and " + supportPhrases[1];
			//ROS_INFO_STREAM("Equals 2: " << supportConcat);
		}
		else if (supportPhrases.size() == 1) {
			supportConcat = supportPhrases[0];
			//ROS_INFO_STREAM("Equals 1: " << supportConcat);
		}
		else if (supportPhrases.size() == 0) {
			if (slightSupportPhrases.size() > 2) {
				for (int i = 0; i < slightSupportPhrases.size()-1; i++) {
					supportConcat = supportConcat + slightSupportPhrases[i] + ", ";
				}
				supportConcat = supportConcat + "and " + slightSupportPhrases[slightSupportPhrases.size()-1];
				//ROS_INFO_STREAM("Greater than 2 Slightly: " << supportConcat);
			}
			else if (slightSupportPhrases.size() == 2) {
				supportConcat = slightSupportPhrases[0] + " and " + slightSupportPhrases[1];
				//ROS_INFO_STREAM("Equals 2 Slightly: " << supportConcat);
			}
			else if (slightSupportPhrases.size() == 1) {
				supportConcat = slightSupportPhrases[0];
				//ROS_INFO_STREAM("Equals 1 Slightly: " << supportConcat);
			}
		}
		
		if (opposePhrases.size() > 2) {
			for (int i = 0; i < opposePhrases.size()-1; i++) {
				opposeConcat = opposeConcat + opposePhrases[i] + ", ";
			}
			opposeConcat = opposeConcat + "and " + opposePhrases[opposePhrases.size()-1];
			//ROS_INFO_STREAM("Greater than 2 Oppose: " << opposeConcat);
			explanation = "Although " + opposeConcat + ", I decided to " + actionText[chosenAction] + " because " + supportConcat + ".";
			//ROS_INFO_STREAM(explanation);
		}
		else if (opposePhrases.size() == 2) {
			opposeConcat = opposePhrases[0] + " and " + opposePhrases[1];
			//ROS_INFO_STREAM("Equals 2 Oppose: " << opposeConcat);
			explanation = "Although " + opposeConcat + ", I decided to " + actionText[chosenAction] + " because " + supportConcat + ".";
			//ROS_INFO_STREAM(explanation);
		}
		else if (opposePhrases.size() == 1) {
			opposeConcat = opposePhrases[0];
			//ROS_INFO_STREAM("Equals 1 Oppose: " << opposeConcat);
			explanation = "Although " + opposeConcat + ", I decided to " + actionText[chosenAction] + " because " + supportConcat + ".";
			//ROS_INFO_STREAM(explanation);
		}
		else if (opposePhrases.size() == 0) {
			if (supportPhrases.size() == 0) {
				if (slightOpposePhrases.size() > 2) {
					for (int i = 0; i < slightOpposePhrases.size()-1; i++) {
						opposeConcat = opposeConcat + slightOpposePhrases[i] + ", ";
					}
					opposeConcat = opposeConcat + "and " + slightOpposePhrases[slightOpposePhrases.size()-1];
					//ROS_INFO_STREAM("Greater than 2 Slightly Oppose: " << opposeConcat);
				}
				else if (slightOpposePhrases.size() == 2) {
					opposeConcat = slightOpposePhrases[0] + " and " + slightOpposePhrases[1];
					//ROS_INFO_STREAM("Equals 2 Slightly Oppose: " << opposeConcat);
				}
				else if (slightOpposePhrases.size() == 1) {
					opposeConcat = slightOpposePhrases[0];
					//ROS_INFO_STREAM("Equals 1 Slightly Oppose: " << opposeConcat);
				}
				explanation = "Although " + opposeConcat + ", I decided to " + actionText[chosenAction] + " because " + supportConcat + ".";
				//ROS_INFO_STREAM(explanation);
			}
			else {
				explanation = "I decided to " + actionText[chosenAction] + " because " + supportConcat + ".";
				//ROS_INFO_STREAM(explanation);
			}
		}
		
		return explanation;
	}

	std::vector<std::string> parseText(string text){
		std::vector<std::string> vstrings;
		std::stringstream ss;
		ss.str(text);
		std::string item;
		char delim = '\t';
		while (std::getline(ss, item, delim)) {
			vstrings.push_back(item);
		}
		//std::stringstream ss(text);
		//std::istream_iterator<std::string> begin(ss);
		//std::istream_iterator<std::string> end;
		//std::vector<std::string> vstrings(begin, end);
		//ROS_DEBUG_STREAM("Log text:" << vstrings[0]);
		return vstrings;
	}
	
	void clearStats() {
		advisors.clear();
		advisorTotal.clear();
		advisorCount.clear();
		advisorMean.clear();
		advisorStandardDeviation.clear();
		advisorTScore.clear();
		actions.clear();
		actionTotal.clear();
		actionCount.clear();
		actionMean.clear();
		actionStandardDeviation.clear();
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
