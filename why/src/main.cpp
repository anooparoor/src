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
#include <algorithm>
#include <sys/time.h>

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
	//! We will be publishing to the "explanations_log" topic
	ros::Publisher explanations_log_pub_;
	//! We will be listening to \decision_log topic
	ros::Subscriber sub_decisionLog_;
	// Current log
	string current_log;
	// Message received
	bool init_message_received;
	// Actions with their associated phrases
	std::map <std::string, std::string> actionText;
	std::map <std::string, std::string> actioningText;
	// t-score intervals with their associated phrases
	std::vector <double> tScoreThreshold;
	std::vector <std::string> tScorePhrase;
	// Advisors with their associated rationales
	std::map <std::string, std::string> advSupportRationales;
	std::map <std::string, std::string> advOpposeRationales;
	// Confidence metrics with their associated phrases
	std::vector <double> confidenceLevelThreshold;
	std::vector <std::string> confidenceLevelPhrase;
	std::vector <double> giniThreshold;
	std::vector <std::string> giniPhrase;
	std::vector <double> overallSupportThreshold;
	std::vector <std::string> overallSupportPhrase;
	// Tier 3 alternate action phrase
	std::vector <double> diffOverallSupportThreshold;
	std::vector <std::string> diffOverallSupportPhrase;
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
	double totalCommentCount=0, totalCommentMean=0, totalCommentStdev=0;
	double gini, overallSupport, confidenceLevel;
	std::vector <double> diffTScores, diffOverallSupports;
	double computationTimeSec=0.0;
	int decisionTier=0;

public:
	//! ROS node initialization
	Explanation(ros::NodeHandle &nh)
	{
		nh_ = nh;
		//set up the publisher for the explanations topic
		explanations_pub_ = nh_.advertise<std_msgs::String>("explanations", 1);
		explanations_log_pub_ = nh_.advertise<std_msgs::String>("explanations_log", 1);
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
			else if (fileLine.find("confidencelevel") != std::string::npos){
				std::vector<std::string> vstrings = parseText(fileLine);
				for(int i=1; i < vstrings.size(); i+=2){
					confidenceLevelThreshold.push_back(atof(vstrings[i].c_str()));
					confidenceLevelPhrase.push_back(vstrings[i+1]);
					ROS_DEBUG_STREAM("File text:" << vstrings[i+1] << " " << vstrings[i] << endl);
				}
				//ROS_DEBUG_STREAM("File text:" << vstrings[0]);
			}
			else if (fileLine.find("gini") != std::string::npos){
				std::vector<std::string> vstrings = parseText(fileLine);
				for(int i=1; i < vstrings.size(); i+=2){
					giniThreshold.push_back(atof(vstrings[i].c_str()));
					giniPhrase.push_back(vstrings[i+1]);
					ROS_DEBUG_STREAM("File text:" << vstrings[i+1] << " " << vstrings[i] << endl);
				}
				//ROS_DEBUG_STREAM("File text:" << vstrings[0]);
			}
			else if (fileLine.find("overallsupport") != std::string::npos){
				std::vector<std::string> vstrings = parseText(fileLine);
				for(int i=1; i < vstrings.size(); i+=2){
					overallSupportThreshold.push_back(atof(vstrings[i].c_str()));
					overallSupportPhrase.push_back(vstrings[i+1]);
					ROS_DEBUG_STREAM("File text:" << vstrings[i+1] << " " << vstrings[i] << endl);
				}
				//ROS_DEBUG_STREAM("File text:" << vstrings[0]);
			}
			else if (fileLine.find("diffOS") != std::string::npos){
				std::vector<std::string> vstrings = parseText(fileLine);
				for(int i=1; i < vstrings.size(); i+=2){
					diffOverallSupportThreshold.push_back(atof(vstrings[i].c_str()));
					diffOverallSupportPhrase.push_back(vstrings[i+1]);
					ROS_DEBUG_STREAM("File text:" << vstrings[i+1] << " " << vstrings[i] << endl);
				}
				//ROS_DEBUG_STREAM("File text:" << vstrings[0]);
			}
			else if (fileLine.find("actioningtext") != std::string::npos){
				std::vector<std::string> vstrings = parseText(fileLine);
				for(int i=1; i < vstrings.size(); i+=2){
					actioningText.insert( std::pair<std::string,std::string>(vstrings[i],vstrings[i+1]));
					ROS_DEBUG_STREAM("File text:" << vstrings[i] << " " << vstrings[i+1] << endl);
				}
				//ROS_DEBUG_STREAM("File text:" << vstrings[0]);
			}
		}


		ros::spinOnce();
	}
	
	void run(){
		std_msgs::String explanationString;
		string vetoedActions, chosenAction, advisorComments;
		ros::Rate rate(30.0);
		timeval cv;
		double start_timecv, end_timecv;
		while(nh_.ok()) {
			while(init_message_received == false){
				ROS_DEBUG("Waiting for first message");
				//wait for some time
				rate.sleep();
				// Sense input 
				ros::spinOnce();
			}
			gettimeofday(&cv,NULL);
			start_timecv = cv.tv_sec + (cv.tv_usec/1000000.0);
			//target = "(" + parseText(current_log)[4] + ", " + parseText(current_log)[5] + ")";
			decisionTier = atoi(parseText(current_log)[10].c_str());
			vetoedActions = parseText(current_log)[11];
			chosenAction = parseText(current_log)[12]+parseText(current_log)[13];
			advisorComments = parseText(current_log)[15];
			//ROS_INFO_STREAM(decisionTier << " " << vetoedActions << " " << chosenAction << " " << advisorComments << endl << endl);

			if (decisionTier == 1){
				explanationString.data = "I could see our target and " + actioningText[chosenAction] + " would get us closer to it.\n" + "Highly confident, since our target is in sensory range and this would get us closer to it.\n" + victoryAlternateActions(chosenAction);
			} else if (vetoedActions == "0 1;0 2;0 3;0 4;0 5;" and chosenAction == "30") {
				//ROS_DEBUG(vetoedActions << endl);
				decisionTier = 1;
				explanationString.data = "I decided to " + actionText[chosenAction] + " because there's not enough room to move forward.\n" + "Highly confident, since there is not enough room to move forward.\n" + vetoedAlternateActions(vetoedActions, chosenAction);
			} else {
				parseTier3Comments(advisorComments);
				advisorTScore = computeTier3TScores(advisorComments, chosenAction);
				computeConfidence(chosenAction);
				explanationString.data = tier3Explanation(chosenAction) + "\n" + confidenceExplanation() + "\n" + vetoedAlternateActions(vetoedActions, chosenAction) + "\n" + tier3AlternateActions(advisorComments, chosenAction);
			}
			gettimeofday(&cv,NULL);
			end_timecv = cv.tv_sec + (cv.tv_usec/1000000.0);
			computationTimeSec = (end_timecv-start_timecv);
			logExplanationData();
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

	std::map <std::string, double> computeTier3TScores(string advisorComments, string chosenAction) {
		std::map <std::string, double> tScores;
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
					tScores[vstrings[0]] = ((atof(vstrings[3].c_str()) - advisorMean[vstrings[0]]) / advisorStandardDeviation[vstrings[0]]);
					//ROS_INFO_STREAM(vstrings[0] << " " << action << ": " << vstrings[3] << ", mean: " << advisorMean[vstrings[0]] << ", stdev: " << advisorStandardDeviation[vstrings[0]] << ", t score: " << (atof(vstrings[3].c_str()) - advisorMean[vstrings[0]]) / advisorStandardDeviation[vstrings[0]]);
				} else {
					tScores[vstrings[0]] = 0;
				}
			}
			vstrings.clear();
		}
		return tScores;
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
		
		return explanation + "\n";
	}
	
	void computeConfidence(string chosenAction) {
		gini = 2 * (actionTotal[chosenAction]/(10*actionCount[chosenAction])) * (1 - (actionTotal[chosenAction]/(10*actionCount[chosenAction])));
		//ROS_INFO_STREAM(actionTotal[chosenAction] << " " << actionCount[chosenAction] << " " << gini);
		std::map <std::string, double>::iterator itr;
		for (itr = actionTotal.begin(); itr != actionTotal.end(); itr++) {
			totalCommentMean += itr->second;
			totalCommentCount++;
		}
		totalCommentMean = totalCommentMean / totalCommentCount;
		for (itr = actionTotal.begin(); itr != actionTotal.end(); itr++) {
			totalCommentStdev += pow((itr->second - totalCommentMean), 2);
		}
		totalCommentStdev = sqrt(totalCommentStdev / totalCommentCount);
		if (totalCommentStdev != 0) {
			overallSupport = (actionTotal[chosenAction] - totalCommentMean)/totalCommentStdev;
		}
		else {
			overallSupport = 0;
		}
		//ROS_INFO_STREAM(totalCommentCount << " " << totalCommentMean << " " << totalCommentStdev << " " << overallSupport);
		confidenceLevel = (0.5 - gini) * overallSupport;
		//ROS_INFO_STREAM(confidenceLevel);
	}
	
	std::string confidenceExplanation() {
		std::string explanation, phraseGini, phraseOverallSupport, phraseConfidenceLevel;
		int giniPhraseID = giniThreshold.size(), overallSupportPhraseID = overallSupportThreshold.size(), confidenceLevelPhraseID = confidenceLevelThreshold.size();
		for (int i = giniThreshold.size()-1; i >= 0; --i) {
			if (gini <= giniThreshold[i]) {
				phraseGini = giniPhrase[i];
				giniPhraseID--;
			}
		}
		for (int i = overallSupportThreshold.size()-1; i >= 0; --i) {
			if (overallSupport <= overallSupportThreshold[i]) {
				phraseOverallSupport = overallSupportPhrase[i];
				overallSupportPhraseID--;
			}
		}
		for (int i = confidenceLevelThreshold.size()-1; i >= 0; --i) {
			if (confidenceLevel <= confidenceLevelThreshold[i]) {
				phraseConfidenceLevel = confidenceLevelPhrase[i];
				confidenceLevelPhraseID--;
			}
		}
		giniPhraseID = -(giniPhraseID-2);
		//ROS_INFO_STREAM(giniPhraseID << " " << overallSupportPhraseID << " " << confidenceLevelPhraseID);
		
		if (giniPhraseID == confidenceLevelPhraseID and overallSupportPhraseID == confidenceLevelPhraseID) {
			explanation = "I'm " + phraseConfidenceLevel + " sure in my decision because " + phraseGini + ". I " + phraseOverallSupport + " to do this most.";
		}
		else if (giniPhraseID == confidenceLevelPhraseID) {
			explanation = "I'm " + phraseConfidenceLevel + " sure in my decision because " + phraseGini + ".";
		}
		else if (overallSupportPhraseID == confidenceLevelPhraseID) {
			explanation = "I'm " + phraseConfidenceLevel + " sure in my decision because I " + phraseOverallSupport + " to do this most.";
		}
		else if (overallSupportPhraseID > confidenceLevelPhraseID and giniPhraseID < confidenceLevelPhraseID) {
			explanation = "I'm " + phraseConfidenceLevel + " sure in my decision because, even though " + phraseGini + ", I " + phraseOverallSupport + " to do this most.";
		}
		else if (giniPhraseID > confidenceLevelPhraseID and overallSupportPhraseID < confidenceLevelPhraseID) {
			explanation = "I'm " + phraseConfidenceLevel + " sure in my decision because, even though I " + phraseOverallSupport + " to do this most, " + phraseGini + ".";
		}
		else {
			explanation = "I'm " + phraseConfidenceLevel + " sure in my decision because " + phraseGini + ". I " + phraseOverallSupport + " to do this most.";
		}		
		//ROS_INFO_STREAM(explanation);
		return explanation + "\n";
	}
	
	std::string victoryAlternateActions(std::string chosenAction) {
		std::string alternateExplanations;
		std::map <std::string, std::string>::iterator itr;
		for (itr = actionText.begin(); itr != actionText.end(); itr++) {
			if (itr->first != chosenAction and itr->first != "30" and (itr->second).length() >0) {
				alternateExplanations = alternateExplanations + "I decided not to " + itr->second + " because I sense our goal and another action would get us closer to it.\n";
			}
		}
		return alternateExplanations;
	}
	
	std::string vetoedAlternateActions(std::string vetoedActions, std::string chosenAction){
		std::string alternateExplanations;

		std::vector<std::string> vstrings;
		std::stringstream ss;
		ss.str(vetoedActions);
		std::string item;
		char delim = ';';
		while (std::getline(ss, item, delim)) {
			item.erase(std::remove(item.begin(), item.end(), ' '), item.end());
			vstrings.push_back(item);
		}
		
		for (int i=0; i < vstrings.size(); i++) {
			if (vstrings[i].at(0) == '0' and (chosenAction.at(0) == '0' or chosenAction.at(0) == '3')) {
				alternateExplanations = alternateExplanations + "I decided not to " + actionText[vstrings[i]] + " because the wall was in the way.\n";
			}
			else if ((vstrings[i].at(0) == '1' or vstrings[i].at(0) == '2') and (chosenAction.at(0) == '1' or chosenAction.at(0) == '2')) {
				alternateExplanations = alternateExplanations + "I decided not to " + actionText[vstrings[i]] + " because I was just facing that way.\n";
			}
		}
		return alternateExplanations;
	}
	
	std::string tier3AlternateActions(std::string advisorComments, std::string chosenAction) {
		std::string alternateExplanations;
		std::map <std::string, double>::iterator atr;
		for (atr = actionTotal.begin(); atr != actionTotal.end(); atr++) {
			std::string supportConcat, opposeConcat, phraseDiffOverallSupport;
			std::vector<std::string> supportPhrases, opposePhrases;
			if (atr->first != chosenAction) {
				double diffOverallSupport;
				if (totalCommentStdev != 0) {
					diffOverallSupport = overallSupport - ((actionTotal[atr->first] - totalCommentMean)/totalCommentStdev);
				}
				else {
					diffOverallSupport = 0;
				}
				//ROS_INFO_STREAM(diffOverallSupportThreshold.size());
				for (int i = diffOverallSupportThreshold.size()-1; i >= 0; --i) {
					//ROS_INFO_STREAM(diffOverallSupport << " " << diffOverallSupportThreshold[i]);
					if (diffOverallSupport <= diffOverallSupportThreshold[i]) {
						phraseDiffOverallSupport = diffOverallSupportPhrase[i];
					}
				}
				//ROS_INFO_STREAM(phraseDiffOverallSupport);
				diffOverallSupports.push_back(diffOverallSupport);
				
				std::map <std::string, double> alternateTScores = computeTier3TScores(advisorComments, atr->first);
				std::map <std::string, double>::iterator itr;
				for (itr = advisorTScore.begin(); itr != advisorTScore.end(); itr++) {
					if ((itr->second - alternateTScores[itr->first]) > 1) {
						supportPhrases.push_back(advSupportRationales[itr->first]);
					}
					else if ((itr->second - alternateTScores[itr->first]) < -1) {
						opposePhrases.push_back(advSupportRationales[itr->first]);
					}
					diffTScores.push_back((itr->second - alternateTScores[itr->first]));
				}
				
				if (supportPhrases.size() > 2) {
					for (int i = 0; i < supportPhrases.size()-1; i++) {
						supportConcat = supportConcat + supportPhrases[i] + ", ";
					}
					supportConcat = supportConcat + "and " + supportPhrases[supportPhrases.size()-1];
				}
				else if (supportPhrases.size() == 2) {
					supportConcat = supportPhrases[0] + " and " + supportPhrases[1];
				}
				else if (supportPhrases.size() == 1) {
					supportConcat = supportPhrases[0];
				}
				
				if (opposePhrases.size() > 2) {
					for (int i = 0; i < opposePhrases.size()-1; i++) {
						opposeConcat = opposeConcat + opposePhrases[i] + ", ";
					}
					opposeConcat = opposeConcat + "and " + opposePhrases[opposePhrases.size()-1];
				}
				else if (opposePhrases.size() == 2) {
					opposeConcat = opposePhrases[0] + " and " + opposePhrases[1];
				}
				else if (opposePhrases.size() == 1) {
					opposeConcat = opposePhrases[0];
				}
				
				
				if (supportPhrases.size() > 0 and opposePhrases.size() > 0) {
					alternateExplanations = alternateExplanations + "I thought about " + actioningText[atr->first] + " because it would let us " + opposeConcat + ", but I felt " + phraseDiffOverallSupport + " strongly about " + actioningText[chosenAction] + " since it lets us " + supportConcat + ".\n";
				}
				else if (supportPhrases.size() > 0 and opposePhrases.size() == 0) {
					alternateExplanations = alternateExplanations + "I thought about " + actioningText[atr->first] + ", but I felt " + phraseDiffOverallSupport + " strongly about " + actioningText[chosenAction] + " since it lets us " + supportConcat + ".\n";
				}
				else if (supportPhrases.size() == 0 and opposePhrases.size() > 0) {
					alternateExplanations = alternateExplanations + "I thought about it because " + actioningText[atr->first] + " would let us " + opposeConcat + ", but I felt " + phraseDiffOverallSupport + " strongly about " + actioningText[chosenAction] + ".\n";
				}
				else if (supportPhrases.size() == 0 and opposePhrases.size() == 0) {
					alternateExplanations = alternateExplanations + "I thought about " + actioningText[atr->first] + ", but I felt " + phraseDiffOverallSupport + " strongly about " + actioningText[chosenAction] + ".\n";
				}
				//ROS_INFO_STREAM(alternateExplanations);
			}
		}
		
		return alternateExplanations;
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
		diffTScores.clear();
		diffOverallSupports.clear();
		
		totalCommentCount=0, totalCommentMean=0, totalCommentStdev=0;
		gini=0, overallSupport=0, confidenceLevel=0;
		computationTimeSec=0.0;
		decisionTier=0;
	}
	
	void logExplanationData() {
		std_msgs::String logData;
		
		std::stringstream tscorestream;
		std::map <std::string, double>::iterator itr;
		for (itr = advisorTScore.begin(); itr != advisorTScore.end(); itr++) {
			tscorestream << itr->second << " ";
		}
		
		std::stringstream difftscoresstream;
		for (int i=0; i < diffTScores.size(); i++) {
			difftscoresstream << diffTScores[i] << " ";
		}
		
		std::stringstream diffoverallsupportsstream;
		for (int i=0; i < diffOverallSupports.size(); i++) {
			diffoverallsupportsstream << diffOverallSupports[i] << " ";
		}
		
		std::stringstream output;
		output << decisionTier << "\t" << computationTimeSec << "\t" << tscorestream.str() << "\t" << gini << "\t" << overallSupport << "\t" << confidenceLevel << "\t" << difftscoresstream.str() << "\t" << diffoverallsupportsstream.str();
		
		logData.data = output.str();
		explanations_log_pub_.publish(logData);
	}
};

// Main file : Load configuration file

int main(int argc, char **argv) {

	//init the ROS node
	ros::init(argc, argv, "why");
	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
		ros::console::notifyLoggerLevelsChanged();
	}
	ros::NodeHandle nh;

	string path = ros::package::getPath("why");
	string text_config = path + "/config/text.conf";
	Explanation explain(nh);
	explain.initialize(text_config);
	ROS_INFO("Explanation Initialized");
	explain.run();

	return 0;
}
