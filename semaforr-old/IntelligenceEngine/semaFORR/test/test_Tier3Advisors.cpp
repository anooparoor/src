/* 
 * File that will test implementation of DescriptiveManager.
 * In line with TDD. Keep this test whenever you refactor the code
 * so you know it works as expected and you did not break anything
 * by your changes.
 *
 * Last modified: April 14, 2014.
 * Created by: Slavisa Djukic <sdjukic@hunter.cuny.edu>
 *
 * Probabily will have to make library from object files.
 *
 * compile with: g++ test_Tier3Advisors.cpp -L ../../../lib -I ../include  -I ../../../include/ParticleFilter/ -I ../../../include/semaFORR/ -lsemaFORR
 */

// This line tells Cathc to provide a main() - do this only in one cpp file
#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include "Tier3Advisor.h"
#include <iostream>

using std::endl;
using std::cout;

const double ERROR = .005;
void test_CloseIn(vector<FORRActionType> actions, Tier3Advisor* my_advisor);
void test_Greedy(vector<FORRActionType> actions, Tier3Advisor* my_advisor);

TEST_CASE("Test Tier3Advisors"){

  Tier3Advisor *my_advisor;

  vector <FORRActionType> actions;
  actions.push_back(NOOP);
  actions.push_back(FORWARD);
  actions.push_back(BACKWARD);
  actions.push_back(RIGHT_TURN);
  actions.push_back(LEFT_TURN);
  actions.push_back(WIDE_RIGHT_TURN);
  actions.push_back(WIDE_LEFT_TURN);
  actions.push_back(PAUSE);
  actions.push_back(HALT);
  
  SECTION("Test Tier3Greedy"){
    //TODO read this from some test file
    string advisor_name = "Greedy";
    string advisor_description = "advisor that favors getting close to the target";
    double advisor_weight = .05;
    double auxiliary_constants[] = {0 ,0 ,0 ,0};
    
    my_advisor = Tier3Advisor::makeAdvisor(actions, advisor_name, advisor_description, advisor_weight, auxiliary_constants, true);

    test_Greedy(actions, my_advisor);
  }

 SECTION("Test Tier3AvoidObstacle"){
    //TODO read this from some test file
    string advisor_name = "ObstacleAvoid";
    string advisor_description = "advisor that wants to stay away from the obstacles";
    double advisor_weight = .05;
    double auxiliary_constants[] = {0 ,0 ,0 ,0};
    
    my_advisor = Tier3Advisor::makeAdvisor(actions, advisor_name, advisor_description, advisor_weight, auxiliary_constants, true);
    
    //this vector has 20 spaces
    for(int i = 0; i < 20; ++i)
      my_advisor->changeVector(i*10+5, i);
    
    FORRAction action = FORRAction(NOOP, 1);
    REQUIRE(my_advisor->actionComment(action) == 0);
    action = FORRAction(PAUSE, 0);
    REQUIRE(my_advisor->actionComment(action) == 0);
    action = FORRAction(HALT, 0);
    REQUIRE(my_advisor->actionComment(action) == 0);
    action = FORRAction(WIDE_RIGHT_TURN, 0);
    REQUIRE(my_advisor->actionComment(action) == -5);
    action = FORRAction(WIDE_LEFT_TURN, 0);
    REQUIRE(my_advisor->actionComment(action) == -5);
    //REQUIRE(my_advisor->actionComment(NOOP) == 0)
    //REQUIRE(my_advisor->actionComment(NOOP) == 0)
    //REQUIRE(my_advisor->actionComment(NOOP) == 0)
    
  }

 SECTION("Test Tier3CloseIn"){
    //TODO read this from some test file
    string advisor_name = "CloseIn";
    string advisor_description = "advisor that zeroes in on the target once robot is close enough";
    double advisor_weight = .05;
    double auxiliary_constants[] = {80 ,0 ,0 ,0};
    
    my_advisor = Tier3Advisor::makeAdvisor(actions, advisor_name, advisor_description, advisor_weight, auxiliary_constants, true);
    
    
    test_CloseIn(actions, my_advisor);
  }

 SECTION("Test Tier3BigStep"){
    //TODO read this from some test file
    string advisor_name = "BigStep";
    string advisor_description = "advisor that favors directions in which it can make big steps";
    double advisor_weight = .05;
    double auxiliary_constants[] = {0 ,0 ,0 ,0};
    
    my_advisor = Tier3Advisor::makeAdvisor(actions, advisor_name, advisor_description, advisor_weight, auxiliary_constants, true);
    
    //this vector has 20 spaces
    for(int i = 0; i < 20; ++i)
      my_advisor->changeVector(i*10+5, i);
    
    my_advisor->setDistanceToTarget(100);
    
    FORRAction action = FORRAction(NOOP, 1);
    CHECK(my_advisor->actionComment(action) == 0);
    action = FORRAction(PAUSE, 0);
    CHECK(my_advisor->actionComment(action) == 0);
    action = FORRAction(HALT, 0);
    CHECK(my_advisor->actionComment(action) == 0);
    action = FORRAction(WIDE_RIGHT_TURN, 0);
    CHECK(my_advisor->actionComment(action) == -5);
    action = FORRAction(WIDE_LEFT_TURN, 0);
    CHECK(my_advisor->actionComment(action) == -5);
    //CHECK(my_advisor->actionComment(NOOP) == 0)
    //CHECK(my_advisor->actionComment(NOOP) == 0)
    //CHECK(my_advisor->actionComment(NOOP) == 0)
    
  }
 
 SECTION("Test Tier3NotOpposite"){
    //TODO read this from some test file
    string advisor_name = "NotOpposite";
    string advisor_description = "advisor that does not want to make rotation opposite of previouse one";
    double advisor_weight = .05;
    double auxiliary_constants[] = {0 ,0 ,0 ,0};
    
    my_advisor = Tier3Advisor::makeAdvisor(actions, advisor_name, advisor_description, advisor_weight, auxiliary_constants, true);
    
    //this vector has 20 spaces
    for(int i = 0; i < 20; ++i)
      my_advisor->changeVector(i*10+5, i);
    
    my_advisor->setDistanceToTarget(100);
    
    FORRAction action = FORRAction(NOOP, 1);
    CHECK(my_advisor->actionComment(action) == 0);
    action = FORRAction(PAUSE, 0);
    CHECK(my_advisor->actionComment(action) == 0);
    action = FORRAction(HALT, 0);
    CHECK(my_advisor->actionComment(action) == 0);
    action = FORRAction(WIDE_RIGHT_TURN, 0);
    CHECK(my_advisor->actionComment(action) == -5);
    action = FORRAction(WIDE_LEFT_TURN, 0);
    CHECK(my_advisor->actionComment(action) == -5);
    //CHECK(my_advisor->actionComment(NOOP) == 0)
    //CHECK(my_advisor->actionComment(NOOP) == 0)
    //CHECK(my_advisor->actionComment(NOOP) == 0)
    
  }

 SECTION("Test Tier3Bold"){
    //TODO read this from some test file
    string advisor_name = "Bold";
    string advisor_description = "advisor that favors making long steps and big rotations";
    double advisor_weight = .05;
    double auxiliary_constants[] = {0 ,0 ,0 ,0};
    
    my_advisor = Tier3Advisor::makeAdvisor(actions, advisor_name, advisor_description, advisor_weight, auxiliary_constants, true);
    
    //this vector has 20 spaces
    for(int i = 0; i < 20; ++i)
      my_advisor->changeVector(i*10+5, i);
    
    my_advisor->setDistanceToTarget(100);
    
    FORRAction action = FORRAction(NOOP, 1);
    CHECK(my_advisor->actionComment(action) == 0);
    action = FORRAction(PAUSE, 0);
    CHECK(my_advisor->actionComment(action) == 0);
    action = FORRAction(HALT, 0);
    CHECK(my_advisor->actionComment(action) == 0);
    action = FORRAction(WIDE_RIGHT_TURN, 0);
    CHECK(my_advisor->actionComment(action) == -5);
    action = FORRAction(WIDE_LEFT_TURN, 0);
    CHECK(my_advisor->actionComment(action) == -5);
    //CHECK(my_advisor->actionComment(NOOP) == 0)
    //CHECK(my_advisor->actionComment(NOOP) == 0)
    //CHECK(my_advisor->actionComment(NOOP) == 0)
    
  }

 SECTION("Test Tier3Ordinary"){
    //TODO read this from some test file
    string advisor_name = "Ordinary";
    string advisor_description = "advisor that favors making middle steps and rotations";
    double advisor_weight = .05;
    double auxiliary_constants[] = {0 ,0 ,0 ,0};
    
    my_advisor = Tier3Advisor::makeAdvisor(actions, advisor_name, advisor_description, advisor_weight, auxiliary_constants, true);
    
    //this vector has 20 spaces
    for(int i = 0; i < 20; ++i)
      my_advisor->changeVector(i*10+5, i);
    
    my_advisor->setDistanceToTarget(100);
    
    FORRAction action = FORRAction(NOOP, 1);
    CHECK(my_advisor->actionComment(action) == 0);
    action = FORRAction(PAUSE, 0);
    CHECK(my_advisor->actionComment(action) == 0);
    action = FORRAction(HALT, 0);
    CHECK(my_advisor->actionComment(action) == 0);
    action = FORRAction(WIDE_RIGHT_TURN, 0);
    CHECK(my_advisor->actionComment(action) == -5);
    action = FORRAction(WIDE_LEFT_TURN, 0);
    CHECK(my_advisor->actionComment(action) == -5);
    //CHECK(my_advisor->actionComment(NOOP) == 0)
    //CHECK(my_advisor->actionComment(NOOP) == 0)
    //CHECK(my_advisor->actionComment(NOOP) == 0)
    
  }

 SECTION("Test Tier3Cautious"){
    //TODO read this from some test file
    string advisor_name = "Cautious";
    string advisor_description = "advisor that favors making smallest steps and rotations";
    double advisor_weight = .05;
    double auxiliary_constants[] = {0 ,0 ,0 ,0};
    
    my_advisor = Tier3Advisor::makeAdvisor(actions, advisor_name, advisor_description, advisor_weight, auxiliary_constants, true);
    
    //this vector has 20 spaces
    for(int i = 0; i < 20; ++i)
      my_advisor->changeVector(i*10+5, i);
    
    my_advisor->setDistanceToTarget(100);
    
    FORRAction action = FORRAction(NOOP, 1);
    CHECK(my_advisor->actionComment(action) == 0);
    action = FORRAction(PAUSE, 0);
    CHECK(my_advisor->actionComment(action) == 0);
    action = FORRAction(HALT, 0);
    CHECK(my_advisor->actionComment(action) == 0);
    action = FORRAction(WIDE_RIGHT_TURN, 0);
    CHECK(my_advisor->actionComment(action) == -5);
    action = FORRAction(WIDE_LEFT_TURN, 0);
    CHECK(my_advisor->actionComment(action) == -5);
    //CHECK(my_advisor->actionComment(NOOP) == 0)
    //CHECK(my_advisor->actionComment(NOOP) == 0)
    //CHECK(my_advisor->actionComment(NOOP) == 0)
    
  }

 /*
 NOTE: At the moment this advisor is not implemented
 SECTION("Test Tier3Cramped"){
    //TODO read this from some test file
    string advisor_name = "Cramped";
    string advisor_description = "advisor that favors making small steps because it is crowded";
    double advisor_weight = .05;
    double auxiliary_constants[] = {0 ,0 ,0 ,0};
    
    my_advisor = Tier3Advisor::makeAdvisor(actions, advisor_name, advisor_description, advisor_weight, auxiliary_constants, true);
    
    //this vector has 20 spaces
    for(int i = 0; i < 20; ++i)
      my_advisor->changeVector(i*10+5, i);
    
    my_advisor->setDistanceToTarget(100);
    
    FORRAction action = FORRAction(NOOP, 1);
    CHECK(my_advisor->actionComment(action) == 0);
    action = FORRAction(PAUSE, 0);
    CHECK(my_advisor->actionComment(action) == 0);
    action = FORRAction(HALT, 0);
    CHECK(my_advisor->actionComment(action) == 0);
    action = FORRAction(WIDE_RIGHT_TURN, 0);
    CHECK(my_advisor->actionComment(action) == -5);
    action = FORRAction(WIDE_LEFT_TURN, 0);
    CHECK(my_advisor->actionComment(action) == -5);
    //CHECK(my_advisor->actionComment(NOOP) == 0)
    //CHECK(my_advisor->actionComment(NOOP) == 0)
    //CHECK(my_advisor->actionComment(NOOP) == 0)
    
  }
 */

 /* 
 NOTE: At the moment this advisor is not implemented
 SECTION("Test Tier3FreeToBe"){
    //TODO read this from some test file
    string advisor_name = "FreeToBe";
    string advisor_description = "advisor that favors making big steps because it has free space";
    double advisor_weight = .05;
    double auxiliary_constants[] = {0 ,0 ,0 ,0};
    
    my_advisor = Tier3Advisor::makeAdvisor(actions, advisor_name, advisor_description, advisor_weight, auxiliary_constants, true);
    
    //this vector has 20 spaces
    for(int i = 0; i < 20; ++i)
      my_advisor->changeVector(i*10+5, i);
    
    my_advisor->setDistanceToTarget(100);
    
    
    FORRAction action = FORRAction(NOOP, 1);
    CHECK(my_advisor->actionComment(action) == 0);
    action = FORRAction(PAUSE, 0);
    CHECK(my_advisor->actionComment(action) == 0);
    action = FORRAction(HALT, 0);
    CHECK(my_advisor->actionComment(action) == 0);
    action = FORRAction(WIDE_RIGHT_TURN, 0);
    CHECK(my_advisor->actionComment(action) == 0);
    action = FORRAction(WIDE_LEFT_TURN, 0);
    CHECK(my_advisor->actionComment(action) == 0);
    //CHECK(my_advisor->actionComment(NOOP) == 0)
    //CHECK(my_advisor->actionComment(NOOP) == 0)
    //CHECK(my_advisor->actionComment(NOOP) == 0)
    
  }
 */

 SECTION("Test Tier3GoAround"){
    //TODO read this from some test file
    string advisor_name = "GoAround";
    string advisor_description = "advisor that favors making sharper turns the closer it gets to the obstacle";
    double advisor_weight = .05;
    double auxiliary_constants[] = {0 ,0 ,0 ,0};
    
    my_advisor = Tier3Advisor::makeAdvisor(actions, advisor_name, advisor_description, advisor_weight, auxiliary_constants, true);
    
    //this vector has 20 spaces
    for(int i = 0; i < 20; ++i)
      my_advisor->changeVector(i*10+5, i);
    
    my_advisor->setDistanceToTarget(100);
    
    FORRAction action = FORRAction(NOOP, 1);
    CHECK(my_advisor->actionComment(action) == 0);
    action = FORRAction(PAUSE, 0);
    CHECK(my_advisor->actionComment(action) == 0);
    action = FORRAction(HALT, 0);
    CHECK(my_advisor->actionComment(action) == 0);
    action = FORRAction(WIDE_RIGHT_TURN, 0);
    CHECK(my_advisor->actionComment(action) == -5);
    action = FORRAction(WIDE_LEFT_TURN, 0);
    CHECK(my_advisor->actionComment(action) == -5);
    //CHECK(my_advisor->actionComment(NOOP) == 0)
    //CHECK(my_advisor->actionComment(NOOP) == 0)
    //CHECK(my_advisor->actionComment(NOOP) == 0)
    
  }

 SECTION("Test Tier3FuzzyGreedy"){
    //TODO read this from some test file
    string advisor_name = "FuzzyGreedy";
    string advisor_description = "advisor that favors making steps in general direction of the target";
    double advisor_weight = .05;
    double auxiliary_constants[] = {0 ,0 ,0 ,0};
    
    my_advisor = Tier3Advisor::makeAdvisor(actions, advisor_name, advisor_description, advisor_weight, auxiliary_constants, true);
    
    //this vector has 20 spaces
    for(int i = 0; i < 20; ++i)
      my_advisor->changeVector(i*10+5, i);
    
    my_advisor->setDistanceToTarget(100);
    
    FORRAction action = FORRAction(NOOP, 1);
    CHECK(my_advisor->actionComment(action) == 0);
    action = FORRAction(PAUSE, 0);
    CHECK(my_advisor->actionComment(action) == 0);
    action = FORRAction(HALT, 0);
    CHECK(my_advisor->actionComment(action) == 0);
    action = FORRAction(WIDE_RIGHT_TURN, 0);
    CHECK(my_advisor->actionComment(action) == -5);
    action = FORRAction(WIDE_LEFT_TURN, 0);
    CHECK(my_advisor->actionComment(action) == -5);
    //CHECK(my_advisor->actionComment(NOOP) == 0)
    //CHECK(my_advisor->actionComment(NOOP) == 0)
    //CHECK(my_advisor->actionComment(NOOP) == 0)
    
  }

 SECTION("Test Tier3Explorer"){
    //TODO read this from some test file
    string advisor_name = "Explorer";
    string advisor_description = "advisor that favors visiting squares that have not been visited";
    double advisor_weight = .05;
    double auxiliary_constants[] = {0 ,0 ,0 ,0};
    
    my_advisor = Tier3Advisor::makeAdvisor(actions, advisor_name, advisor_description, advisor_weight, auxiliary_constants, true);
    
    //this vector has 20 spaces
    for(int i = 0; i < 20; ++i)
      my_advisor->changeVector(i*10+5, i);
    
    my_advisor->setDistanceToTarget(100);
    
    FORRAction action = FORRAction(NOOP, 1);
    CHECK(my_advisor->actionComment(action) == 0);
    action = FORRAction(PAUSE, 0);
    CHECK(my_advisor->actionComment(action) == 0);
    action = FORRAction(HALT, 0);
    CHECK(my_advisor->actionComment(action) == 0);
    action = FORRAction(WIDE_RIGHT_TURN, 0);
    CHECK(my_advisor->actionComment(action) == -5);
    action = FORRAction(WIDE_LEFT_TURN, 0);
    CHECK(my_advisor->actionComment(action) == -5);
    //CHECK(my_advisor->actionComment(NOOP) == 0)
    //CHECK(my_advisor->actionComment(NOOP) == 0)
    //CHECK(my_advisor->actionComment(NOOP) == 0)
    
  }

}

void test_CloseIn(vector<FORRActionType> actions, Tier3Advisor* my_advisor){
  // but this advisor uses only first 5 places of this vector
  // 0 - robot x position
  // 1 - robot y position
  // 2 - robot yaw
  // 3 - target x position
  // 4 - target y position
 

  my_advisor->setDistanceToTarget(30);
  my_advisor->set_commenting(); // call this method to update commenting to this distance 
  // advisor should be active at this distance to the target
  CHECK(my_advisor->is_commenting() == true);
  my_advisor->setDistanceToTarget(75); 
  my_advisor->set_commenting();
  // advisor should be active at this distance to the target
  CHECK(my_advisor->is_commenting() == true);
  my_advisor->setDistanceToTarget(90);
  my_advisor->set_commenting(); 
  // advisor should not be active at this distance to the target
  CHECK(my_advisor->is_commenting() == false);
  my_advisor->setDistanceToTarget(150);
  my_advisor->set_commenting(); 
  // advisor should not be active at this distance to the target
  CHECK(my_advisor->is_commenting() == false);
  my_advisor->setDistanceToTarget(550);
  my_advisor->set_commenting(); 
  // advisor should not be active at this distance to the target
  CHECK(my_advisor->is_commenting() == false);
 

 double first_setup[] = {50, 50, 0, 100, 50}; 
  for(int i = 0; i < 5; ++i)
    my_advisor->changeVector(first_setup[i], i);
  
  my_advisor->setDistanceToTarget(50);
  
  for(int i = 0; i < 9; ++i){
    for(int j = 1; j < 6; ++j){
      
      FORRAction action = FORRAction(actions[i],j);
      if(i == 0)
	REQUIRE(my_advisor->actionComment(action) == 0);
      if(i == 1){
	switch(j){
	case 1:
	  CHECK(abs(my_advisor->actionComment(action) - .3) < ERROR);
	  break;
	case 2:
	  CHECK(abs(my_advisor->actionComment(action) - .7) < ERROR);
	  break;
	case 3:
	  CHECK(abs(my_advisor->actionComment(action) - 2) < ERROR);
	    break;
	case 4:
	  CHECK(abs(my_advisor->actionComment(action) - 2.5) < ERROR);
	  break;
	case 5:
	  REQUIRE(abs(my_advisor->actionComment(action) - (-5)) < ERROR);
	  break;
	}
      }
      // this part is not implemented moving backward
      /*
	if(i == 2){
	switch(j){
	  case 0:
	  CHECK(my_advisor->actionComment(action) == 0.3);
	  break;
	  case 1:
	  CHECK(abs(my_advisor->actionComment(action) - (- .3)) < ERROR);
	  break;
	  case 2:
	  CHECK(abs(my_advisor->actionComment(action) - (- .7)) < ERROR);
	  break;
	  case 3:
	  CHECK(abs(my_advisor->actionComment(action) - (- 2)) < ERROR);
	    break;
	    case 4:
	    CHECK(abs(my_advisor->actionComment(action) - (- 2.5)) < ERROR);
	    break;
	    }
	    }
      */
      if(i == 3){
	switch(j){
	case 1:
	  CHECK(abs(my_advisor->actionComment(action) - (-5)) < ERROR);
	  break;
	case 2:
	  CHECK(abs(my_advisor->actionComment(action) - (-5)) < ERROR);
	  break;
	case 3:
	  CHECK(abs(my_advisor->actionComment(action) - (-5)) < ERROR);
	  break;
	case 4:
	  CHECK(abs(my_advisor->actionComment(action) - (-5)) < ERROR);
	  break;
	case 5:
	  CHECK(abs(my_advisor->actionComment(action) - (-5)) < ERROR);
	  break;
	}
      }
      if(i == 4){
	switch(j){
	case 1:
	  CHECK(abs(my_advisor->actionComment(action) - (-5)) < ERROR);
	  break;
	case 2:
	  CHECK(abs(my_advisor->actionComment(action) - (-5)) < ERROR);
	  break;
	case 3:
	  CHECK(abs(my_advisor->actionComment(action) - (-5)) < ERROR);
	  break;
	case 4:
	  CHECK(abs(my_advisor->actionComment(action) - (-5)) < ERROR);
	  break;
	case 5:
	  CHECK(abs(my_advisor->actionComment(action) - (-5)) < ERROR);
	  break;
	}
      } 
      if(i == 5)
	CHECK(my_advisor->actionComment(action) == -5);
      if(i == 6)
	CHECK(my_advisor->actionComment(action) == -5);
      if(i == 7)
	CHECK(my_advisor->actionComment(action) == 0);
      if(i == 8)
	CHECK(my_advisor->actionComment(action) == 0);
    }
  }

  double second_setup[] = {50, 50, 0, 100, 100}; // at 45 degree to left 
  for(int i = 0; i < 5; ++i)
    my_advisor->changeVector(second_setup[i], i);
  
  my_advisor->setDistanceToTarget(sqrt(2500+2500));
  
  for(int i = 0; i < 9; ++i){
    for(int j = 1; j < 6; ++j){
      
      FORRAction action = FORRAction(actions[i],j);
      if(i == 0)
	REQUIRE(my_advisor->actionComment(action) == 0);
      if(i == 1){
	switch(j){
	case 1:
	  CHECK(abs(my_advisor->actionComment(action) - .15) < ERROR);
	  break;
	case 2:
	  CHECK(abs(my_advisor->actionComment(action) - .35) < ERROR);
	  break;
	case 3:
	  CHECK(abs(my_advisor->actionComment(action) - 1) < ERROR);
	    break;
	case 4:
	  CHECK(abs(my_advisor->actionComment(action) - 1.25) < ERROR);
	  break;
	case 5:
	  REQUIRE(abs(my_advisor->actionComment(action) - (-.25)) < ERROR);
	  break;
	}
      }
      // this part is not implemented moving backward
      /*
	if(i == 2){
	switch(j){
	  case 0:
	  CHECK(my_advisor->actionComment(action) == 0.3);
	  break;
	  case 1:
	  CHECK(abs(my_advisor->actionComment(action) - (- .3)) < ERROR);
	  break;
	  case 2:
	  CHECK(abs(my_advisor->actionComment(action) - (- .7)) < ERROR);
	  break;
	  case 3:
	  CHECK(abs(my_advisor->actionComment(action) - (- 2)) < ERROR);
	    break;
	    case 4:
	    CHECK(abs(my_advisor->actionComment(action) - (- 2.5)) < ERROR);
	    break;
	    }
	    }
      */
      if(i == 3){
	switch(j){
	case 1:
	  CHECK(abs(my_advisor->actionComment(action) - (-.985)) < ERROR);
	  break;
	case 2:
	  CHECK(abs(my_advisor->actionComment(action) - (-1.94)) < ERROR);
	  break;
	case 3:
	  CHECK(abs(my_advisor->actionComment(action) - (-4.138)) < ERROR);
	  break;
	case 4:
	  CHECK(abs(my_advisor->actionComment(action) - (-5)) < ERROR);
	  break;
	case 5:
	  CHECK(abs(my_advisor->actionComment(action) - (-5)) < ERROR);
	  break;
	}
      }
      if(i == 4){
	switch(j){
	case 1:
	  CHECK(abs(my_advisor->actionComment(action) - .985) < ERROR);
	  break;
	case 2:
	  CHECK(abs(my_advisor->actionComment(action) - 1.94) < ERROR);
	  break;
	case 3:
	  CHECK(abs(my_advisor->actionComment(action) - 4.138) < ERROR);
	  break;
	case 4:
	  CHECK(abs(my_advisor->actionComment(action) - (-3.276)) < ERROR);
	  break;
	case 5:
	  CHECK(abs(my_advisor->actionComment(action) - (-5)) < ERROR);
	  break;
	}
      } 
      if(i == 5)
	CHECK(my_advisor->actionComment(action) == -5);
      if(i == 6)
	CHECK(my_advisor->actionComment(action) == -5);
      if(i == 7)
	CHECK(my_advisor->actionComment(action) == 0);
      if(i == 8)
	CHECK(my_advisor->actionComment(action) == 0);
    }
  }
  
}

void test_Greedy(vector<FORRActionType> actions, Tier3Advisor* my_advisor){
  // Here we are not interested in exact values that advisor produces
  // only that values correspond to the best action
  // i.e. if target is right in front of robot move forward should get the
  // highest vote
  double first_setup[] = {65, 
    for(int i = 0; i < 20; ++i)
      my_advisor->changeVector(i*10+5, i);
    
    my_advisor->setDistanceToTarget(100);
    
    FORRAction action = FORRAction(NOOP, 1);
    REQUIRE(my_advisor->actionComment(action) == -5);
    action = FORRAction(PAUSE, 0);
    REQUIRE(my_advisor->actionComment(action) == -5);
    action = FORRAction(HALT, 0);
    REQUIRE(my_advisor->actionComment(action) == -5);
    action = FORRAction(WIDE_RIGHT_TURN, 0);
    REQUIRE(my_advisor->actionComment(action) == -5);
    action = FORRAction(WIDE_LEFT_TURN, 0);
    REQUIRE(my_advisor->actionComment(action) == -5);
    //REQUIRE(my_advisor->actionComment(NOOP) == 0)
    //REQUIRE(my_advisor->actionComment(NOOP) == 0)
    //REQUIRE(my_advisor->actionComment(NOOP) == 0)
    
}
