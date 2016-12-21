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
 * compile with: g++ test_DescriptiveManager.cpp -I ../include/ -I ../../../Utils/MetroCommunication/include/ -I ../../../Utils/MetroTimer/include/ -I ../../../Utils/semaFORR/include/ -I ../../../Utils/ParticleFilter/include/ -I ../../RSWL/include/ -L ../../../Utils/MetroCommunication/lib/ -L ../../../Utils/MetroTimer/lib/ -L ../../../Utils/semaFORR/lib/ -L ../../../Utils/ParticleFilter/lib/ -L ../../RSWL/lib/ -L ../lib/ -lDManager -lMetroCommunication -lboost_system -lboost_thread -lboost_filesystem -lglut -lGLU -lMetroTimer -lsemaFORR -lParticleFilter -lrswl -lpthread
 */

// This line tells Cathc to provide a main() - do this only in one cpp file
#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include "DescriptiveManager.h"
#include <iostream>

using std::endl;
using std::cout;

// Tests for CartesianPoint
TEST_CASE("Test DescriptiveManager implementation"){
    CartesianPoint p1 = CartesianPoint(450, 80);
    CartesianPoint p2 = CartesianPoint(150, 200);
    // comment to myself map is defined as height, width and buffer size
    Map my_map = Map(500, 600, 20);

    // so have to fake CS and get some robot and its position 
    // also
    SECTION(""){
      vector<CartesianPoint> targets;
      targets.push_back(p1);
      targets.push_back(p2);
      DescriptiveManager my_manager = DescriptiveManager(my_map, targets);
      CartesianPoint a_target = my_manager.get_target();
      cout << "A target is " << a_target.get_x() << ", " << a_target.get_y() << endl;
      
    }
}
