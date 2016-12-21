/* 
 * File that will test implementation of FORRGeometry.
 * In line with TDD. Keep this test whenever you refactor the code
 * so you know it works as expected and you did not break anything
 * by your changes.
 *
 * Last modified: April 13, 2014.
 * Created by: Slavisa Djukic <sdjukic@hunter.cuny.edu>
 * compile with: g++ test_FORRGeometry.cpp -L../../../lib -I../include -lsemaFORR
 */

// This line tells Cathc to provide a main() - do this only in one cpp file
#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include "FORRGeometry.h"

// Tests for CartesianPoint
TEST_CASE("Test CartesianPoint class implementation"){
    CartesianPoint p1 = CartesianPoint(5, 8);
    CartesianPoint p2 = CartesianPoint();

    SECTION("We should be able to access coordinates of the point"){
        REQUIRE(p1.get_x() == 5);
        REQUIRE(p1.get_y() == 8);
        REQUIRE(p2.get_x() == 0);
        REQUIRE(p2.get_y() == 0);
    }
    SECTION("We should be able to change coordinates of the point"){
        p1.set_x(12.5);
        p1.set_y(13.7);
        REQUIRE(p1.get_x() == 12.5);
        REQUIRE(p1.get_y() == 13.7);
    }
    SECTION("We should be able to check whether two points are equal"){
	REQUIRE((CartesianPoint(3, 5) == CartesianPoint(5, 8)) == false);
	REQUIRE((CartesianPoint(7, 9) == CartesianPoint(7.0001, 8.99999)) == true);
	REQUIRE((CartesianPoint(12, -15) == CartesianPoint(12, -15)) == true);
    }
    SECTION("We should get the distance from two points"){
	// the order of arguments should not matter
	REQUIRE(distance(p1, p2) == distance(p2, p1));
	p1.set_x(10);
	p1.set_y(0);
	REQUIRE(distance(p1, p2) == 10);
    }
}
    
// Tests for Line
TEST_CASE("Test Line class implementation"){
	Line l1 = Line(0,0, 10, 10);
	CartesianPoint p1 = CartesianPoint(0, 5);
	CartesianPoint p2 = CartesianPoint(5, 0);
	Line l2 = Line(p1, p2);
    SECTION("Create line and access its members"){
	REQUIRE(l1.get_value_a() == 10);
	REQUIRE(l1.get_value_b() == -10);
	REQUIRE(l1.get_value_c() == 0);
	REQUIRE(l2.get_value_a() == -5);
	REQUIRE(l2.get_value_b() == -5);
	REQUIRE(l2.get_value_c() == -25);
    }
    SECTION("Change line parameters"){
	l2.set_value_a(21.4);
	l2.set_value_b(-11.8);
	l2.set_value_c(212.1);
	REQUIRE(l2.get_value_a() == 21.4);
	REQUIRE(l2.get_value_b() == -11.8);
	REQUIRE(l2.get_value_c() == 212.1);
    }
    SECTION("We should be able to get distance between point and line"){
	CartesianPoint p3 = CartesianPoint(1, -1);
	CartesianPoint p1 = CartesianPoint(0, 5);
	CartesianPoint p2 = CartesianPoint(5, 0);
	Line l2 = Line(p1, p2);
        REQUIRE(abs(distance(p3, l1) - sqrt(2)) < ERROR);
	REQUIRE(distance(p2, l2) == 0);
    }   
    SECTION("We should be able to check line intersection"){
	CartesianPoint intersection;

	Line l3 = Line(-2, 0, 5, 0);
	Line l4 = Line(5,3, -5, 3);
	REQUIRE(do_intersect(l1, l2, intersection) == true);
	REQUIRE(do_intersect(l3,l4, intersection) == false);
        // TODO check intersection point that gets returned from this function
    }
    SECTION("We should be able to check whether the point is on the line"){
	REQUIRE(is_point_on_line(p1, l2) == true);
	REQUIRE(is_point_on_line(p1, l1) == false);
	REQUIRE(is_point_on_line(CartesianPoint(4,1), l2) == true);
	REQUIRE(is_point_on_line(CartesianPoint(3,3), l1) == true);
    }
    SECTION("We should be able to get the slope of the line"){
	REQUIRE(l1.get_slope() == 1);
	REQUIRE(l2.get_slope() == -1);
	REQUIRE(Line(CartesianPoint(5,0) , CartesianPoint(-5,0)).get_slope() == 0);
	REQUIRE(Line(CartesianPoint(3,3), CartesianPoint(3, 12)).get_slope() == std::numeric_limits<double>::infinity());
	REQUIRE(Line(CartesianPoint(5,3), CartesianPoint(5, -12)).get_slope() == std::numeric_limits<double>::infinity());
    }
}

// Tests for LineSegment
TEST_CASE("Test functionality for LineSegment class"){
	LineSegment l1 = LineSegment(0,0, 10, 10);
	CartesianPoint p1 = CartesianPoint(0, 5);
	CartesianPoint p2 = CartesianPoint(5, 0);
	LineSegment l2 = LineSegment(p1, p2);
	LineSegment l3 = LineSegment(1,3,1,17);
	LineSegment l4 = LineSegment(0,0,0,5);
    SECTION("We should be able to check whether line segments intersect"){
        CartesianPoint intersect;
	REQUIRE(do_intersect(l1, l2, intersect) == true);
	REQUIRE(do_intersect(l1, l3, intersect) == false);
	REQUIRE(do_intersect(l1, l4, intersect) == true);
	REQUIRE(do_intersect(l4, l2, intersect) == true);
    }
    SECTION("We should be able to check the distance of the point from the line segment"){
        REQUIRE(abs(distance(p1, l1) - sqrt(12.5)) < ERROR);
	REQUIRE(distance(CartesianPoint(12, 10), l1) == 2);
	REQUIRE(distance(CartesianPoint(13, 13), l1) == sqrt(18));
	REQUIRE(distance(CartesianPoint(-1, -1), l1) == sqrt(2));
	REQUIRE(distance(CartesianPoint(1, 1), l1) == 0);
    }
    SECTION("We should be able to check if point is in the line segment"){
      	LineSegment lss4 = LineSegment(CartesianPoint(602,0), CartesianPoint(602,538));
	CartesianPoint p1 = CartesianPoint(602, 157.346);
	CartesianPoint p2 = CartesianPoint(602, 182);
	LineSegment lss3 = LineSegment(CartesianPoint(305.684, 254.75), CartesianPoint(1255.67, -57.5279));
	CartesianPoint p4 = CartesianPoint(205.0, 128.658);
	LineSegment lss1 = LineSegment(CartesianPoint(205.0, 0.0), CartesianPoint(205.0, 206.0));
	LineSegment lss2 = LineSegment(CartesianPoint(208.515, 130.039), CartesianPoint(-722.223, -235.65));
	LineSegment lss6 = LineSegment(CartesianPoint(317.224, 309.05), CartesianPoint(1313.89, 227.475));
	CartesianPoint p5 = CartesianPoint(602, 285.742);

	CartesianPoint inter;

	REQUIRE(is_point_in_segment(p1, lss4) == true);
	REQUIRE(is_point_in_segment(p2, lss4) == true);
	REQUIRE(is_point_in_segment(p1, lss3) == true);
	//REQUIRE(is_point_in_segment(p2, lss3) == true);
	REQUIRE(is_point_in_segment(p4, lss1) == true);
	REQUIRE(is_point_in_segment(p4, lss2) == true);
	REQUIRE(is_point_in_segment(p5, lss4) == true);
	// test whether these segments intersect
	REQUIRE(do_intersect(lss4, lss3, inter) == true);
	REQUIRE(do_intersect(lss1, lss2, inter) == true);
	REQUIRE(do_intersect(lss4, lss6, inter) == true);
	
    }
	
}
// Tests for Vector
TEST_CASE("Test Vector class implementation"){
     	Vector v1 = Vector(CartesianPoint(0,0), M_PI/4, 5);
	Vector v2 = Vector(CartesianPoint(0,5), -M_PI/4, 12);
	Vector v3 = Vector(CartesianPoint(-3,0), 13*M_PI/6, 1);
        Vector v4 = Vector(7, 7, -M_PI/3, 5);
	LineSegment ls1 = LineSegment(CartesianPoint(0,0), CartesianPoint(10,10));
	LineSegment ls2 = LineSegment(CartesianPoint(5,-10), CartesianPoint(5,15));

	Vector vs = Vector(CartesianPoint(214,175), M_PI, 1000);
	LineSegment lss1 = LineSegment(CartesianPoint(205,-10), CartesianPoint(205,216));
	Vector vs1 = Vector(CartesianPoint(213,144), M_PI, 1000);
	Vector vs2 = Vector(CartesianPoint(143, 216), -M_PI/1.19, 1000);
	LineSegment lss2 = LineSegment(CartesianPoint(95,206), CartesianPoint(215,206));
	
	LineSegment lss3 = LineSegment(CartesianPoint(305,254), CartesianPoint(1255,-57)) ;
	LineSegment lss4 = LineSegment(CartesianPoint(602,0), CartesianPoint(602,538));
	LineSegment lss5 = LineSegment(CartesianPoint(0,0), CartesianPoint(612,0));
	Vector vs3 = Vector(CartesianPoint(305,254), -M_PI/10.13, 1000);
	Vector vs4 = Vector(CartesianPoint(317.224, 309.05), -0.0818, 1000);

	CartesianPoint inters;

    SECTION("Test vector accessors"){
      REQUIRE((v1.get_origin() == CartesianPoint(0,0)) == true);
      REQUIRE(abs(v1.get_slope() - M_PI/4) < ERROR);
      REQUIRE(abs(v3.get_slope() - M_PI/6) < ERROR); // point is to show we can reduce any angle to -Pi Pi range
      REQUIRE(v2.get_intensity() == 12);
      REQUIRE((v4.get_origin() == CartesianPoint(7,7)) == true);
    }
    SECTION("Test vector and line segment intersection"){
      REQUIRE(do_intersect(vs, lss1, inters) == true);
      REQUIRE(do_intersect(vs1, lss1, inters) == true);
      REQUIRE(do_intersect(vs2, lss2, inters) == true);
      REQUIRE(do_intersect(lss3, lss4, inters) == true);
      REQUIRE(do_intersect(lss5, lss3, inters) == false);
      REQUIRE(do_intersect(vs3, lss4, inters) == true);
      REQUIRE(do_intersect(vs3, lss5, inters) == false);
      REQUIRE(do_intersect(vs4, lss4, inters) == true);
    } 
      
}
