/*
 * Utils.h
 *
 *  Created on: Dec 23, 2008
 *      Author: George Rabanca
 *  Modified on : Jul 24, 2011
 *      Author: Tuna Ozgelen
 */

#ifndef UTILS_H_
#define UTILS_H_

#include <math.h>
#include <string>
#include <stdlib.h>
#include <iostream>
#include <climits>
#include <vector>
using namespace std;

#define PI acos((double)-1)
#define _E 2.7182

class Utils {
public:
  //transforms any radian value angle to (-PI, PI] angle.
  static double normalizeAngle(double alpha) {
    int quotent = (int) (alpha / (2 * PI));
    double remainder = alpha - quotent * 2 * PI;
    
    if (remainder <= -1 * PI)
      remainder += 2 * PI;
    if (remainder > PI)
      remainder -= 2 * PI;
    return remainder;
  }
  
  static double gaussian(double x, double mean, double variance) {
    double coef = 1 / sqrt(2 * PI * variance);
    double exponent = -1 * (x - mean) * (x - mean) / (2 * variance);
    double retVal = coef * pow(M_E, exponent);
    return retVal;
  }

  static double getMean(vector<double> dv){
    double acc = 0.0; 
    vector<double>::iterator iter; 
    for( iter = dv.begin(); iter != dv.end(); iter++ ){
      acc += (*iter); 
    }
    return ( acc / dv.size() ) ;
  }

  static double getStdDev(vector<double> dv, double mean) {
    return sqrt(getVariance(dv, mean));
  }

  static double getVariance(vector<double> dv, double mean) {
    double var = 0.0; 
    vector<double>::iterator iter; 
    for( iter = dv.begin(); iter != dv.end(); iter++ ){
      var += pow(((*iter)-mean),2); 
    }    
    return ( var / dv.size() ); 
  }
  
  static double applyRandomness(double x, double maxRandom) {
    return (x + maxRandom * getRandom(-1, 1));
  }
  
  //transforms an angle from radians to degrees
  static double toDegrees(double radians) {
    return 180 * radians / PI;
  }
  
  //transforms an angle from radians to degrees
  static double toRadians(double degrees) {
    return PI * degrees / 180;
  }
  
  static void initRandom() {
    srand(time(NULL));
  }
  
  static double getRandom(double lowerBound, double upperBound) {
    double intervalSize = upperBound - lowerBound;
    return (rand() * intervalSize / RAND_MAX + lowerBound);
  }
  
  static double getRandom() {
    return getRandom(0, 1);
  }
  
  static string getTime() {
    time_t timer = time(NULL);
    char * date = asctime(localtime(&timer));
    string date_s = string(date);
    return date_s;
  }
  
  // Returns 1 if the lines intersect, otherwise 0. In addition, if the lines 
  // intersect the intersection point may be stored in the double i_x and i_y.
  static bool get_line_intersection(double p0_x, double p0_y, double p1_x, double p1_y, 
			     double p2_x, double p2_y, double p3_x, double p3_y, double *i_x, double *i_y) 
  {
    double s1_x, s1_y, s2_x, s2_y;
    s1_x = p1_x - p0_x;     s1_y = p1_y - p0_y;
    s2_x = p3_x - p2_x;     s2_y = p3_y - p2_y;
    
    double s, t;
    s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) / (-s2_x * s1_y + s1_x * s2_y);
    t = ( s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / (-s2_x * s1_y + s1_x * s2_y);
    
    if (s >= 0 && s <= 1 && t >= 0 && t <= 1) {
        // Collision detected
        if (i_x != NULL)
	  *i_x = p0_x + (t * s1_x);
        if (i_y != NULL)
	  *i_y = p0_y + (t * s1_y);
	
        return true;
      }
    
    return false; // No collision
  }
  
  static double get_euclidian_distance(double x0, double y0, double x1, double y1){
    return ( sqrt( pow((x0 - x1),2) + pow((y0 - y1),2) ) ); 
  } 
  
  static int get_manhattan_distance(int x0, int y0, int x1, int y1){
    return ( abs(x1-x0) + abs(y1-y0) );
  }

  static bool isPointOnLine(double x, double y, double x1, double y1, double x2, double y2, double tolerance) {
    if ( isBetween(x, x1, x2, tolerance) && isBetween(y, y1, y2, tolerance) ){
      if ( (x2 - x1) == 0.0 )
	return true; 
      
      double slope = (y2 - y1) / (x2 - x1);
      double yintercept = y1 - slope * x1;
      
      return ( fabs( y - ( slope * x + yintercept )) <= tolerance );
    }
    return false; 
  }

  static bool isBetween(double a, double b, double c, double tolerance ){
    return (( a >= (b - tolerance) && a <= (c + tolerance) ) || 
	    ( a >= (c - tolerance) && a <= (b + tolerance) ));
  }

  static double calcHeading(double rx, double ry){
    // make sure the div by zero doesn't happen.
    if ( rx == 0 )
      rx = 0.01;
    
    double angle = atan(ry/rx);
    if ( rx < 0 && ry < 0 ) 
      angle = -1 * ( PI - angle ) ;
    if ( rx < 0 && ry > 0 )
      angle += PI ; 
    
    return angle; 
  }
};

#endif /* UTILS_H_ */
