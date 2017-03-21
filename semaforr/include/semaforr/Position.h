/*
 * Position.h
 *
 *  Created on: Dec 26, 2008
 *      Author: richardmarcley
 */

#ifndef POSITION_H_
#define POSITION_H_

#include <string>
#include <math.h>
#include <vector>


using namespace std;

class Position {
public:
  Position () {
    Position (0,0,0);
  }
  Position (double x, double y, double theta);
  
  double getX() const;
  void setX(double x);
  double getY() const;
  void setY(double y);
  double getTheta() const;
  void setTheta(double theta);
  
  double getDistance(Position other);
  double getDistance(double x1, double y1);
  
  bool operator==(Position p);
private:
  double x;
  double y;
  double theta;
};

#endif /* POSITION_H_ */
