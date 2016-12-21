/*!
 * Teammate.h
 *
 * \brief Represents the state of a teammate robot
 *
 * Note: this class is based on previous robotsProp.h used by the CollisionAvoider
 * 
 * \author A. Tuna Ozgelen
 * \date 1/15/2014 Created
 */

#ifndef TEAMMATE_H
#define TEAMMATE_H

// Radius
#define DETECTION_RADIUS 40
#define DETECTION_BUFFER 10

// Angle at which to watch out for collisions in degrees
#define FOV_ANGLE_DEGREES 160

#include <iostream>
#include "Position.h"
#include "Utils.h"
using namespace std;

class Teammate {
 public:
  Teammate(long id=-1, double x=0, double y=0, double theta=0, int size=0);


  bool operator==(Teammate p) const { return (p.getId() == this->id_); }


  bool operator<(Teammate p) const { return (p.getId() < this->id_); }

  
  //<! returns the session id of the teammate
  long getId() const { return id_; }


  //<! \brief returns the last reported position of the teammate
  Position getPosition() const { return curr_pos_; }


  //<! \brief updates the current position of the teammate with p, and the prev_pos 
  void setPosition(Position p) {
    prev_pos_ = curr_pos_;
    curr_pos_ = p;
  }


  //<! \brief returns the reported size of the teammate robot
  int getSize() const { return size_; }


  //<! \brief returns the prior position (one before the last reported position) of the teammate 
  Position getPreviousPosition() const { return prev_pos_; }


  /*! /brief returns true if the teammate robot is within collision zone and it is relatively in 
   *          front of the robot
   */
  bool isInCollisionCourse() { 
    return (proximity_ <= DETECTION_RADIUS) &&
      (this->relative_angle_ <= Utils::toRadians(FOV_ANGLE_DEGREES/2)); 
  }


  bool isInCollisionZone() { return (proximity_ <= DETECTION_RADIUS); }


  bool isInBufferZone() { return (proximity_ <= DETECTION_RADIUS + DETECTION_BUFFER); }


  bool isBehind() { return !(this->relative_angle_ <= Utils::toRadians(FOV_ANGLE_DEGREES/2)); }


  /*! \brief computes the proximity of our robot to this teammate robot and 
   *         its relative heading. 
   *
   *  It is called when we have a new position information for the teammate robot
   */
  void updateCollisionStatus(Position p);


  //<! \brief prints the status of the teammate for debugging purposes.
  void print() const;

 protected:
  
  //<! session_id of the teammate robot
  long id_;


  //<! size of the teammate robot 
  int size_;


  //<! last reported position of the teammate robot
  Position curr_pos_;


  //<! previous to the last reported position of the teammate robot
  Position prev_pos_;


  //<! euclidian distance between our robot and the teammate robot
  double proximity_;


  //<! the heading of the teammate robot from the perspective of our robot 
  double relative_angle_;

};

#endif  /* TEAMMATE_H */

