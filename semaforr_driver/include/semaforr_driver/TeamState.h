/*!
 * TeamState.h
 *
 * \brief Manages information regarding all other robots (teammates)
 *
 * \author A. Tuna Ozgelen
 * \date 1/15/2014 Created
 */
#ifndef TEAMSTATE_H
#define TEAMSTATE_H

#include "Teammate.h"
#include "Position.h"
#include <vector>

class TeamState { 

 public:
  TeamState() {
    teammates_.clear();
  }
  
  /*! \brief updates the teammate collection 
   * 
   * This function should be called everytime a UPDATE_TEAMMATE message is received. 
   * It will first check if there is already a teammate with the session_id exists in the collection
   * if not it will create a new Teammate object otherwise will update the existing one. 
   */ 
  void updateTeammate(long id, double x, double y, double theta, int size);

  
  /*! \brief removes the teammate object from the collecion
   * 
   * If the teammate object with session_id = id is found and erased it will return true
   * otherwise false
   */
  bool removeTeammate(long id);


  /*! \brief returns true if there is a teammate robot with the session_id provided
   *         false otherwise.
   */
  bool isRobotTeammate(long id);


  /*! \brief wrapper for Teammate::isInCollisionCourse
   *
   * if the teammate with the id is not found this function returns false. 
   * From this returned value, it is impossible to tell the difference between a valid teammate that 
   * is not in collision course with the robot and an invalid teammate id. 
   * Therefore -> call isRobotTeammate() before this function
   * 
   * \sa isRobotTeammate
   */
  bool isTeammateInCollisionCourse(long id);


  //! \brief returns true if teammate is within a circle defined by a proximity threshold (collision zone)
  bool isTeammateInCollisionZone(long id); 


  //! \brief returns true if teammate is within a circle defined by a proximity + buffer (buffer zone)
  bool isTeammateInBufferZone(long id); 


  /*! \brief returns true if teammate is outside of the detection angle 
   *         which we call it behind but if the angle is very small this term becomes confusing 
   */
  bool isTeammateBehind(long id); 


  /*! \brief wrapper for Teammate::updateCollisionStatus
   * 
   * if the teammate id provided is valid it will compute the collision info for the teammate robot
   * otherwise will do nothing
   */ 
  void updateCollisionStatus(long id, Position p);


  //! \brief returns the current position of the teammate with session id passed as a param
  Position getPosition(long id); 


  //! \brief returns the previous position of the teammate with session id passed as a param
  Position getPreviousPosition(long id); 


  //! \brief returns the size of the teammate with session id passed as a param
  int getSize(long id); 

  //! \brief returns a vector of known teammate session ids
  std::vector<long> getTeammateIDs();
  // return the sum of distances of all teammates from a given position within a certain range
  //double getSumOfDistances(Position p, double range);

 private:
  
  //! collection pointers to known teammate robots
  std::vector<Teammate> teammates_;

  /*! \brief returns the pointer to the teammate with session_id = id. 
   *         if the object doesn't exists, returns a new teammate object pointer
   *         which has the session_id = -1
   */ 
  Teammate* getTeammate(long id);

};

#endif
