/*!
 * TeamState.cpp
 *
 */
#include "TeamState.h"

void TeamState::updateTeammate(long id =-1, double x=0, double y=0, double theta=0, int size=0) { 
  if(!isRobotTeammate(id)) {
    Teammate t(id, x, y, theta, size); 
    teammates_.push_back(t); 
  }
  else {
    Teammate * t = getTeammate(id);
    Position pose(x, y, theta); 
    t->setPosition(pose); 
  }
}

//computes the sum of the distances of the teammates from a given position (x,y) within a given RANGE
/*
double TeamState::getSumOfDistances(Position p, double range){
  vector<long> teammateIds = getTeammateIDs();
  double distance = 0;
  for(int i = 0; i < teammateIds.size(); i++){
    Position teammatePosition = getPosition(teammateIds[i]);
    double teammateDistance = p.getDistance(teammatePosition);
    if(teammateDistance < range)
      distance += teammateDistance;
  }
  return distance;
}
*/

bool TeamState::removeTeammate(long id) {
  std::vector<Teammate>::iterator iter; 
  for (iter = teammates_.begin(); iter!= teammates_.end(); iter++) {
    if(iter->getId() == id) {
      teammates_.erase(iter); 
      return true; 
    }
  }
  return false;
} 


bool TeamState::isRobotTeammate(long id) {
  std::vector<Teammate>::iterator iter; 
  for (iter = teammates_.begin(); iter!= teammates_.end(); iter++) {
    if(iter->getId() == id) {
      return true;
    }
  }
  return false;
}


bool TeamState::isTeammateInCollisionCourse(long id) {
  Teammate * t = getTeammate(id); 
  if(t->getId() != -1) 
    return t->isInCollisionCourse();
  return false; 
}


bool TeamState::isTeammateInCollisionZone(long id) {
  Teammate * t = getTeammate(id); 
  if(t->getId() != -1) 
    return t->isInCollisionZone();
  return false; 
}


bool TeamState::isTeammateInBufferZone(long id) {
  Teammate * t = getTeammate(id); 
  if(t->getId() != -1) 
    return t->isInBufferZone();
  return false; 
}


bool TeamState::isTeammateBehind(long id) {
  Teammate * t = getTeammate(id); 
  if(t->getId() != -1) 
    return t->isBehind();
  return false; 
}


Teammate* TeamState::getTeammate(long id) {
  std::vector<Teammate>::iterator iter; 
  for (iter = teammates_.begin(); iter != teammates_.end(); iter++) {
    if(iter->getId() == id) {
      return &(*iter);
    }
  }
  return new Teammate();
}

void TeamState::updateCollisionStatus(long id, Position p) {
  Teammate * t = getTeammate(id);
  if(t->getId() != -1)
    t->updateCollisionStatus(p); 
}



Position TeamState::getPosition(long id) {
  Teammate * t = getTeammate(id);
  if(t->getId() != -1)
    return t->getPosition(); 
  Position p(-1,-1,-1); 
  return p;
}



Position TeamState::getPreviousPosition(long id) {
  Teammate * t = getTeammate(id);
  if(t->getId() != -1)
    return t->getPreviousPosition(); 
  Position p(-1,-1,-1); 
  return p;
}


int TeamState::getSize(long id) { 
  Teammate * t = getTeammate(id);
  if(t->getId() != -1)
    return t->getSize();
  return 0;
}

std::vector<long> TeamState::getTeammateIDs() { 
  std::vector<long> tmate_ids; 
  std::vector<Teammate>::iterator iter; 
  for(iter = teammates_.begin(); iter != teammates_.end(); iter++)
    tmate_ids.push_back(iter->getId()); 
  return tmate_ids; 
}
