/*
 * Surveyor.h
 *
 *  Created on: Jul 1, 2010
 *      Author: robotics
 */

#ifndef SURVEYOR_H_
#define SURVEYOR_H_

#include "Localization.h"

class Surveyor: public Localization {
public:
  Surveyor(Map * map) : Localization(map, 41){
    setObservationVariance(1);
  }
};

#endif /* SURVEYOR_H_ */
