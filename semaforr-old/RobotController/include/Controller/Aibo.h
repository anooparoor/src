/*
 * Aibo.h
 *
 *  Created on: Jul 1, 2010
 *      Author: robotics
 */

#ifndef AIBO_H
#define AIBO_H_

#include "Localization.h"

class Aibo: public Localization {
public:
	Aibo(Map * map) : Localization(map, 59){
		setObservationVariance(.2);

		printf("Set motor enable\n");
		p2d->SetMotorEnable(true);
	}
};

#endif /* AIBO_H_ */
