/*
 * MCLPositionEstimator.h
 *
 *  Created on: Jan 21, 2009
 *  Author: George Rabanca
 */
#include <vector>
#include <map>
#include "Position.h"
#include "Particle.h"

#ifndef MCLPOSITIONESTIMATOR_H_
#define MCLPOSITIONESTIMATOR_H_

class MCLPositionEstimator {
public:
	MCLPositionEstimator(const int length, const int height);

	Position getPosition(vector<Particle>, double &confidence);

	static const int partitionNum = 10;
private:
	int cubeX;
	int cubeY;
	int cubeT;

	Position averagePositions(vector<Particle> particles);

	Position getBestK(vector<Particle>, double&, int); 
	Position getBestCluster(vector<Particle>, double&);

	friend class MCLPositionEstimatorTest;
};

#endif /* MCLPOSITIONESTIMATOR_H_ */
