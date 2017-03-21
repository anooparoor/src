/*
 * Position.cpp
 *
 *  Created on: Dec 26, 2008
 *      Author: richardmarcley
 */

#include "Position.h"
#include <stdio.h>
#include <math.h>


Position::Position(double x, double y, double theta) {
	this->x = x;
	this->y = y;
	this->theta = theta;
}

double Position::getDistance(Position other) {
	double dx = x - other.getX();
	double dy = y - other.getY();
	double distance = sqrt(dx * dx + dy * dy);
	return distance;
}

double Position::getDistance(double x1, double y1) {
	double dx = x - x1;
	double dy = y - y1;
	double distance = sqrt(dx * dx + dy * dy);
	return distance;
}

double Position::getX() const {
	return x;
}

void Position::setX(double x) {
	this->x = x;
}

double Position::getY() const {
	return y;
}

void Position::setY(double y) {
	this->y = y;
}

double Position::getTheta() const {
	return theta;
}

void Position::setTheta(double theta) {
	this->theta = theta;
}

bool Position::operator==(Position p){
	return (x == p.getX() && y == p.getY() && theta == p.getTheta());
}
