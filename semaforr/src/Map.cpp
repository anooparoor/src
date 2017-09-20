/*
 * Map.cpp
 *
 *  Created on: June 17, 2017
 *      Author: Anoop Aroor
 */


#include "Map.h"
#include <iostream>
using namespace std;

Map::Map(){}

// in cms
Map::Map(double length, double height) {
  this->length = length;
  this->height = height;
  cout << length  << " " << height << " " << length/50 << " " << height/50 << endl;  
  factor = 50;
  for(int j = 0 ; j <= length/factor; j++){
	vector<bool> column;
  	for(int i = 0 ; i <= height/factor; i++){
		column.push_back(false);
  	}
	occupancyGrid.push_back(column);
  }
}

void Map::addWall(double x1, double y1, double x2, double y2){
  Wall wall;
  wall.x1 = x1;
  wall.x2 = x2;
  wall.y1 = y1;
  wall.y2 = y2;
  walls.push_back(wall);
  double distance = Map::distance(x1,y1,x2,y2);
  double stepSize = 2; //cms
  cout << "Wall : " << x1 << " " << y1 << " " << x2 << " " << y2 << " " << endl;
  for(int step = -5; step <= distance+5; step += stepSize ){
	double t = step/distance;
  	double xtest = (x1 * t) + ((1-t)*x2);
  	double ytest = (y1 * t) + ((1-t)*y2);
	int gridx = (int)(xtest / factor);
	int gridy = (int)(ytest / factor);
	if(gridx >= 0 && gridy >= 0 && gridx <= length/factor && gridy <= height/factor){
		//cout << "Test : " << xtest << " " << ytest << " " << gridx << " " << gridy << " " << endl;
		occupancyGrid[gridx][gridy] = true;
	}
  }
  occupancyGrid[(int)(x2/factor)][(int)(y2/factor)] = true;
}


//read xml file and add walls in cms
bool Map::readMapFromXML(string filename){
	const char * c = filename.c_str();
	TiXmlDocument xml(c);
	bool valid = xml.LoadFile();
	TiXmlHandle hDoc(&xml);
	TiXmlElement* obstacleNode;
	TiXmlElement* vertexNode;
	TiXmlHandle hObstacle(0);
	TiXmlHandle hRoot(0);
	if ( !valid ) {	// load xml file
		cout << "Could not load map xml " << filename << ".\n";
		return false;
	}

	TiXmlElement* rootNode = xml.RootElement();	
	if( ! rootNode ) {
		cout << "Root element does not exist\n.";
		return false;
	}
	hRoot = TiXmlHandle(rootNode);

	obstacleNode = hRoot.FirstChild("ObstacleSet").FirstChild("Obstacle").Element();
	for( obstacleNode; obstacleNode; obstacleNode=obstacleNode->NextSiblingElement()){
		hObstacle = TiXmlHandle(obstacleNode);
		vertexNode = hObstacle.FirstChild("Vertex").Element();
		double x1 = atof(vertexNode->Attribute("p_x")) * 100;// to convert from m to cms
		double y1 = atof(vertexNode->Attribute("p_y")) * 100;
		vertexNode = vertexNode->NextSiblingElement();
		double x2 = atof(vertexNode->Attribute("p_x")) * 100;
		double y2 = atof(vertexNode->Attribute("p_y")) * 100;
		addWall(x1,y1,x2,y2);
		//cout << "Adding wall ("<< x1 <<"," << y1<<")->("<<x2 <<"," << y2<<")"<<endl;		
	}
}

bool Map::isWithinBorders(double x, double y){
  if ( x >= 0 && x < length && y >= 0 && y < height ) 
    return true;
  return false; 
}


/*! 
  \brief This function returns true if a point is within a Wall buffer, meaning too close to a wall.
*/
bool Map::isPointInBuffer(double x, double y){ 
  //cout << "is point in buffer " << x << " " << y << endl;
  int gridx = (int)(x/factor);
  int gridy = (int)(y/factor);
  return occupancyGrid[gridx][gridy];
  /*for(int i = gridx - 1; i <= gridx + 1; i++){
	for(int j = gridy - 1; j <= gridx + 1; j++){
		if(i >= 0 and j >= 0 and i <= height/10 and j <= length/10){
			if(occupancyGrid[i][j] == true){
				return true;		
			}
		}
	}
  }   
  return false;
  */
}

bool Map::isPathObstructed(double x0, double y0, double x1, double y1 ){
      //cout << "In path obstructed " << x0 << " " << y0 << "-" << x1 << " " << y1 << endl;
      double stepSize = 5; //cms
      double distance = Map::distance(x0,y0,x1,y1);
      if(Map::isPointInBuffer(x0,y0)) return true;
      if(Map::isPointInBuffer(x1,y1)) return true;
      if(distance > (stepSize * 2)){
      	for(int step = 0; step <= distance; step += stepSize ){
		double t = step/distance;
  		double xtest = (x0 * t) + ((1-t)*x1);
  		double ytest = (y0 * t) + ((1-t)*y1);
		if(Map::isPointInBuffer(xtest,ytest) == true){
			return true;
		}
      	}
      }
      return false;
}



bool Map::isAccessible(double x, double y){
	if(isWithinBorders(x,y) and (not isPointInBuffer(x,y))){
		return true;
	} 
	return false;
}

// returns distance between two points 
double Map::distance(double x1, double y1, double x2, double y2){
	double dx = x1 - x2;
	double dy = y1 - y2;
	return sqrt((dx*dx) + (dy*dy));  
} 

// returns distance between a point and a wall
double Map::distanceFromWall(double x, double y, int wallIndex){
	Wall wall = walls[wallIndex];
	return distanceFromSegment(wall.x1, wall.y1, wall.x2, wall.y2, x, y);
} 

// returns distance between a point and a line segment
double Map::distanceFromSegment(double x1, double y1, double x2, double y2, double pointX, double pointY)
{
    double diffX = x2 - x1;
    float diffY = y2 - y1;
    if ((diffX == 0) && (diffY == 0))
    {
        diffX = pointX - x1;
        diffY = pointY - y1;
        return sqrt(diffX * diffX + diffY * diffY);
    }

    float t = ((pointX - x1) * diffX + (pointY - y1) * diffY) / (diffX * diffX + diffY * diffY);

    if (t < 0)
    {
        //point is nearest to the first point i.e x1 and y1
        diffX = pointX - x1;
        diffY = pointY - y1;
    }
    else if (t > 1)
    {
        //point is nearest to the end point i.e x2 and y2
        diffX = pointX - x2;
        diffY = pointY - y2;
    }
    else
    {
        //if perpendicular line intersect the line segment.
        diffX = pointX - (x1 + t * diffX);
        diffY = pointY - (y1 + t * diffY);
    }

    //returning shortest distance
    return sqrt(diffX * diffX + diffY * diffY);
}


