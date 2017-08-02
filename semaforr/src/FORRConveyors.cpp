/**************
Implementation file for FORRConveyors class, see
FORRConveyors.h for function and variable descriptions

Written by Matthew Evanusa, 2014
***************/

#include "FORRConveyors.h"


using namespace std;


int FORRConveyors::getMaxGridValue(){
  int max_value = 0;
  for(int x = 0; x < boxes_width; x++){
    for(int y = 0; y < boxes_height; y++){
      if(conveyors[x][y] > max_value)
	max_value = conveyors[x][y];
    }
  }
  return max_value;
}


// Return the value in the given grid position
int FORRConveyors::getGridValue(double map_x, double map_y){
  pair<int,int> grid_coords = convertToGridCoordinates(map_x, map_y);
  return conveyors[grid_coords.first][grid_coords.second];
}



//Populate grid by walking along the path of robot travel
void FORRConveyors::populateGridFromPositionHistory(vector<Position> *pos_hist){
	vector<Position> position = *pos_hist;
	pair<int,int> prev, next;
	prev.first = -1;
	prev.second = -2;
	for(int i = 0 ; i < position.size() - 1; i++){
		next = updateGridFromLine(position[i].getX(), position[i].getY(), position[i+1].getX(), position[i+1].getY(), prev);
		prev = next;
	}
}

//Populate grid by walking along a line segment
pair<int,int> FORRConveyors::updateGridFromLine(double x1, double y1, double x2, double y2, pair<int,int> prev){
	double step_size = 0.1;
	double tx,ty;
	pair<int,int> grid_point;
	grid_point.first = -1;
	grid_point.second = -2;
	//cout << "POints " << x1 << "," << y1 << " ; " << x2 << "," << y2 << endl; 
	for(double step = 0; step <= 1; step += step_size){
		tx = (x1 * step) + (x2 * (1-step));
		ty = (y1 * step) + (y2 * (1-step));
		//cout << tx << "," << ty << endl; 
		grid_point = convertToGridCoordinates(tx, ty);
		//cout << "Grid index : " << grid_point.first << "," << grid_point.second << endl;
		if(grid_point != prev){
			conveyors[grid_point.first][grid_point.second] += 1;
		}
		prev = grid_point;
	}
	return prev;
}


void FORRConveyors::outputConveyors(string filename){
  ofstream output;
  

  output.open(filename.c_str());
  for(int j = boxes_height-1; j >=0; j--){
    
    for(int i = 0; i < boxes_width; i++){
      output << conveyors[i][j] << " ";
    }
    output << endl;
  }
  
  output.close();

}


//converts from map coordinates to grid coordinates
pair<int,int> FORRConveyors::convertToGridCoordinates(double x, double y){
  return make_pair((int)((x/(map_width*1.0)) * boxes_width), (int)((y/(map_height * 1.0)) * boxes_height));
}


//returns the midpoint of the block that has the highest count and is adjacent
//to the current block
pair<int, int> FORRConveyors::getNextGridPosition(double curr_x, double curr_y){
  int converted_x, converted_y;
  int count = 0;
  int new_x, new_y;
  converted_x = (int)((curr_x/(map_width*1.0)) * boxes_width);     
  converted_y = (int)((curr_y/(map_height*1.0)) * boxes_height);
  //if not on the edge, necessary so it doesn't check out of bounds
  //also, if previous count is set, will update in the following 
  //if conditions
  if(converted_x < (boxes_width-1)){
    if(conveyors[converted_x+1][converted_y] > count){
      count = conveyors[converted_x+1][converted_y];
      new_x = converted_x+1;
      new_y = converted_y;
    }
  }
  if(converted_x > 0){
    if(conveyors[converted_x-1][converted_y] > count){
      count = conveyors[converted_x-1][converted_y];
      new_x = converted_x-1;
      new_y = converted_y;
    }
  }
  if(converted_y < (boxes_height-1)){
    if(conveyors[converted_x][converted_y+1] > count){
      count = conveyors[converted_x][converted_y+1];
      new_x = converted_x;
      new_y = converted_y+1;
    }
    
  }
  if(converted_y > 0){
    if(conveyors[converted_x][converted_y-1] > count){
      //count = conveyors[curr_x][curr_y-1];
      new_x = converted_x;
      new_y = converted_y-1;
    }
  }
  
  int reconverted_x = ((new_x*granularity)+((new_x+1)*granularity))/2;
  int reconverted_y = ((new_y*granularity)+((new_y+1)*granularity))/2;
  return make_pair(reconverted_x, reconverted_y);

}




void FORRConveyors::clearConveyors(){
  for(int i = 0; i < boxes_width; i++)
    for(int j = 0; j < boxes_height; j++)
      conveyors[i][j] = 0;
}

