/**************
Implementation file for FORRWaypoints class, see
FORRWaypoints.h for function and variable descriptions

Written by Matthew Evanusa, 2014
***************/



#include <iostream>
#include "FORRWaypoints.h"
#include <vector>
#include <fstream>
#include <sstream>

using namespace std;


int FORRWaypoints::getMaxGridValue(){
  int max_value = 0;
  for(int x = 0; x < boxes_width; x++){
    for(int y = 0; y < boxes_height; y++){
      if(waypoints[x][y] > max_value)
	max_value = waypoints[x][y];

    }



  }

  return max_value;
}


void FORRWaypoints::setGrid(int width, int height){
  cout << "Entered setgrid."<<endl;
  boxes_width = width/granularity;
  boxes_height = height/granularity;
  map_height = height;
  map_width = width;
  for(int i = 0; i < boxes_width; i++){
    vector<int> row;
    for(int j = 0; j < boxes_height; j++){
      row.push_back(0);
    }
    waypoints.push_back(row);
  }
  cout << "Exit setgrid."<<endl;
  max_grid_value = 0;
}


int FORRWaypoints::getGridValue(double map_x, double map_y){
  pair<int,int> grid_coords = convertToGridCoordinates(map_x, map_y);
  
  return waypoints[grid_coords.first][grid_coords.second];


}



//second method for populating the grid, instead of just
//counting up the endpoints, counts up the times the line went through 
//generates the four line segments around each box and sees if the current line being
//investigated intersects that box, if so increment
void FORRWaypoints::populateGridFromFullLine(){
  ifstream pathstream;
  pathstream.open("paths.conf");
  string fileLine;
  //bool first_values = true;
  while(!pathstream.eof()){
    getline(pathstream, fileLine);
    stringstream ss(fileLine);
    int x1, y1, x2, y2 = 0;
    
    string first_x, first_y;
    string second_x, second_y;
    string third_x, third_y;
    cout << "Before reading in line."<<endl;
    bool first_values = true;
    while(ss >> first_x){
      ss >> first_y;
      
      if(first_values && !ss.eof()){
	ss >> second_x; 
	ss >> second_y;
      }
      
      x1 = atoi(first_x.c_str());
      y1 = atoi(first_y.c_str());
      cout << "Loading in lines, x1: "<<x1<<", y1: "<<y1<<"...";
      CartesianPoint p1 = CartesianPoint(x1,y1);
      LineSegment current_path_line;

      if(first_values){
	x2 = atoi(second_x.c_str());
	y2 = atoi(second_y.c_str());
	
	third_x = second_x;
	third_y = second_y;

	first_values = false;
      }
      else{
	x2 = atoi(third_x.c_str());
	y2 = atoi(third_y.c_str());
	
	third_x = first_x;
	third_y = first_y;	
      }
      cout << "x2: "<<x2<<", y2: "<<y2<<endl;
      CartesianPoint p2 = CartesianPoint(x2,y2);
      current_path_line = LineSegment(p1,p2);
      
    
    CartesianPoint corner1, corner2, corner3, corner4;

    //look through each box and see if the line intersects with it
    //these are all to check for the corner cases.  if there is a better way 
    //I will change it
    for(int i = 0; i < waypoints.size(); i++){
      for(int j = 0; j < waypoints[i].size(); j++){

	corner1 = CartesianPoint((i * granularity), (j * granularity));

	if(i < waypoints.size()-1){
	 
	  corner2 = CartesianPoint(((i+1) * granularity), (j * granularity));
  
	  if(j < waypoints[i].size()-1){
	    
	    corner4 = CartesianPoint((i * granularity), ((j+1) * granularity));
	    corner3 = CartesianPoint(((i+1) * granularity), ((j+1) * granularity));
	  }
	  else{
	    corner4 = CartesianPoint((i * granularity), map_height); //if at the corner, use the map height
	    corner3 = CartesianPoint((i+1 * granularity), map_height);	   	    
	  }
	}
	else{
	  corner2 = CartesianPoint(map_width, (j * granularity));

	  if(j < waypoints[i].size()-1){
	    corner4 = CartesianPoint((i * granularity), ((j+1) * granularity));
	    corner3 = CartesianPoint(map_width, ((j+1) * granularity));
	  }
	  else{
	    corner4 = CartesianPoint((i * granularity), map_height);
	    corner3 = CartesianPoint(map_width, map_height);
	  }
	}
	
	CartesianPoint c;
	LineSegment l1 = LineSegment(corner1, corner2);
	LineSegment l2 = LineSegment(corner2, corner3);
	LineSegment l3 = LineSegment(corner3, corner4);
	LineSegment l4 = LineSegment(corner4, corner1);
	
	//Check if any line segments of the box intersect with the path line
	if(do_intersect(current_path_line, l1, c) || do_intersect(current_path_line, l2, c) ||
	   do_intersect(current_path_line, l3, c) || do_intersect(current_path_line, l4, c)){
	  waypoints[i][j] += 1;
	  
	}
      }
    }
    }
  }

  max_grid_value = getMaxGridValue();

}



void FORRWaypoints::populateGrid(){
  cout << "Entered populateGrid"<<endl;
  ifstream pathstream;
  pathstream.open("paths.saved");
  string fileLine;
  
  while(!pathstream.eof()){
    getline(pathstream, fileLine);
    stringstream ss(fileLine);
    // bool first_values;
    int x1, y1 = 0;
    int previous_x, previous_y = -1;
    string x,y = "";
    int converted_x, converted_y = 0;
   
    while(ss>>x){
      ss>>y;
      
      x1 = atoi(x.c_str());
      y1 = atoi(y.c_str());

      //check to remove duplicate from directly previous move, but only per-target run
      if((x1 != previous_x) || (y1 != previous_y)){
	cout << "Boxes width: " << boxes_width << endl;
	cout << "Relation: " << x1/(map_width*1.0) << endl;
	converted_x = (int)((x1/(map_width*1.0)) * boxes_width);
	cout << "Converted x: "<<converted_x<<endl;
	converted_y = (int)((y1/(map_height*1.0)) * boxes_height);
	waypoints[converted_x][converted_y] += 1;
	cout << "Converted y: "<<converted_y<<endl<<endl;

      }
      
      previous_x = x1;
      previous_y = y1;

    }
  }

  pathstream.close();
  cout << "Exited populate grid."<<endl;
}


void FORRWaypoints::outputWaypoints(string filename){
  ofstream output;
  

  output.open(filename.c_str());
  for(int j = boxes_height-1; j >=0; j--){
    
    for(int i = 0; i < boxes_width; i++){
      output << waypoints[i][j] << " ";
    }
    output << endl;
  }
  
  output.close();

}


//converts from map coordinates to grid coordinates
pair<int,int> FORRWaypoints::convertToGridCoordinates(double x, double y){
  return make_pair((int)((x/(map_width*1.0)) * boxes_width), (int)((y/(map_height * 1.0)) * boxes_height));
}


//returns the midpoint of the block that has the highest count and is adjacent
//to the current block
pair<int, int> FORRWaypoints::getNextGridPosition(double curr_x, double curr_y){
  int converted_x, converted_y;
  int count = 0;
  int new_x, new_y;
  converted_x = (int)((curr_x/(map_width*1.0)) * boxes_width);     
  converted_y = (int)((curr_y/(map_height*1.0)) * boxes_height);
  //if not on the edge, necessary so it doesn't check out of bounds
  //also, if previous count is set, will update in the following 
  //if conditions
  if(converted_x < (boxes_width-1)){
    if(waypoints[converted_x+1][converted_y] > count){
      count = waypoints[converted_x+1][converted_y];
      new_x = converted_x+1;
      new_y = converted_y;
    }
  }
  if(converted_x > 0){
    if(waypoints[converted_x-1][converted_y] > count){
      count = waypoints[converted_x-1][converted_y];
      new_x = converted_x-1;
      new_y = converted_y;
    }
  }
  if(converted_y < (boxes_height-1)){
    if(waypoints[converted_x][converted_y+1] > count){
      count = waypoints[converted_x][converted_y+1];
      new_x = converted_x;
      new_y = converted_y+1;
    }
    
  }
  if(converted_y > 0){
    if(waypoints[converted_x][converted_y-1] > count){
      //count = waypoints[curr_x][curr_y-1];
      new_x = converted_x;
      new_y = converted_y-1;
    }
  }
  
  int reconverted_x = ((converted_x*granularity)+((converted_x+1)*granularity))/2;
  int reconverted_y = ((converted_y*granularity)+((converted_y+1)*granularity))/2;
  return make_pair(reconverted_x, reconverted_y);

}




void FORRWaypoints::clearWaypoints(){
  for(int i = 0; i < boxes_width; i++)
    for(int j = 0; j < boxes_height; j++)
      waypoints[i][j] = 0;
}

