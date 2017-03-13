#include<vector>
#include<FORRGeometry.h>
#include<FORRTrails.h>
#include<fstream>
#include<sstream>


using namespace std;

#define EPSILON 20


//if it does find a trail marker that sees, return the index.  otherwise, return -1
int FORRTrails::doesTrailHaveVisiblePointToTarget(CartesianPoint target_point, int trail_index){
  for(int j = 0; j < trails[trail_index].size(); j++){
    if(canTrailMarkerSeeTarget(trails[trail_index][j].wallVectorEndpoints, target_point, trails[trail_index][j].coordinates, EPSILON)){
      //target point is visible along trail at trail marker trails[trail_index][j]
      cout << "Trail at coordinate "<<trails[trail_index][j].coordinates.get_x()<<","<<
	trails[trail_index][j].coordinates.get_y()<<" can see target"<<endl;
      cout<< "Target located at: ("<<target_point.get_x()<<","<<target_point.get_y()<<endl;
      //trail_marker_sees_point = j;
      return j;
      }
  }


    //else nothing visible, return false
    return -1;
}



void FORRTrails::updateTrails(){

  //ifstream trailstream;
  ifstream vectorstream;
  //trailstream.open("trail.conf");
  vectorstream.open("corrected_wallvectors.conf");

  vector<TrailMarker> trail;
  
  //string trail_list =  "";
  string vector_list = "";
  if(vectorstream.eof()) return; //if the trail is empty (because there was only a tier1 move, return)
  
  

  while(!vectorstream.eof()){
    getline(vectorstream, vector_list);
    stringstream vs(vector_list);
    string x_vector, y_vector = "";
    string x_trail, y_trail = "";
    
    vs >> x_vector;
    vs >> y_vector; // remove the starting coordinates, these are the trail x,y coordinates

    int x_int_vector, y_int_vector = 0;

    x_int_vector = atoi(x_vector.c_str());
    y_int_vector = atoi(y_vector.c_str());
    
    CartesianPoint trail_point = CartesianPoint(x_int_vector, y_int_vector);
    cout << "Added trail_point ("<<x_int_vector<<","<<y_int_vector<<")"<<endl;

    vector<CartesianPoint> vectorEndPoints;
    

    //collect the wall distance vector endpoints after the initial x,y trail coordinates
    while(vs >> x_vector){
      vs >> y_vector;
      x_int_vector = atoi(x_vector.c_str());
      y_int_vector = atoi(y_vector.c_str());
      
      CartesianPoint c = CartesianPoint(x_int_vector, y_int_vector);

      vectorEndPoints.push_back(c);

    }

    TrailMarker t = TrailMarker (trail_point, vectorEndPoints);
    if(x_int_vector !=0 && y_int_vector !=0) //in case it reached the end, error catching
      trail.push_back(t);  // add to the current trail

    
  }

  //Finally, push the full trail into the trails vector in FORRTrails.h
  trails.push_back(trail);

 
}






/*
void FORRTrails::readTrails(string filename){
  ifstream inputstream;
  string fileline = "";
  
  inputstream.open(filename.c_str());
  if(!inputstream.is_open()) return; //file doesn't exist, break
  while(!inputstream.eof()){
    getline(inputstream, fileline);
    stringstream ss(fileline);
    string x, y = "";
    int x_int, y_int = 0;
    vector<TrailMarker> trail;
    while(ss >> x){
      ss >> y;
      x_int = atoi(x.c_str());
      y_int = atoi(y.c_str());
      
      //trail.push_back(TrailMarker(CartesianPoint(x_int, y_int)));


    }
    trails.push_back(trail);
    

  }
}
 
*/
//returns true if there is a point that is "visible" by the wall distance vectors to some epsilon.  
//OLD VERSION::A point is visible if the distance to a wall distance vector line is < epsilon.
//This version takes in the distance vectors as distance values

//NEW VERSION::sees if the wall distance vector intersects with a segment created by 2
//trailpoints
bool FORRTrails::canRobotSeeTrailSegment(vector<double> &distanceVectors, CartesianPoint trailpoint1, CartesianPoint trailpoint2,
					CartesianPoint current_point){
  
  double rots[] =  {0, .1548, -.1548, .3048, -.3048, .65, -.65, 1.3, -1.3, 3.39, -3.39};
  
  CartesianPoint intersection_point(0,0);
  for(int i = 0; i < distanceVectors.size(); i++){
    
    //this can be cleaned up or put into one function, but need to convert the distance to a linesegment to calculate
    //the distance from a point to a line
    Vector v = Vector(current_point, rots[i], distanceVectors[i]);
    CartesianPoint endpoint = v.get_endpoint();
    LineSegment distance_vector_line = LineSegment(current_point, endpoint);
    //double distance_to_point = distance(trailpoint, distance_vector_line);
    if( sqrt((endpoint.get_x() - current_point.get_x())*(endpoint.get_x() - current_point.get_x()) + 
	     (endpoint.get_y() - current_point.get_y())*(endpoint.get_y() - current_point.get_y())) < 125){
    
      LineSegment trail_segment = LineSegment(trailpoint1, trailpoint2);
    
      //  if(distance_to_point < epsilon){
      //return true;
      //}

  
      
      if(do_intersect(distance_vector_line, trail_segment, intersection_point)){
	
        
	return true;
      
      }
      
    
    
    
    }

  }
  //else, not visible
  return false;
}




//returns true if there is a point that is "visible" by the wall distance vectors to some epsilon.  
//A point is visible if the distance to a wall distance vector line is < epsilon.
bool FORRTrails::canTrailMarkerSeeTarget(vector<CartesianPoint> &distanceVectorEndpoints, CartesianPoint target_point, 
					 CartesianPoint current_point, double epsilon){


  for(int i = 0; i < distanceVectorEndpoints.size(); i++){

     
    //this can be cleaned up or put into one function, but need to convert the distance to a linesegment to calculate
    //the distance from a point to a line
    

    LineSegment l = LineSegment(current_point, distanceVectorEndpoints[i]);
    double distance_to_point = distance(target_point, l);
    if(distance_to_point < epsilon){
      cout << "Distance vector endpoint visible: ("<<distanceVectorEndpoints[i].get_x()<<","<<
	distanceVectorEndpoints[i].get_y()<<")"<<endl; 
      cout << "Distance: "<<distance_to_point<<endl;
      return true;
    }
  }



  //else, not visible
  return false;


}





void FORRTrails::findNearbyTrail(CartesianPoint target, CartesianPoint start, vector<double> &currentDistanceVectors){
  //first, check to see if there is a trail that is near your current position.
  //could also do this the other way around and check to see first if there are any points along the 
  //trail that are "seeable" (within some epsilon to a distance vector) to the target.
  //either way, these both need to be satisfied before a trail is "found"
  int trail_marker_seen_target = 0;

  //loop through all your trails and see if there is any trail point along those that you can see from where the robot is currently
  for(unsigned int i = 0; i < trails.size(); i++){
  
    trail_marker_seen_target = doesTrailHaveVisiblePointToTarget(target,i);
    
    //if not -1, then has a trail marker that can see the target
    if(trail_marker_seen_target > 0){

      //size-1 because we're checking pairs of trailmarkers now that generate a line
      for(unsigned int j = 0; j < trails[i].size()-1; j++){
	//for checking to see our current vector, we use the version of ispointvisibletoanotherpoint() 
	//that takes the distance vectors as distance doubles, as that's what's getting passed
	//from robotcontroller
	//check to see if any of our robot's current (i.e. the location the robot is at right now) wall vectors
	//are within epsilon to the point on the trail
	//say epsilon distance units for now as the epsilon
	

	//NEW VERSION::looks at the intersection between segments
		if(canRobotSeeTrailSegment(currentDistanceVectors, trails[i][j].coordinates, trails[i][j+1].coordinates,
		start)){
	
	  //OLD VERSION
	//if(canRobotSeeTrailMarker(currentDistanceVectors, trails[i][j].coordinates, start, EPSILON)){
	  cout << "Trail marker at coordinates ("<<trails[i][j].coordinates.get_x()<<","<<
	    trails[i][j].coordinates.get_y()<<") visible to robot"<<endl;
	  cout << "----------TRAIL FOUND!---------- i ="<<i<<endl;
	  chosen_trail = i;
	  
	  //if the trail marker that saw the target is greater along the sequence than what you're seeing,
	  //then you're following the trail in the same direction it was created
	  //otherwise, you need to follow the trail in reverse
	  if(trail_marker_seen_target > j){
	    setDirection(POSITIVE);
	  }
	  else{
	    setDirection(NEGATIVE);
	  }
	  
	}

      }
    }
  }
  
  //otherwise, chosen_trail remains -1
}


CartesianPoint FORRTrails::getFurthestVisiblePointOnChosenTrail(CartesianPoint current, vector<double> &currentDistanceVectors){
  
  //if no trail found, return a dummy point
  if(chosen_trail == -1) return CartesianPoint(-1,-1);


  //go backwards to grab a further point first if possible
  for(int j = trails[chosen_trail].size(); j >= 1; j--){
    if(canRobotSeeTrailSegment(currentDistanceVectors, trails[chosen_trail][j].coordinates,
    			      trails[chosen_trail][j-1].coordinates, current)){
      //if(canRobotSeeTrailMarker(currentDistanceVectors, trails[chosen_trail][j].coordinates,
      //		      current, EPSILON)){
      can_see_trail = true;
      return trails[chosen_trail][j].coordinates;
    }
  }
  
  can_see_trail = false;
  return CartesianPoint(-1,-1);


}


void FORRTrails::printTrails(){
  for(int i = 0; i < trails.size(); i++){
    for(int j = 0; j < trails[i].size(); j++){
      cout << "("<<trails[i][j].coordinates.get_x()<<","<<trails[i][j].coordinates.get_y()<<") ";


    }
    cout <<endl;
  }
 
}
