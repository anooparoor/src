// Conveyor is a heatmap of all locations the robot has been to along its travel

int Beliefs::times_at_location(CartesianPoint location){
  int square_x = location.get_x() / square_size;
  int square_y = location.get_y() / square_size;

  if(square_x > locations_size || square_y > locations_size){
    cout << "ERROR: out of map " << endl;
    exit(1);
  }
  return visited_locations[square_x][square_y];
}


void Beliefs::create_locations(){
  const int MAZE = 600; // this is magic number for maze
  locations_size = MAZE / square_size;
  visited_locations = new double*[locations_size];
  for(int i = 0; i < locations_size; ++i)
    visited_locations[i] = new double[locations_size];

  // zero out all the values
  for(int i = 0; i < locations_size; ++i)
    for(int j = 0; j < locations_size; ++j)
      visited_locations[i][j] = 0;
}


void Beliefs::display_visited_map(){
  
  for(int j = boxes_height-1; j >=0; j--){
    
    for(int i = 0; i < boxes_width; i++){
      //cout << visited_grid[i][j] << " ";
    }
    //cout << endl;
  }
  
  //cout << "Square size: " << square_size<<endl;
  
}


void Beliefs::location_lookup(){
  int movements[] = {3, 7, 20, 25, 105};
  int newX, newY, square_x, square_y;
  Position robot_position = getCurrentPosition();
  double robot_yaw = robot_position.getTheta();
  // determine where is the robot at the moment
  square_x = robot_position.getX() / square_size;
  square_y = robot_position.getY() / square_size;
   // used for all rotations
  targetDistanceVector[0] = visited_locations[square_x][square_y];
  //cout << "In FD: for Explorer: 0 "  << targetDistances[0] << endl;
       
  for(int i = 1; i < 6; ++i){
    // going forward
    newX = robot_position.getX() + movements[i-1]  * cos(robot_yaw);  
    newY = robot_position.getY() + movements[i-1]  * sin(robot_yaw);

    if(newX < 600 && newX > 0 && newY < 600 && newY > 0){// make sure you are in the map
      square_x = newX / square_size;
      square_y = newY / square_size;
      // reusing targetDistances vector
      targetDistanceVector[i] = visited_locations[square_x][square_y];
    }
    else
      // this line ultimately does not matter since this step will be commented by Tier1
      targetDistanceVector[i] = 10; // if step is out of the map, magic value
    // going backward
    newX = robot_position.getX() - movements[i-1]  * cos(robot_yaw);
    newY = robot_position.getY() - movements[i-1]  * sin(robot_yaw);
    if(newX < 600 && newX > 0 && newY < 600 && newY > 0){
      // determine where is the robot at the moment
      square_x = newX / square_size;
      square_y = newY / square_size;
      targetDistanceVector[i+5] = visited_locations[square_x][square_y];
    }
    else
      // this line ultimately does not matter since this step will be commented by Tier1
      targetDistanceVector[i] = 10; // if step is out of the map, magic value
    //cout << "In FD: for Explorer: " << i << "  " << targetDistances[i] << endl;
  }
}

double Beliefs::get_visited_locations_value(Position expectedPosition){
  double x = expectedPosition.getX();
  double y = expectedPosition.getY();
  int mapHeight = getMap()->getHeight();
  int mapWidth = getMap()->getLength();
  //cout << "In get_visited_locations_value: "<<endl;
  //cout << "x before lowering: "<<x << endl;
  //cout << "y before lowering: "<<y << endl;
  
  if(x > mapWidth) x = mapWidth-1;
  if(y > mapHeight) y = mapHeight-1;
  //cout << "x after lowering: "<<x << endl;
  //cout << "y after lowering: "<<y << endl;

  if((x > 0) && (x < mapWidth) && (y > 0) && (y < mapHeight)){
    //cout << "Int x:" <<(int)(x/square_size)<<", Int y: "<< (int)(y/square_size) <<endl;
    //cout << "Visited grid width: " << visited_grid.size()<<endl;
    //cout << "Visited grid height: "<< visited_grid[0].size()<<endl;
    return visited_grid[(int)(x/square_size)][(int)(y/square_size)];
  }
  else return 10;

}




void Beliefs::set_visited_grid(int width, int height){
  //cout << "Entered set visited grid."<<endl;
  //cout << "Square size: " << square_size<<endl;
  boxes_width = ceil(width/square_size*1.0);
  boxes_height = ceil(height/square_size*1.0);
  
  for(int i = 0; i < boxes_width; i++){
    vector<int> row;
    for(int j = 0; j < boxes_height; j++){
      row.push_back(0);
    }
    visited_grid.push_back(row);
  }
  //cout << "Exit set visited grid."<<endl;

}

void Beliefs::reset_visited_grid(){
  for(int i = 0; i < visited_grid.size(); i++){
    for(int j = 0; j < visited_grid[i].size(); j++){
      visited_grid[i][j] = 0;
    }

  }
}


