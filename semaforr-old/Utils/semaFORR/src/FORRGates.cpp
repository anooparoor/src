#include<vector>
#include<FORRGeometry.h>
#include<FORRGates.h>
#include<fstream>
#include<sstream>

using namespace std;

void FORRGates::add_gate(CartesianPoint point_from, CartesianPoint point_to, int quad_from, int quad_to){
  gates.push_back(Gate(point_from, point_to, quad_from, quad_to));
}


int FORRGates::region(int x, int y, int max_x, int max_y){
 int quad_x = max_x / 2;
 int quad_y = max_y / 2;
if ((x < quad_x) && (y < quad_y))
  return 3;
 else if((x < quad_x) && (y > quad_y))
   return 2;
 else if((x > quad_x) && (y < quad_y))
   return 4;
 else return 1;
}


void FORRGates::learn_gate(int previous_x, int previous_y, int curr_x,
			   int curr_y, int map_x, int map_y){
  int previous_region = region(previous_x, previous_y, map_x, map_y);
  cout << "Previous region: "<<previous_region<<endl;
  int curr_region = region(curr_x, curr_y, map_x, map_y);
  cout << "New Region: "<<curr_region<<endl;
  if(previous_region != curr_region){
    //CartesianPoint previous_point = CartesianPoint(previous_x, previous_y);
    //CartesianPoint curr_point = CartesianPoint(curr_x, curr_y);
    cout<< "Gate found!"<<endl;
    //previous gate is opposite direction
    add_gate(CartesianPoint(previous_x, previous_y), CartesianPoint(curr_x, curr_y), curr_region, previous_region);
    add_gate(CartesianPoint(curr_x, curr_y), CartesianPoint(previous_x, previous_y), previous_region, curr_region);
    
  }


}

void FORRGates::load_gates_from_file(string filename){
  ifstream inputstream;
  string newL = "";
  inputstream.open(filename.c_str());
  if(!inputstream.is_open()) return; // if file doesn't exist leave
  while(!inputstream.eof()){
    getline(inputstream, newL);
    istringstream istring(newL);
    
    //format of gate file is predetermined
    string temp = "";
    int x1,y1,x2,y2,r1,r2;
    istring >> temp;
    x1 = atoi(temp.c_str());
    istring >> temp;
    y1 = atoi(temp.c_str());
    istring >> temp;
    x2 = atoi(temp.c_str());
    istring >> temp;
    y2 = atoi(temp.c_str());
    istring >> temp;
    r1 = atoi(temp.c_str());
    istring >> temp;
    r2 = atoi(temp.c_str());
    //printf("x1:%d,y1:%d,x2:%d,y2:%d,r1:%d,r2:%d \n",x1,y1,x2,y2,r1,r2);
    if(r1 != 0)
      add_gate(CartesianPoint(x1,y1),CartesianPoint(x2,y2),r1,r2);
    
  }
  
  inputstream.close();
}

void FORRGates::remove_gate(int index){
  gates.erase(gates.begin()+index);
}

void FORRGates::write_gates_to_file(string filename){
  ofstream gatestream;
  gatestream.open(filename.c_str());
  for(unsigned int i = 0; i < gates.size(); i++){
    CartesianPoint p = gates[i].point_from;
    CartesianPoint p1 = gates[i].point_to;
    int quad_from_ = gates[i].quad_from;
    int quad_to_ = gates[i].quad_to;
    gatestream << p.get_x()<<" "<<p.get_y()<<" "<<p1.get_x()<<" "<<p1.get_y()<<" "<<quad_from_<<" "<<quad_to_<<endl;
  }

  gatestream.close();
}
