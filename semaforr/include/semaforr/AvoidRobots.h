// Header file of a class which contains functions which is resposible for collision avoidance with other robots also called n body reciprocal collision
// avoidance problem 

# include "Position.h"
# include "FORRGeometry.h"
# include <math.h>
# include "FORRAction.h"
# include "Beliefs.h"

class Vertex{
 public:
  Position robot;
  double weight;
  list<Position> adj;
  bool operator<(const Vertex &other) const { return weight < other.weight; }  
  void printdetails() const {
    cout << "Robot position: " << robot.getX() << "," << robot.getY() << endl;
    cout << "weight: " << weight << endl;
    cout << "Adjacency list :" << endl;
    std::list<Position>::const_iterator iterator;
    for (iterator = adj.begin() ; iterator != adj.end(); ++iterator){
      cout << "position " << (*iterator).getX() << "," << (*iterator).getY() << endl;
    }
  }
};

class AvoidRobots{
 public:
  void avoid_robots(Beliefs *beliefs);
  bool generate_collision_graph(vector<Position> team_pose, list<Vertex> *collision_graph);
  double check_for_collision(Position robot1, Position robot2);
  double check_inline_collision(Position robot1_pos, Position robot2_pos);
  void add_collision_info(list<Vertex> *collision_graph ,Position robot1,Position robot2);
  bool checkequal(Position position1, Position position2);
  void add_weight_info(list<Vertex> *collision_graph ,Position robot, double weight);
  void veto_inline_collisions(Beliefs *beliefs);
  bool veto_angular_collisions(list<Vertex> *collision_graph, set<FORRAction> *vetoed_actions ,Position robot);
  bool in_maximal_set(set<Vertex> maximal_independent_set, Position robot);
  double compute_weight(set<Vertex> independent_set);
  bool is_independent(set<Vertex> input_set);
  double distance(CartesianPoint first, CartesianPoint second);
  double distance(Position first, Position second);
  void print_point(CartesianPoint point);
  set < set < Vertex > > generatePowerSet( list<Vertex> *inputSet);
  bool is_facing(Position robot1, Position robot2);
};




