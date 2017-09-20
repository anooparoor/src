#ifndef GRAPH_H
#define GRAPH_H
#include "Node.h"
#include "Edge.h"
#include <vector>
#include <queue>
#include <math.h>
#include "Map.h"
#include <assert.h>
#include <fstream>

class Graph {
private:
  vector<Node*> nodes; 
  vector<Edge*> edges;
  vector< vector <int> > nodeIndex; 

  double proximity;           // proximity between 2 nodes ( in cm, 1m = 100cm ) 

  Map * map ; 

  void generateNavGraph();    

  bool isEdge(Edge e); 
 
public: 
  Graph(Map*, int);

  ~Graph();

  Map* getMap() { return map; }

  int getProximity() const { return proximity; }

  vector<Node*> getNodes() const { return nodes; }

  vector<Edge*> getEdges() const { return edges; }

  // this function is used to retreive node id that is at (x,y). Used during populating neigbors of nodes, 
  // during construction of navGraph
  int getNodeID(int x, int y);

  Node getNode(int n);
  
  Node* getNodePtr(int i); 

  Edge* getEdge(int n1, int n2);

  void updateEdgeCost(int i, double costfromto, double costtofrom){
	edges[i]->setCost(costfromto, costtofrom);
  }

  //! returns the neigbors of the node with index n. Calls directly Node::getNeighbors 
  vector<int> getNeighbors(Node n); 

  //! populates the graph with nodes and assigns their immediate neighbors
  void populateNodeNeighbors(); 

  bool isNode(Node n); 

  int numNodes() const { return nodes.size(); }

  int numEdges() const { return edges.size(); }

  void printGraph() ;

  void outputGraph();

  vector<Node*> getNodesInRegion( int x, int y, double r);

  vector<Node*> getNodesInRegion( int x1, int y1, int x2, int y2);

  //! sets all the nodes in the graph as accessible
  void clearGraph();

  double calcCost(Node, Node);
};

#endif
