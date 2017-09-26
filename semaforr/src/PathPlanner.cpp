/*!
  \file PathPlanner.cpp
  \addtogroup PathPlanner
  @{
 */

#include "PathPlanner.h"
#include <limits.h>

#define PATH_DEBUG true

/*!
  \brief Calculates the shortest path from a start point to a destination point on the navigation graph.

  This function makes necessary calls to populate the PathPlanner::path list, which is a list of node indexes. The list doesn't contain the source (start point) and the target (destination point), only the nodes that the robot needs to get to in sequence, in order to reach its destination.

  First A* algorithm is run on the navigation graph and then the path is smoothed by removing unnecessary waypoints.

  \b Warning a source and a target must be specified prior to this function call.

  \return 0 if path is calculated, 1 if source is not a node or accessible to a valid node, 2 if target is not a node or accessible to a valid node, 3 if no path is found between the source and the target

 */
int PathPlanner::calcPath(bool cautious){
  const string signature = "PathPlanner::calcPath()> ";
  
  if ( source.getID() != Node::invalid_node_index && target.getID() != Node::invalid_node_index ){
    
    if(PATH_DEBUG) {
      cout << signature << "Source:"; 
      source.printNode(); 
      cout << endl;
      cout << signature << "Target:"; 
      target.printNode();
      cout << endl;
    }

    Node s, t;
    if ( navGraph->isNode(source) ) {
      if(PATH_DEBUG)
	cout << signature << "Source is a valid Node in the navigation graph" << endl; 
      s = source ;
    }
    else {
      if(PATH_DEBUG)
	cout << signature << "Source is not a valid Node in the navigation graph. Getting closest valid node." << endl; 
      s = getClosestNode(source, target);
    }
    //cout << signature << "Checking if source node is invalid" << endl;
    if ( s.getID() == Node::invalid_node_index )
      return 1;

    if ( navGraph->isNode(target) ) {
      if(PATH_DEBUG)
	cout << signature << "Target is a valid Node in the navigation graph" << endl; 
      t = target ;
    }
    else {
      if(PATH_DEBUG)
	cout << signature << "Target is not a valid Node in the navigation graph. Getting closest valid node." << endl; 
      t = getClosestNode(target, source);
    }
    //cout << signature << "Checking if target node is invalid" << endl;
    if ( t.getID() == Node::invalid_node_index )
      return 2;

    //cout << signature << "Completed finding source and destination nodes" << endl;
    if(PATH_DEBUG) {
      cout << signature << "s:"; 
      s.printNode(); 
      cout << endl;
      cout << signature << "t:"; 
      t.printNode();
      cout << endl;
    }
    //cout << signature << "Updating nav graph" << endl;
    // update the nav graph with the latest crowd model to change the edge weights
    updateNavGraph();
    astar newsearch(*navGraph, s, t);
    if ( newsearch.isPathFound() ) {
      path = newsearch.getPathToTarget();
      objectiveSet = false;
      pathCompleted = false;
      
      if(!cautious)
	smoothPath(path, s, t);
      
      pathCost = calcPathCost(path); 
      pathCalculated = true;
    }
    else {
      return 3;
    }
  }
  return 0;
}

void PathPlanner::updateNavGraph(){
	cout << "Updating nav graph before with the current crowd model" << endl;
	if(crowdModel.densities.size() == 0){
		cout << "crowdModel not recieved" << endl; 
	}
	else{
		cout << crowdModel.height << endl;
		for(int i = 0 ; i < crowdModel.densities.size(); i++){
			cout << crowdModel.densities[i] << endl;
		} 
		vector<Edge*> edges = navGraph->getEdges();
		// compute the extra cost imposed by crowd model on each edge in navGraph
		for(int i = 0; i < edges.size(); i++){
			Node toNode = navGraph->getNode(edges[i]->getTo());
			Node fromNode = navGraph->getNode(edges[i]->getFrom());
			double oldcost = edges[i]->getDistCost();
			double newEdgeCostft = computeNewEdgeCost(fromNode, toNode, true, oldcost);
			double newEdgeCosttf = computeNewEdgeCost(fromNode, toNode, false, oldcost); 
			navGraph->updateEdgeCost(i, newEdgeCostft, newEdgeCosttf);
			//cout << "Edge Cost " << oldcost << " -> " << newEdgeCostft << " -> " << newEdgeCosttf << endl;  
		}
	}
}


double PathPlanner::computeNewEdgeCost(Node s, Node d, bool direction, double oldcost){
	int b = 30;
	double s_cost = cellCost(s.getX(), s.getY(), b);
	double d_cost = cellCost(d.getX(), d.getY(), b);

	int k = 4;

	//double flowcost = computeCrowdFlow(s,d) + 2;
	//cout << "Node flow cost " << flowcost << endl;
	//cout << "Node penalty : " << d_density << " * " << s_density << endl;
	//double newEdgeCost = (oldcost * flowcost); 
	double newEdgeCost = (oldcost + ((s_cost + d_cost) * 100 * k)/2);
	//cout << "Old cost : " << oldcost << " new cost : " << newEdgeCost << std::endl; 
	return newEdgeCost;
}


double PathPlanner::cellCost(int nodex, int nodey, int buffer){
	int x = (int)((nodex/100.0)/crowdModel.resolution);
	int x1 = (int)(((nodex+buffer)/100.0)/crowdModel.resolution);
	int x2 = (int)(((nodex-buffer)/100.0)/crowdModel.resolution);
	int y = (int)((nodey/100.0)/crowdModel.resolution);
	int y1 = (int)(((nodey+buffer)/100.0)/crowdModel.resolution);
	int y2 = (int)(((nodey-buffer)/100.0)/crowdModel.resolution);


	//std::cout << "x " << x << " y " << y;
	double d = crowdModel.densities[(y * crowdModel.width) + x];
	double d1 = crowdModel.densities[(y1 * crowdModel.width) + x];
	double d2 = crowdModel.densities[(y2 * crowdModel.width) + x];
	double d3 = crowdModel.densities[(y * crowdModel.width) + x1];
	double d4 = crowdModel.densities[(y * crowdModel.width) + x2];
	//std::cout << " Cell cost " << d << std::endl;
	return (d + d1 + d2 + d3 + d4)/5;
	//return d;
}

 

// Projection of crowd flow vectors on vector at s and d and then take the average
double PathPlanner::computeCrowdFlow(Node s, Node d){
	int s_x_index = (int)((s.getX()/100.0)/crowdModel.resolution);
	int s_y_index = (int)((s.getY()/100.0)/crowdModel.resolution);
	int d_x_index = (int)((d.getX()/100.0)/crowdModel.resolution);
	int d_y_index = (int)((d.getY()/100.0)/crowdModel.resolution);
	//Assuming crowd densities are normalized between 0 and 1
	double s_l = crowdModel.left[(s_y_index * crowdModel.width) + s_x_index];
	double d_l = crowdModel.left[(d_y_index * crowdModel.width) + d_x_index];
	double s_r = crowdModel.right[(s_y_index * crowdModel.width) + s_x_index];
	double d_r = crowdModel.right[(d_y_index * crowdModel.width) + d_x_index];
	double s_u = crowdModel.up[(s_y_index * crowdModel.width) + s_x_index];
	double d_u = crowdModel.up[(d_y_index * crowdModel.width) + d_x_index];
	double s_d = crowdModel.down[(s_y_index * crowdModel.width) + s_x_index];
	double d_d = crowdModel.down[(d_y_index * crowdModel.width) + d_x_index];

	double s_ul = crowdModel.up_left[(s_y_index * crowdModel.width) + s_x_index];
	double d_ul = crowdModel.up_left[(d_y_index * crowdModel.width) + d_x_index];
	double s_ur = crowdModel.up_right[(s_y_index * crowdModel.width) + s_x_index];
	double d_ur = crowdModel.up_right[(d_y_index * crowdModel.width) + d_x_index];
	double s_dl = crowdModel.down_left[(s_y_index * crowdModel.width) + s_x_index];
	double d_dl = crowdModel.down_left[(d_y_index * crowdModel.width) + d_x_index];
	double s_dr = crowdModel.down_right[(s_y_index * crowdModel.width) + s_x_index];
	double d_dr = crowdModel.down_right[(d_y_index * crowdModel.width) + d_x_index];

	//cout << "Left : " << d_l << " * " << s_l << endl;
	//cout << "Right : " << d_r << " * " << s_r << endl;
	//cout << "Up : " << d_u << " * " << s_u << endl;
	//cout << "Down : " << d_d << " * " << s_d << endl;
	//cout << "Up-right : " << d_ur << " * " << s_ur << endl;
	//cout << "Up-left : " << d_ul << " * " << s_ul << endl;
	//cout << "Down-right : " << d_dr << " * " << s_dr << endl;
	//cout << "Down-left : " << d_dl << " * " << s_dl << endl;

	double pi = 3.145;
	double u1 = projection(pi/2, d_u, s.getX(), s.getY(), d.getX(), d.getY()); 
	double d1 = projection(3*pi/2, d_d, s.getX(), s.getY(), d.getX(), d.getY()); 
	double r1 = projection(0, d_r, s.getX(), s.getY(), d.getX(), d.getY()); 
	double l1 = projection(pi, d_l, s.getX(), s.getY(), d.getX(), d.getY()); 

	double ur1 = projection(pi/4, d_ur, s.getX(), s.getY(), d.getX(), d.getY()); 
	double ul1 = projection(3*pi/4, d_ul, s.getX(), s.getY(), d.getX(), d.getY()); 
	double dr1 = projection(7*pi/4, d_dr, s.getX(), s.getY(), d.getX(), d.getY()); 
	double dl1 = projection(5*pi/4, d_dl, s.getX(), s.getY(), d.getX(), d.getY()); 

	double final1 = u1 + d1 + r1 + l1 + ur1 + ul1 + dr1 + dl1;
	
	double u2 = projection(pi/2, d_u, d.getX(), d.getY(), s.getX(), s.getY()); 
	double d2 = projection(3*pi/2, d_d, d.getX(), d.getY(), s.getX(), s.getY()); 
	double r2 = projection(0, d_r, d.getX(), d.getY(), s.getX(), s.getY()); 
	double l2 = projection(pi, d_l, d.getX(), d.getY(), s.getX(), s.getY()); 

	double ur2 = projection(pi/4, d_ur, d.getX(), d.getY(), s.getX(), s.getY()); 
	double ul2 = projection(3*pi/3, d_ul, d.getX(), d.getY(), s.getX(), s.getY()); 
	double dr2 = projection(7*pi/4, d_dr, d.getX(), d.getY(), s.getX(), s.getY()); 
	double dl2 = projection(5*pi/4, d_dl, d.getX(), d.getY(), s.getX(), s.getY());

	double final2 = u2 + d2 + r2 + l2 + ur2 + ul2 + dr2 + dl2;
	double cost = (final1 + final2)/2;
	return cost;
}


double PathPlanner::projection(double flow_angle, double flow_length, double xs, double ys, double xd, double yd){
	double pi = 3.145;
	double edge_angle = atan2((ys - yd),(xs - xd));
	edge_angle = (edge_angle > 0 ? edge_angle : (2*pi + edge_angle));
	double theta = flow_angle - edge_angle;
	//cout << "Flow angle: " << flow_angle << " Edge angle: " << edge_angle << " Diff: " << theta << endl; 
	return flow_length * cos(theta);
}




bool PathPlanner::isAccessible(Node s, Node t) {
  if ( !navGraph->isNode(s) )
    s = getClosestNode(s, t);

  if ( !navGraph->isNode(t) )
    t = getClosestNode(t, s);

  if ( s.getID() == Node::invalid_node_index || t.getID() == Node::invalid_node_index )
    return false;

  astar newsearch(*navGraph, s, t);
  if ( newsearch.isPathFound() )
    return true;

  return false;
}


list<pair<int,int> > PathPlanner::getPathXYBetween(int x1, int y1, int x2, int y2){
  
  // flags show if the (x1, y1) and (x2, y2) are not valid nodes in the navgraph
  bool s_invalid = false; 
  bool t_invalid = false; 

  // flags represent what happens after attempting to get the closest valid node to (x1, y1) and (x2, y2)
  bool no_source = false; 
  bool no_target = false; 

  // create temp nodes for pairs 
  Node temp_s(1, x1, y1); 
  Node temp_t(1, x2, y2);

  /* check if the (x1, y1) is a valid node in the graph, if so get a copy of the node and assign it to s
   * else attempt to get the closest valid node from the graph, if it fails set the no_source flag to true
   * if it succeeds assign the closest valid node to s and set the s_invalid flag to true
   */
  Node s;
  int s_id = navGraph->getNodeID(x1, y1);
  if(s_id == Node::invalid_node_index) {
    s = getClosestNode(temp_s, temp_t);
 
    if(!navGraph->isNode(s))
      no_source = true; 

    s_invalid = true; 
  }
  else {
    s = navGraph->getNode(s_id);
  }

  /* check if the (x2, y2) is a valid node in the graph, if so get a copy of the node and assign it to t
   * else attempt to get the closest valid node from the graph, if it fails set the no_target flag to true
   * if it succeeds assign the closest valid node to t and set the t_invalid flag to true
   */
  Node t;
  int t_id = navGraph->getNodeID(x2, y2);
  if(t_id == Node::invalid_node_index) {
    t = getClosestNode(temp_t, temp_s); 

    if(!navGraph->isNode(t))
      no_target = true; 

    t_invalid = true; 
  }

  // computed path list containing node ids
  list<int> path_c;

  // computed path list containing (x,y) values of the nodes
  list< pair<int,int> > path_c_points;

  /* if either source or the target is invalid, astar can't find a path. 
   * to inform the caller of this function if this situation arises, a special pair <source, target>
   * will be added to the path and the function will return. if any of the values in the pair is -1
   * it will mean the path wasn't found due to invalid source, target or both
   */ 
  if(no_source || no_target) {
    int sval = (no_source) ? -1 : 0; 
    int tval = (no_target) ? -1 : 0;
    pair<int,int> p(sval, tval); 
    path_c_points.push_back(p); 

    return path_c_points;
  }

  // calculate the path. if no path is found return the path_points list empty  
  astar newsearch(*navGraph, s, t);
  if(newsearch.isPathFound()) {
    path_c = newsearch.getPathToTarget();
    smoothPath(path_c, s, t);
  }
  else {
    return path_c_points;
  }
  
  // compute the distance
  if(s_invalid) {
    pair<int, int> st(x1,y1);
    path_c_points.push_back(st);
  }
  
  list<int>::iterator iter;
  for ( iter = path_c.begin(); iter != path_c.end(); iter++ ){
    Node d = navGraph->getNode(*iter);
    pair<int, int> p(d.getX(), d.getY());
    path_c_points.push_back(p);
  }

  if(t_invalid) {
    pair<int, int> tg(x2,y2);
    path_c_points.push_back(tg);
  }

  return path_c_points;
}


int PathPlanner::getPathLength(list<pair<int,int> > path){
  double length = 0;
  list<pair<int,int> >::iterator iter, iter_next;
  for ( iter = path.begin(); iter != path.end(); iter++ ){
    iter_next = iter;
    iter_next++;
    if ( iter_next != path.end() ) {
      length += Map::distance(iter->first, iter->second, iter_next->first, iter_next->second);
    }
  }
  return static_cast<int>(length);
}


double PathPlanner::calcPathCost(list<int> p){
  double pcost = 0;
  list<int>::iterator iter;
  int first;
  Edge * e;

  for( iter = p.begin(); iter != p.end() ; iter++ ){
    first = *iter++;
    if (iter != p.end()){
      e = navGraph->getEdge(first, *iter);
      pcost += e->getCost(true);
    }
    iter--;
  }
  // add reaching from source and to target costs
  // Note: This will double count when called from estimateCost() and calcPath()

  if ( source.getID() != Node::invalid_node_index && target.getID() != Node::invalid_node_index ){
    if ( !p.empty() ){
      pcost += Map::distance(source.getX(), source.getY(),
					     navGraph->getNode(p.front()).getX(),
					     navGraph->getNode(p.front()).getY());
      pcost += Map::distance(navGraph->getNode(p.back()).getX(),
					     navGraph->getNode(p.back()).getY(),
					     target.getX(), target.getY());
    }
    else
      pcost += Map::distance(source.getX(), source.getY(),
					     target.getX(), target.getY());
  }

  return pcost;
}

double PathPlanner::estimateCost(int x1, int y1, int x2, int y2) {
  Node source(1, x1, y1);
  Node target(1, x2, y2);

  return estimateCost(source, target, 1);
}

double PathPlanner::estimateCost(Node s, Node t, int l){
  Node sn = getClosestNode(s, t);
  Node tn = getClosestNode(t, s);

  if(tn.getID() < 0 || sn.getID() < 0)
    return INT_MAX;

  double pathCost = Map::distance(s.getX(), s.getY(), sn.getX(), sn.getY());
  pathCost += Map::distance(t.getX(), t.getY(), tn.getX(), tn.getY());

  astar nsearch(*navGraph, sn, tn);
  if ( nsearch.isPathFound() ){
    list<int> p = nsearch.getPathToTarget();
    pathCost += calcPathCost(p);
  }
  else {
    if(PATH_DEBUG)
      cout << "PathPlanner::estimateCost> no path found to target! "
	   << "Source accessible: " << sn.isAccessible()
	   << ", target accessible: " << tn.isAccessible() << endl;
    pathCost = INT_MAX;
  }

  return pathCost;
}

double PathPlanner::getRemainingPathLength(double x, double y) {
  list<pair<int,int> > path_xy;

  pair<int,int> s(x, y);
  path_xy.push_back(s); 

  list<int>::iterator iter;
  for(iter = path.begin(); iter != path.end(); iter++) {
    Node d = navGraph->getNode(*iter);
    pair<int, int> p(d.getX(), d.getY());
    path_xy.push_back(p);
  }
  
  pair<int,int> t(target.getX(), target.getY());
  path_xy.push_back(t); 

  return getPathLength(path_xy);
}


/*!
  \brief Returns the closest Node in the navigation graph to an arbirary \f$(x, y)\f$ position.

  \param Node \c n, any \f$(x, y)\f$ on the map which we are searching for the closest node
  \param Node \c ref, any \f$(x,y)\f$ on the map, which we are using as a guiding point
  \return Node, this is a member node of the navigation graph, or an invalid node if not found

  This function is used to determine the closest member nodes of the navigation graph to target and the source, 
  since the A* runs only over the member nodes.

  It first asks for the nodes within a region from the navigation graph and returns an accessible node in 
  that area, that has the minimum total distances from the itself to \c n and \c ref nodes. 

  The radius of the search region is initially set to the \c proximity defined by the \c Graph class. 
  If no accessible nodes are found, the radius of the search area is increased and the search is repeated. 
  The maximum search radius is set to 1.5 * proximity. If there are no accessible nodes within that distance
  the robot is probably surrounded by obstacles, therefore this function returns an invalid node.

 */
Node PathPlanner::getClosestNode(Node n, Node ref){
  const string signature = "PathPlanner::getClosestNode()> ";
  
  Node temp;
  double s_radius = navGraph->getProximity(); 
  double max_radius = navGraph->getProximity() * 1.5; 

  do {
    
    if(PATH_DEBUG)
      cout << signature << "Searching for the closest node within " << s_radius << endl; 

    vector<Node*> nodes = navGraph->getNodesInRegion(n.getX(), n.getY(), s_radius);

    double dist = INT_MAX;

    vector<Node*>::iterator iter;
    for( iter = nodes.begin(); iter != nodes.end(); iter++ ){
      double d = Map::distance( (*iter)->getX(), (*iter)->getY(), n.getX(), n.getY() );
      
      if(PATH_DEBUG){
	cout << "\tChecking ";
	(*iter)->printNode();
	cout << endl; 
	cout << "\tDistance between the n and this node: " << d << endl; 
      }
      
      double d_t = 0.0;
      if(ref.getID() != Node::invalid_node_index)
	d_t = Map::distance((*iter)->getX(), (*iter)->getY(), ref.getX(), ref.getY());

      if(PATH_DEBUG) 
	cout << "\tDistance between this node to ref: " << d_t << endl; 
      // d + d_t < dist, was the earlier version, not sure why?
      if (( d < dist ) &&
	  !map.isPathObstructed( (*iter)->getX(), (*iter)->getY(), n.getX(), n.getY()) &&
	  (*iter)->isAccessible()) {
	//cout << "Checking if node is accesible : " << (*iter)->getX() << " " << (*iter)->getY()  << endl; 
	dist = d + d_t;
	temp = (*(*iter));
	if(PATH_DEBUG) {
	  cout << "\tFound a new candidate!: "; 
	  temp.printNode();
	  cout << endl << endl;
	}
      }
    }
    
    if(temp.getID() == Node::invalid_node_index) {
      s_radius += 0.1 * s_radius; 
      if(PATH_DEBUG) 
	cout << signature << "Didn't find a suitable candidate. Increasing search radius to: " << s_radius << endl; 
    }

  } while(temp.getID() == Node::invalid_node_index && s_radius <= max_radius); 

  return temp;
}

/*!
  \brief Used to remove extra points off of the path
 */
void PathPlanner::smoothPath(list<int>& pathCalc, Node s, Node t){
  int proximity = navGraph->getProximity();

  if ( pathCalc.size() > 1 ) {
    // smooth the end points
    // if getting to second node from source is shorter and not obstructed remove first node.
    list<int>::iterator iter = pathCalc.begin();
    Node first = navGraph->getNode(*iter++);
    Node second = navGraph->getNode(*iter);
    iter--; // point back to the first element

    if ( PATH_DEBUG ) {
      cout << "source: ";
      s.printNode();
      cout << " - first: " ;
      first.printNode();
      cout << " - second: " ;
      second.printNode();
      cout << endl ;
    }

    if ( !map.isPathObstructed(s.getX(), s.getY(), second.getX(), second.getY()) &&
	 Map::distance( s.getX(), s.getY(), second.getX(), second.getY() ) + proximity * 0.5 <
	 ( Map::distance( s.getX(), s.getY(), first.getX(), first.getY() ) +
	   Map::distance( first.getX(), first.getY(), second.getX(), second.getY() ) )){
      if ( PATH_DEBUG ) cout << "Erasing first" << endl;
      pathCalc.erase(iter);
    }
  }

  if ( pathCalc.size() > 1 ) {
    // if getting to second node from source is shorter and not obstructed remove first node.
    list<int>::iterator iter = pathCalc.end();
    Node last = navGraph->getNode(*(--iter));
    Node onebeforelast = navGraph->getNode(*(--iter));
    iter++;

    if ( PATH_DEBUG ){
      cout << "onebeforelast: ";
      onebeforelast.printNode();
      cout << " - last: " ;
      last.printNode();
      cout << " - target: " ;
      t.printNode();
      cout << endl;
    }

    if ( !map.isPathObstructed(onebeforelast.getX(), onebeforelast.getY(), t.getX(), t.getY()) &&
	 Map::distance( onebeforelast.getX(), onebeforelast.getY(), t.getX(), t.getY() ) + proximity * 0.5 <
	 ( Map::distance( onebeforelast.getX(), onebeforelast.getY(), last.getX(), last.getY() ) +
	   Map::distance( last.getX(), last.getY(), t.getX(), t.getY() ) )){
      if ( PATH_DEBUG ) cout << "Erasing last" << endl ;
      pathCalc.erase(iter);
    }
  }

  if ( PATH_DEBUG ) {
    cout << "after smoothing: " << endl;
    printPath(pathCalc);
  }
}

/*!
  \brief Prints the path node by node
 */
void PathPlanner::printPath(){
  list<int>::iterator it;
  for ( it = path.begin(); it != path.end(); it++ ){
    navGraph->getNode(*it).printNode() ;
    cout << endl;
  }
}

void PathPlanner::printPath(list<int> p){
  list<int>::iterator it ;
  for ( it = p.begin(); it != p.end(); it++ ){
    navGraph->getNode(*it).printNode() ;
    cout << endl;
  }
}


void PathPlanner::printPath(list<pair<int,int> > p) {
  list<pair<int,int> >::iterator it ;
  for ( it = p.begin(); it != p.end(); it++ ){
    int nodeId = navGraph->getNodeID(it->first, it->second);
    if ( nodeId == -1 ) {
      cout << "Not a graph node - (" << it->first << ", " << it->second << ")" << endl;
    }
    else {
      navGraph->getNode(nodeId).printNode() ;
      cout << endl;
    }
  }
}


bool PathPlanner::allWaypointsValid() {
  list<int>::iterator it;
  for(it = path.begin(); it != path.end(); ++it) {
    if(!navGraph->getNode(*it).isAccessible())
      return false;
  }
  return true;
}

/*! @} */
