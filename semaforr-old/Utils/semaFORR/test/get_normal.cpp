#include "FORRGeometry.h"
#include <iostream>

using std::cout;
using std::endl;

double get_normal(double slope);
pair<CartesianPoint, CartesianPoint> get_points_at_distance(CartesianPoint point, double distance, Line line);

int main(){

  Line my_line = Line(1,-1,15,-15);
  double dist = 9;
  CartesianPoint la_point = CartesianPoint(3, -3);
  cout << "Slope is " << my_line.get_slope() << endl;
  cout << "And the normal is " << get_normal(my_line.get_slope()) << endl;

  pair<CartesianPoint, CartesianPoint> result_pair = get_points_at_distance(la_point, dist, my_line);
  cout << "One point is: " << result_pair.first.get_x() << " " <<  result_pair.first.get_y() << endl;
  cout << "One point is: " << result_pair.second.get_x() << " " <<  result_pair.second.get_y() << endl;

  return 0;
}

double get_normal(double slope){
  double answer;
  if(slope == std::numeric_limits<double>::infinity())
    answer = 0;
  else if(slope == 0)
    answer = std::numeric_limits<double>::infinity();
 else
   answer = -1 / slope;

  return answer;
}

pair<CartesianPoint, CartesianPoint> get_points_at_distance(CartesianPoint point, double distance, Line line){
  double normal = get_normal(line.get_slope());
  double normal_angle = atan(normal);
  CartesianPoint first = CartesianPoint(point.get_x() + distance*cos(normal_angle), point.get_y() - distance*sin(normal_angle));
  CartesianPoint second = CartesianPoint(point.get_x() - distance*cos(normal_angle), point.get_y() + distance*sin(normal_angle));

  return std::make_pair(first, second);
}
