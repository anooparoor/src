/*
 * Implementation file for our FORRGeometry module.
 *
 * Last modified: March 25, 2014.
 * Created by: Slavisa Djukic <sdjukic@hunter.cuny.edu>
 *
 */

#include <iostream>
#include "FORRGeometry.h"

using std::cout;
using std::endl;
using std::min;
using std::max;

/**********************************************************************
                 CartesianPoint constructor section
***********************************************************************/
CartesianPoint::CartesianPoint(): x(0), y(0) {}

CartesianPoint::CartesianPoint(double x_c, double y_c): x(x_c), y(y_c) {}

CartesianPoint::CartesianPoint(const CartesianPoint& other){
  x = other.x;
  y = other.y;
}

/**********************************************************************
                 Cartesian Point getters and setters
**********************************************************************/
void CartesianPoint::set_x(double new_x) { x = new_x; }

void CartesianPoint::set_y(double new_y) { y = new_y; }

double CartesianPoint::get_x() const { return x; }

double CartesianPoint::get_y() const { return y; }

double CartesianPoint::get_distance(CartesianPoint point){
	
	return sqrt((x - point.x)*(x - point.x) + (y - point.y)*(y - point.y));
}

/**********************************************************************
                  equality operator
**********************************************************************/
bool CartesianPoint::operator==(const CartesianPoint& lhs) const{
  return (abs(this->x - lhs.x) < ERROR && abs(this->y - lhs.y) < ERROR);
}



/***********************************************************************
                Line class implementation
***********************************************************************/
Line::Line(CartesianPoint first, CartesianPoint second): 
  coefficient_a(second.get_y() - first.get_y()),
  coefficient_b(first.get_x() - second.get_x()),
  coefficient_c(coefficient_a * first.get_x() + coefficient_b * first.get_y()) {}

Line::Line(double x1, double y1, double x2, double y2):
  coefficient_a(y2 - y1),
  coefficient_b(x1 - x2),
  coefficient_c(coefficient_a * x1 + coefficient_b * y1) {}

Line::Line(const Line& other) {
  coefficient_a = other.coefficient_a;
  coefficient_b = other.coefficient_b;
  coefficient_c = other.coefficient_c;
}

  
/**********************************************************************
                  Line getters and setters
***********************************************************************/
double Line::get_value_a() const { return coefficient_a; }
double Line::get_value_b() const { return coefficient_b; }
double Line::get_value_c() const { return coefficient_c; }

void Line::set_value_a(double new_a) { coefficient_a = new_a; }
void Line::set_value_b(double new_b) { coefficient_b = new_b; }
void Line::set_value_c(double new_c) { coefficient_c = new_c; }

double Line::get_slope() const{
  double answer;
  if(coefficient_a == 0)
    answer = 0;
  else if(coefficient_b == 0)   // don't divide by zero
    answer = std::numeric_limits<double>::infinity();
  else
    answer = -coefficient_a / coefficient_b;

  return answer;
}
	      


/**********************************************************************
                  LineSegment constructors
**********************************************************************/
LineSegment::LineSegment(CartesianPoint first, CartesianPoint second): Line(first, second), 
								       end_point_1(first), 
								       end_point_2(second) {}

LineSegment::LineSegment(double x1, double y1, double x2, double y2): 
  Line(x1, y1, x2, y2), 
  end_point_1(CartesianPoint(x1, y1)), 
  end_point_2(CartesianPoint(x2, y2)) {}


/*********************************************************************
                        Accessors
**********************************************************************/
/*
 * Does not make sense to return single end point of LineSegment
 * because there is no way we can consistently distinguish them.
 * Well there is less than operator, but I did not implement it 
 * and that would just transfer problem to higher level of abstraction.
 * This way we get the end points and we can do whatever needs to
 * be done.
 */
pair<CartesianPoint, CartesianPoint> LineSegment::get_endpoints(){
  return std::make_pair(end_point_1, end_point_2);
}
/*LineSegment::LineSegment(const LineSegment& other): 
  coefficient_a(other.coefficient_a), 
  coefficient_b(other.coefficient_b), 
  coefficient_c(other.coefficient_c), 
  end_point_1(other.end_point_1), 
  end_point_2(other.end_point_2) {}*/



/**********************************************************************
                  Vector constructors
**********************************************************************/
Vector::Vector(CartesianPoint point, double angle, double intensty): 
  origin(point), theta(angle), intensity(intensty) {
  int ratio;
  if((angle > (2 * M_PI)) || (angle < -2 * M_PI)){
    ratio = static_cast<int>(angle / (2 * M_PI));
    theta = angle - ratio * 2 * M_PI;
  } 
 else
     theta = angle;
}

Vector::Vector(double x1, double y1, double angle, double intensty):
  origin(CartesianPoint(x1, y1)), theta(angle), intensity(intensty) {}


/**********************************************************************
                  accessors and mutators
**********************************************************************/
CartesianPoint Vector::get_origin() const{ return origin; }

double Vector::get_angle() const { return theta; }

double Vector::get_intensity() const { return intensity; }

double Vector::turn_vector(double angle){
  
   double new_angle = this->get_angle() + angle;
   //new_angle = Utils::normalizeAngle(new_angle);
   return new_angle;
}

CartesianPoint Vector::get_endpoint(){
  return get_point(this->get_intensity(), 0);
}

CartesianPoint Vector::get_point(double distance, double angle){

  double turned_angle = this->turn_vector(angle);
  CartesianPoint point = this->get_origin();

  //cout << "Casting new points at angle: " << normal_angle << " and from " << point.get_x() << " " << point.get_y() << endl;
  return CartesianPoint(point.get_x() + distance*cos(turned_angle), point.get_y() + distance*sin(turned_angle));
}



/**********************************************************************
     friend functions for common interclass computations
*********************************************************************/
double distance(CartesianPoint first, CartesianPoint second){
  return sqrt((first.x - second.x)*(first.x - second.x) + (first.y - second.y)*(first.y - second.y));
}


// Formula taken from Wikipedia
double distance(CartesianPoint point, Line line){
  return abs(line.coefficient_a * point.x + line.coefficient_b * point.y - line.coefficient_c) / sqrt (line.coefficient_a*line.coefficient_a + line.coefficient_b*line.coefficient_b);
}


double distance(CartesianPoint point, LineSegment segment){
  double tmp_distance;
  if((point.x > min(segment.end_point_1.x , segment.end_point_2.x) 
      && point.x < max(segment.end_point_1.x , segment.end_point_2.x)) 
     ||(point.y > min(segment.end_point_1.y , segment.end_point_2.y) 
	&& point.y < max(segment.end_point_1.y , segment.end_point_2.y)))
    // In this case we can reuse distance between point and line function
    tmp_distance = distance(point, static_cast<Line>(segment));
  
  else
    tmp_distance = min(distance(point, segment.end_point_1), distance(point, segment.end_point_2));

  return tmp_distance;
}


bool is_point_on_line(CartesianPoint point, Line line){
  return abs(line.coefficient_a * point.x + line.coefficient_b * point.y - line.coefficient_c) < ERROR;
}


bool is_point_in_segment(CartesianPoint point, LineSegment segment){
  // when all else failed I used triangular equation  
  return (abs(distance(point, segment.end_point_1) + distance(point, segment.end_point_2) - distance(segment.end_point_1, segment.end_point_2)) < ERROR);
}

//constructors
Circle::Circle(CartesianPoint center, double radius):center(center), radius(radius){}

//getters and setters
CartesianPoint Circle::get_center(){
  return center;
}

double Circle::get_radius(){
  return radius;
}

CartesianPoint intersection_point(Circle circle, LineSegment line_segment){
  double r = circle.get_radius();
  double cx = circle.get_center().get_x();
  double cy = circle.get_center().get_y();
  double m = line_segment.get_slope();
  double b = (line_segment.get_value_c() / line_segment.get_value_b());
  double A = (m*m) + 1;
  double B = 2*(m*b - m*cy - cx);
  double C = (cx*cx) + (b*b) + (cy*cy) - (2*b*cy) - (r*r);
  //cout << r << " " << cx << " " << cy << " " << m << " " << b << " " << A << " " << B << " " << C << endl;
  // solution of the equation of point of intersection of the line segment and the circle assuming they intersect
  double firstx = (-B + sqrt( ((B*B) - (4*A*C)) ))/(2*A);
  double firsty = m*firstx + b;
  double secondx = (-B - sqrt( ((B*B) - (4*A*C)) ))/(2*A);
  double secondy = m*secondx + b;
  //cout << firstx << " " << firsty << " " << secondx << " " << secondy << endl;
  pair<CartesianPoint, CartesianPoint> endpoints = line_segment.get_endpoints();
  if(endpoints.first.get_x() < endpoints.second.get_x() && firstx > endpoints.first.get_x() && firstx < endpoints.second.get_x()) {
    return CartesianPoint(firstx, firsty);
  } else if(endpoints.first.get_x() > endpoints.second.get_x() && firstx > endpoints.second.get_x() && firstx < endpoints.first.get_x()) {
    return CartesianPoint(firstx, firsty);
  } else if(endpoints.first.get_x() < endpoints.second.get_x() && secondx > endpoints.first.get_x() && secondx < endpoints.second.get_x()) {
    return CartesianPoint(secondx, secondy);
  } else if(endpoints.first.get_x() > endpoints.second.get_x() && secondx > endpoints.second.get_x() && secondx < endpoints.first.get_x()) {
    return CartesianPoint(secondx, secondy);
  }
}

// solve equations for the line and circle and check if there is a solution (b^2 - 4ac > 0)
bool do_intersect(Circle circle, Line line){
  double r = circle.get_radius();
  double p = circle.get_center().get_x();
  double q = circle.get_center().get_y();
  double m = line.get_slope();
  double c = -(line.get_value_c() / line.get_value_a());
  double A = (m*m) + 1;
  double B = 2*(m*c - m*q - p);
  double C = (q*q) - (r*r) + (p*p) - (2*c*q) + (c*c);
  // solution of the equation of point of intersection of the line and the circle
  if((B*B - 4*A*C) <= 0)
    return false;
  else
    return true;
}

bool do_intersect (Line first, Line second, CartesianPoint& point_of_intersection){
  bool answer;
  double determinant = 1.0 * first.coefficient_a * second.coefficient_b - second.coefficient_a * first.coefficient_b;

  if(determinant == 0)
    answer = false;
  
  else{
    // coordinates of the intersection point
    point_of_intersection.x = (second.coefficient_b * first.coefficient_c - first.coefficient_b * second.coefficient_c) / determinant;
    point_of_intersection.y = (second.coefficient_c * first.coefficient_a - first.coefficient_c * second.coefficient_a) / determinant;
    answer = true;
  }
  
  return answer;
}


bool do_intersect(LineSegment first, LineSegment second, CartesianPoint& intersection){
  bool answer;
  if(do_intersect(static_cast<Line>(first), static_cast<Line>(second), intersection)){
    if((is_point_in_segment(intersection, first))  && (is_point_in_segment(intersection, second)))
      answer = true;
    else
      answer = false;
  }
  else
    answer = false;

  return answer;
}


bool do_intersect(Vector vector, Line line, CartesianPoint& intersection){
  /* LineSegment from_vector = LineSegment(vector.origin, 
					CartesianPoint(vector.origin.x+cos(vector.theta)*vector.intensity, 
						       vector.origin.y + sin(vector.theta) * vector.intensity));

  
						       return do_line_segments_intersect(from_vector, line_segment, intersection);*/
   return false;
}


bool do_intersect(Vector vector, LineSegment line_segment, CartesianPoint& intersection){
  
  LineSegment from_vector = LineSegment(vector.origin, CartesianPoint(vector.origin.x + cos(vector.theta) * vector.intensity, vector.origin.y + sin(vector.theta) * vector.intensity));

  return do_intersect(line_segment,from_vector, intersection);
}


bool do_intersect(Vector vector1, Vector vector2, CartesianPoint& intersection){
  
  LineSegment from_vector1 = LineSegment(vector1.origin, CartesianPoint(vector1.origin.x + cos(vector1.theta) * vector1.intensity, vector1.origin.y + sin(vector1.theta) * vector1.intensity));

  LineSegment from_vector2 = LineSegment(vector2.origin, CartesianPoint(vector2.origin.x + cos(vector2.theta) * vector2.intensity, vector2.origin.y + sin(vector2.theta) * vector2.intensity));
  
  return do_intersect(from_vector1, from_vector2, intersection);
}


