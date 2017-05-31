
#ifndef FORRGEOMETRY_H
#define FORRGEOMETRY_H
/*
 * Declaration of geometry objects and functions that will form
 * foundation of our simulator. We will use this functions for
 * predictions of movements and fake sensing.
 *
 * Last modified: March 25, 2014.
 * Created by: Slavisa Djukic <sdjukic@hunter.cuny.edu>
 *
 */

/**********************************************************************
                     TO DO:
 - Line should be created by point and slope in radians
 - Should there be PolarPoint class there ?
 - Line and LineSegment should throw exception when we try to create them by passing same point (there is no line between single point)
 - Vector cannot have negative intensity
*********************************************************************/
#include <cmath>
#include <algorithm>
#include <limits>
#include <utility>

// our computation is involving doubles we have to define error
// we are comfortable with
#define ERROR 0.01

using std::pair;

// forward declaration of this class
class Line;
class LineSegment;
class Vector;
class Circle;

/****************************************************************
                Class CartesianPoint
 defined by ordered pair (x,y) which determine its position in 
 the cartesian coordinate system.
***************************************************************/
class CartesianPoint{
 public:
  /********************************************************************
                    constructors
  ********************************************************************/
  CartesianPoint ();                       // initializes point to (0,0)
  CartesianPoint (double x_c, double y_c); 
  CartesianPoint (const CartesianPoint& other);    


  /********************************************************************
                     equality operator 
  ********************************************************************/
  bool operator==(const CartesianPoint& lhs) const;

  /********************************************************************
                       accessors and mutators
  ********************************************************************/
  void set_x(double new_x);
  void set_y(double new_y);
  double get_x() const;
  double get_y() const;
  double get_distance(CartesianPoint point);


  /********************************************************************
                        friends
  ********************************************************************/
  friend double distance(CartesianPoint first, CartesianPoint second);
  
  friend double distance(CartesianPoint point, Line line);
  
  friend double distance(CartesianPoint point, LineSegment segment);
  
  friend bool is_point_on_line (CartesianPoint point, Line line);
  
  friend bool is_point_in_segment(CartesianPoint point, LineSegment segment);
  
  friend bool do_intersect (Line first, Line second, CartesianPoint& point_of_intersection);

  friend bool do_intersect(Vector vector, LineSegment line_segment, CartesianPoint& intersection);

  friend bool do_intersect(Vector vector1, Vector vector2, CartesianPoint& intersection);

  /*******************************************************************
                       data members
  ********************************************************************/
 private:
  double x;
  double y;
};



/*********************************************************************
                        Class PolarPoint
 this point is defined by its distance from the origin and the angle
 with x-axis at which this distance is measured.
*********************************************************************/

/*********************************************************************
                       Class Line
 class represents line. In our case line is defined by any two points
 that lie on it.
*********************************************************************/
class Line{
 public:
  /********************************************************************
                    constructors
  ********************************************************************/
  Line (CartesianPoint first, CartesianPoint second);
  Line (double x1, double y1, double x2, double y2);
  Line (const Line& other);
  Line (){};

  /********************************************************************
                      accessors and mutators
   Internally line is represented as Ax + By = C  i.e. line is not defined by its two points but these parameters.
  ********************************************************************/
  double get_value_a() const;
  double get_value_b() const;
  double get_value_c() const;

  void set_value_a(double new_a);
  void set_value_b(double new_b);
  void set_value_c(double new_c);

  double get_slope() const;
  /********************************************************************
          friends for common functionality between classes
  ********************************************************************/
  friend double distance(CartesianPoint point, Line line);

  // this function will return true or false whether two lines intersect
  // third argument here is in case lines intersect it stores point of intersection
  friend bool do_intersect (Line first, Line second, CartesianPoint& point_of_intersection);

  friend bool is_point_on_line (CartesianPoint point, Line line); 


  /*******************************************************************
                       data members
  *******************************************************************/
 protected:
  double coefficient_a;
  double coefficient_b;
  double coefficient_c;
};



/**********************************************************************
                      Class LineSegment
    the same as line, except it is bounded by its endpoints
**********************************************************************/
class LineSegment: public Line{
 public:
  /******************************************************************
                        Constructors
  ******************************************************************/
  LineSegment(CartesianPoint first, CartesianPoint second);
  LineSegment(double x1, double y1, double x2, double y2);
  LineSegment(){};
  //LineSegment(const LineSegment& other);


  /*******************************************************************
                         Accessors
  ********************************************************************/
  pair<CartesianPoint, CartesianPoint> get_endpoints();  

  /*  
    function as in do_lines_intersect will store the point of intersection
    if two segments intersect
    if they don't intersect the value is undefined
  */
  friend double distance(CartesianPoint point, LineSegment segment);
  
  friend bool is_point_in_segment(CartesianPoint point, LineSegment segment);

  friend bool do_intersect(LineSegment first, LineSegment second, CartesianPoint& intersection);

  friend bool do_intersect(Vector vector, LineSegment line_segment, CartesianPoint& intersection);

  /*******************************************************************
                        data members
  ********************************************************************/
 protected:
  CartesianPoint end_point_1;
  CartesianPoint end_point_2;
};



/***********************************************************************
                    Class Vector
 - defined by a CartesianPoint (I call it vector's origin)
 - angle theta in radians (angle between positive x-axis and direction of the vector)
 - intensity
***********************************************************************/
class Vector{
 public:
  /********************************************************************
                          Constructors
  ********************************************************************/
  Vector(CartesianPoint point, double angle, double intensity);
  Vector(double x1, double y1, double angle, double intensity);


  /********************************************************************
                     accessors and mutators
  *********************************************************************/
  CartesianPoint get_origin() const;
  double get_angle() const;
  double get_intensity() const;
  double turn_vector(double angle);
  CartesianPoint get_endpoint();
  CartesianPoint get_point(double distance, double angle);

  /********************************************************************
                     friends for interclass calculations
  ********************************************************************/
  friend bool do_intersect(Vector vector1, Vector vector2, CartesianPoint& intersection);

  friend bool do_intersect(Vector vector, Line line, CartesianPoint& intersection);  

  friend bool do_intersect(Vector vector, LineSegment line_segment, CartesianPoint& intersection);


  /********************************************************************
                       data members
  *********************************************************************/
 private:
  CartesianPoint origin;
  double theta;
  double intensity;
};

class Circle{
 public:
  //constructors
  Circle(CartesianPoint center, double radius);
  //getters and setters
  CartesianPoint get_center();
  double get_radius();
  //verify intersection 
  friend bool do_intersect(Circle circle, Line line);
  friend CartesianPoint intersection_point(Circle circle, LineSegment line_segment);
 private:
  CartesianPoint center;
  double radius;
};


#endif
