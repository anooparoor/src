#include "Teammate.h"

Teammate::Teammate(long i, double x, double y, double t, int s) 
  : id_(i), size_(s), proximity_(0) {
  curr_pos_.setX(x);
  curr_pos_.setY(y);
  curr_pos_.setTheta(t);
}

void Teammate::updateCollisionStatus(Position p) {
  // update proximity
  proximity_ = Utils::get_euclidian_distance(curr_pos_.getX(), 
					     curr_pos_.getY(), 
					     p.getX(), 
					     p.getY()) ;

  // update the relative heading
  double delta_y, delta_x, theta;
  delta_y = curr_pos_.getY() - p.getY();
  delta_x = curr_pos_.getX() - p.getX();
  theta = atan2(delta_y, delta_x);
  relative_angle_ =  fabs(p.getTheta() - theta);
}

void Teammate::print() const {
    cout << "R-" << id_ << ":("
        << static_cast<int>(curr_pos_.getX()) << "," << static_cast<int>(curr_pos_.getY()) << ","
        << static_cast<int>(Utils::toDegrees(curr_pos_.getTheta())) << ")-d:"
        << static_cast<int>(proximity_) ;
}

