#pragma once

#include "kalman_filter/matrix.hpp"

namespace kalman
{

class State : public kalman::Vector<6>
{
public:
  KALMAN_VECTOR(State, 6)
  enum : uint8_t
  {
    X,
    Y,
    THETA,
    NU,
    OMEGA,
    ALPHA
  };

  inline double x() const {return (*this)[X];}
  inline double y() const {return (*this)[Y];}
  inline double theta() const {return (*this)[THETA];}
  inline double nu() const {return (*this)[NU];}
  inline double omega() const {return (*this)[OMEGA];}
  inline double alpha() const {return (*this)[ALPHA];}

  inline double & x() {return (*this)[X];}
  inline double & y() {return (*this)[Y];}
  inline double & theta() {return (*this)[THETA];}
  inline double & nu() {return (*this)[NU];}
  inline double & omega() {return (*this)[OMEGA];}
  inline double & alpha() {return (*this)[ALPHA];}
};

} // namespace kalman
