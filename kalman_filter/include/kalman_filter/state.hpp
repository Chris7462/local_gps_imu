#pragma once

#include <Eigen/Dense>

namespace kalman
{

constexpr int StateSize = 6;

class State : public Eigen::Matrix<double, StateSize, 1>
{
public:
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

using StateCov = Eigen::Matrix<double, StateSize, StateSize>;

} // namespace kalman
