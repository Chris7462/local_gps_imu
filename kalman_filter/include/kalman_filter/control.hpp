#pragma once

#include <Eigen/Dense>


namespace kalman
{

constexpr int ControlSize = 2;

class Control : public Eigen::Matrix<double, ControlSize, 1>
{
public:
  enum : uint8_t
  {
    OMEGA,
    ALPHA
  };

  Control() = default;
  ~Control() = default;

  inline double omega() const {return (*this)[OMEGA];}
  inline double alpha() const {return (*this)[ALPHA];}

  inline double & omega() {return (*this)[OMEGA];}
  inline double & alpha() {return (*this)[ALPHA];}
};

using ControlCov = Eigen::Matrix<double, ControlSize, ControlSize>;

} // namespace kalman
