#pragma once

#include <Eigen/Dense>

namespace kalman
{

constexpr int ImuMeasurementSize = 2;

class ImuMeasurement : public Eigen::Matrix<double, ImuMeasurementSize, 1>
{
public:
  enum : uint8_t
  {
    OMEGA,
    ALPHA
  };

  ImuMeasurement() = default;
  ~ImuMeasurement() = default;

  inline double omega() const {return (*this)[OMEGA];}
  inline double alpha() const {return (*this)[ALPHA];}

  inline double & omega() {return (*this)[OMEGA];}
  inline double & alpha() {return (*this)[ALPHA];}
};

using ImuMeasurementCov = Eigen::Matrix<double, ImuMeasurementSize, ImuMeasurementSize>;

} // namespace kalman
