#pragma once

#include <Eigen/Dense>


namespace kalman
{

constexpr int GpsMeasurementSize = 2;

class GpsMeasurement : public Eigen::Matrix<double, GpsMeasurementSize, 1>
{
public:
  enum : uint8_t
  {
    X,
    Y
  };

  GpsMeasurement() = default;
  ~GpsMeasurement() = default;

  inline double x() const {return (*this)[X];}
  inline double y() const {return (*this)[Y];}

  inline double & x() {return (*this)[X];}
  inline double & y() {return (*this)[Y];}
};

using GpsMeasurementCov = Eigen::Matrix<double, GpsMeasurementSize, GpsMeasurementSize>;

} // namespace kalman
