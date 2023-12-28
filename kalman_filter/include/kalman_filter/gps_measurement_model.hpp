#pragma once

#include <Eigen/Dense>

#include "kalman_filter/state.hpp"
#include "kalman_filter/gps_measurement.hpp"


namespace kalman
{

class ExtendedKalmanFilter;

using GpsMeasurementJacobian = Eigen::Matrix<double, GpsMeasurementSize, StateSize>;
using GpsNoiseJacobian = Eigen::Matrix<double, GpsMeasurementSize, GpsMeasurementSize>;
using GpsMeasurementCov = Eigen::Matrix<double, GpsMeasurementSize, GpsMeasurementSize>;

class GpsMeasurementModel
{
public:
  GpsMeasurementModel();
  ~GpsMeasurementModel() = default;

  void setCovariance(const GpsMeasurementCov & R);
  GpsMeasurement h(const State & x) const;
  void updateJacobians(const State & x);

  friend class kalman::ExtendedKalmanFilter;

private:
  GpsMeasurementJacobian H_;
  GpsNoiseJacobian V_;
  GpsMeasurementCov R_;
};

} // namespace kalman
