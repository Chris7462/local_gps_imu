#pragma once

#include <Eigen/Dense>

#include "kalman_filter/state.hpp"
#include "kalman_filter/imu_measurement.hpp"


namespace kalman
{

class ExtendedKalmanFilter;

using ImuMeasurementJacobian = Eigen::Matrix<double, ImuMeasurementSize, StateSize>;
using ImuNoiseJacobian = Eigen::Matrix<double, ImuMeasurementSize, ImuMeasurementSize>;
using ImuMeasurementCov = Eigen::Matrix<double, ImuMeasurementSize, ImuMeasurementSize>;

class ImuMeasurementModel
{
public:
  ImuMeasurementModel();
  ~ImuMeasurementModel() = default;

  void setCovariance(const ImuMeasurementCov & R);
  ImuMeasurement h(const State & x) const;
  void updateJacobians(const State & x);

  friend class kalman::ExtendedKalmanFilter;

private:
  ImuMeasurementJacobian H_;
  ImuNoiseJacobian V_;
  ImuMeasurementCov R_;
};

} // namespace kalman
