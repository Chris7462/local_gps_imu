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
  void updateJacobian(const State & x);

  friend class kalman::ExtendedKalmanFilter;

private:
  ImuMeasurementJacobian H_;
  ImuNoiseJacobian V_;
  ImuMeasurementCov R_;
};


ImuMeasurementModel::ImuMeasurementModel()
{
  H_ = ImuMeasurementJacobian::Zero();
  V_ = ImuNoiseJacobian::Zero();
  R_ = ImuMeasurementCov::Identity();
}

void ImuMeasurementModel::setCovariance(const ImuMeasurementCov & R)
{
  R_ = R;
}

ImuMeasurement ImuMeasurementModel::h(const State & x) const
{
  ImuMeasurement measurement;
  measurement.omega() = x.omega();
  measurement.alpha() = x.alpha();

  return measurement;
}

void ImuMeasurementModel::updateJacobian(const State & x)
{
  (void)x;  // not using state. Just put this line to silence compiler warning.

  // H = dh/dx (Jacobian of measurement transition w.r.t the state)
  H_.setZero();
  H_(ImuMeasurement::OMEGA, State::OMEGA) = 1.0;
  H_(ImuMeasurement::ALPHA, State::ALPHA) = 1.0;

  // V = dh/ddelta (Jacobian of measurement trasition w.r.t the noise)
  V_.setIdentity();
}

} // namespace kalman
