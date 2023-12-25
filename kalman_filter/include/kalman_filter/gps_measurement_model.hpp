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
  void updateJacobian(const State & x);

  friend class kalman::ExtendedKalmanFilter;

private:
  GpsMeasurementJacobian H_;
  GpsNoiseJacobian V_;
  GpsMeasurementCov R_;
};


GpsMeasurementModel::GpsMeasurementModel()
{
  H_ = GpsMeasurementJacobian::Zero();
  V_ = GpsNoiseJacobian::Zero();
  R_ = GpsMeasurementCov::Identity();
}

void GpsMeasurementModel::setCovariance(const GpsMeasurementCov & R)
{
  R_ = R;
}

GpsMeasurement GpsMeasurementModel::h(const State & x) const
{
  GpsMeasurement measurement;
  measurement.x() = x.x();
  measurement.y() = x.y();

  return measurement;
}

void GpsMeasurementModel::updateJacobian(const State & x)
{
  (void)x;  // not using state. Just put this line to silence compiler warning.

  // H = dh/dx (Jacobian of measurement transition w.r.t the state)
  H_.setZero();
  H_(GpsMeasurement::X, State::X) = 1.0;
  H_(GpsMeasurement::Y, State::Y) = 1.0;

  // V = dh/ddelta (Jacobian of measurement trasition w.r.t the noise)
  V_.setIdentity();
}

} // namespace kalman
