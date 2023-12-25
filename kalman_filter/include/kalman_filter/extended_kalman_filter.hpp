#pragma once

#include "kalman_filter/system_model.hpp"
#include "kalman_filter/gps_measurement_model.hpp"
#include "kalman_filter/imu_measurement_model.hpp"


namespace kalman
{

using MeasurementCov = Eigen::MatrixXd;
using KalmanGain = Eigen::MatrixXd;

class ExtendedKalmanFilter
{
public:
  ExtendedKalmanFilter();
  ~ExtendedKalmanFilter() = default;

  void init(const State & x, const StateCov & P);

  void predict(SystemModel & s, const double dt);
  void predict(SystemModel & s, const Control & u, const double dt);

  template<typename MeasurementModel, typename Measurement>
  void update(MeasurementModel & m, const Measurement & z);

private:
  State x_;
  StateCov P_;
};


ExtendedKalmanFilter::ExtendedKalmanFilter()
{
  x_.setZero();
  P_.setIdentity();
}

void ExtendedKalmanFilter::init(const State & x, const StateCov & P)
{
  x_ = x;
  P_ = P;
}

void ExtendedKalmanFilter::predict(SystemModel & s, const double dt)
{
  // predict without control
  Control u;
  u.setZero();
  return predict(s, u, dt);
}

void ExtendedKalmanFilter::predict(SystemModel & s, const Control & u, const double dt)
{
  s.updateJacobian(x_, u, dt);

  // predict state
  x_ = s.f(x_, u, dt);

  // predict covariance
  P_ = s.F_ * P_ * s.F_.transpose() + s.W_ * s.Q_ * s.W_.transpose();
}

template<typename MeasurementModel, typename Measurement>
void ExtendedKalmanFilter::update(MeasurementModel & m, const Measurement & z)
{
  m.updateJacobian(x_);

  // compute innovation covariance
  MeasurementCov S = (m.H_ * P_ * m.H_.transpose()) + (m.V_ * m.R_ * m.V_.transpose());

  // compute kalman gain
  KalmanGain K = P_ * m.H_.transpose() * S.inverse();

  // update state estimate
  x_ += K * (z - m.h(x_));

  // update covariance
  P_ -= K * m.H_ * P_;
}

} // namespace kalman
