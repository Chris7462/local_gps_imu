#include "kalman_filter/extended_kalman_filter.hpp"


namespace kalman
{

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

} // namespace kalman
