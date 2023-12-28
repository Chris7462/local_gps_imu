#include "kalman_filter/extended_kalman_filter.hpp"


namespace kalman
{

ExtendedKalmanFilter::ExtendedKalmanFilter()
{
  x_.setZero();
  P_.setIdentity();
  I_.setIdentity();
}

void ExtendedKalmanFilter::init(const State & x, const StateCov & P)
{
  x_ = x;
  P_ = P;
}

const State & ExtendedKalmanFilter::getState() const
{
  return x_;
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
  s.updateJacobians(x_, u, dt);

  // predict state
  x_ = s.f(x_, u, dt);

  // predict covariance
  P_ = s.F_ * P_ * s.F_.transpose() + s.W_ * s.Q_ * s.W_.transpose();
}

void ExtendedKalmanFilter::wrapStateYaw()
{
  x_.theta() = wrap2pi(x_.theta());
}

double ExtendedKalmanFilter::limitMeasurementYaw(double yaw)
{
  double yaw_max = x_.theta() + M_PI;
  double yaw_min = x_.theta() - M_PI;
  while ((yaw > yaw_max) || (yaw < yaw_min)) {
    if (yaw > yaw_max) {
      yaw -= 2.0 * M_PI;
    } else if (yaw < yaw_min) {
      yaw += 2.0 * M_PI;
    }
  }
  return yaw;
}

double ExtendedKalmanFilter::wrap2pi(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

} // namespace kalman
