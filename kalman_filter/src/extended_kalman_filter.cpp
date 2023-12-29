#include "kalman_filter/extended_kalman_filter.hpp"


namespace kalman
{

ExtendedKalmanFilter::ExtendedKalmanFilter()
{
  x_.setZero();
  P_.setIdentity();
  I_.setIdentity();
}

void ExtendedKalmanFilter::predict(SystemModel & s)
{
  // predict without control
  Control u;
  u.setZero();
  return predict(s, u);
}

void ExtendedKalmanFilter::predict(SystemModel & s, const Control & u)
{
  s.updateJacobians(x_, u);

  // predict state
  x_ = s.f(x_, u);

  // predict covariance
  P_ = s.F_ * P_ * s.F_.transpose() + s.W_ * s.getCovariance() * s.W_.transpose();
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
