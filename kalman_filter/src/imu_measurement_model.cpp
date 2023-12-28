#include "kalman_filter/imu_measurement_model.hpp"


namespace kalman
{

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
  measurement.theta() = x.theta();
  measurement.omega() = x.omega();
  measurement.alpha() = x.alpha();

  return measurement;
}

void ImuMeasurementModel::updateJacobians(const State & x)
{
  (void)x;  // not using state. Just put this line to silence compiler warning.

  // H = dh/dx (Jacobian of measurement transition w.r.t the state)
  H_.setZero();
  H_(ImuMeasurement::THETA, State::THETA) = 1.0;
  H_(ImuMeasurement::OMEGA, State::OMEGA) = 1.0;
  H_(ImuMeasurement::ALPHA, State::ALPHA) = 1.0;

  // V = dh/ddelta (Jacobian of measurement trasition w.r.t the noise)
  V_.setIdentity();
}

} // namespace kalman
