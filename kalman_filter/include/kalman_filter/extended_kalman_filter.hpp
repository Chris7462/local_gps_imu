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
  const State & getState() const;

  void predict(SystemModel & s, const double dt);
  void predict(SystemModel & s, const Control & u, const double dt);

  template<typename MeasurementModel, typename Measurement>
  void update(MeasurementModel & m, const Measurement & z);

  template<typename MeasurementModel, typename Measurement>
  double mahalanobis(MeasurementModel & m, const Measurement & z);

  void wrapStateYaw();
  double limitMeasurementYaw(const double offset);

protected:
  double wrap2pi(const double angle);

private:
  State x_;
  StateCov P_;
  StateCov I_;
};

template<typename MeasurementModel, typename Measurement>
void ExtendedKalmanFilter::update(MeasurementModel & m, const Measurement & z)
{
  m.updateJacobians(x_);

  // compute innovation covariance
  MeasurementCov S = (m.H_ * P_ * m.H_.transpose()) + (m.V_ * m.R_ * m.V_.transpose());

  // compute kalman gain
  KalmanGain K = P_ * m.H_.transpose() * S.inverse();

  // update state estimate
  x_ += K * (z - m.h(x_));

  // update covariance
  //P_ -= K * m.H_ * P_;
  P_ = ((I_ - K * m.H_) * P_ * (I_ - K * m.H_).transpose()) +
    (K * m.V_ * m.R_ * m.V_.transpose() * K.transpose());
}

template<typename MeasurementModel, typename Measurement>
double ExtendedKalmanFilter::mahalanobis(MeasurementModel & m, const Measurement & z)
{
  m.updateJacobians(x_);

  // compute innovation covariance
  MeasurementCov S = (m.H_ * P_ * m.H_.transpose()) + (m.V_ * m.R_ * m.V_.transpose());

  return (z - m.h(x_)).transpose() * S.inverse() * (z - m.h(x_));
}

} // namespace kalman
