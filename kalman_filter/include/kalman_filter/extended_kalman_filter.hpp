#pragma once

#include "kalman_filter/state.hpp"
#include "kalman_filter/filter_base.hpp"
#include "kalman_filter/standard_base.hpp"
#include "kalman_filter/system_model.hpp"


namespace kalman
{

class ExtendedKalmanFilter : public KalmanFilterBase<State>,
  public StandardBase<State>
{
public:
  ExtendedKalmanFilter();
  ~ExtendedKalmanFilter() = default;

  void predict(SystemModel & s);
  void predict(SystemModel & s, const Control & u);

  template<typename MeasurementModel, typename Measurement>
  void update(MeasurementModel & m, const Measurement & z);

  template<typename MeasurementModel, typename Measurement>
  double mahalanobis(MeasurementModel & m, const Measurement & z);

  void wrapStateYaw();
  double limitMeasurementYaw(const double offset);

protected:
  double wrap2pi(const double angle);

private:
  Covariance<State> I_;
};

template<typename MeasurementModel, typename Measurement>
void ExtendedKalmanFilter::update(MeasurementModel & m, const Measurement & z)
{
  m.updateJacobians(x_);

  // compute innovation covariance
  Covariance<Measurement> S = (m.H_ * P_ * m.H_.transpose()) +
    (m.V_ * m.getCovariance() * m.V_.transpose());

  // compute kalman gain
  KalmanGain<State, Measurement> K = P_ * m.H_.transpose() * S.inverse();

  // update state estimate
  x_ += K * (z - m.h(x_));

  // update covariance
  //P_ -= K * m.H_ * P_;
  P_ = ((I_ - K * m.H_) * P_ * (I_ - K * m.H_).transpose()) +
    (K * m.V_ * m.getCovariance() * m.V_.transpose() * K.transpose());
}

template<typename MeasurementModel, typename Measurement>
double ExtendedKalmanFilter::mahalanobis(MeasurementModel & m, const Measurement & z)
{
  m.updateJacobians(x_);

  // compute innovation covariance
  Covariance<Measurement> S = (m.H_ * P_ * m.H_.transpose()) +
    (m.V_ * m.getCovariance() * m.V_.transpose());

  return (z - m.h(x_)).transpose() * S.inverse() * (z - m.h(x_));
}

} // namespace kalman
