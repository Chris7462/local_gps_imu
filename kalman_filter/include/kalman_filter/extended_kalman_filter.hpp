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
  bool update(MeasurementModel & m, const Measurement & z);

  template<typename Measurement>
  double mahalanobis(const Measurement & y, const Covariance<Measurement> & S_inv) const;

  void wrapStateYaw();
  double limitMeasurementYaw(const double offset);

protected:
  double wrap2pi(const double angle);

private:
  Covariance<State> I_;
};

template<typename MeasurementModel, typename Measurement>
bool ExtendedKalmanFilter::update(MeasurementModel & m, const Measurement & z)
{
  m.updateJacobians(x_);

  // compute innovation
  Measurement y = z - m.h(x_);

  // compute innovation covariance
  Covariance<Measurement> S = (m.H_ * P_ * m.H_.transpose()) +
    (m.V_ * m.getCovariance() * m.V_.transpose());

  // inverse of the innovation covariance
  Covariance<Measurement> S_inv = S.inverse();

  double dist = mahalanobis(y, S_inv);

  if (dist < m.threshold_) {
    // compute kalman gain
    KalmanGain<State, Measurement> K = P_ * m.H_.transpose() * S_inv;

    // update state estimate
    x_ += K * y;

    // update covariance
    //P_ -= K * m.H_ * P_;
    P_ = ((I_ - K * m.H_) * P_ * (I_ - K * m.H_).transpose()) +
      (K * m.V_ * m.getCovariance() * m.V_.transpose() * K.transpose());

    return true;
  } else {
    return false;
  }
}

template<typename Measurement>
double ExtendedKalmanFilter::mahalanobis(
  const Measurement & y, const Covariance<Measurement> & S_inv) const
{
  return (y.transpose() * S_inv * y).value();
}

} // namespace kalman
