#pragma once

#include <iostream>

#include "kalman_filter/filter_base.hpp"
#include "kalman_filter/standard_base.hpp"
#include "kalman_filter/linearized_system_model.hpp"
#include "kalman_filter/linearized_measurement_model.hpp"


namespace kalman
{

template<typename StateType>
class ExtendedKalmanFilter
  : public FilterBase<StateType>, public StandardBase<StateType>
{
public:
  using KalmanFilterBase = FilterBase<StateType>;
  using StandardFilterBase = StandardBase<StateType>;

  using typename KalmanFilterBase::State;

  template<typename Measurement, template<class> typename CovarianceBase>
  using MeasurementModelType = LinearizedMeasurementModel<State, Measurement, CovarianceBase>;

  template<typename Control, template<class> typename CovarianceBase>
  using SystemModelType = LinearizedSystemModel<State, Control, CovarianceBase>;

  template<typename Measurement>
  using KalmanGain = KalmanGain<State, Measurement>;

  ExtendedKalmanFilter()
  {
    x_.setZero();
    P_.setIdentity();
    I_.setIdentity();
  }
  ~ExtendedKalmanFilter() = default;

  template<typename Control, template<class> typename CovarianceBase>
  void predict(SystemModelType<Control, CovarianceBase> & s)
  {
    // predict state (without control)
    Control u;
    u.setZero();
    return predict(s, u);
  }

  template<typename Control, template<class> typename CovarianceBase>
  void predict(SystemModelType<Control, CovarianceBase> & s, const Control & u)
  {
    s.updateJacobians(x_, u);

    // predict state
    x_ = s.f(x_, u);

    // predict covariance
    P_ = s.F_ * P_ * s.F_.transpose() + s.W_ * s.getCovariance() * s.W_.transpose();
  }

  template<typename Measurement, template<class> typename CovarianceBase>
  bool update(MeasurementModelType<Measurement, CovarianceBase> & m, const Measurement & z)
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

    if (dist < m.chisq_threshold_) {
      // compute kalman gain
      KalmanGain<Measurement> K = P_ * m.H_.transpose() * S_inv;

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
  double mahalanobis(const Measurement & y, const Covariance<Measurement> & S_inv) const
  {
    return (y.transpose() * S_inv * y).value();
  }

protected:
  using KalmanFilterBase::x_;
  using StandardFilterBase::P_;

private:
  Covariance<State> I_;
};

} // namespace kalman
