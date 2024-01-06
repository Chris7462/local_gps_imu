#pragma once

#include "kalman_filter/measurement_model.hpp"


namespace kalman
{

template<class StateType>
class ExtendedKalmanFilter;

template<typename StateType, typename MeasurementType,
  template<class> typename CovarianceBase = StandardBase>
class LinearizedMeasurementModel
  : public MeasurementModel<StateType, MeasurementType, CovarianceBase>
{
  friend class ExtendedKalmanFilter<StateType>;

public:
  using Base = MeasurementModel<StateType, MeasurementType, CovarianceBase>;
  using typename Base::State;
  using typename Base::Measurement;

protected:
  LinearizedMeasurementModel()
  {
    H_.setZero();
    V_.setIdentity();
  }
  ~LinearizedMeasurementModel() = default;

  virtual void updateJacobians(const State & x)
  {
    (void)x;
  }

  double chisq_threshold_;
  Jacobian<Measurement, State> H_;
  Jacobian<Measurement, Measurement> V_;
};

} // namespace kalman
