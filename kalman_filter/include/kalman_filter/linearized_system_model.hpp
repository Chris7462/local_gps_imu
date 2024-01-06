#pragma once

#include "kalman_filter/system_model.hpp"


namespace kalman
{

template<typename StateType>
class ExtendedKalmanFilter;

template<typename StateType, typename ControlType = Vector<0>,
  template<class> typename CovarianceBase = StandardBase>
class LinearizedSystemModel
  : public SystemModel<StateType, ControlType, CovarianceBase>
{
  friend class ExtendedKalmanFilter<StateType>;

public:
  using Base = SystemModel<StateType, ControlType, CovarianceBase>;
  using typename Base::State;
  using typename Base::Control;

protected:
  LinearizedSystemModel()
  {
    F_.setIdentity();
    W_.setZero();
  }
  ~LinearizedSystemModel() = default;

  virtual void updateJacobians(const State & x, const Control & u)
  {
    (void)x;
    (void)u;
  }

  Jacobian<State, State> F_;
  Jacobian<State, State> W_;
};

} // namespace kalman
