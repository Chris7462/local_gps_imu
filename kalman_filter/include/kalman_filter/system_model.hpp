#pragma once

#include <cmath>

#include "kalman_filter/state.hpp"
#include "kalman_filter/control.hpp"


namespace kalman
{

class ExtendedKalmanFilter;

using StateJacobian = Eigen::Matrix<double, StateSize, StateSize>;
using NoiseJacobian = Eigen::Matrix<double, StateSize, StateSize>;
using SystemCov = Eigen::Matrix<double, StateSize, StateSize>;

class SystemModel
{
public:
  SystemModel();
  ~SystemModel() = default;

  void setCovariance(const SystemCov & Q);
  State f(const State & x, const Control & u, const double dt) const;
  void updateJacobians(const State & x, const Control & u, const double dt);

  friend class kalman::ExtendedKalmanFilter;

private:
  StateJacobian F_;
  NoiseJacobian W_;
  SystemCov Q_;
};

} // namespace kalman
