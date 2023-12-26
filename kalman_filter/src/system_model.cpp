#include "kalman_filter/system_model.hpp"


namespace kalman
{

SystemModel::SystemModel()
{
  F_ = StateJacobian::Identity();
  W_ = NoiseJacobian::Zero();
  Q_ = SystemCov::Identity();
}

void SystemModel::setCovariance(const SystemCov & Q)
{
  Q_ = Q;
}

State SystemModel::f(const State & x, const Control & u, const double dt) const
{
  (void)u;  // not using control here. Just put this line to silence compile warning.

  State state;
  // Predicted state vector after transition
  double dt2 = dt * dt;
  double dl = x.nu() * dt + 0.5 * x.alpha() * dt2;
  double dtheta = x.theta() + 0.5 * x.omega() * dt;

  state.x() = x.x() + dl * std::cos(dtheta);
  state.y() = x.y() + dl * std::sin(dtheta);
  state.theta() = x.theta() + x.omega() * dt;
  state.nu() = x.nu() + x.alpha() * dt;
  state.omega() = x.omega();
  state.alpha() = x.alpha();

  return state;
}

void SystemModel::updateJacobian(const State & x, const Control & u, const double dt)
{
  (void)u;  // not using control here. Just put this line to silence compile warning.

  double dt2 = dt * dt;
  double dl = x.nu() * dt + 0.5 * x.alpha() * dt2;
  double dtheta = x.theta() + 0.5 * x.omega() * dt;
  double sin_dtheta = std::sin(dtheta);
  double cos_dtheta = std::cos(dtheta);

  // F = df/dx (Jacobian of state transition w.r.t the state)
  F_.setIdentity();

  // partial derivative of f_x w.r.t theta, nu, omega, alpha
  F_(State::X, State::THETA) = -dl * sin_dtheta;
  F_(State::X, State::NU) = dt * cos_dtheta;
  F_(State::X, State::OMEGA) = -0.5 * dt * dl * sin_dtheta;
  F_(State::X, State::ALPHA) = 0.5 * dt2 * cos_dtheta;

  // partial derivative of f_y w.r.t theta, nu, omega, alpha
  F_(State::Y, State::THETA) = dl * cos_dtheta;
  F_(State::Y, State::NU) = dt * sin_dtheta;
  F_(State::Y, State::OMEGA) = 0.5 * dt * dl * cos_dtheta;
  F_(State::Y, State::ALPHA) = 0.5 * dt2 * sin_dtheta;

  // partial derivative of f_theta w.r.t omega
  F_(State::THETA, State::OMEGA) = dt;

  // partial derivative of f_nu w.r.t alpah
  F_(State::NU, State::ALPHA) = dt;

  // W = df/depsilon  (Jacobian of state transition w.r.t the noise)
  // Assume the noise only coming from omega and alpha
  W_.setZero();

  // partial derivative of f_x w.r.t omega, alpha
  W_(State::X, State::OMEGA) = -0.5 * dt * dl * sin_dtheta;
  W_(State::X, State::ALPHA) = 0.5 * dt2 * cos_dtheta;

  // partial derivative of f_y w.r.t omega, alpha
  W_(State::Y, State::OMEGA) = 0.5 * dt * dl * cos_dtheta;
  W_(State::Y, State::ALPHA) = 0.5 * dt2 * sin_dtheta;

  // partial derivative of f_theta w.r.t omega
  W_(State::THETA, State::OMEGA) = dt;

  // parital derivative of f_nu w.r.t alpah
  W_(State::NU, State::ALPHA) = dt;

  // parital derivative of f_omega w.r.t omega
  W_(State::OMEGA, State::OMEGA) = 1.0;

  // parital derivative of f_alpha w.r.t alpha
  W_(State::ALPHA, State::ALPHA) = 1.0;
}

} // namespace kalman
