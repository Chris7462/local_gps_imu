#pragma once

#include "kalman_filter/matrix.hpp"
#include "kalman_filter/state.hpp"
#include "kalman_filter/standard_base.hpp"


namespace kalman
{

class ExtendedKalmanFilter;

class Control : public kalman::Vector<2>
{
public:
  KALMAN_VECTOR(Control, 2)
  enum : uint8_t
  {
    OMEGA,
    ALPHA
  };

  inline double omega() const {return (*this)[OMEGA];}
  inline double alpha() const {return (*this)[ALPHA];}

  inline double & omega() {return (*this)[OMEGA];}
  inline double & alpha() {return (*this)[ALPHA];}
};

class SystemModel : public StandardBase<State>
{
  friend class kalman::ExtendedKalmanFilter;

public:
  SystemModel(const double dt);
  ~SystemModel() = default;

  State f(const State & x, const Control & u) const;
  void updateJacobians(const State & x, const Control & u);

private:
  double dt_;
  Jacobian<State, State> F_;
  Jacobian<State, State> W_;
};

} // namespace kalman
