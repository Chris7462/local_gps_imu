#pragma once

#include "kalman_filter/matrix.hpp"
#include "kalman_filter/state.hpp"
#include "kalman_filter/standard_base.hpp"


namespace kalman
{

class ExtendedKalmanFilter;

class GpsMeasurement : public kalman::Vector<2>
{
public:
  KALMAN_VECTOR(GpsMeasurement, 2)
  enum : uint8_t
  {
    X,
    Y
  };

  inline double x() const {return (*this)[X];}
  inline double y() const {return (*this)[Y];}

  inline double & x() {return (*this)[X];}
  inline double & y() {return (*this)[Y];}
};

class GpsMeasurementModel : public StandardBase<GpsMeasurement>
{
  friend class kalman::ExtendedKalmanFilter;

public:
  GpsMeasurementModel();
  ~GpsMeasurementModel() = default;

  GpsMeasurement h(const State & x) const;
  void updateJacobians(const State & x);

private:
  double threshold_;
  Jacobian<GpsMeasurement, State> H_;
  Jacobian<GpsMeasurement, GpsMeasurement> V_;
};

} // namespace kalman
