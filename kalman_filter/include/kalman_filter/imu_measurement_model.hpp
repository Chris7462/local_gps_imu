#pragma once

#include "kalman_filter/matrix.hpp"
#include "kalman_filter/state.hpp"
#include "kalman_filter/standard_base.hpp"


namespace kalman
{

class ExtendedKalmanFilter;

constexpr int ImuMeasurementSize = 3;

class ImuMeasurement : public Vector<ImuMeasurementSize>
{
public:
  KALMAN_VECTOR(ImuMeasurement, ImuMeasurementSize)
  enum : uint8_t
  {
    THETA,
    OMEGA,
    ALPHA
  };

  inline double theta() const {return (*this)[THETA];}
  inline double omega() const {return (*this)[OMEGA];}
  inline double alpha() const {return (*this)[ALPHA];}

  inline double & theta() {return (*this)[THETA];}
  inline double & omega() {return (*this)[OMEGA];}
  inline double & alpha() {return (*this)[ALPHA];}
};

class ImuMeasurementModel : public StandardBase<ImuMeasurement>
{
  friend class ExtendedKalmanFilter;

public:
  ImuMeasurementModel();
  ~ImuMeasurementModel() = default;

  ImuMeasurement h(const State & x) const;
  void updateJacobians(const State & x);

private:
  double threshold_;
  Jacobian<ImuMeasurement, State> H_;
  Jacobian<ImuMeasurement, ImuMeasurement> V_;
};

} // namespace kalman
