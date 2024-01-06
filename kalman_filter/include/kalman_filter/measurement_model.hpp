#pragma once

#include "kalman_filter/standard_base.hpp"


namespace kalman
{

template<typename StateType, typename MeasurementType,
  template<class> typename CovarianceBase = StandardBase>
class MeasurementModel : public CovarianceBase<MeasurementType>
{
  static_assert(
    StateType::RowsAtCompileTime == Dynamic || StateType::RowsAtCompileTime > 0,
    "State vector must contain at least 1 element or be dynamic");
  static_assert(
    MeasurementType::RowsAtCompileTime == Dynamic || MeasurementType::RowsAtCompileTime > 0,
    "Measurement vector must contain at least 1 element or be dynamic");

public:
  using State = StateType;
  using Measurement = MeasurementType;
  virtual Measurement h(const State & x) const = 0;

protected:
  MeasurementModel() = default;
  virtual ~MeasurementModel() = default;
};

} // namespace kalman
