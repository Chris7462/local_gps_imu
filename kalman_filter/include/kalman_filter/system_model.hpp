#pragma once

#include "kalman_filter/matrix.hpp"
#include "kalman_filter/standard_base.hpp"


namespace kalman
{

template<typename StateType, typename ControlType = Vector<0>,
  template<class> typename CovarianceBase = StandardBase>
class SystemModel : public CovarianceBase<StateType>
{
  static_assert(
    StateType::RowsAtCompileTime == Dynamic || StateType::RowsAtCompileTime > 0,
    "State vector must contain at least 1 element or be dynamic");
  static_assert(
    ControlType::RowsAtCompileTime == Dynamic || ControlType::RowsAtCompileTime >= 0,
    "Control vector must contain at least 0 elements or be dynamic");

public:
  using State = StateType;
  using Control = ControlType;
  virtual State f(const State & x, const Control & u) const = 0;

protected:
  SystemModel() = default;
  virtual ~SystemModel() = default;
};

} // namespace kalman
