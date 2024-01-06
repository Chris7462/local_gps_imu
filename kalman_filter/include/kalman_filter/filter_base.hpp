#pragma once

#include "kalman_filter/matrix.hpp"

namespace kalman
{

template<typename StateType>
class FilterBase
{
public:
  static_assert(
    StateType::RowsAtCompileTime == Dynamic || StateType::RowsAtCompileTime > 0,
    "State vector must contain at least 1 element or be dynamic");

  static_assert(
    StateType::ColsAtCompileTime == 1,
    "State type must be a column vector");

  using State = StateType;

  void init(const State & initialState)
  {
    x_ = initialState;
  }

  const State & getState() const
  {
    return x_;
  }

protected:
  FilterBase() = default;

  State x_;
};

} // namespace kalman
