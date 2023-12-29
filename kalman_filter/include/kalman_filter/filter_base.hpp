#pragma once

namespace kalman
{

template<typename StateType>
class KalmanFilterBase
{
public:
  static_assert(
    StateType::RowsAtCompileTime > 0,
    "State vector must contain at least 1 element");

  static_assert(
    StateType::ColsAtCompileTime == 1,
    "State type must be a column vector");

  void init(const StateType & initialState)
  {
    x_ = initialState;
  }

  const StateType & getState() const
  {
    return x_;
  }

protected:
  KalmanFilterBase() = default;

  StateType x_;
};

} // namespace kalman
