#pragma once

// Eigen header
#include <Eigen/Dense>


#define KALMAN_VECTOR(NAME, N) \
  using Base = kalman::Vector<N>; \
  using Base::RowsAtCompileTime; \
  using Base::ColsAtCompileTime; \
  using Base::SizeAtCompileTime; \
 \
  NAME(void) \
    : kalman::Vector<N>() \
  { \
  } \
 \
  template<typename OtherDerived> \
  NAME(const Eigen::MatrixBase<OtherDerived> & other) \
    : kalman::Vector<N>(other) \
  { \
  } \
 \
  template<typename OtherDerived> \
  NAME & operator=(const Eigen::MatrixBase<OtherDerived> & other) \
  { \
    this->Base::operator=(other); \
    return *this; \
  }

namespace kalman
{

template<int rows, int cols>
using Matrix = Eigen::Matrix<double, rows, cols>;

template<int N>
class Vector : public Matrix<N, 1>
{
public:
  using Base = Matrix<N, 1>;
  using Base::RowsAtCompileTime;
  using Base::ColsAtCompileTime;
  using Base::SizeAtCompileTime;

  Vector(void)
  : Matrix<N, 1>()
  {
  }

  template<typename OtherDerived>
  Vector(const Eigen::MatrixBase<OtherDerived> & other)
  : Matrix<N, 1>(other)
  {
  }

  template<typename OtherDerived>
  Vector & operator=(const Eigen::MatrixBase<OtherDerived> & other)
  {
    this->Base::operator=(other);
    return *this;
  }
};

} // namespace kalman
