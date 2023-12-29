#pragma once

// local header
#include "kalman_filter/matrix.hpp"


namespace kalman
{

template<int N>
using SquareMatrix = Matrix<N, N>;

template<typename Type>
using Covariance = SquareMatrix<Type::RowsAtCompileTime>;

template<typename StateType, typename MeasurementType>
using KalmanGain = Matrix<StateType::RowsAtCompileTime, MeasurementType::RowsAtCompileTime>;

template<typename A, typename B>
using Jacobian = Matrix<A::RowsAtCompileTime, B::RowsAtCompileTime>;

} // namespace kalman
