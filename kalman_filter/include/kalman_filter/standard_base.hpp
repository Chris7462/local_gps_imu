#pragma once

// local header
#include "kalman_filter/types.hpp"


namespace kalman
{

template<typename Type>
class StandardBase
{
public:
  const Covariance<Type> & getCovariance() const
  {
    return P_;
  }

  void setCovariance(const Covariance<Type> & covariance)
  {
    P_ = covariance;
  }

protected:
  StandardBase()
  {
    P_.setIdentity();
  }

  Covariance<Type> P_;
};

} // namespace kalman
