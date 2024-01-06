#include <gtest/gtest.h>

#define private public
#include "kalman_filter/extended_kalman_filter.hpp"
#undef protected


using namespace kalman;

class ExtendedKalmanFilterTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    ekf = std::make_unique<EKFv3>();
  }
  void TearDown() override
  {
  }

  using EKFv3 = ExtendedKalmanFilter<Vector<3>>;
  std::unique_ptr<EKFv3> ekf;
};

TEST_F(ExtendedKalmanFilterTest, Contructor_TC1)
{
  EXPECT_EQ(ekf->getState().size(), 3);
  EXPECT_TRUE(ekf->getState().isZero());
  EXPECT_TRUE(ekf->getCovariance().isIdentity());
  EXPECT_TRUE(ekf->I_.isIdentity());
}
