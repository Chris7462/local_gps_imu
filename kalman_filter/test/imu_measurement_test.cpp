#include <gtest/gtest.h>
#define private public
#include "kalman_filter/imu_measurement.hpp"
#undef private


// IMU
class ImuMeasurementTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    imu = std::make_unique<kalman::ImuMeasurement>();
  }
  void TearDown() override
  {
  }

  std::unique_ptr<kalman::ImuMeasurement> imu;
};

TEST_F(ImuMeasurementTest, Contructor_TC1)
{
  EXPECT_EQ(imu->size(), kalman::ImuMeasurementSize);
}

TEST_F(ImuMeasurementTest, SetGet_TC1)
{
  imu->theta() = 1.0;
  imu->omega() = 2.0;
  imu->alpha() = 3.0;

  EXPECT_DOUBLE_EQ(imu->theta(), 1.0);
  EXPECT_DOUBLE_EQ(imu->omega(), 2.0);
  EXPECT_DOUBLE_EQ(imu->alpha(), 3.0);
}
