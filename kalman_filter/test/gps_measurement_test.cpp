#include <gtest/gtest.h>
#define private public
#include "kalman_filter/gps_measurement.hpp"
#undef private


// GPS
class GpsMeasurementTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    gps = std::make_unique<kalman::GpsMeasurement>();
  }
  void TearDown() override
  {
  }

  std::unique_ptr<kalman::GpsMeasurement> gps;
};

TEST_F(GpsMeasurementTest, Contructor_TC1)
{
  EXPECT_EQ(gps->size(), kalman::GpsMeasurementSize);
}

TEST_F(GpsMeasurementTest, SetGet_TC1)
{
  gps->x() = 1.0;
  gps->y() = 2.0;

  EXPECT_DOUBLE_EQ(gps->x(), 1.0);
  EXPECT_DOUBLE_EQ(gps->y(), 2.0);
}
