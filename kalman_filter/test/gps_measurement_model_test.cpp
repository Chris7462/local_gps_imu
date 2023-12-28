#include <gtest/gtest.h>
#define private public
#include "kalman_filter/gps_measurement_model.hpp"
#undef private


class GpsMeasurementModelTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    meas = std::make_unique<kalman::GpsMeasurementModel>();
    x.setOnes();
  }
  void TearDown() override
  {
  }

  std::unique_ptr<kalman::GpsMeasurementModel> meas;
  kalman::State x;
};

TEST_F(GpsMeasurementModelTest, Contructor_TC1)
{
  EXPECT_TRUE(meas->H_.isZero());
  EXPECT_TRUE(meas->V_.isZero());
  EXPECT_TRUE(meas->R_.isIdentity());
}

TEST_F(GpsMeasurementModelTest, setCovariance_TC1)
{
  kalman::GpsMeasurementCov R = kalman::GpsMeasurementCov::Zero();
  R.diagonal() << 1.0, 2.0;
  meas->setCovariance(R);

  EXPECT_EQ(meas->R_.rows(), kalman::GpsMeasurementSize);
  EXPECT_EQ(meas->R_.cols(), kalman::GpsMeasurementSize);
  EXPECT_TRUE(meas->R_.isDiagonal());
  EXPECT_DOUBLE_EQ(meas->R_(kalman::GpsMeasurement::X, kalman::GpsMeasurement::X), 1.0);
  EXPECT_DOUBLE_EQ(meas->R_(kalman::GpsMeasurement::Y, kalman::GpsMeasurement::Y), 2.0);
}

TEST_F(GpsMeasurementModelTest, h_TC1)
{
  kalman::GpsMeasurement measurement;
  measurement = meas->h(x);

  EXPECT_DOUBLE_EQ(measurement.x(), 1.0);
  EXPECT_DOUBLE_EQ(measurement.y(), 1.0);
}

TEST_F(GpsMeasurementModelTest, updateJacobians_TC1)
{
  meas->updateJacobians(x);
  EXPECT_DOUBLE_EQ(meas->H_(kalman::GpsMeasurement::X, kalman::State::X), 1.0);
  EXPECT_DOUBLE_EQ(meas->H_(kalman::GpsMeasurement::Y, kalman::State::Y), 1.0);

  EXPECT_TRUE(meas->V_.isIdentity());
}
