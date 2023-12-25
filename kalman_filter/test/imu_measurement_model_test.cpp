#include <gtest/gtest.h>
#define private public
#include "kalman_filter/imu_measurement_model.hpp"
#undef private


class ImuMeasurementModelTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    meas = std::make_unique<kalman::ImuMeasurementModel>();
    x.setOnes();
  }
  void TearDown() override
  {
  }

  std::unique_ptr<kalman::ImuMeasurementModel> meas;
  kalman::State x;
};

TEST_F(ImuMeasurementModelTest, Contructor_TC1)
{
  EXPECT_TRUE(meas->H_.isZero());
  EXPECT_TRUE(meas->V_.isZero());
  EXPECT_TRUE(meas->R_.isIdentity());
}

TEST_F(ImuMeasurementModelTest, setCovariance_TC1)
{
  kalman::ImuMeasurementCov R = kalman::ImuMeasurementCov::Zero();
  R.diagonal() << 1.0, 2.0;
  meas->setCovariance(R);

  EXPECT_EQ(meas->R_.rows(), kalman::ImuMeasurementSize);
  EXPECT_EQ(meas->R_.cols(), kalman::ImuMeasurementSize);
  EXPECT_TRUE(meas->R_.isDiagonal());
  EXPECT_DOUBLE_EQ(meas->R_(kalman::ImuMeasurement::OMEGA, kalman::ImuMeasurement::OMEGA), 1.0);
  EXPECT_DOUBLE_EQ(meas->R_(kalman::ImuMeasurement::ALPHA, kalman::ImuMeasurement::ALPHA), 2.0);
}

TEST_F(ImuMeasurementModelTest, h_TC1)
{
  kalman::ImuMeasurement measurement;
  measurement = meas->h(x);

  EXPECT_DOUBLE_EQ(measurement.omega(), 1.0);
  EXPECT_DOUBLE_EQ(measurement.alpha(), 1.0);
}

TEST_F(ImuMeasurementModelTest, updateJacobian_TC1)
{
  meas->updateJacobian(x);
  EXPECT_DOUBLE_EQ(meas->H_(kalman::ImuMeasurement::OMEGA, kalman::State::OMEGA), 1.0);
  EXPECT_DOUBLE_EQ(meas->H_(kalman::ImuMeasurement::ALPHA, kalman::State::ALPHA), 1.0);

  EXPECT_TRUE(meas->V_.isIdentity());
}
