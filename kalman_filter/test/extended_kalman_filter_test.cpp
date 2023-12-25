#include <gtest/gtest.h>
#define private public
#include "kalman_filter/extended_kalman_filter.hpp"
#undef private

#include "kalman_filter/system_model.hpp"
#include "kalman_filter/gps_measurement_model.hpp"
#include "kalman_filter/imu_measurement_model.hpp"


class ExtendedKalmanFilterTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    ekf = std::make_unique<kalman::ExtendedKalmanFilter>();
    init_x.setOnes();
    init_P.setIdentity();

    sys = std::make_unique<kalman::SystemModel>();
    dt = 0.1;

    gps = std::make_unique<kalman::GpsMeasurementModel>();
    zg << 1.1, 1.1;

    imu = std::make_unique<kalman::ImuMeasurementModel>();
    zi << 1.5, 1.5;
  }

  void TearDown() override
  {
  }

  std::unique_ptr<kalman::ExtendedKalmanFilter> ekf;
  kalman::State init_x;
  kalman::StateCov init_P;

  std::unique_ptr<kalman::SystemModel> sys;
  double dt;

  std::unique_ptr<kalman::GpsMeasurementModel> gps;
  kalman::GpsMeasurement zg;

  std::unique_ptr<kalman::ImuMeasurementModel> imu;
  kalman::ImuMeasurement zi;
};

TEST_F(ExtendedKalmanFilterTest, Contructor_TC1)
{
  EXPECT_TRUE(ekf->x_.isZero());
  EXPECT_TRUE(ekf->P_.isIdentity());
  EXPECT_EQ(ekf->x_.rows(), kalman::StateSize);
  EXPECT_EQ(ekf->P_.rows(), kalman::StateSize);
  EXPECT_EQ(ekf->P_.cols(), kalman::StateSize);
}

TEST_F(ExtendedKalmanFilterTest, Init_TC1)
{
  ekf->init(init_x, init_P * 2);
  EXPECT_DOUBLE_EQ(ekf->x_(kalman::State::X), 1.0);
  EXPECT_DOUBLE_EQ(ekf->x_(kalman::State::Y), 1.0);
  EXPECT_DOUBLE_EQ(ekf->x_(kalman::State::THETA), 1.0);
  EXPECT_DOUBLE_EQ(ekf->x_(kalman::State::NU), 1.0);
  EXPECT_DOUBLE_EQ(ekf->x_(kalman::State::OMEGA), 1.0);
  EXPECT_DOUBLE_EQ(ekf->x_(kalman::State::ALPHA), 1.0);

  EXPECT_TRUE(ekf->P_.isDiagonal());
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::X, kalman::State::X), 2.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::Y, kalman::State::Y), 2.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::THETA, kalman::State::THETA), 2.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::NU, kalman::State::NU), 2.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::OMEGA, kalman::State::OMEGA), 2.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::ALPHA, kalman::State::ALPHA), 2.0);
}

TEST_F(ExtendedKalmanFilterTest, Predict_TC1)
{
  ekf->init(init_x, init_P);
  ekf->predict(*sys, dt);
  EXPECT_DOUBLE_EQ(ekf->x_(kalman::State::X), 1.0522449600286314);
  EXPECT_DOUBLE_EQ(ekf->x_(kalman::State::Y), 1.0910794386873719);
  EXPECT_DOUBLE_EQ(ekf->x_(kalman::State::THETA), 1.1);
  EXPECT_DOUBLE_EQ(ekf->x_(kalman::State::NU), 1.1);
  EXPECT_DOUBLE_EQ(ekf->x_(kalman::State::OMEGA), 1.0);
  EXPECT_DOUBLE_EQ(ekf->x_(kalman::State::ALPHA), 1.0);

  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::X, kalman::State::X), 1.0108250897967503423);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::X, kalman::State::Y), -0.0004446067744095863);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::X, kalman::State::THETA), -0.09199023307424552);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::X, kalman::State::NU), 0.0502546758370644);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::X, kalman::State::OMEGA), -0.009107943868737180);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::X, kalman::State::ALPHA), 0.004975710478917271);

  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::Y, kalman::State::X), -0.0004446067744095863);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::Y, kalman::State::Y), 1.0103050352032492132);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::Y, kalman::State::THETA), 0.05276740962891765);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::Y, kalman::State::NU), 0.08760974578499572);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::Y, kalman::State::OMEGA), 0.005224496002863135);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::Y, kalman::State::ALPHA), 0.008674232255940171);

  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::THETA, kalman::State::X), -0.0919902330742455171);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::THETA, kalman::State::Y), 0.0527674096289176484);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::THETA, kalman::State::THETA), 1.02);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::THETA, kalman::State::NU), 0.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::THETA, kalman::State::OMEGA), 0.2);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::THETA, kalman::State::ALPHA), 0.0);

  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::NU, kalman::State::X), 0.0502546758370644164);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::NU, kalman::State::Y), 0.0876097457849957240);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::NU, kalman::State::THETA), 0.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::NU, kalman::State::NU), 1.02);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::NU, kalman::State::OMEGA), 0.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::NU, kalman::State::ALPHA), 0.2);

  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::OMEGA, kalman::State::X), -0.0091079438687371797);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::OMEGA, kalman::State::Y), 0.0052244960028631346);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::OMEGA, kalman::State::THETA), 0.2);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::OMEGA, kalman::State::NU), 0.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::OMEGA, kalman::State::OMEGA), 2.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::OMEGA, kalman::State::ALPHA), 0.0);

  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::ALPHA, kalman::State::X), 0.0049757104789172708);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::ALPHA, kalman::State::Y), 0.0086742322559401706);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::ALPHA, kalman::State::THETA), 0.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::ALPHA, kalman::State::NU), 0.2);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::ALPHA, kalman::State::OMEGA), 0.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::ALPHA, kalman::State::ALPHA), 2.0);
}

TEST_F(ExtendedKalmanFilterTest, GpsUpdate_TC1)
{
  ekf->init(init_x, init_P);
  ekf->update(*gps, zg);

  EXPECT_DOUBLE_EQ(ekf->x_(kalman::State::X), 1.05);
  EXPECT_DOUBLE_EQ(ekf->x_(kalman::State::Y), 1.05);
  EXPECT_DOUBLE_EQ(ekf->x_(kalman::State::THETA), 1.0);
  EXPECT_DOUBLE_EQ(ekf->x_(kalman::State::NU), 1.0);
  EXPECT_DOUBLE_EQ(ekf->x_(kalman::State::OMEGA), 1.0);
  EXPECT_DOUBLE_EQ(ekf->x_(kalman::State::ALPHA), 1.0);

  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::X, kalman::State::X), 0.5);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::X, kalman::State::Y), 0.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::X, kalman::State::THETA), 0.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::X, kalman::State::NU), 0.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::X, kalman::State::OMEGA), 0.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::X, kalman::State::ALPHA), 0.0);

  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::Y, kalman::State::X), 0.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::Y, kalman::State::Y), 0.5);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::Y, kalman::State::THETA), 0.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::Y, kalman::State::NU), 0.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::Y, kalman::State::OMEGA), 0.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::Y, kalman::State::ALPHA), 0.0);

  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::THETA, kalman::State::X), 0.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::THETA, kalman::State::Y), 0.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::THETA, kalman::State::THETA), 1.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::THETA, kalman::State::NU), 0.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::THETA, kalman::State::OMEGA), 0.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::THETA, kalman::State::ALPHA), 0.0);

  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::NU, kalman::State::X), 0.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::NU, kalman::State::Y), 0.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::NU, kalman::State::THETA), 0.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::NU, kalman::State::NU), 1.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::NU, kalman::State::OMEGA), 0.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::NU, kalman::State::ALPHA), 0.0);

  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::OMEGA, kalman::State::X), 0.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::OMEGA, kalman::State::Y), 0.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::OMEGA, kalman::State::THETA), 0.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::OMEGA, kalman::State::NU), 0.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::OMEGA, kalman::State::OMEGA), 1.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::OMEGA, kalman::State::ALPHA), 0.0);

  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::ALPHA, kalman::State::X), 0.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::ALPHA, kalman::State::Y), 0.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::ALPHA, kalman::State::THETA), 0.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::ALPHA, kalman::State::NU), 0.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::ALPHA, kalman::State::OMEGA), 0.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::ALPHA, kalman::State::ALPHA), 1.0);
}

TEST_F(ExtendedKalmanFilterTest, ImuUpdate_TC1)
{
  ekf->init(init_x, init_P);
  ekf->update(*imu, zi);

  EXPECT_DOUBLE_EQ(ekf->x_(kalman::State::X), 1.0);
  EXPECT_DOUBLE_EQ(ekf->x_(kalman::State::Y), 1.0);
  EXPECT_DOUBLE_EQ(ekf->x_(kalman::State::THETA), 1.0);
  EXPECT_DOUBLE_EQ(ekf->x_(kalman::State::NU), 1.0);
  EXPECT_DOUBLE_EQ(ekf->x_(kalman::State::OMEGA), 1.25);
  EXPECT_DOUBLE_EQ(ekf->x_(kalman::State::ALPHA), 1.25);

  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::X, kalman::State::X), 1.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::X, kalman::State::Y), 0.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::X, kalman::State::THETA), 0.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::X, kalman::State::NU), 0.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::X, kalman::State::OMEGA), 0.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::X, kalman::State::ALPHA), 0.0);

  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::Y, kalman::State::X), 0.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::Y, kalman::State::Y), 1.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::Y, kalman::State::THETA), 0.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::Y, kalman::State::NU), 0.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::Y, kalman::State::OMEGA), 0.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::Y, kalman::State::ALPHA), 0.0);

  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::THETA, kalman::State::X), 0.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::THETA, kalman::State::Y), 0.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::THETA, kalman::State::THETA), 1.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::THETA, kalman::State::NU), 0.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::THETA, kalman::State::OMEGA), 0.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::THETA, kalman::State::ALPHA), 0.0);

  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::NU, kalman::State::X), 0.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::NU, kalman::State::Y), 0.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::NU, kalman::State::THETA), 0.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::NU, kalman::State::NU), 1.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::NU, kalman::State::OMEGA), 0.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::NU, kalman::State::ALPHA), 0.0);

  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::OMEGA, kalman::State::X), 0.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::OMEGA, kalman::State::Y), 0.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::OMEGA, kalman::State::THETA), 0.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::OMEGA, kalman::State::NU), 0.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::OMEGA, kalman::State::OMEGA), 0.5);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::OMEGA, kalman::State::ALPHA), 0.0);

  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::ALPHA, kalman::State::X), 0.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::ALPHA, kalman::State::Y), 0.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::ALPHA, kalman::State::THETA), 0.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::ALPHA, kalman::State::NU), 0.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::ALPHA, kalman::State::OMEGA), 0.0);
  EXPECT_DOUBLE_EQ(ekf->P_(kalman::State::ALPHA, kalman::State::ALPHA), 0.5);
}
