#include <gtest/gtest.h>
#define private public
#include "kalman_filter/system_model.hpp"
#undef private


class SystemModelTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    sys = std::make_unique<kalman::SystemModel>();
    //  x.x() = 1.0;
    //  x.y() = 1.0;
    //  x.theta() = 1.0;
    //  x.nu() = 1.0;
    //  x.omega() = 1.0;
    //  x.alpha() = 1.0;

    x.setOnes();
    u.setZero();

    dt = 0.1;
  }
  void TearDown() override
  {
  }

  std::unique_ptr<kalman::SystemModel> sys;
  kalman::State x;
  kalman::Control u;
  double dt;
};

TEST_F(SystemModelTest, Contructor_TC1)
{
  EXPECT_TRUE(sys->F_.isIdentity());
  EXPECT_TRUE(sys->W_.isZero());
  EXPECT_TRUE(sys->Q_.isIdentity());
}

TEST_F(SystemModelTest, setCovariance_TC1)
{
  kalman::SystemCov Q = kalman::SystemCov::Zero();
  Q.diagonal() << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;
  sys->setCovariance(Q);

  EXPECT_EQ(sys->Q_.rows(), kalman::StateSize);
  EXPECT_EQ(sys->Q_.cols(), kalman::StateSize);
  EXPECT_TRUE(sys->Q_.isDiagonal());
  EXPECT_DOUBLE_EQ(sys->Q_(kalman::State::X, kalman::State::X), 1.0);
  EXPECT_DOUBLE_EQ(sys->Q_(kalman::State::Y, kalman::State::Y), 2.0);
  EXPECT_DOUBLE_EQ(sys->Q_(kalman::State::THETA, kalman::State::THETA), 3.0);
  EXPECT_DOUBLE_EQ(sys->Q_(kalman::State::NU, kalman::State::NU), 4.0);
  EXPECT_DOUBLE_EQ(sys->Q_(kalman::State::OMEGA, kalman::State::OMEGA), 5.0);
  EXPECT_DOUBLE_EQ(sys->Q_(kalman::State::ALPHA, kalman::State::ALPHA), 6.0);
}

TEST_F(SystemModelTest, f_TC1)
{
  kalman::State state;
  state = sys->f(x, u, dt);
  EXPECT_DOUBLE_EQ(state.x(), 1.0522449600286314);
  EXPECT_DOUBLE_EQ(state.y(), 1.0910794386873719);
  EXPECT_DOUBLE_EQ(state.theta(), 1.1);
  EXPECT_DOUBLE_EQ(state.nu(), 1.1);
  EXPECT_DOUBLE_EQ(state.omega(), 1.0);
  EXPECT_DOUBLE_EQ(state.alpha(), 1.0);
}

TEST_F(SystemModelTest, updateJacobian_TC1)
{
  sys->updateJacobian(x, u, dt);
  EXPECT_DOUBLE_EQ(sys->F_(kalman::State::X, kalman::State::THETA), -0.091079438687371797);
  EXPECT_DOUBLE_EQ(sys->F_(kalman::State::X, kalman::State::NU), 0.049757104789172696);
  EXPECT_DOUBLE_EQ(sys->F_(kalman::State::X, kalman::State::OMEGA), -0.00455397193436859);
  EXPECT_DOUBLE_EQ(sys->F_(kalman::State::X, kalman::State::ALPHA), 0.002487855239458635);

  EXPECT_DOUBLE_EQ(sys->F_(kalman::State::Y, kalman::State::THETA), 0.05224496002863133);
  EXPECT_DOUBLE_EQ(sys->F_(kalman::State::Y, kalman::State::NU), 0.0867423225594017);
  EXPECT_DOUBLE_EQ(sys->F_(kalman::State::Y, kalman::State::OMEGA), 0.002612248001431567);
  EXPECT_DOUBLE_EQ(sys->F_(kalman::State::Y, kalman::State::ALPHA), 0.004337116127970085);

  EXPECT_DOUBLE_EQ(sys->F_(kalman::State::THETA, kalman::State::OMEGA), 0.1);
  EXPECT_DOUBLE_EQ(sys->F_(kalman::State::NU, kalman::State::ALPHA), 0.1);

  EXPECT_DOUBLE_EQ(sys->W_(kalman::State::X, kalman::State::OMEGA), -0.00455397193436859);
  EXPECT_DOUBLE_EQ(sys->W_(kalman::State::X, kalman::State::ALPHA), 0.002487855239458635);

  EXPECT_DOUBLE_EQ(sys->W_(kalman::State::Y, kalman::State::OMEGA), 0.002612248001431567);
  EXPECT_DOUBLE_EQ(sys->W_(kalman::State::Y, kalman::State::ALPHA), 0.004337116127970085);

  EXPECT_DOUBLE_EQ(sys->W_(kalman::State::THETA, kalman::State::OMEGA), 0.1);

  EXPECT_DOUBLE_EQ(sys->W_(kalman::State::NU, kalman::State::ALPHA), 0.1);

  EXPECT_DOUBLE_EQ(sys->W_(kalman::State::OMEGA, kalman::State::OMEGA), 1.0);

  EXPECT_DOUBLE_EQ(sys->W_(kalman::State::ALPHA, kalman::State::ALPHA), 1.0);
}
