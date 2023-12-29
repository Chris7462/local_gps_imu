#include <gtest/gtest.h>

#define private public
#include "kalman_filter/system_model.hpp"
#undef private


using namespace kalman;

class ControlTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    control = std::make_unique<Control>();
  }
  void TearDown() override
  {
  }

  std::unique_ptr<Control> control;
};

TEST_F(ControlTest, Contructor_TC1)
{
  EXPECT_EQ(control->size(), 2);
}

TEST_F(ControlTest, SetGet_TC1)
{
  control->omega() = 1.0;
  control->alpha() = 2.0;

  EXPECT_DOUBLE_EQ(control->omega(), 1.0);
  EXPECT_DOUBLE_EQ(control->alpha(), 2.0);
}


class SystemModelTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    sys = std::make_unique<SystemModel>(dt);
    x.setOnes();
    u.setZero();
  }
  void TearDown() override
  {
  }

  std::unique_ptr<SystemModel> sys;
  State x;
  Control u;
  double dt = 0.1;
};

TEST_F(SystemModelTest, Contructor_TC1)
{
  EXPECT_TRUE(sys->F_.isIdentity());
  EXPECT_TRUE(sys->W_.isZero());
  EXPECT_TRUE(sys->getCovariance().isIdentity());
}

TEST_F(SystemModelTest, setCovariance_TC1)
{
  Covariance<State> Q;
  Q.setZero();
  Q.diagonal() << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;
  sys->setCovariance(Q);

  EXPECT_EQ(sys->getCovariance().rows(), 6);
  EXPECT_EQ(sys->getCovariance().cols(), 6);
  EXPECT_TRUE(sys->getCovariance().isDiagonal());
  EXPECT_DOUBLE_EQ(sys->getCovariance()(State::X, State::X), 1.0);
  EXPECT_DOUBLE_EQ(sys->getCovariance()(State::Y, State::Y), 2.0);
  EXPECT_DOUBLE_EQ(sys->getCovariance()(State::THETA, State::THETA), 3.0);
  EXPECT_DOUBLE_EQ(sys->getCovariance()(State::NU, State::NU), 4.0);
  EXPECT_DOUBLE_EQ(sys->getCovariance()(State::OMEGA, State::OMEGA), 5.0);
  EXPECT_DOUBLE_EQ(sys->getCovariance()(State::ALPHA, State::ALPHA), 6.0);
}

TEST_F(SystemModelTest, f_TC1)
{
  State state;
  state = sys->f(x, u);
  EXPECT_DOUBLE_EQ(state.x(), 1.0522449600286314);
  EXPECT_DOUBLE_EQ(state.y(), 1.0910794386873719);
  EXPECT_DOUBLE_EQ(state.theta(), 1.1);
  EXPECT_DOUBLE_EQ(state.nu(), 1.1);
  EXPECT_DOUBLE_EQ(state.omega(), 1.0);
  EXPECT_DOUBLE_EQ(state.alpha(), 1.0);
}

TEST_F(SystemModelTest, updateJacobians_TC1)
{
  sys->updateJacobians(x, u);
  EXPECT_DOUBLE_EQ(sys->F_(State::X, State::THETA), -0.091079438687371797);
  EXPECT_DOUBLE_EQ(sys->F_(State::X, State::NU), 0.049757104789172696);
  EXPECT_DOUBLE_EQ(sys->F_(State::X, State::OMEGA), -0.00455397193436859);
  EXPECT_DOUBLE_EQ(sys->F_(State::X, State::ALPHA), 0.002487855239458635);

  EXPECT_DOUBLE_EQ(sys->F_(State::Y, State::THETA), 0.05224496002863133);
  EXPECT_DOUBLE_EQ(sys->F_(State::Y, State::NU), 0.0867423225594017);
  EXPECT_DOUBLE_EQ(sys->F_(State::Y, State::OMEGA), 0.002612248001431567);
  EXPECT_DOUBLE_EQ(sys->F_(State::Y, State::ALPHA), 0.004337116127970085);

  EXPECT_DOUBLE_EQ(sys->F_(State::THETA, State::OMEGA), 0.1);
  EXPECT_DOUBLE_EQ(sys->F_(State::NU, State::ALPHA), 0.1);

  EXPECT_DOUBLE_EQ(sys->W_(State::X, State::OMEGA), -0.00455397193436859);
  EXPECT_DOUBLE_EQ(sys->W_(State::X, State::ALPHA), 0.002487855239458635);

  EXPECT_DOUBLE_EQ(sys->W_(State::Y, State::OMEGA), 0.002612248001431567);
  EXPECT_DOUBLE_EQ(sys->W_(State::Y, State::ALPHA), 0.004337116127970085);

  EXPECT_DOUBLE_EQ(sys->W_(State::THETA, State::OMEGA), 0.1);

  EXPECT_DOUBLE_EQ(sys->W_(State::NU, State::ALPHA), 0.1);

  EXPECT_DOUBLE_EQ(sys->W_(State::OMEGA, State::OMEGA), 1.0);

  EXPECT_DOUBLE_EQ(sys->W_(State::ALPHA, State::ALPHA), 1.0);
}
