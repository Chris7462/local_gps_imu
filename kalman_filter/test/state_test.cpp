#include <gtest/gtest.h>
#define private public
#include "kalman_filter/state.hpp"
#undef private


class StateTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    state = std::make_unique<kalman::State>();
  }
  void TearDown() override
  {
  }

  std::unique_ptr<kalman::State> state;
};

TEST_F(StateTest, Contructor_TC1)
{
  EXPECT_EQ(state->size(), kalman::StateSize);
}

TEST_F(StateTest, SetGet_TC1)
{
  state->x() = 1.0;
  state->y() = 2.0;
  state->theta() = 3.0;
  state->nu() = 4.0;
  state->omega() = 5.0;
  state->alpha() = 6.0;

  EXPECT_DOUBLE_EQ(state->x(), 1.0);
  EXPECT_DOUBLE_EQ(state->y(), 2.0);
  EXPECT_DOUBLE_EQ(state->theta(), 3.0);
  EXPECT_DOUBLE_EQ(state->nu(), 4.0);
  EXPECT_DOUBLE_EQ(state->omega(), 5.0);
  EXPECT_DOUBLE_EQ(state->alpha(), 6.0);
}
