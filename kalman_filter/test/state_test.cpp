#include <gtest/gtest.h>

#include "kalman_filter/state.hpp"


using namespace kalman;

class StateTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    state = std::make_unique<State>();
  }
  void TearDown() override
  {
  }

  std::unique_ptr<State> state;
};

TEST_F(StateTest, Contructor_TC1)
{
  EXPECT_EQ(state->size(), 6);
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
