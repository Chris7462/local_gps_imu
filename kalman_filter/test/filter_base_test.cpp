#include <gtest/gtest.h>

#define protected public
#include "kalman_filter/filter_base.hpp"
#undef protected


using namespace kalman;

class FilterBaseTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    state = std::make_unique<FBv3>();
  }
  void TearDown() override
  {
  }

  using FBv3 = FilterBase<Vector<3>>;
  std::unique_ptr<FBv3> state;
};

TEST_F(FilterBaseTest, Contructor_TC1)
{
  EXPECT_EQ(state->x_.size(), 3);
}

TEST_F(FilterBaseTest, init_TC1)
{
  Vector<3> init_x;
  init_x.setOnes();
  state->init(init_x);

  EXPECT_EQ(state->x_.size(), 3);
  EXPECT_TRUE(state->x_.isOnes());
}

TEST_F(FilterBaseTest, getState_TC1)
{
  Vector<3> init_x;
  init_x.setOnes();
  state->init(init_x);

  EXPECT_EQ(state->getState().size(), 3);
  EXPECT_TRUE(state->getState().isOnes());
}
