#include <gtest/gtest.h>
#define private public
#include "kalman_filter/control.hpp"
#undef private


class ControlTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    control = std::make_unique<kalman::Control>();
  }
  void TearDown() override
  {
  }

  std::unique_ptr<kalman::Control> control;
};

TEST_F(ControlTest, Contructor_TC1)
{
  EXPECT_EQ(control->size(), kalman::ControlSize);
}

TEST_F(ControlTest, SetGet_TC1)
{
  control->omega() = 1.0;
  control->alpha() = 2.0;

  EXPECT_DOUBLE_EQ(control->omega(), 1.0);
  EXPECT_DOUBLE_EQ(control->alpha(), 2.0);
}
