#include <gtest/gtest.h>

#include "kalman_filter/matrix.hpp"


using namespace kalman;

class VectorTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    state = std::make_unique<State>();
  }
  void TearDown() override
  {
  }

  using State = Vector<3>;
  std::unique_ptr<State> state;
};

TEST_F(VectorTest, Contructor_TC1)
{
  EXPECT_EQ(state->size(), 3);
}

TEST_F(VectorTest, CopyConstructor_TC1)
{
  state->setOnes();
  State s(*state);

  EXPECT_EQ(s.size(), 3);
  EXPECT_TRUE(s.isOnes());
}

TEST_F(VectorTest, AssignmentOperator_TC1)
{
  state->setOnes();
  State s = *state;

  EXPECT_EQ(s.size(), 3);
  EXPECT_TRUE(s.isOnes());
}
