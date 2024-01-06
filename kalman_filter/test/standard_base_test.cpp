#include <gtest/gtest.h>

#define protected public
#include "kalman_filter/standard_base.hpp"
#undef protected


using namespace kalman;

class StandardBaseTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    cov = std::make_unique<SBv3>();
  }
  void TearDown() override
  {
  }

  using SBv3 = StandardBase<Vector<3>>;
  std::unique_ptr<SBv3> cov;
};

TEST_F(StandardBaseTest, Contructor_TC1)
{
  EXPECT_EQ(cov->P_.rows(), 3);
  EXPECT_EQ(cov->P_.cols(), 3);
  EXPECT_EQ(cov->P_.size(), 9);
  EXPECT_TRUE(cov->P_.isIdentity());
}

TEST_F(StandardBaseTest, setCovariance_TC1)
{
  Covariance<Vector<3>> Q;
  Q.setOnes();

  cov->setCovariance(Q);

  EXPECT_EQ(cov->P_.rows(), 3);
  EXPECT_EQ(cov->P_.cols(), 3);
  EXPECT_EQ(cov->P_.size(), 9);
  EXPECT_TRUE(cov->P_.isOnes());
}

TEST_F(StandardBaseTest, getCovariance_TC1)
{
  Covariance<Vector<3>> Q;
  Q.setOnes();

  cov->setCovariance(Q);

  EXPECT_EQ(cov->getCovariance().rows(), 3);
  EXPECT_EQ(cov->getCovariance().cols(), 3);
  EXPECT_EQ(cov->getCovariance().size(), 9);
  EXPECT_TRUE(cov->getCovariance().isOnes());
}
