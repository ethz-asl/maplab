#include "sparse-graph/common/representative-node.h"

#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>

namespace spg {

class RepresentativeNodeTest : public ::testing::Test {
 protected:
  RepresentativeNodeTest() : ::testing::Test() {}

  virtual void SetUp() {}
};

TEST_F(RepresentativeNodeTest, TestEqualTo) {
  aslam::Transformation pose;
  RepresentativeNode lhs(pose, 0, 0);
  RepresentativeNode lhs_copy(pose, 0, 0);
  RepresentativeNode lhs_2(pose, 10, 10);
  RepresentativeNode lhs_2_copy(pose, 10, 10);

  EXPECT_TRUE(lhs.isEqualTo(lhs_copy));
  EXPECT_TRUE(lhs == lhs_copy);
  EXPECT_TRUE(lhs_2.isEqualTo(lhs_2_copy));
  EXPECT_TRUE(lhs_2 == lhs_2_copy);
}

TEST_F(RepresentativeNodeTest, TestNotEqualTo) {
  aslam::Transformation pose;
  RepresentativeNode lhs(pose, 0, 0);
  RepresentativeNode lhs_copy(pose, 0, 0);
  RepresentativeNode lhs_2(pose, 10, 10);
  RepresentativeNode lhs_2_copy(pose, 10, 10);

  EXPECT_FALSE(lhs.isEqualTo(lhs_2_copy));
  EXPECT_TRUE(lhs != lhs_2_copy);
  EXPECT_FALSE(lhs_2.isEqualTo(lhs_copy));
  EXPECT_TRUE(lhs_2 != lhs_copy);
}

}  // namespace spg

MAPLAB_UNITTEST_ENTRYPOINT
