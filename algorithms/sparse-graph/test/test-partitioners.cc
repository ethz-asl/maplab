#include "sparse-graph/sparse-graph.h"

#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>

namespace spg {

class PartitionerTest : public ::testing::Test {
 protected:
  PartitionerTest() : ::testing::Test() {}

  virtual void SetUp() {}

  pose_graph::VertexIdList createRandomIds(const std::size_t n_ids) {
    pose_graph::VertexIdList all_vertices;

    for (std::size_t i = 0; i < n_ids; ++i) {
      pose_graph::VertexId id = aslam::createRandomId<pose_graph::VertexId>();
      all_vertices.emplace_back(std::move(id));
    }

    return all_vertices;
  }
};

TEST_F(PartitionerTest, TestAveragePartitioner) {}

}  // namespace spg

MAPLAB_UNITTEST_ENTRYPOINT
