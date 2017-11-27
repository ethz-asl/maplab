#include <memory>
#include <random>
#include <unordered_map>
#include <unordered_set>

#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/distortion-fisheye.h>
#include <aslam/cameras/ncamera.h>
#include <aslam/frames/visual-frame.h>
#include <glog/logging.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>
#include <maplab-common/unique-id.h>
#include <vi-map/check-map-consistency.h>
#include <vi-map/pose-graph.h>
#include <vi-map/vi-map.h>

#include "map-optimization-legacy/test/6dof-test-trajectory-gen.h"
#include "map-optimization-legacy/test/6dof-vi-map-gen.h"

namespace map_optimization_legacy {

class ViwlsGraph : public ::testing::Test {
 protected:
  virtual ~ViwlsGraph() {}

 protected:
  SixDofVIMapGenerator vimap_gen_;
};

TEST_F(ViwlsGraph, VIMapGetRandomVertexIdTest) {
  vimap_gen_.generateVIMap();
  pose_graph::VertexIdList all_vertex_ids;
  pose_graph::VertexId rand_vertex_a, rand_vertex_b, rand_vertex_c;

  vi_map::VIMap& vi_map = vimap_gen_.vi_map_;
  vi_map.getAllVertexIds(&all_vertex_ids);

  vi_map.setRandIntGeneratorSeed(2);
  vi_map.getRandomVertexId(&rand_vertex_a);
  vi_map.getRandomVertexId(&rand_vertex_b);
  vi_map.getRandomVertexId(&rand_vertex_c);
  EXPECT_TRUE(rand_vertex_a.isValid());
  EXPECT_TRUE(rand_vertex_b.isValid());
  EXPECT_TRUE(rand_vertex_c.isValid());
  EXPECT_NE(rand_vertex_a.hexString(), rand_vertex_b.hexString());
  EXPECT_NE(rand_vertex_a.hexString(), rand_vertex_c.hexString());
}

}  // namespace map_optimization_legacy

MAPLAB_UNITTEST_ENTRYPOINT
