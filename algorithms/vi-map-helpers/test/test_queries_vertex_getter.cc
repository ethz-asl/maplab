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
#include <vi-map/test/vi-map-generator.h>
#include <vi-map/vi-map.h>

#include "vi-map-helpers/vi-map-queries.h"

namespace vi_map_helpers {

class ViMapQueriesTest : public ::testing::Test {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static constexpr size_t kRandomSeed = 42u;
  static constexpr size_t kNumVertices = 5u;

 protected:
  ViMapQueriesTest() : map_(), generator_(map_, kRandomSeed) {}

  void generateMap() {
    T_G_M_.getPosition().setZero();

    mission_id_ = generator_.createMission(T_G_M_);

    for (size_t i = 0u; i < kNumVertices; ++i) {
      T_G_V_[i].getPosition().setZero();
      vertex_ids_[i] = generator_.createVertex(mission_id_, T_G_V_[i]);
    }

    for (size_t i = 0u; i < 2 * kNumVertices; ++i) {
      pose_graph::VertexIdList observer_ids;
      // Add two random coobservations.
      observer_ids.push_back(vertex_ids_[(i / 2 + 2) % kNumVertices]);
      observer_ids.push_back(vertex_ids_[(i / 2 + 4) % kNumVertices]);
      generator_.createLandmark(
          Eigen::Vector3d(0, 0, 1), vertex_ids_[i / 2], {});
    }

    generator_.generateMap();
  }

  vi_map::VIMap map_;
  vi_map::VIMapGenerator generator_;

 private:
  vi_map::MissionId mission_id_;
  pose_graph::VertexId vertex_ids_[kNumVertices];
  pose::Transformation T_G_M_;
  pose::Transformation T_G_V_[kNumVertices];
};

TEST_F(ViMapQueriesTest, VIMapGetBoundaryVertexIdsTest) {
  generateMap();
  pose_graph::VertexIdList all_vertex_ids, boundary_vertices;
  pose_graph::VertexIdSet center_vertex_set;
  pose_graph::VertexId center_vertex;

  map_.getAllVertexIds(&all_vertex_ids);

  map_.setRandIntGeneratorSeed(kRandomSeed);
  map_.getRandomVertexId(&center_vertex);
  VIMapQueries map_query(map_);
  map_query.getBoundaryVertexIds(center_vertex_set, &boundary_vertices);

  for (const pose_graph::VertexId& boundary_vertex_id : boundary_vertices) {
    bool is_boundary = false;
    pose_graph::VertexId next_vertex_id, prev_vertex_id;

    if (map_.getNextVertex(
            boundary_vertex_id, pose_graph::Edge::EdgeType::kViwls,
            &next_vertex_id)) {
      is_boundary =
          center_vertex_set.find(next_vertex_id) != center_vertex_set.end();
    }
    if (map_.getPreviousVertex(
            boundary_vertex_id, pose_graph::Edge::EdgeType::kViwls,
            &prev_vertex_id)) {
      is_boundary = is_boundary || (center_vertex_set.find(next_vertex_id) !=
                                    center_vertex_set.end());
    }
    EXPECT_TRUE(is_boundary);
  }
}

TEST_F(ViMapQueriesTest, VIMapGetCoobservingVertices) {
  generateMap();
  pose_graph::VertexIdList all_vertex_ids;
  pose_graph::VertexIdSet center_vertex_set, coobserved_vertices;
  pose_graph::VertexId center_vertex;

  map_.getAllVertexIds(&all_vertex_ids);

  map_.setRandIntGeneratorSeed(kRandomSeed);
  map_.getRandomVertexId(&center_vertex);
  VIMapQueries map_query(map_);

  constexpr double kMinCoobservedFeaturesRatio = 0.0;
  constexpr size_t kMinCoobservedFeaturesCount = 1u;
  map_query.getCoobservingVertices(
      center_vertex_set, kMinCoobservedFeaturesRatio,
      kMinCoobservedFeaturesCount, &coobserved_vertices);
  vi_map::LandmarkIdList landmarks;
  map_.getVertex(center_vertex).getAllObservedLandmarkIds(&landmarks);

  for (const pose_graph::VertexId& vertex_id : coobserved_vertices) {
    bool is_coobserving = false;
    vi_map::LandmarkIdList potentially_coobserved_landmarks;
    map_.getVertex(vertex_id).getAllObservedLandmarkIds(
        &potentially_coobserved_landmarks);

    for (const vi_map::LandmarkId& landmark_id :
        potentially_coobserved_landmarks) {
      is_coobserving =
          is_coobserving ||
          (std::find(landmarks.begin(), landmarks.end(), landmark_id) !=
           landmarks.end());
    }
    EXPECT_TRUE(is_coobserving);
  }
}
}  // namespace vi_map_helpers

MAPLAB_UNITTEST_ENTRYPOINT
