#include <random>

#include <gtest/gtest.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>
#include <vi-map/test/vi-map-generator.h>

namespace map_optimization_legacy {

class RemoveVertexEdgeTest : public testing::Test {
 protected:
  RemoveVertexEdgeTest() : map_(), generator_(map_, kRandomSeed) {}

  virtual void SetUp() {
    // Verify if constants make sense.
    static_assert(
        kNumRandomVertexIndex < kNumVertices,
        "Random vertex index should be smaller than the total "
        "number of vertices");

    addPosegraph();
    addLandmarks();

    generator_.generateMap();
  }

  void addPosegraph() {
    pose::Transformation T_G_V;
    pose::Transformation T_G_M;

    mission_ = generator_.createMission(T_G_M);

    vertices_.resize(kNumVertices);

    // Create first mission vertices.
    for (size_t i = 0; i < kNumVertices; ++i) {
      vertices_[i] = generator_.createVertex(mission_, T_G_V);
    }
  }

  void addLandmarks() {
    CHECK(!vertices_.empty());

    // Just a random landmark position that will be visible from the
    // default vertex (keyframe) positions.
    Eigen::Vector3d p_G_fi(0, 0, 1);

    std::mt19937 gen(kRandomSeed);
    std::uniform_int_distribution<> dis(0, kNumVertices - 1);
    for (size_t i = 0; i < kNumLandmarks; ++i) {
      const size_t vertex_index = dis(gen);
      CHECK_LT(vertex_index, vertices_.size());

      pose_graph::VertexIdList observer_vertices(
          vertices_.begin(), vertices_.end());
      for (size_t j = 0; j < observer_vertices.size(); ++j) {
        if (observer_vertices[j] == vertices_[vertex_index]) {
          observer_vertices.erase(observer_vertices.begin() + j);
          break;
        }
      }

      generator_.createLandmark(
          p_G_fi, vertices_[vertex_index], observer_vertices);
    }
  }

  void addResource(size_t vertex_index) {
    CHECK_LT(vertex_index, vertices_.size());
    vi_map::Vertex& vertex = map_.getVertex(vertices_[vertex_index]);

    backend::ResourceId resource_id;
    common::generateId(&resource_id);

    static constexpr size_t kFrameIndex = 0;
    vertex.addFrameResourceIdOfType(
        kFrameIndex, backend::ResourceType::kRawImage, resource_id);
  }

  void removeStoreLandmarksOfVertex(size_t vertex_index) {
    CHECK_LT(vertex_index, vertices_.size());
    vi_map::Vertex& vertex = map_.getVertex(vertices_[vertex_index]);

    vi_map::LandmarkIdList store_landmarks;
    vertex.getStoredLandmarkIdList(&store_landmarks);

    for (const vi_map::LandmarkId& landmark_id : store_landmarks) {
      map_.removeLandmark(landmark_id);
    }
  }

  void removeObservedLandmarksOfVertex(size_t vertex_index) {
    CHECK_LT(vertex_index, vertices_.size());
    vi_map::Vertex& vertex = map_.getVertex(vertices_[vertex_index]);

    vi_map::LandmarkId invalid_global_landmark_id;

    for (unsigned int frame_idx = 0; frame_idx < vertex.numFrames();
         ++frame_idx) {
      vi_map::LandmarkIdList frame_landmark_ids;
      vertex.getFrameObservedLandmarkIds(frame_idx, &frame_landmark_ids);

      for (size_t keypoint_idx = 0; keypoint_idx < frame_landmark_ids.size();
           ++keypoint_idx) {
        if (frame_landmark_ids[keypoint_idx].isValid()) {
          vi_map::Landmark& landmark =
              map_.getLandmark(frame_landmark_ids[keypoint_idx]);

          // Dereference on the landmark side.
          landmark.removeAllObservationsOfVertexAndFrame(
              vertices_[vertex_index], frame_idx);

          // Dereference on the vertex side.
          vertex.setObservedLandmarkId(
              frame_idx, keypoint_idx, invalid_global_landmark_id);
        }
      }
    }
  }

  void removeEdgesOfVertex(size_t vertex_index) {
    CHECK_LT(vertex_index, vertices_.size());
    vi_map::Vertex& vertex = map_.getVertex(vertices_[vertex_index]);

    pose_graph::EdgeIdSet edges;
    vertex.getAllEdges(&edges);

    for (const pose_graph::EdgeId& edge_id : edges) {
      map_.removeEdge(edge_id);
    }
  }

  pose_graph::VertexIdList vertices_;
  vi_map::MissionId mission_;
  vi_map::VIMap map_;

  // Some random vertex in the middle of the posegraph.
  static constexpr size_t kNumRandomVertexIndex = 16;

 private:
  vi_map::VIMapGenerator generator_;

  static constexpr size_t kNumVertices = 30;
  static constexpr size_t kNumLandmarks = 300;
  static constexpr size_t kRandomSeed = 9;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

TEST_F(RemoveVertexEdgeTest, RemoveEdge) {
  pose_graph::EdgeIdList edges;
  map_.getAllEdgeIds(&edges);
  ASSERT_FALSE(edges.empty());

  pose_graph::EdgeId edge_to_be_deleted = *edges.begin();

  EXPECT_TRUE(map_.hasEdge(edge_to_be_deleted));
  map_.removeEdge(edge_to_be_deleted);
  EXPECT_FALSE(map_.hasEdge(edge_to_be_deleted));
}

TEST_F(RemoveVertexEdgeTest, RemoveVertex) {
  removeStoreLandmarksOfVertex(kNumRandomVertexIndex);
  removeObservedLandmarksOfVertex(kNumRandomVertexIndex);
  removeEdgesOfVertex(kNumRandomVertexIndex);

  EXPECT_TRUE(map_.hasVertex(vertices_[kNumRandomVertexIndex]));
  map_.removeVertex(vertices_[kNumRandomVertexIndex]);
  EXPECT_FALSE(map_.hasVertex(vertices_[kNumRandomVertexIndex]));
}

TEST_F(RemoveVertexEdgeTest, RemoveVertexWithEdge) {
  removeStoreLandmarksOfVertex(kNumRandomVertexIndex);
  removeObservedLandmarksOfVertex(kNumRandomVertexIndex);

  EXPECT_TRUE(map_.hasVertex(vertices_[kNumRandomVertexIndex]));
  EXPECT_DEATH(map_.removeVertex(vertices_[kNumRandomVertexIndex]), "edge");
}

TEST_F(RemoveVertexEdgeTest, RemoveVertexWithLandmarks) {
  removeObservedLandmarksOfVertex(kNumRandomVertexIndex);
  removeEdgesOfVertex(kNumRandomVertexIndex);

  EXPECT_TRUE(map_.hasVertex(vertices_[kNumRandomVertexIndex]));
  EXPECT_DEATH(
      map_.removeVertex(vertices_[kNumRandomVertexIndex]), "landmarks");
}

TEST_F(RemoveVertexEdgeTest, RemoveVertexWithLandmarkObservations) {
  removeStoreLandmarksOfVertex(kNumRandomVertexIndex);
  removeEdgesOfVertex(kNumRandomVertexIndex);

  EXPECT_TRUE(map_.hasVertex(vertices_[kNumRandomVertexIndex]));
  EXPECT_DEATH(
      map_.removeVertex(vertices_[kNumRandomVertexIndex]), "references");
}

TEST_F(RemoveVertexEdgeTest, RemoveVertexWithResources) {
  removeStoreLandmarksOfVertex(kNumRandomVertexIndex);
  removeObservedLandmarksOfVertex(kNumRandomVertexIndex);
  removeEdgesOfVertex(kNumRandomVertexIndex);

  addResource(kNumRandomVertexIndex);

  EXPECT_TRUE(map_.hasVertex(vertices_[kNumRandomVertexIndex]));
  EXPECT_DEATH(map_.removeVertex(vertices_[kNumRandomVertexIndex]), "resource");
}

}  // namespace map_optimization_legacy

MAPLAB_UNITTEST_ENTRYPOINT
