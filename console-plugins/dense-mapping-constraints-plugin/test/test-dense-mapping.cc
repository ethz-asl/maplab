#include <aslam/common/timer.h>
#include <atomic>
#include <chrono>
#include <functional>
#include <map-manager/map-manager.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>
#include <registration-toolbox/common/registration-gflags.h>
#include <thread>
#include <vi-map/vi-map.h>
#include <vi-mapping-test-app/vi-mapping-test-app.h>
#include <visualization/resource-visualization.h>
#include <visualization/rviz-visualization-sink.h>
#include <visualization/viwls-graph-plotter.h>

#include "dense-mapping/dense-mapping-parallel-process.h"
#include "dense-mapping/dense-mapping.h"

DECLARE_bool(vis_lc_edge_covariances);

namespace dense_mapping {

class DenseMappingTest : public ::testing::Test {
 protected:
  DenseMappingTest() : ::testing::Test() {
    constexpr bool kVisualizeMap = false;
    if (kVisualizeMap) {
      visualization::RVizVisualizationSink::init();
      plotter_.reset(new visualization::ViwlsGraphRvizPlotter);
    }
  }

  virtual void SetUp() {
    const std::string map_path = "./test_maps/dense_mapping_test_map/";
    test_app_.loadDataset(map_path);
    CHECK_NOTNULL(test_app_.getMapMutable());

    LOG(INFO) << "Loaded map with " << test_app_.numVerticesOnMap()
              << " vertices from '" << map_path << "'";
  }

  size_t getNumLoopClosureEdges(const vi_map::VIMap& map) {
    size_t num_loop_closure_edges = 0u;

    pose_graph::EdgeIdList edge_ids;
    map.getAllEdgeIds(&edge_ids);
    for (const pose_graph::EdgeId& edge_id : edge_ids) {
      CHECK(edge_id.isValid());
      if (map.getEdgeType(edge_id) ==
          pose_graph::Edge::EdgeType::kLoopClosure) {
        ++num_loop_closure_edges;
      }
    }
    return num_loop_closure_edges;
  }

  void visualizeMap() {
    if (plotter_) {
      LOG(INFO) << "Visualizing map...";
      const vi_map::VIMap& map = *test_app_.getMapMutable();
      plotter_->visualizeMap(map);
      std::this_thread::sleep_for(std::chrono::seconds(2));
      plotter_->visualizeMap(map);
      std::this_thread::sleep_for(std::chrono::seconds(2));

      vi_map::MissionIdList mission_ids;
      map.getAllMissionIds(&mission_ids);
      FLAGS_vis_pointcloud_accumulated_before_publishing = true;
      visualization::visualizeReprojectedDepthResource(
          backend::ResourceType::kPointCloudXYZI, mission_ids, map);

      std::this_thread::sleep_for(std::chrono::seconds(2));
    }
  }

  visual_inertial_mapping::VIMappingTestApp test_app_;
  std::unique_ptr<visualization::ViwlsGraphRvizPlotter> plotter_;
};

TEST_F(DenseMappingTest, TestDenseMapping) {
  FLAGS_vis_lc_edge_covariances = true;
  FLAGS_tf_map_frame = "darpa";
  FLAGS_dm_candidate_search_enable_intra_mission_consecutive = true;
  FLAGS_dm_candidate_search_enable_intra_mission_proximity = true;
  FLAGS_dm_candidate_search_enable_inter_mission_proximity = true;
  regbox::FLAGS_regbox_pcl_downsample_leaf_size_m = 0.1;

  vi_map::VIMap* map_ptr = CHECK_NOTNULL(test_app_.getMapMutable());

  // Initial loop closures in map. These were created by the stationary submap
  // logic.
  EXPECT_EQ(getNumLoopClosureEdges(*map_ptr), 0);

  vi_map::MissionIdList mission_ids;
  map_ptr->getAllMissionIds(&mission_ids);

  const Config config = Config::fromGflags();
  timing::TimerImpl timer_first("addDenseMappingConstraintsToMap (run 1)");
  EXPECT_TRUE(addDenseMappingConstraintsToMap(config, mission_ids, map_ptr));
  timer_first.Stop();

  EXPECT_NEAR(getNumLoopClosureEdges(*map_ptr), 44, 5);

  timing::TimerImpl timer_second("addDenseMappingConstraintsToMap (run 2)");
  EXPECT_TRUE(addDenseMappingConstraintsToMap(config, mission_ids, map_ptr));
  timer_second.Stop();

  // No new edges are computed.
  EXPECT_NEAR(getNumLoopClosureEdges(*map_ptr), 44, 5);

  visualizeMap();

  LOG(INFO) << timing::Timing::Print();
}

TEST_F(DenseMappingTest, TestParallelProcessEqualThreads) {
  const std::size_t num_threads = 4;
  const std::size_t start = 0;
  const std::size_t end = 4;
  std::atomic<std::size_t> thread_accum{0u};
  std::function<void(std::size_t, std::size_t, std::size_t)> functor =
      [start, end, &thread_accum](
          std::size_t thread_idx, std::size_t start_idx, std::size_t end_idx) {
        EXPECT_GE(start_idx, start);
        EXPECT_LE(end_idx, end);
        EXPECT_GE(thread_idx, start_idx);
        EXPECT_LE(thread_idx, end_idx);
        ++thread_accum;
      };

  const std::size_t actual_num_threads =
      parallelProcess(functor, start, end, num_threads);
  EXPECT_EQ(actual_num_threads, num_threads);
  EXPECT_EQ(thread_accum.load(), end - start);
}

TEST_F(DenseMappingTest, TestParallelProcessLessThreads) {
  const std::size_t num_threads = 4;
  const std::size_t start = 0;
  const std::size_t end = 6;
  std::atomic<std::size_t> thread_accum{0u};
  std::function<void(std::size_t, std::size_t, std::size_t)> functor =
      [start, end, &thread_accum](
          std::size_t thread_idx, std::size_t start_idx, std::size_t end_idx) {
        EXPECT_GE(start_idx, start);
        EXPECT_LE(end_idx, end);
        EXPECT_GE(thread_idx, start);
        EXPECT_LE(thread_idx, end);
        for (std::size_t i = start_idx; i < end_idx; ++i) {
          ++thread_accum;
        }
      };

  const std::size_t actual_num_threads =
      parallelProcess(functor, start, end, num_threads);
  EXPECT_GT(actual_num_threads, 0);
  EXPECT_LE(actual_num_threads, num_threads);
  EXPECT_EQ(thread_accum.load(), end - start);
}

TEST_F(DenseMappingTest, TestParallelProcessMoreThreads) {
  const std::size_t num_threads = 4;
  const std::size_t start = 0;
  const std::size_t end = 2;
  std::atomic<std::size_t> thread_accum{0u};
  std::function<void(std::size_t, std::size_t, std::size_t)> functor =
      [start, end, &thread_accum](
          std::size_t thread_idx, std::size_t start_idx, std::size_t end_idx) {
        EXPECT_GE(start_idx, start);
        EXPECT_LE(end_idx, end);
        EXPECT_GE(thread_idx, start);
        EXPECT_LE(thread_idx, end);
        for (std::size_t i = start_idx; i < end_idx; ++i) {
          ++thread_accum;
        }
      };

  const std::size_t actual_num_threads =
      parallelProcess(functor, start, end, num_threads);
  EXPECT_GT(actual_num_threads, 0);
  EXPECT_LE(actual_num_threads, num_threads);
  EXPECT_EQ(thread_accum.load(), end - start);
}

TEST_F(DenseMappingTest, TestEdgeRemoval) {
  FLAGS_vis_lc_edge_covariances = true;
  FLAGS_tf_map_frame = "darpa";
  FLAGS_dm_candidate_search_enable_intra_mission_consecutive = true;
  FLAGS_dm_candidate_search_enable_intra_mission_proximity = true;
  FLAGS_dm_candidate_search_enable_inter_mission_proximity = true;
  regbox::FLAGS_regbox_pcl_downsample_leaf_size_m = 0.1;
  vi_map::VIMap* map_ptr = CHECK_NOTNULL(test_app_.getMapMutable());
  const Config config = Config::fromGflags();

  EXPECT_EQ(getNumLoopClosureEdges(*map_ptr), 0);
  vi_map::MissionIdList mission_ids;
  map_ptr->getAllMissionIds(&mission_ids);
  EXPECT_TRUE(addDenseMappingConstraintsToMap(config, mission_ids, map_ptr));

  const size_t n_edges = getNumLoopClosureEdges(*map_ptr);
  EXPECT_GT(n_edges, 0);

  pose_graph::VertexIdList all_vertices;
  map_ptr->getAllVertexIdsAlongGraphsSortedByTimestamp(&all_vertices);

  const size_t n_to_remove = 20u;
  const std::size_t n_vertices = all_vertices.size();
  std::size_t n_removed_vertices = 0u;
  CHECK_GT(n_vertices, n_to_remove);
  for (const pose_graph::VertexId& v : all_vertices) {
    if (removeConstraintsFromVertex(v, map_ptr)) {
      ++n_removed_vertices;
    }
  }
  EXPECT_GT(n_removed_vertices, 0u);
  EXPECT_LT(getNumLoopClosureEdges(*map_ptr), n_edges);
}

}  // namespace dense_mapping

MAPLAB_UNITTEST_ENTRYPOINT
