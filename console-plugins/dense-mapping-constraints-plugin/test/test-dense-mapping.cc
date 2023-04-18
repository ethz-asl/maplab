#include "dense-mapping/dense-mapping.h"

#include <atomic>
#include <chrono>
#include <functional>
#include <thread>

#include <aslam/common/timer.h>
#include <map-manager/map-manager.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>
#include <registration-toolbox/common/registration-gflags.h>
#include <vi-map/vi-map.h>
#include <vi-mapping-test-app/vi-mapping-test-app.h>
#include <visualization/resource-visualization.h>
#include <visualization/rviz-visualization-sink.h>
#include <visualization/viwls-graph-plotter.h>

#include "dense-mapping/dense-mapping-parallel-process.h"

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
    const std::string map_path = "./test_maps/common_test_map/";
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
  FLAGS_dm_candidate_search_enable_intra_mission_consecutive = true;
  FLAGS_dm_candidate_search_enable_intra_mission_proximity = true;
  FLAGS_dm_candidate_search_enable_inter_mission_proximity = true;
  regbox::FLAGS_regbox_pcl_downsample_leaf_size_m = 0.1;

  vi_map::VIMap* map_ptr = CHECK_NOTNULL(test_app_.getMapMutable());

  EXPECT_EQ(getNumLoopClosureEdges(*map_ptr), 0);

  vi_map::MissionIdList mission_ids;
  map_ptr->getAllMissionIds(&mission_ids);

  const Config config = Config::fromGflags();
  EXPECT_TRUE(addDenseMappingConstraintsToMap(config, mission_ids, map_ptr));
  EXPECT_GE(getNumLoopClosureEdges(*map_ptr), 1);

  visualizeMap();
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

}  // namespace dense_mapping

MAPLAB_UNITTEST_ENTRYPOINT
