#include <Eigen/Core>
#include <aslam/common/pose-types.h>
#include <gtest/gtest.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <vi-map/test/vi-map-generator.h>

#include "vi-map-helpers/mission-clustering-coobservation.h"

namespace vi_map_helpers {
// Create the following test map:
//   Mission_0    Mission_1    Mission_2    Mission_3    Mission_4
//    |      \    /                |   \   /                |
//    L0       L1-------------------     L2                 L3
//                                                          |
//                                          Mission_6    Mission_5
//                                            \_ L4
void createTestMap(vi_map::VIMap* map, vi_map::MissionIdList* missions) {
  CHECK_NOTNULL(map);
  CHECK_NOTNULL(missions);
  vi_map::VIMapGenerator generator(*map, /*random_seed=*/0);

  // Create empty missions and a single vertex to it.
  vi_map::MissionIdList& M = *missions;
  pose_graph::VertexIdList V;
  for (int i = 0; i < 7; ++i) {
    aslam::Transformation dummy_pose;
    M.emplace_back(generator.createMission(dummy_pose));
    V.emplace_back(generator.createVertex(M.back(), dummy_pose));
  }

  // Create landmarks.
  Eigen::Vector3d dummy_position(0, 0, 100);
  vi_map::LandmarkIdList L(5);
  L[0] = generator.createLandmark(dummy_position, V[0], {V[1]});
  L[1] = generator.createLandmark(dummy_position, V[0], {V[1], V[2]});
  L[2] = generator.createLandmark(dummy_position, V[2], {V[3]});
  L[3] = generator.createLandmark(dummy_position, V[4], {V[5]});
  L[4] = generator.createLandmark(dummy_position, V[6], {});

  generator.generateMap();
}

bool clustersEqualWithoutOrdering(
    const std::vector<vi_map::MissionIdSet>& clusters_a,
    const std::vector<vi_map::MissionIdSet>& clusters_b) {
  CHECK_EQ(clusters_a.size(), clusters_b.size());

  // Find unique matches for each element in A to an element in B.
  std::unordered_set<size_t> matched_indices_of_b;
  for (const vi_map::MissionIdSet& a_set : clusters_a) {
    bool found_match = false;
    size_t index_b = 0u;
    for (const vi_map::MissionIdSet& b_set : clusters_b) {
      if (a_set == b_set && matched_indices_of_b.count(index_b) == 0) {
        found_match = true;
        matched_indices_of_b.insert(index_b);
        break;
      }
      ++index_b;
    }
    if (!found_match) {
      return false;
    }
  }
  CHECK_EQ(matched_indices_of_b.size(), clusters_b.size());
  return true;
}

void printClusters(const std::vector<vi_map::MissionIdSet>& clusters) {
  for (const vi_map::MissionIdSet& cluster : clusters) {
    std::stringstream ss;
    ss << "Cluster: ";
    for (const vi_map::MissionId& id : cluster) {
      ss << id << ", ";
    }
    LOG(INFO) << ss.str();
  }
}

TEST(MissionClusteringCoobservation, ClusterMissionByLandmarkCoobservations) {
  vi_map::VIMap map;
  vi_map::MissionIdList map_missions;
  createTestMap(&map, &map_missions);

  // Clustering using all missions.
  vi_map::MissionIdSet mission_to_query(
      map_missions.begin(), map_missions.end());
  std::vector<vi_map::MissionIdSet> clusters =
      clusterMissionByLandmarkCoobservations(map, mission_to_query);

  std::vector<vi_map::MissionIdSet> expected_clusters;
  const vi_map::MissionIdList& M = map_missions;
  expected_clusters.emplace_back(vi_map::MissionIdSet{M[0], M[1], M[2], M[3]});
  expected_clusters.emplace_back(vi_map::MissionIdSet{M[4], M[5]});
  expected_clusters.emplace_back(vi_map::MissionIdSet{M[6]});
  EXPECT_TRUE(clustersEqualWithoutOrdering(expected_clusters, clusters));

  // Clustering all missions without mission 2.
  mission_to_query.erase(M[2]);
  clusters = clusterMissionByLandmarkCoobservations(map, mission_to_query);

  expected_clusters.clear();
  expected_clusters.emplace_back(vi_map::MissionIdSet{M[0], M[1]});
  expected_clusters.emplace_back(vi_map::MissionIdSet{M[3]});
  expected_clusters.emplace_back(vi_map::MissionIdSet{M[4], M[5]});
  expected_clusters.emplace_back(vi_map::MissionIdSet{M[6]});
  EXPECT_TRUE(clustersEqualWithoutOrdering(expected_clusters, clusters));
}

}  // namespace vi_map_helpers

MAPLAB_UNITTEST_ENTRYPOINT
