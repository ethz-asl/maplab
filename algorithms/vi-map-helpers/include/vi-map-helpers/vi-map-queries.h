#ifndef VI_MAP_HELPERS_VI_MAP_QUERIES_H_
#define VI_MAP_HELPERS_VI_MAP_QUERIES_H_

#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <aslam/common/pose-types.h>
#include <posegraph/unique-id.h>
#include <vi-map/loop-constraint.h>
#include <vi-map/unique-id.h>

namespace vi_map {
class Vertex;
class VIMap;

typedef std::pair<vi_map::MissionId, pose_graph::VertexIdList>
    MissionVertexIdPair;
typedef std::unordered_map<vi_map::MissionId, pose_graph::VertexIdList>
    MissionVertexIdList;
}  // namespace vi_map

namespace vi_map_helpers {

// Finds a vertex with the largest number of overlapping (i.e. commonly
// observed) landmarks. Useful for creation of loop closure edges.
pose_graph::VertexId getVertexIdWithMostOverlappingLandmarks(
    const pose_graph::VertexId& query_vertex_id,
    const vi_map::VertexKeyPointToStructureMatchList& structure_matches,
    const vi_map::VIMap& map, vi_map::LandmarkIdSet* overlap_landmarks);

// A collection of queries on a VIMap which involve more logic than simple
// data retrieval.
class VIMapQueries {
 public:
  explicit VIMapQueries(const vi_map::VIMap& map);

  // ==================
  // GLOBAL MAP QUERIES
  // ==================
  void getIdsOfVerticesWithLandmarkObservations(
      pose_graph::VertexIdList* result);

  void getAllWellConstrainedLandmarkIds(
      vi_map::LandmarkIdList* landmark_ids) const;

  void getAllNotWellConstrainedLandmarkIds(
      vi_map::LandmarkIdList* landmark_ids) const;

  // =============
  // MISSION ID IN
  // =============
  void forIdsOfObservedLandmarksOfEachVertexWhile(
      const vi_map::MissionId& mission_id,
      const std::function<
          bool(const vi_map::LandmarkIdSet& store_landmarks)>&  // NOLINT
      action) const;
  void forIdsOfObservedLandmarksOfEachVertexAlongGraphWhile(
      const vi_map::MissionId& mission_id,
      const std::function<
          bool(const vi_map::LandmarkIdSet& store_landmarks)>&  // NOLINT
      action) const;
  void getLandmarksObservedByMission(
      const vi_map::MissionId& query_mission,
      vi_map::LandmarkIdSet* observed_landmarks) const;
  bool observesLandmarksObservedByMultipleMissions(
      const vi_map::MissionId& query_mission) const;
  void getMissionsStoringLandmarksObservedByMultipleMissions(
      const vi_map::MissionId& query_mission,
      vi_map::MissionIdSet* result) const;
  bool storesLandmarksObservedByOtherMissions(
      const vi_map::MissionId& query_mission) const;

  void getVertexIdsAlongGraphForMissions(
      const vi_map::MissionIdList& mission_id_list,
      vi_map::MissionVertexIdList* mission_to_vertex_ids_map) const;

  // ====================
  // LANDMARK ID IN
  // ====================
  void getCommonObserversForLandmarks(
      const vi_map::LandmarkIdList& landmarks,
      pose_graph::VertexIdList* result) const;
  bool isLandmarkObservedByMultipleMissions(
      const vi_map::LandmarkId& landmark_id) const;

  // ===============
  // VISUAL FRAME IN
  // ===============
  void getVisualFramesWithCommonLandmarksSortedMostToLeast(
      const vi_map::Vertex& vertex, const size_t frame_index,
      const size_t min_common_landmarks,
      vi_map::VisualFrameIdentifierList* result) const;
  size_t getNumWellConstrainedLandmarks(
      const vi_map::Vertex& vertex, const size_t frame_index) const;

  // ============
  // VERTEX ID IN
  // ============
  void getFollowingVertexIdsAlongGraph(
      const pose_graph::VertexId& starting_vertex,
      const bool include_starting_vertex,
      pose_graph::VertexIdList* result) const;
  // Includes starting vertex.
  void getFollowingVertexIdsAlongGraph(
      const pose_graph::VertexId& starting_vertex,
      pose_graph::VertexIdList* result) const;
  int getVerticesWithCommonLandmarks(
      const pose_graph::VertexId& vertex_id, int num_matches_to_return,
      int min_number_common_landmarks,
      pose_graph::VertexIdList* coobserver_vertex_ids) const;

  struct VertexCommonLandmarksCount {
    int in_common;
    pose_graph::VertexId vertex_id;
    VertexCommonLandmarksCount(int common_count, const pose_graph::VertexId& id)
        : in_common(common_count), vertex_id(id) {}
  };
  typedef std::vector<VertexCommonLandmarksCount>
      VertexCommonLandmarksCountVector;

  int getVerticesWithCommonLandmarks(
      const pose_graph::VertexId& vertex_id, int min_number_common_landmarks,
      VertexCommonLandmarksCountVector* coobserver_vertex_ids) const;
  int getVertexWithMostLandmarksInCommon(
      const pose_graph::VertexId& vertex_id,
      pose_graph::VertexId* best_match_vertex_id) const;
  void getIdsOfLandmarksObservedByVertex(
      const pose_graph::VertexId& vertex_id,
      vi_map::LandmarkIdSet* result) const;

  // =============
  // VERTEX IDS IN
  // =============
  int getNumberOfCommonLandmarks(
      const pose_graph::VertexId& vertex_1,
      const pose_graph::VertexId& vertex_2,
      vi_map::LandmarkIdSet* landmarks) const;
  int getNumberOfCommonLandmarks(
      const pose_graph::VertexId& vertex_1,
      const pose_graph::VertexId& vertex_2) const;

  void getCoobservingVertices(
      const pose_graph::VertexIdSet& given_vertices,
      const double min_coobserved_features_ratio,
      const size_t min_coobsered_features_count,
      std::unordered_set<pose_graph::VertexId>* coobserving_vertices) const;

  /// Check if a given vertex shares a given minimum percentage of observed
  /// landmarks with a given set of vertices.
  bool isCoobservingBasedOnPercentage(
      double minimum_percentage_of_shared_observations,
      const pose_graph::VertexIdSet& given_vertices,
      const pose_graph::VertexId& query_vertex_id) const;

  /// Check if a given vertex has at least one observed landmark with a
  /// minimum of observers from a given set of vertices.
  bool isCoobservingBasedOnAbsoluteCountOfLandmarkObservers(
      unsigned int minimum_observers_needed,
      const pose_graph::VertexIdSet& given_vertices,
      const pose_graph::VertexId& query_vertex_id) const;
  void getBoundaryVertexIds(
      const pose_graph::VertexIdSet& inner_vertices,
      pose_graph::VertexIdList* boundary_vertices) const;
  void getBoundaryVertexIds(
      const pose_graph::VertexIdList& inner_vertices,
      pose_graph::VertexIdList* boundary_vertices) const;

  template <typename VertexIdContainerType>
  inline void getConsecutiveLandmarkObserverGroupsFromMission(
      const vi_map::LandmarkId& store_landmark_id,
      const vi_map::MissionId& mission_id,
      std::vector<VertexIdContainerType>* clustered_vertex_ids) const;

  /// Clusters a set of vertex ids from a single mission to groups of
  /// consecutive vertices based on the mission trajectory.
  template <typename VertexIdContainerType>
  inline void clusterConsecutiveVerticesFromOneMission(
      const pose_graph::VertexIdSet& vertices,
      std::vector<VertexIdContainerType>* clustered_vertex_ids) const;

  // =========
  // VERTEX IN
  // =========
  void forEachWellConstrainedObservedLandmark(
      const vi_map::Vertex& vertex,
      const std::function<void(
          const size_t frame_idx, const vi_map::LandmarkId& landmark_id,
          const vi_map::Vertex& storing_vertex)>& action) const;

 private:
  struct VertexCommonLandmarksCountComparator {
    bool operator()(
        const VertexCommonLandmarksCount& i,
        const VertexCommonLandmarksCount& j) const {
      return i.in_common > j.in_common;
    }
  };

  const vi_map::VIMap& map_;
};

}  // namespace vi_map_helpers

#include "vi-map-helpers/vi-map-queries-inl.h"

#endif  // VI_MAP_HELPERS_VI_MAP_QUERIES_H_
