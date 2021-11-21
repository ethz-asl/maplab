#ifndef VI_MAP_HELPERS_VI_MAP_MANIPULATION_H_
#define VI_MAP_HELPERS_VI_MAP_MANIPULATION_H_

#include <stddef.h>

#include <unordered_map>

#include <posegraph/unique-id.h>

#include "vi-map-helpers/vi-map-geometry.h"
#include "vi-map-helpers/vi-map-queries.h"

namespace vi_map {
class VIMap;
}  // namespace vi_map

namespace vi_map_helpers {

class VIMapManipulation {
 public:
  typedef std::unordered_map<int, vi_map::LandmarkId> TrackIndexToLandmarkIdMap;
  typedef std::unordered_map<int, vi_map::SemanticLandmarkId>
      TrackIndexToSemanticLandmarkIdMap;
  explicit VIMapManipulation(vi_map::VIMap* map);

  // =====================
  // GLOBAL MAP OPERATIONS
  // =====================
  void rotate(const size_t dimension, const double degrees);
  void artificiallyDisturbVertices();

  // =============
  // MISSION ID IN
  // =============
  // Rotate the mission until variance in z is minimized.
  // Among all the possible rotations that achieve this, we pick the one which
  // conserves the projection of bv_G_R_A into the xy plane. bv_G_R_A is the
  // bearing vector from the root vertex to the average vertex position.
  void alignToXYPlane(const vi_map::MissionId& mission_id);

  // Get all the Viwls edges that do not contain IMU measurements.
  // Edges are returned in graph traversal order.
  void getViwlsEdgesWithoutImuMeasurements(
      const vi_map::MissionId& mission_id,
      pose_graph::EdgeIdList* corrupt_edge_ids) const;

  // Assuming consecutive edges cannot be corrupt.
  void fixViwlsEdgesWithoutImuMeasurements(const vi_map::MissionId& mission_id);

  // Merges landmarks based on the track IDs stored in the frames. Please note
  // that not all of the odometry pipelines provide these IDs.
  size_t mergeLandmarksBasedOnTrackIds(const vi_map::MissionId& mission_id);

  // ============
  // VERTEX ID IN
  // ============
  // Currently this assumes that the concerned vertices don't have landmark
  // observations. Come the use case, this should be easy to generalize.
  void removePosegraphAfter(
      const pose_graph::VertexId& vertex_id,
      pose_graph::VertexIdList* removed_vertex_ids);

  // =============
  // VERTEX IDS IN
  // =============
  // Currently this assumes that the concerned vertices don't have landmark
  // observations. Come the use case, this should be easy to generalize.
  void removeVerticesAndIncomingEdges(
      const pose_graph::VertexIdList& vertex_ids);

  // For each track id in each vertex, will either add a new landmark or a
  // backlink to an existing landmark, if the landmark id can be looked
  // up. Since it's typically expected that landmarks are stored at the first
  // observation, you should pass in the vertices in order.
  size_t initializeLandmarksFromUnusedFeatureTracksOfMission(
      const vi_map::MissionId& mission_id,
      vi_map::LandmarkIdList* initialized_landmark_ids = nullptr);
  size_t initializeLandmarksFromUnusedFeatureTracksOfMission(
      const vi_map::MissionId& mission_id,
      const pose_graph::VertexId& starting_vertex_id,
      vi_map::LandmarkIdList* initialized_landmark_ids = nullptr);
  void initializeLandmarksFromUnusedFeatureTracksOfOrderedVertices(
      const pose_graph::VertexIdList& ordered_vertex_ids,
      TrackIndexToLandmarkIdMap* trackid_landmarkid_map);
  void initializeLandmarksFromUnusedFeatureTracksOfVertex(
      const pose_graph::VertexId& vertex_id,
      TrackIndexToLandmarkIdMap* trackid_landmarkid_map);

  // For each track id in each vertex, will either add a new semantic landmark
  // or a backlink to an existing semantic landmark, if the semantic landmark
  // id can be looked up. Since it's typically expected that semantic landmarks
  // are stored at the first observation, you should pass in the vertices
  // in order.
  size_t initializeSemanticLandmarksFromUnusedFeatureTracksOfMission(
      const vi_map::MissionId& mission_id);
  void initializeSemanticLandmarksFromUnusedFeatureTracksOfOrderedVertices(
      const pose_graph::VertexIdList& ordered_vertex_ids,
      TrackIndexToSemanticLandmarkIdMap* trackid_semanticlandmarkid_map);
  void initializeSemanticLandmarksFromUnusedFeatureTracksOfVertex(
      const pose_graph::VertexId& vertex_id,
      TrackIndexToSemanticLandmarkIdMap* trackid_semanticlandmarkid_map);

  // Releases image from the vertices that are older than
  // image_removal_age_threshold frames. Used by online odometry pipelines.
  // The method only releases the images stored in Visual Frames. Images
  // stored as resources are kept.
  void releaseOldVisualFrameImages(
      const pose_graph::VertexId& current_vertex_id,
      const int image_removal_age_threshold);

  // Remove all landmarks from the map that are flagged as bad.
  size_t removeBadLandmarks();

  // This function will remove all vertices and data before the new root vertex.
  // The intended use case is to seriallize the current map state and then drop
  // everything except the latest vertex.
  // NOTE: currently only tested when calling it with the last vertex of the
  // mission as new root vertex.
  void dropMapDataBeforeVertex(
      const vi_map::MissionId& mission_id,
      const pose_graph::VertexId& new_root_vertex,
      const bool delete_resources_from_file_system);

  // Adds TransformationEdges of type kOdometry between vertices
  // that have fewer common landmarks than 'min_number_of_common_landmarks'. If
  // the threshold is set to 0, we add odometry edges between all vertices.
  uint32_t addOdometryEdgesBetweenVertices(
      const uint32_t min_number_of_common_landmarks = 0u);

  // If a (sub-)map is not moving at all, landmarks sometimes fail to constrain
  // the position, which leads to the vertices drifting away or rotating on the
  // spot, even when constrained with 6DoF odometry edges. This measure can be
  // justified by the fact that most laser and vision based odometry can perform
  // a drift-free re-localization against local geometry/features if the robot
  // does not move.
  bool constrainStationarySubmapWithLoopClosureEdge(
      const double max_translation_m, const double max_rotation_m);

 private:
  vi_map::VIMap& map_;
  VIMapGeometry geometry_;
  VIMapQueries queries_;
};

}  // namespace vi_map_helpers

#endif  // VI_MAP_HELPERS_VI_MAP_MANIPULATION_H_
