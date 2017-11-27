#ifndef VI_MAP_HELPERS_VI_MAP_MANIPULATION_H_
#define VI_MAP_HELPERS_VI_MAP_MANIPULATION_H_

#include <stddef.h>

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

  explicit VIMapManipulation(vi_map::VIMap* map);

  // =====================
  // GLOBAL MAP OPERATIONS
  // =====================
  void rotate(const size_t dimension, const double degrees);

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
      const vi_map::MissionId& mission_id);
  void initializeLandmarksFromUnusedFeatureTracksOfOrderedVertices(
      const pose_graph::VertexIdList& ordered_vertex_ids,
      TrackIndexToLandmarkIdMap* trackid_landmarkid_map);
  void initializeLandmarksFromUnusedFeatureTracksOfVertex(
      const pose_graph::VertexId& vertex_id,
      TrackIndexToLandmarkIdMap* trackid_landmarkid_map);

  // Releases image from the vertices that are older than
  // image_removal_age_threshold frames. Used by online odometry pipelines.
  // The method only releases the images stored in Visual Frames. Images
  // stored as resources are kept.
  void releaseOldVisualFrameImages(
      const pose_graph::VertexId& current_vertex_id,
      const int image_removal_age_threshold);

  // Remove all landmarks from the map that are flagged as bad.
  size_t removeBadLandmarks();

 private:
  vi_map::VIMap& map_;
  VIMapGeometry geometry_;
  VIMapQueries queries_;
};

}  // namespace vi_map_helpers

#endif  // VI_MAP_HELPERS_VI_MAP_MANIPULATION_H_
