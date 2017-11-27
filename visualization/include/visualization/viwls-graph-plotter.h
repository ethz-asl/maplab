#ifndef VISUALIZATION_VIWLS_GRAPH_PLOTTER_H_
#define VISUALIZATION_VIWLS_GRAPH_PLOTTER_H_

#include <string>
#include <vector>

#include <Eigen/Core>
#include <aslam/common/pose-types.h>
#include <vi-map-helpers/near-camera-pose-sampling.h>
#include <vi-map/mission-baseframe.h>
#include <vi-map/unique-id.h>
#include <vi-map/vi-map.h>

#include "visualization/common-rviz-visualization.h"
#include "visualization/viz-primitives.h"

namespace visualization {

class ViwlsGraphRvizPlotter {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MAPLAB_POINTER_TYPEDEFS(ViwlsGraphRvizPlotter);

  ViwlsGraphRvizPlotter();

  void visualizeMap(
      const vi_map::VIMap& map, bool publish_baseframes, bool publish_vertices,
      bool publish_edges, bool publish_landmarks) const;
  void visualizeMap(const vi_map::VIMap& map) const;

  void visualizeMissions(
      const vi_map::VIMap& map, const vi_map::MissionIdList& mission_ids,
      bool publish_baseframes, bool publish_vertices, bool publish_edges,
      bool publish_landmarks) const;

  void plotSlidingWindowLocalizationResult(
      const aslam::Transformation& T_G_B, size_t marker_id) const;

  void plotPartitioning(
      const vi_map::VIMap& map,
      const std::vector<pose_graph::VertexIdList>& partitioning) const;

  void plotPartitioning(
      const vi_map::VIMap& map,
      const std::vector<pose_graph::VertexIdList>& partitioning,
      const std::vector<visualization::Color>& colors) const;

  void plotVisualFramePartitioning(
      const vi_map::VIMap& map,
      const vi_map::FrameIdToFrameIdentifierMap& frame_to_frame_identifier_map,
      const std::vector<aslam::FrameIdList>& partitioning) const;

  void publishTF(
      aslam::Transformation T_G_I, const std::string& frame_id,
      const std::string& child_frame_id) const;
  void publishVertexPoseAsTF(
      const vi_map::VIMap& map, pose_graph::VertexId vertex_id) const;
  void publishEdges(
      const vi_map::VIMap& map, const vi_map::MissionIdList& missions) const;
  void publishEdges(
      const vi_map::VIMap& map, const vi_map::MissionIdList& missions,
      pose_graph::Edge::EdgeType edge_type,
      const visualization::Color& color) const;
  void publishEdges(
      const vi_map::VIMap& map, const pose_graph::EdgeIdList& edges,
      const visualization::Color& color, unsigned int marker_id,
      const std::string& topic_extension) const;
  void publishVertices(
      const vi_map::VIMap& map, const vi_map::MissionIdList& missions) const;
  void publishVertices(
      const vi_map::VIMap& map, const pose_graph::VertexIdList& vertices) const;
  void publishBaseFrames(
      const vi_map::VIMap& map, const vi_map::MissionIdList& missions) const;
  void publishBaseFrames(
      const vi_map::VIMap& map,
      const vi_map::MissionBaseFrameIdList& baseframes) const;
  void publishLandmarks(
      const vi_map::VIMap& map, const vi_map::MissionIdList& missions) const;
  void publishLandmarks(const Eigen::Matrix3Xd& W_landmarks) const;
  // If the topic is empty uses the default landmark topic instead.
  void publishLandmarks(
      const Eigen::Matrix3Xd& W_landmarks, const visualization::Color& color,
      const std::string& topic) const;
  void appendLandmarksToSphereVector(
      const vi_map::VIMap& map, const vi_map::MissionIdList& missions,
      visualization::SphereVector* spheres) const;
  void appendLandmarksToSphereVector(
      const vi_map::VIMap& map,
      const pose_graph::VertexIdList& storing_vertices,
      const visualization::Color& color,
      visualization::SphereVector* spheres) const;

  void publishPosesInGlobalFrame(
      const aslam::TransformationVector& transformations) const;

  void publishStructureMatches(
      const Eigen::Vector3d& G_vertex_position,
      const Eigen::Matrix3Xd& G_landmarks) const;

  void publishCamPredictions(
      const vi_map_helpers::NearCameraPoseSampling& sampling,
      const std::vector<double>& predictions);

  void visualizeNCameraExtrinsics(
      const vi_map::VIMap& map, const vi_map::MissionId& mission_id) const;

  void visualizeAllOptionalSensorsExtrinsics(const vi_map::VIMap& map);

  void publishReferenceMap() const;
  void setReferenceMap(const vi_map::VIMap& map);

  static const std::string kCamPredictionTopic;
  static const std::string kEdgeTopic;
  static const std::string kBoundingBoxTopic;
  static const std::string kBaseframeTopic;
  static const std::string kVertexTopic;
  static const std::string kVertexPartitioningTopic;
  static const std::string kBoxTopic;
  static const std::string kLoopclosureTopic;
  static const std::string kLandmarkPairsTopic;
  static const std::string kLandmarkTopic;
  static const std::string kMeshTopic;
  static const std::string kLandmarkNormalsTopic;
  static const std::string kSlidingWindowLocalizationResultTopic;
  static const std::string kUniqueKeyFramesTopic;
  static const std::string kNcamExtrinsicsTopic;
  static const std::string kSensorExtrinsicsTopic;

 private:
  visualization::LineSegmentVector reference_edges_line_segments_;
  Eigen::Vector3d origin_;
};

}  // namespace visualization

#endif  // VISUALIZATION_VIWLS_GRAPH_PLOTTER_H_
