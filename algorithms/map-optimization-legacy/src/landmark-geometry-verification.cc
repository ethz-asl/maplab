#include <map-optimization-legacy/landmark-geometry-verification.h>
#include <vi-map/vi-mission.h>

#include <maplab-common/accessors.h>
#include <maplab-common/quaternion-math.h>

namespace map_optimization_legacy {

LandmarkGeometryVerification::LandmarkGeometryVerification(
    const vi_map::PoseGraph& posegraph,
    const LandmarkIdToVertexIdMap& landmarks,
    const vi_map::MissionMap& missions,
    const vi_map::MissionBaseFrameMap& mission_base_frames,
    const vi_map::SensorManager& sensor_manager)
    : posegraph_(posegraph),
      landmarks_(landmarks),
      missions_(missions),
      mission_base_frames_(mission_base_frames),
      sensor_manager_(sensor_manager) {}

void LandmarkGeometryVerification::getMergeReprojectionErrors(
    const vi_map::LandmarkId& change_from, const vi_map::LandmarkId& change_to,
    std::vector<double>* reprojection_errors_before,
    std::vector<double>* reprojection_errors_after) {
  CHECK_NOTNULL(reprojection_errors_before);
  CHECK_NOTNULL(reprojection_errors_after);

  getReprojectionErrors(change_from, change_from, reprojection_errors_before);
  getReprojectionErrors(change_from, change_to, reprojection_errors_after);
}

// Calculate reprojection errors if the keypoint_landmark_id is merged
// into map_landmark_id.
void LandmarkGeometryVerification::getReprojectionErrors(
    const vi_map::LandmarkId& keypoint_landmark_id,
    const vi_map::LandmarkId& map_landmark_id,
    std::vector<double>* reprojection_errors) {
  CHECK_NOTNULL(reprojection_errors);

  // Find keypoint landmark.
  const pose_graph::VertexId& keypoint_landmark_store_vertex_id =
      common::getChecked(landmarks_, keypoint_landmark_id);

  const vi_map::Vertex& keypoint_landmark_vertex =
      posegraph_.getVertex(keypoint_landmark_store_vertex_id)
          .getAs<vi_map::Vertex>();

  const vi_map::Landmark& keypoint_landmark =
      keypoint_landmark_vertex.getLandmarks().getLandmark(keypoint_landmark_id);

  // Find map landmark.
  const pose_graph::VertexId& map_landmark_store_vertex_id =
      common::getChecked(landmarks_, map_landmark_id);

  const vi_map::Vertex& map_landmark_vertex =
      posegraph_.getVertex(map_landmark_store_vertex_id)
          .getAs<vi_map::Vertex>();

  const Eigen::Vector3d LM_p_fi =
      map_landmark_vertex.getLandmark_p_LM_fi(map_landmark_id);
  const vi_map::VIMission& landmark_mission =
      common::getChecked(missions_, map_landmark_vertex.getMissionId())
          ->getAs<const vi_map::VIMission>();
  const vi_map::MissionBaseFrame& LM_baseframe = common::getChecked(
      mission_base_frames_, landmark_mission.getBaseFrameId());
  const Eigen::Vector3d G_p_fi =
      LM_baseframe.transformPointInMissionFrameToGlobalFrame(LM_p_fi);

  // Iterate through all vertices that see the map landmark.
  reprojection_errors->resize(keypoint_landmark.numberOfObservations());
  keypoint_landmark.forEachObservation(
      [&](const vi_map::KeypointIdentifier& observation, const size_t i) {
        const vi_map::Vertex* keyframe_vertex =
            dynamic_cast<const vi_map::Vertex*>(  // NOLINT
                posegraph_.getVertexPtr(observation.frame_id.vertex_id));
        CHECK_NOTNULL(keyframe_vertex);

        // Find appropriate mission.
        const vi_map::VIMission& keyframe_mission =
            common::getChecked(missions_, keyframe_vertex->getMissionId())
                ->getAs<vi_map::VIMission>();

        const aslam::NCamera& ncamera =
            sensor_manager_.getNCameraForMission(keyframe_mission.id());

        const pose::Transformation& T_C_I =
            ncamera.get_T_C_B(observation.frame_id.frame_index);

        const vi_map::MissionBaseFrame& keyframe_mission_baseframe =
            common::getChecked(
                mission_base_frames_, keyframe_mission.getBaseFrameId());
        pose::Transformation G_T_M = keyframe_mission_baseframe.get_T_G_M();

        const Eigen::Vector3d M_p_fi = G_T_M.inverse() * G_p_fi;
        pose::Transformation M_T_I = keyframe_vertex->get_T_M_I();

        // Calculate C_p_fi for the keypoint frame.
        const Eigen::Vector3d C_p_fi = T_C_I * M_T_I.inverse() * M_p_fi;

        // Reproject landmark to each frame.
        Eigen::Vector2d reprojected_point;
        keyframe_vertex->getCamera(observation.frame_id.frame_index)
            ->project3(C_p_fi, &reprojected_point);

        // Calculate error when compared to VisualFrame keypoint.
        (*reprojection_errors)[i] =
            (reprojected_point -
             keyframe_vertex->getVisualFrame(observation.frame_id.frame_index)
                 .getKeypointMeasurement(observation.keypoint_index))
                .norm();
      });
}

}  // namespace map_optimization_legacy
