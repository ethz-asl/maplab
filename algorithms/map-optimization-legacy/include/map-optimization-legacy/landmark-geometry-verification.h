#ifndef MAP_OPTIMIZATION_LEGACY_LANDMARK_GEOMETRY_VERIFICATION_H_
#define MAP_OPTIMIZATION_LEGACY_LANDMARK_GEOMETRY_VERIFICATION_H_

#include <memory>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <glog/logging.h>
#include <vi-map/landmark-index.h>
#include <vi-map/landmark.h>
#include <vi-map/pose-graph.h>
#include <vi-map/sensor-manager.h>
#include <vi-map/unique-id.h>
#include <vi-map/vertex.h>

namespace map_optimization_legacy {

typedef std::unordered_map<vi_map::LandmarkId, pose_graph::VertexId>
    LandmarkIdToVertexIdMap;

class LandmarkGeometryVerification final {
 public:
  LandmarkGeometryVerification() = delete;
  LandmarkGeometryVerification(
      const vi_map::PoseGraph& posegraph,
      const LandmarkIdToVertexIdMap& landmarks,
      const vi_map::MissionMap& missions,
      const vi_map::MissionBaseFrameMap& mission_base_frames,
      const vi_map::SensorManager& sensor_manager);

  void getMergeReprojectionErrors(
      const vi_map::LandmarkId& change_from,
      const vi_map::LandmarkId& change_to,
      std::vector<double>* reprojection_errors_before,
      std::vector<double>* reprojection_errors_after);

  void getReprojectionErrors(
      const vi_map::LandmarkId& keypoint_landmark_id,
      const vi_map::LandmarkId& map_landmark_id,
      std::vector<double>* reprojection_errors);

 private:
  const vi_map::PoseGraph& posegraph_;
  const LandmarkIdToVertexIdMap& landmarks_;
  const vi_map::MissionMap& missions_;
  const vi_map::MissionBaseFrameMap& mission_base_frames_;
  const vi_map::SensorManager& sensor_manager_;
};

}  // namespace map_optimization_legacy

#endif  // MAP_OPTIMIZATION_LEGACY_LANDMARK_GEOMETRY_VERIFICATION_H_
