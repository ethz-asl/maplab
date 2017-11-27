#include "dense-reconstruction/stereo-pair-detection.h"

#include <sstream>
#include <unordered_map>

#include <aslam/cameras/camera.h>
#include <map-resources/resource-common.h>
#include <vi-map/sensor-manager.h>
#include <vi-map/vi-map.h>

namespace dense_reconstruction {
namespace stereo {

DEFINE_double(dense_stereo_max_angle_between_opt_axes, 0.785, "");
DEFINE_double(dense_stereo_min_baseline, 0.02, "");

// Metrics and indicator to determine if two cameras are suitable for planar
// rectification.
struct StereoPairMetrics {
  double baseline_absolute;
  double baseline_x_offset;

  bool first_camera_is_left;

  Eigen::Vector2d epipole_first_image;
  Eigen::Vector2d epipole_second_image;

  bool first_camera_is_in_front;
  bool second_camera_is_in_front;

  bool epipole_inside_first_image;
  bool epipole_inside_second_image;

  double angle_between_optical_axis;
};

bool isStereoCamera(const StereoPairMetrics& stereo_metrics) {
  // TODO(mfehr): Port the rectification cost function of
  // mono_dense_reconstruction here.

  const bool epipoles_in_img =
      (stereo_metrics.epipole_inside_first_image ||
       stereo_metrics.epipole_inside_second_image);

  const bool viewing_angle_too_different =
      stereo_metrics.angle_between_optical_axis >
      FLAGS_dense_stereo_max_angle_between_opt_axes;

  const bool minimal_baseline_ok =
      stereo_metrics.baseline_absolute > FLAGS_dense_stereo_min_baseline;

  const bool is_stereo_camera = !epipoles_in_img && minimal_baseline_ok &&
                                !viewing_angle_too_different &&
                                stereo_metrics.first_camera_is_left;

  VLOG(2) << "\nStereo pair evaluation:\n"
          << "\tEpipole in image: " << epipoles_in_img << " -> "
          << (epipoles_in_img ? "rejected" : "ok") << "\n"
          << "\tDelta viewing angle: "
          << stereo_metrics.angle_between_optical_axis << " -> "
          << (viewing_angle_too_different ? "rejected" : "ok") << "\n"
          << "\tBaseline: " << stereo_metrics.baseline_absolute << "m -> "
          << (!minimal_baseline_ok ? "rejected" : "ok") << "\n"
          << "\tFirst camera is left / x offset: "
          << stereo_metrics.first_camera_is_left << "/"
          << stereo_metrics.baseline_x_offset << " -> "
          << (!stereo_metrics.first_camera_is_left ? "rejected" : "ok") << "\n"
          << "\t===> "
          << (is_stereo_camera ? "Is stereo camera!" : "Is no stereo camera!");

  return is_stereo_camera;
}

void computeStereoPairMetrics(
    const aslam::Camera& first_camera, const aslam::Camera& second_camera,
    const aslam::Transformation T_C2_C1, StereoPairMetrics* stereo_metrics) {
  CHECK_NOTNULL(stereo_metrics);

  const Eigen::Matrix4d T_C2_C1_mat = T_C2_C1.getTransformationMatrix();
  const Eigen::Matrix4d T_C1_C2_mat =
      T_C2_C1.inverse().getTransformationMatrix();

  // Camera origin expressed in the other cameras coordinate frame.
  const Eigen::Vector3d p2_C1 = T_C2_C1_mat.block<3, 1>(0, 3);
  const Eigen::Vector3d p1_C2 = T_C1_C2_mat.block<3, 1>(0, 3);

  // Compute absolute distance between the two camera origins.
  stereo_metrics->baseline_absolute = p1_C2.norm();
  CHECK_LT(p1_C2.norm() - p2_C1.norm(), 1e-6);

  // The x-component of the baseline expressed in the C1 frame gives us
  // information about which camera is the one on the left.
  stereo_metrics->baseline_x_offset = T_C2_C1_mat(0, 3);
  stereo_metrics->first_camera_is_left =
      (stereo_metrics->baseline_x_offset < 0.0) &&
      (std::abs(stereo_metrics->baseline_x_offset) > 1e-6);

  // Project the camera origin of C1 into C2 and vice versa.
  aslam::ProjectionResult result_second_image =
      second_camera.project3(p1_C2, &(stereo_metrics->epipole_second_image));
  aslam::ProjectionResult result_first_image =
      first_camera.project3(p2_C1, &(stereo_metrics->epipole_first_image));

  // NOTE(mfehr): The epipole could also be inside an image even if the camera
  // origin of the other camera is behind the image plane. Worst case both
  // cameras are facing away from each other and both of these checks would not
  // pick it up, however we hope the other checks should catch such an extreme
  // condition.
  stereo_metrics->epipole_inside_first_image =
      (result_first_image == aslam::ProjectionResult::KEYPOINT_VISIBLE);
  stereo_metrics->epipole_inside_second_image =
      (result_second_image == aslam::ProjectionResult::KEYPOINT_VISIBLE);

  stereo_metrics->first_camera_is_in_front =
      stereo_metrics->epipole_inside_first_image ||
      (result_first_image ==
       aslam::ProjectionResult::KEYPOINT_OUTSIDE_IMAGE_BOX);
  stereo_metrics->second_camera_is_in_front =
      stereo_metrics->epipole_inside_second_image ||
      (result_second_image ==
       aslam::ProjectionResult::KEYPOINT_OUTSIDE_IMAGE_BOX);

  stereo_metrics->angle_between_optical_axis = std::acos(
      Eigen::Vector3d(T_C2_C1_mat.block<3, 1>(0, 2))
          .dot(Eigen::Vector3d(0.0, 0.0, 1.0)));
}

void printStereoCamerasPerMission(
    const StereoPairsPerMissionMap& stereo_camera_ids_per_mission,
    const int verbosity) {
  std::stringstream ss;
  ss << "\nFound the following stereo cameras:\n"
     << "+------------+----------------------------------+\n"
     << "| mission id |            camera ids            |\n"
     << "+------------+----------------------------------+\n";
  for (const StereoPairIdsPerMission& pair_per_mission :
       stereo_camera_ids_per_mission) {
    for (const StereoPairIdentifier& stereo_pair_identifier :
         pair_per_mission.second) {
      const vi_map::MissionId& mission_id = pair_per_mission.first;
      const aslam::CameraId& first_camera_id =
          stereo_pair_identifier.first_camera_id;
      const aslam::CameraId& second_camera_id =
          stereo_pair_identifier.second_camera_id;

      ss << "| " << mission_id << " | " << first_camera_id << " |\n"
         << "|            | " << second_camera_id << " |\n"
         << "+------------+----------------------------------+\n";
    }
  }
  VLOG(verbosity) << ss.str();
}

void findAllStereoCameras(
    const vi_map::VIMap& vi_map,
    StereoPairsPerMissionMap* stereo_camera_ids_per_mission) {
  CHECK_NOTNULL(stereo_camera_ids_per_mission);

  vi_map::MissionIdList all_mission_ids;
  vi_map.getAllMissionIds(&all_mission_ids);
  for (const vi_map::MissionId& mission_id : all_mission_ids) {
    const vi_map::SensorManager& sensor_manager = vi_map.getSensorManager();
    const aslam::NCamera& ncamera =
        sensor_manager.getNCameraForMission(mission_id);

    std::unordered_map<size_t, size_t> pairs_found;

    for (size_t first_camera_idx = 0u;
         first_camera_idx < ncamera.getNumCameras(); ++first_camera_idx) {
      const aslam::Camera& first_camera = ncamera.getCamera(first_camera_idx);

      for (size_t second_camera_idx = 0u;
           second_camera_idx < ncamera.getNumCameras(); ++second_camera_idx) {
        if (first_camera_idx == second_camera_idx) {
          continue;
        }

        // Make sure we didn't discover the opposite direciton already.
        if (pairs_found.count(second_camera_idx) > 0u) {
          if (pairs_found[second_camera_idx] == first_camera_idx) {
            continue;
          }
        }

        CHECK_LT(second_camera_idx, ncamera.getNumCameras());
        const aslam::Camera& second_camera =
            ncamera.getCamera(second_camera_idx);

        const aslam::Transformation& T_C1_B =
            ncamera.get_T_C_B(first_camera_idx);
        const aslam::Transformation& T_C2_B =
            ncamera.get_T_C_B(second_camera_idx);
        const aslam::Transformation T_C2_C1 = T_C2_B * T_C1_B.inverse();

        StereoPairMetrics stereo_metrics;
        computeStereoPairMetrics(
            first_camera, second_camera, T_C2_C1, &stereo_metrics);

        if (!isStereoCamera(stereo_metrics)) {
          continue;
        }

        // Write the pair into this map to prevent discovery of another stereo
        // camera with these two cameras.
        pairs_found[first_camera_idx] = second_camera_idx;

        // Store stereo pair.
        StereoPairIdentifier identifier;
        identifier.first_camera_id = first_camera.getId();
        identifier.second_camera_id = second_camera.getId();
        identifier.T_C2_C1 = T_C2_C1;

        (*stereo_camera_ids_per_mission)[mission_id].push_back(identifier);

        // Found a stereo pair with the current first_camera_idx as first
        // camera. This camera should not be the first camera for another
        // pair, because it would overwrite the depth resource of the first
        // reconstruction.
        break;
      }
    }
  }
}

}  // namespace stereo
}  // namespace dense_reconstruction
