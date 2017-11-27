#include "dense-reconstruction/pmvs-common.h"

#include <map>
#include <stdlib.h>

#include <Eigen/Core>
#include <aslam/cameras/camera-factory.h>
#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/camera-unified-projection.h>
#include <aslam/cameras/camera.h>
#include <aslam/common/timer.h>
#include <aslam/common/undistort-helpers.h>
#include <aslam/frames/visual-frame.h>
#include <aslam/frames/visual-nframe.h>
#include <glog/logging.h>
#include <landmark-triangulation/pose-interpolator.h>
#include <maplab-common/accessors.h>
#include <maplab-common/file-system-tools.h>
#include <maplab-common/vector-window-operations.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <vi-map/unique-id.h>

namespace dense_reconstruction {
ObserverCamera::ObserverCamera(
    const vi_map::VIMap& vi_map, const PmvsConfig& config,
    const vi_map::MissionId& _mission_id, const aslam::CameraId& _camera_id,
    const unsigned int _frame_idx, const bool _is_optional_camera)
    : mission_id(_mission_id),
      camera_id(_camera_id),
      frame_idx(_frame_idx),
      is_optional_camera(_is_optional_camera) {
  VLOG(2) << "Create ObserverCamera from camera " << camera_id;
  const aslam::Camera* camera = nullptr;
  if (is_optional_camera) {
    const backend::CameraWithExtrinsics& cam_with_extrinsics =
        vi_map.getMission(mission_id)
            .getOptionalCameraWithExtrinsics(camera_id);
    camera = cam_with_extrinsics.second.get();
    T_B_C = cam_with_extrinsics.first.inverse();
    VLOG(2) << "  -> is optional camera.";
  } else {
    const aslam::NCamera& n_camera =
        vi_map.getSensorManager().getNCameraForMission(mission_id);
    camera = &(n_camera.getCamera(frame_idx));
    T_B_C = n_camera.get_T_C_B(frame_idx).inverse();
    VLOG(2) << "  -> is nframe camera.";
  }

  const aslam::Camera::Type camera_type = camera->getType();
  aslam::Camera::Ptr original_camera;
  switch (camera_type) {
    case aslam::Camera::Type::kPinhole:
      VLOG(2) << "  -> is pinhole camera.";
      if (camera->getDistortion().getType() !=
          aslam::Distortion::Type::kNoDistortion) {
        undistorter = aslam::createMappedUndistorter(
            *camera, config.pmvs_undistortion_alpha,
            config.pmvs_undistortion_scale,
            config.kUndistortionInterpolationMethod);

        camera_matrix_undistorted = static_cast<const aslam::PinholeCamera&>(
                                        undistorter->getOutputCamera())
                                        .getCameraMatrix();

        VLOG(2)
            << "  -> is distorted, initializing pinhole mapped undistorter!";
        CHECK(undistorter != nullptr);
      } else {
        camera_matrix_undistorted =
            static_cast<const aslam::PinholeCamera*>(camera)->getCameraMatrix();
        VLOG(2) << "  -> is already undistorted!";
      }
      break;
    case aslam::Camera::Type::kUnifiedProjection:
      VLOG(2) << "  -> is unified projection camera.";
      if (camera->getDistortion().getType() !=
          aslam::Distortion::Type::kNoDistortion) {
        undistorter = aslam::createMappedUndistorterToPinhole(
            *static_cast<const aslam::UnifiedProjectionCamera*>(camera),
            config.pmvs_undistortion_alpha, config.pmvs_undistortion_scale,
            config.kUndistortionInterpolationMethod);

        camera_matrix_undistorted =
            static_cast<const aslam::UnifiedProjectionCamera&>(
                undistorter->getOutputCamera())
                .getCameraMatrix();
        VLOG(2) << "  -> is distorted, initializing unified projection mapped "
                   "undistorter!";
        CHECK(undistorter != nullptr);
      } else {
        camera_matrix_undistorted =
            static_cast<const aslam::UnifiedProjectionCamera*>(camera)
                ->getCameraMatrix();
        VLOG(2) << "  -> is already undistorted!";
      }

      break;
    default:
      LOG(FATAL) << "Unsupported camera type: " << camera_type;
  }
}

ObserverCamera::ObserverCamera(
    const vi_map::VIMap& vi_map, const PmvsConfig& config,
    const vi_map::MissionId& _mission_id, const aslam::CameraId& _camera_id)
    : ObserverCamera(vi_map, config, _mission_id, _camera_id, 0u, true) {}

void ObserverCamera::undistortImage(
    const cv::Mat& image, cv::Mat* undistorted_image) const {
  CHECK(!image.empty());
  CHECK_NOTNULL(undistorted_image);
  CHECK(undistorter != nullptr);
  undistorter->processImage(image, undistorted_image);
}

ObserverPose::ObserverPose(
    const vi_map::VIMap& vi_map, const size_t _camera_number,
    const vi_map::MissionId& _mission_id,
    const pose_graph::VertexId& _vertex_id, const int64_t _timestamp_ns,
    const ObserverCamera& observer_camera,
    const backend::ResourceType& _image_type)
    : camera_number(_camera_number),
      mission_id(_mission_id),
      vertex_id(_vertex_id),
      timestamp_ns(_timestamp_ns),
      camera_id(observer_camera.camera_id),
      is_optional_camera_image(observer_camera.is_optional_camera),
      image_type(_image_type),
      frame_idx(observer_camera.frame_idx) {
  aslam::Transformation T_M_I;
  if (needsInterpolation(vi_map)) {
    Eigen::Matrix<int64_t, 1, Eigen::Dynamic> pose_timestamps;
    pose_timestamps.resize(1);
    pose_timestamps(0, 0) = timestamp_ns;
    aslam::TransformationVector poses_M_I;
    landmark_triangulation::PoseInterpolator pose_interpolator;
    pose_interpolator.getPosesAtTime(
        vi_map, mission_id, pose_timestamps, &poses_M_I);
    T_M_I = poses_M_I.at(0u);
  } else {
    T_M_I = vi_map.getVertex(vertex_id).get_T_M_I();
  }

  const aslam::Transformation& T_G_M =
      vi_map.getMissionBaseFrameForMission(mission_id).get_T_G_M();

  T_G_C = T_G_M * T_M_I * observer_camera.T_B_C;
  p_G = T_G_C.getPosition();

  const Eigen::Matrix<double, 3, 4> T_GC =
      T_G_C.inverse().getTransformationMatrix().template topLeftCorner<3, 4>();

  P_undistorted = observer_camera.camera_matrix_undistorted * T_GC;
}

void ObserverPose::loadImage(
    const vi_map::VIMap& vi_map, cv::Mat* image) const {
  CHECK_NOTNULL(image);
  if (is_optional_camera_image) {
    CHECK(
        vi_map.getOptionalCameraResource(
            vi_map.getMission(mission_id), image_type, camera_id, timestamp_ns,
            image));
  } else {
    const vi_map::Vertex& vertex = vi_map.getVertex(vertex_id);
    CHECK(vi_map.getFrameResource(vertex, frame_idx, image_type, image));
  }
}

bool ObserverPose::needsUndistortion() const {
  return (image_type == backend::ResourceType::kRawImage) ||
         (image_type == backend::ResourceType::kRawColorImage);
}

bool ObserverPose::needsInterpolation(const vi_map::VIMap& vi_map) const {
  return vi_map.getVertex(vertex_id).getMinTimestampNanoseconds() !=
         timestamp_ns;
}

}  // namespace dense_reconstruction
