#ifndef DENSE_RECONSTRUCTION_PMVS_COMMON_H_
#define DENSE_RECONSTRUCTION_PMVS_COMMON_H_

#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include <Eigen/Core>
#include <aslam/pipeline/undistorter-mapped.h>
#include <glog/logging.h>
#include <vi-map/unique-id.h>
#include <vi-map/vi-map.h>
#include <vi-map/vi-mission.h>

#include "dense-reconstruction/pmvs-config.h"

namespace dense_reconstruction {

struct ObservedLandmark {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  ObservedLandmark() {}

  vi_map::LandmarkId landmark_id;
  size_t landmark_number;

  AlignedUnorderedSet<size_t> observer_pose_numbers;
  Eigen::Vector3d p_G;
};
typedef AlignedUnorderedMap<vi_map::LandmarkId, ObservedLandmark>
    ObservedLandmarks;

struct ObserverCamera {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  ObserverCamera(
      const vi_map::VIMap& vi_map, const PmvsConfig& config,
      const vi_map::MissionId& _mission_id, const aslam::CameraId& _camera_id,
      const unsigned int _frame_idx, const bool _is_optional_camera);

  ObserverCamera(
      const vi_map::VIMap& vi_map, const PmvsConfig& config,
      const vi_map::MissionId& _mission_id, const aslam::CameraId& _camera_id);

  void undistortImage(const cv::Mat& image, cv::Mat* undistorted_image) const;

  const vi_map::MissionId mission_id;
  const aslam::CameraId camera_id;
  unsigned int frame_idx;
  const bool is_optional_camera;

  Eigen::Matrix3d camera_matrix_undistorted;
  aslam::Transformation T_B_C;

  std::unique_ptr<aslam::MappedUndistorter> undistorter;
};
typedef AlignedUnorderedMap<aslam::CameraId, ObserverCamera> ObserverCameraMap;

struct ObserverPose {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  ObserverPose(
      const vi_map::VIMap& vi_map, const size_t _camera_number,
      const vi_map::MissionId& _mission_id,
      const pose_graph::VertexId& _vertex_id, const int64_t _timestamp_ns,
      const ObserverCamera& observer_camera,
      const backend::ResourceType& _image_type);

  void loadImage(const vi_map::VIMap& vi_map, cv::Mat* image) const;

  bool needsUndistortion() const;

  bool needsInterpolation(const vi_map::VIMap& vi_map) const;

  // Observer identification data
  size_t camera_number;
  vi_map::MissionId mission_id;
  pose_graph::VertexId vertex_id;
  int64_t timestamp_ns;
  aslam::CameraId camera_id;
  bool is_optional_camera_image;
  backend::ResourceType image_type;
  unsigned int frame_idx;

  // Observer position-specific data.
  aslam::Transformation T_G_C;
  Eigen::Vector3d p_G;
  Eigen::Matrix<double, 3, 4> P_undistorted;

  struct Hash {
    size_t operator()(const ObserverPose& observer_pose) const {
      return observer_pose.camera_number;
    }
  };

  struct Pred {
    bool operator()(
        const ObserverPose& observer_pose_A,
        const ObserverPose& observer_pose_B) const {
      return observer_pose_A.camera_number == observer_pose_B.camera_number;
    }
  };
};

typedef std::unordered_set<ObserverPose, ObserverPose::Hash, ObserverPose::Pred,
                           Eigen::aligned_allocator<ObserverPose>>
    ObserverPoseSet;

typedef AlignedUnorderedMap<pose_graph::VertexId, ObserverPoseSet>
    ObserverPosesMap;

}  // namespace dense_reconstruction

#endif  // DENSE_RECONSTRUCTION_PMVS_COMMON_H_
