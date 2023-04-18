#include <aslam/geometric-vision/match-outlier-rejection-twopt.h>
#include <aslam/matcher/match.h>
#include <aslam/visualization/basic-visualization.h>
#include <visualization/common-rviz-visualization.h>

#include "feature-tracking/vo-outlier-rejection-pipeline.h"

namespace feature_tracking {

VOOutlierRejectionPipeline::VOOutlierRejectionPipeline(
    const aslam::Camera::ConstPtr& camera, const int cam_idx,
    const aslam::Quaternion& q_C_B, const vi_map::FeatureType feature_type,
    const FeatureTrackingOutlierSettings& outlier_settings)
    : camera_(camera),
      cam_idx_(cam_idx),
      q_C_B_(q_C_B),
      feature_type_(static_cast<int>(feature_type)),
      feature_type_string_(vi_map::FeatureTypeToString(feature_type)),
      outlier_settings_(outlier_settings),
      initialized_(false) {}

VOOutlierRejectionPipeline::~VOOutlierRejectionPipeline() {}

void VOOutlierRejectionPipeline::rejectMatchesFrame(
    const aslam::Quaternion& q_Bkp1_Bk, aslam::VisualFrame* frame_kp1,
    aslam::VisualFrame* frame_k) {
  CHECK_NOTNULL(frame_kp1);
  CHECK_NOTNULL(frame_k);
  CHECK_GT(
      frame_kp1->getTimestampNanoseconds(), frame_k->getTimestampNanoseconds());
  CHECK(camera_.get() == frame_kp1->getCameraGeometry().get());

  if (!initialized_) {
    initialized_ = true;
  } else if (frame_k->getTimestampNanoseconds() != k_frame_timestamp_) {
    LOG(WARNING) << "Frames to the outlier rejection pipeline have to be "
                 << "sequential! Resetting outlier rejection pipeline!";
    reset();
  }

  // Maintaining a consistent locking order (i.e. temporal) is very important
  // to avoid potential deadlocking with other trackers running in parallel
  frame_k->lock();
  frame_kp1->lock();

  if (visualize_keypoint_detections_) {
    cv::Mat image;
    aslam_cv_visualization::drawKeypoints(*frame_kp1, &image, feature_type_);
    const std::string topic = feature_tracking_ros_base_topic_ +
                              "/keypoints_raw_cam" + std::to_string(cam_idx_) +
                              "_" + feature_type_string_;
    visualization::RVizVisualizationSink::publish(topic, image);
  }

  CHECK(frame_k->getNumKeypointMeasurementsOfType(feature_type_));
  CHECK(frame_k->hasDescriptors());
  CHECK(frame_k->hasDescriptorType(feature_type_));
  CHECK(frame_kp1->getNumKeypointMeasurementsOfType(feature_type_));
  CHECK(frame_kp1->hasDescriptors());
  CHECK(frame_kp1->hasDescriptorType(feature_type_));

  // Get the relative motion of the camera using the extrinsics of the camera.
  aslam::Quaternion q_Ckp1_Ck = q_C_B_ * q_Bkp1_Bk * q_C_B_.inverse();

  // We have to reconstruct the matches from the track ids
  aslam::FrameToFrameMatches matches_kp1_k;
  Eigen::VectorBlock<Eigen::VectorXi> track_ids_k =
      frame_k->getTrackIdsOfTypeMutable(feature_type_);
  Eigen::VectorBlock<Eigen::VectorXi> track_ids_kp1 =
      frame_kp1->getTrackIdsOfTypeMutable(feature_type_);

  std::unordered_map<int, int> track_id_to_idx_k;
  for (int i = 0; i < track_ids_k.size(); i++) {
    if (track_ids_k(i) != -1) {
      CHECK(track_id_to_idx_k.emplace(track_ids_k(i), i).second)
          << "Can't have two keypoints in one frame with the same track id";
    }
  }

  std::unordered_set<int> kp1_outlier_track_ids;
  for (int i = 0; i < track_ids_kp1.size(); i++) {
    if (track_ids_kp1(i) != -1) {
      // If the track was previously an outlier do not keep it
      // and mark it for removal also in the next frame
      if (k_outlier_track_ids.count(track_ids_kp1(i))) {
        kp1_outlier_track_ids.emplace(track_ids_kp1(i));
        // Keep the detection, but remove the track id
        track_ids_kp1(i) = -1;
        continue;
      }

      auto match = track_id_to_idx_k.find(track_ids_kp1(i));
      if (match != track_id_to_idx_k.end()) {
        matches_kp1_k.emplace_back(i, match->second);
      }
    }
  }

  VLOG_IF(10, kp1_outlier_track_ids.size() > 0)
      << "Removed " << kp1_outlier_track_ids.size() << " matches on camera "
      << cam_idx_ << " with feature type " << feature_type_string_
      << " because the tracks were previously disconnected by an outlier.";

  // Find the inlier and outlier matches based on the rotation.
  aslam::FrameToFrameMatches inlier_matches_kp1_k;
  aslam::FrameToFrameMatches outlier_matches_kp1_k;

  bool ransac_success = aslam::geometric_vision::
      rejectOutlierFeatureMatchesTranslationRotationSAC(
          *frame_kp1, *frame_k, q_Ckp1_Ck, feature_type_, matches_kp1_k,
          outlier_settings_.deterministic,
          outlier_settings_.two_pt_ransac_threshold,
          outlier_settings_.two_pt_ransac_max_iterations, &inlier_matches_kp1_k,
          &outlier_matches_kp1_k);

  LOG_IF(WARNING, !ransac_success)
      << "Match outlier rejection RANSAC failed on camera " << cam_idx_
      << " with feature type " << feature_type_string_ << ".";
  const size_t num_outliers = outlier_matches_kp1_k.size();
  VLOG_IF(5, num_outliers > 0)
      << "Removed " << num_outliers << " outliers of " << matches_kp1_k.size()
      << " matches on camera " << cam_idx_ << " with feature type "
      << feature_type_string_ << ".";

  // Remove the detected outlier tracks from the frame
  for (const auto& outlier_match_kp1_k : outlier_matches_kp1_k) {
    const int idx_kp1 = outlier_match_kp1_k.first;
    const int outlier_track_id = track_ids_kp1(idx_kp1);

    kp1_outlier_track_ids.emplace(outlier_track_id);
    track_ids_kp1(idx_kp1) = -1;

    // If the track id was not valid it means the track length is now one,
    // so we can remove the track completely by deleting it also from the
    // previous frame.
    if (!k_valid_track_ids_.count(outlier_track_id)) {
      const int idx_k = outlier_match_kp1_k.second;
      track_ids_k(idx_k) = -1;
    }
  }

  if (visualize_keypoint_matches_) {
    cv::Mat inlier_image;
    aslam_cv_visualization::drawKeypointMatches(
        *frame_kp1, *frame_k, inlier_matches_kp1_k, feature_type_,
        &inlier_image);
    const std::string topic =
        feature_tracking_ros_base_topic_ + "/keypoint_matches_camera_" +
        std::to_string(cam_idx_) + "_" + feature_type_string_;
    visualization::RVizVisualizationSink::publish(topic, inlier_image);

    cv::Mat outlier_image;
    aslam_cv_visualization::drawKeypointMatches(
        *frame_kp1, *frame_k, outlier_matches_kp1_k, feature_type_,
        &outlier_image);
    const std::string outlier_topic =
        feature_tracking_ros_base_topic_ + "/keypoint_outliers_camera_" +
        std::to_string(cam_idx_) + "_" + feature_type_string_;
    visualization::RVizVisualizationSink::publish(outlier_topic, outlier_image);
  }

  // Next frame becomes the previous frame for the next iteration, so we
  // renew timestamp and outlier / inlier information
  k_frame_timestamp_ = frame_kp1->getTimestampNanoseconds();

  k_valid_track_ids_.clear();
  for (const auto& inlier_match_kp1_k : inlier_matches_kp1_k) {
    const int idx_kp1 = inlier_match_kp1_k.first;
    k_valid_track_ids_.emplace(track_ids_kp1(idx_kp1));
  }

  k_outlier_track_ids.swap(kp1_outlier_track_ids);

  frame_kp1->unlock();
  frame_k->unlock();
}

void VOOutlierRejectionPipeline::reset() {
  k_outlier_track_ids.clear();
  k_valid_track_ids_.clear();
}

}  // namespace feature_tracking
