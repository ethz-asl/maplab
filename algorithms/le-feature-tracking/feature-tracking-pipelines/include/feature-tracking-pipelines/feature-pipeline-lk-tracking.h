#ifndef FEATURE_TRACKING_PIPELINES_FEATURE_PIPELINE_LK_TRACKING_H_
#define FEATURE_TRACKING_PIPELINES_FEATURE_PIPELINE_LK_TRACKING_H_

#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <aslam/cameras/ncamera.h>
#include <aslam/matcher/match-helpers.h>
#include <glog/logging.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>

#include "feature-tracking-pipelines/feature-describer-base.h"
#include "feature-tracking-pipelines/feature-detector-base.h"
#include "feature-tracking-pipelines/feature-pipeline-base.h"
#include "feature-tracking-pipelines/flags.h"
#include "feature-tracking-pipelines/helpers.h"
#include "feature-tracking-pipelines/keyframe-features.h"
#include "feature-tracking-pipelines/pnp-ransac.h"

namespace feature_tracking_pipelines {

struct LkTrackingSettings {
  size_t max_num_features = FLAGS_NEW_feature_tracker_num_features;
};

LkTrackingSettings InitLkTrackingSettingsFromGFlags() {
  LkTrackingSettings settings;
  CHECK_GT(FLAGS_NEW_feature_tracker_num_features, 0);
  settings.max_num_features = FLAGS_NEW_feature_tracker_num_features;
  return settings;
}

class FeaturePipelineLkTracking : public FeatureTrackingPipelineBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FeaturePipelineLkTracking(
      const LkTrackingSettings& settings, const RansacSettings& ransac_settings,
      std::shared_ptr<FeatureDetectorBase> feature_detector,
      std::shared_ptr<FeatureDescriberBase> feature_describer)
      : FeatureTrackingPipelineBase(
            ransac_settings, feature_detector, feature_describer),
        settings_(settings) {}
  virtual ~FeaturePipelineLkTracking() = default;

  virtual void processImages(
      const aslam::NCamera& ncamera, const aslam::Quaternion& q_Icurr_Iprev,
      const std::vector<cv::Mat>& curr_camera_images,
      const std::vector<cv::Mat>& prev_camera_images,
      const std::vector<KeyframeFeatures>& previous_keyframe,
      std::vector<KeyframeFeatures>* current_keyframe_ptr,
      FeaturePipelineDebugData* optional_debug_data) {
    // optional_debug_data is optional and can be a nullptr.

    CHECK_EQ(ncamera.numCameras(), curr_camera_images.size());
    CHECK_EQ(ncamera.numCameras(), previous_keyframe.size());
    CHECK_NOTNULL(current_keyframe_ptr)->clear();

    std::vector<KeyframeFeatures>& current_keyframe = *current_keyframe_ptr;
    current_keyframe.resize(ncamera.numCameras());

    std::vector<aslam::FrameToFrameMatches> inlier_matches_kp1_k,
        outlier_matches_kp1_k;
    inlier_matches_kp1_k.resize(ncamera.numCameras());
    outlier_matches_kp1_k.resize(ncamera.numCameras());

    // TODO(schneith): Parallelize the loop over camera images.
    for (size_t frame_idx = 0u; frame_idx < ncamera.numCameras(); ++frame_idx) {
      // Copy over the keypoints from the last tracking round.
      CHECK_EQ(previous_keyframe[frame_idx].frame_idx, frame_idx);
      current_keyframe[frame_idx].keypoint_detector_name =
          feature_detector_->getDetectorName();
      current_keyframe[frame_idx].keypoint_descriptor_name =
          feature_describer_->getDescriptorName();

      // Predict feature locations considering a rotation.
      std::vector<cv::Point2f> keypoints_prev;
      KeyframeFeaturesToCvPoints(
          previous_keyframe[frame_idx].keypoint_measurements, &keypoints_prev);

      // Get the relative motion of the camera using the extrinsics of the
      // camera system.
      const aslam::Quaternion& q_C_B =
          ncamera.get_T_C_B(frame_idx).getRotation();
      aslam::Quaternion q_Ckp1_Ck = q_C_B * q_Icurr_Iprev * q_C_B.inverse();

      Eigen::Matrix2Xd predicted_keypoints =
          previous_keyframe[frame_idx].keypoint_measurements;

      /*
      Eigen::Matrix2Xd predicted_keypoints;
      const Eigen::Matrix2Xd &prev_keypoints 
          = previous_keyframe[frame_idx].keypoint_measurements;
      std::vector<unsigned char> prediction_success;
      aslam::predictKeypointsByRotation(
          ncamera.getCamera(frame_idx), prev_keypoints, q_Ckp1_Ck,
          &predicted_keypoints, &prediction_success);
      CHECK_EQ(
          prediction_success.size(), predicted_keypoints.cols());
        */

      std::vector<cv::Point2f> keypoints_curr;
      KeyframeFeaturesToCvPoints(predicted_keypoints, &keypoints_curr);

      // Track features from last to current frame using LK.
      std::vector<float> tracking_error;
      std::vector<unsigned char> tracking_successful;
      if (!keypoints_prev.empty()) {
        // TODO(schneith): Expose options to settings.

        static constexpr double kMinEigenThreshold = 0.001;
        static constexpr size_t kMaxPyramidLevel = 4u;
        static constexpr size_t kOperationFlag = 0;
        const cv::TermCriteria kTerminationCriteria = cv::TermCriteria(
            cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03);
        const cv::Size kWindowSize = cv::Size(21, 21);

        cv::calcOpticalFlowPyrLK(
            prev_camera_images[frame_idx], curr_camera_images[frame_idx],
            keypoints_prev, keypoints_curr, tracking_successful, tracking_error,
            kWindowSize, kMaxPyramidLevel, kTerminationCriteria, kOperationFlag,
            kMinEigenThreshold);

        CHECK_EQ(keypoints_curr.size(), keypoints_prev.size());
        CvKeypointsToKeyframeFeatures(
            keypoints_curr, &current_keyframe[frame_idx].keypoint_measurements);
      }

      // Track IDs remain the same as in the previous frame.
      current_keyframe[frame_idx].keypoint_track_ids =
          previous_keyframe[frame_idx].keypoint_track_ids;

      // TODO(schneith): Re-evaluate the orientation and scale at the new
      // keypoint location. For now we propagate the value from the initial
      // keypoint detection; except for the orientation.
      current_keyframe[frame_idx].keypoint_scales =
          previous_keyframe[frame_idx].keypoint_scales;
      current_keyframe[frame_idx].keypoint_scores =
          previous_keyframe[frame_idx].keypoint_scores;
      current_keyframe[frame_idx].keypoint_orientations_rad.setZero(
          2, current_keyframe[frame_idx].keypoint_measurements.cols());

      // Remove points for which the tracking failed.
      std::vector<size_t> failed_indices;
      GetElementIndicesInVector(
          tracking_successful, /*find_value=*/0u, &failed_indices);

      LOG(WARNING) << "Removing " << failed_indices.size() << " of "
                   << current_keyframe[frame_idx].keypoint_measurements.cols()
                   << " keypoints for which tracking failed.";
      RemoveKeypoints(failed_indices, &current_keyframe[frame_idx]);
      LOG(WARNING) << "REMOVED";

      // Reject outliers using PNP-ransac.
      const bool ransac_success = PerformTemporalFrameToFrameRansac(
          ncamera.getCamera(frame_idx), current_keyframe[frame_idx],
          previous_keyframe[frame_idx], ransac_settings_, q_Icurr_Iprev,
          &inlier_matches_kp1_k[frame_idx], &outlier_matches_kp1_k[frame_idx]);
      LOG_IF(WARNING, !ransac_success)
          << "Ransac failed on camera: " << frame_idx;

      std::vector<size_t> outlier_indices;
      outlier_indices.resize(outlier_matches_kp1_k[frame_idx].size());
      std::transform(
          outlier_matches_kp1_k[frame_idx].begin(),
          outlier_matches_kp1_k[frame_idx].end(), outlier_indices.begin(),
          [](const aslam::FrameToFrameMatch& outlier_match_kp1_k) {
            return outlier_match_kp1_k.getKeypointIndexAppleFrame();
          });

      // TODO(schneith): Returning of outlier tracks does not work anymore.
      // Earlier we set the TrackId to -1 but now we delete it, so the indices
      // change....

      VLOG_IF(1, !outlier_indices.empty())
          << "Removing " << outlier_indices.size()
          << " outlier matches from a total of "
          << current_keyframe[frame_idx].keypoint_measurements.cols()
          << " matches.";
      RemoveKeypoints(outlier_indices, &current_keyframe[frame_idx]);

      // Remove the non-describable features.
      // TODO(schneith): figure out the effect of this.
      std::vector<size_t> non_describable_indices;
      if (feature_describer_->hasNonDescribableFeatures(
              current_keyframe[frame_idx].keypoint_measurements,
              current_keyframe[frame_idx].keypoint_scales,
              &non_describable_indices)) {
        RemoveKeypoints(non_describable_indices, &current_keyframe[frame_idx]);
      }

      // Remove features that have converged, preferring to keep longer tracks
      // (which is equal to a lower track id).
      constexpr double kMinDistance = 3.0;
      cv::Mat occupancy_image(
          curr_camera_images[frame_idx].rows,
          curr_camera_images[frame_idx].cols, CV_8UC1, cv::Scalar(255));

      std::vector<size_t> keypoint_indices_sorted_by_track_id;
      GetKeypointIndicesSortedByTrackId(
          current_keyframe[frame_idx], &keypoint_indices_sorted_by_track_id);

      std::vector<size_t> converged_keypoint_indices;
      for (int kp_idx : keypoint_indices_sorted_by_track_id) {
        CHECK_LT(
            kp_idx, current_keyframe[frame_idx].keypoint_measurements.cols());
        CHECK_GE(current_keyframe[frame_idx].keypoint_track_ids(0, kp_idx), 0);

        cv::Point keypoint(
            current_keyframe[frame_idx].keypoint_measurements(0, kp_idx),
            current_keyframe[frame_idx].keypoint_measurements(1, kp_idx));

        if (occupancy_image.at<uchar>(keypoint) == 255) {
          cv::circle(
              occupancy_image, keypoint, kMinDistance, cv::Scalar(0),
              CV_FILLED);
        } else {
          converged_keypoint_indices.emplace_back(kp_idx);
        }
      }

      VLOG_IF(1, !converged_keypoint_indices.empty())
          << "Removing " << converged_keypoint_indices.size()
          << " converged features "
          << " from a total of "
          << current_keyframe[frame_idx].keypoint_measurements.cols()
          << " features.";
      RemoveKeypoints(converged_keypoint_indices, &current_keyframe[frame_idx]);

      // Create a detection mask to prevent new detections close to features
      // that are already being tracked.
      // TODO(schneith): Expose as settings.
      constexpr size_t kRejectFeatureRadiusPx = 10u;

      cv::Mat detection_mask(
          curr_camera_images[frame_idx].rows,
          curr_camera_images[frame_idx].cols, CV_8UC1, cv::Scalar(255));

      keypoint_indices_sorted_by_track_id.clear();
      GetKeypointIndicesSortedByTrackId(
          current_keyframe[frame_idx], &keypoint_indices_sorted_by_track_id);

      for (int kp_idx : keypoint_indices_sorted_by_track_id) {
        CHECK_LT(
            kp_idx, current_keyframe[frame_idx].keypoint_measurements.cols());
        CHECK_GE(current_keyframe[frame_idx].keypoint_track_ids(0, kp_idx), 0);

        cv::Point keypoint(
            current_keyframe[frame_idx].keypoint_measurements(0, kp_idx),
            current_keyframe[frame_idx].keypoint_measurements(1, kp_idx));

        if (detection_mask.at<uchar>(keypoint) == 255) {
          cv::circle(
              detection_mask, keypoint, kRejectFeatureRadiusPx, cv::Scalar(0),
              CV_FILLED);
        }
      }

      /* This is not threadsafe!
      cv::namedWindow("rejection_mask");
      cv::imshow("rejection_mask", detection_mask);
      cv::waitKey(1);
      */

      // Fill up the keypoint slots using new detections.
      // We delay redetections for performance reasons until the successfully
      // tracked feature count drops under kRedetectThresholdPercent percent of
      // the max feature count.
      constexpr double kRedetectThresholdPercent = 0.6;
      const bool should_redetect =
          (current_keyframe[frame_idx].keypoint_measurements.cols() <
           (kRedetectThresholdPercent * settings_.max_num_features));

      if (should_redetect) {
        const int num_to_detect =
            settings_.max_num_features -
            current_keyframe[frame_idx].keypoint_measurements.cols();
        CHECK_GE(num_to_detect, 0);

        KeyframeFeatures new_detections;
        feature_detector_->detectFeatures(
            curr_camera_images[frame_idx], num_to_detect, detection_mask,
            &new_detections);
        const int num_deteted = new_detections.keypoint_measurements.cols();
        LOG_IF(WARNING, num_deteted < num_to_detect)
            << "Tried to detect " << num_to_detect << " keypoints on camera "
            << frame_idx << " but only got " << num_deteted << ".";

        // Assign track ids to new detections.
        std::pair<size_t, size_t> ids_start_end =
            track_id_provider_.getIds(num_deteted);
        new_detections.keypoint_track_ids.resize(Eigen::NoChange, num_deteted);
        new_detections.keypoint_track_ids.setLinSpaced(
            ids_start_end.first, ids_start_end.second - 1);
        CHECK_EQ(
            new_detections.keypoint_track_ids.cols(),
            new_detections.keypoint_measurements.cols());

        AppendKeypoints(new_detections, &current_keyframe[frame_idx]);
      }

      // Extract the descriptors for all newly detected and also for the new
      // location of the tracked keypoints.
      feature_describer_->describeFeatures(
          curr_camera_images[frame_idx], &current_keyframe[frame_idx]);

      // Assume a constant measurement uncertainty.
      // This needs to done after the feature extraction
      // since the number of keypoints can change in there.
      std::size_t num_keypoints =
          current_keyframe[frame_idx].keypoint_measurements.cols();
      Eigen::VectorXd uncertainties(num_keypoints);
      uncertainties.setConstant(0.8);
      current_keyframe[frame_idx].keypoint_measurement_uncertainties =
          uncertainties;
    }

    if (optional_debug_data != nullptr) {
      optional_debug_data->inlier_matches_kp1_k.swap(inlier_matches_kp1_k);
      optional_debug_data->outlier_matches_kp1_k.swap(outlier_matches_kp1_k);
    }
  }

 private:
  const LkTrackingSettings settings_;

  TrackIdProvider track_id_provider_;
};

}  // namespace feature_tracking_pipelines

#endif  // FEATURE_TRACKING_PIPELINES_FEATURE_PIPELINE_LK_TRACKING_H_
