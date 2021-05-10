#ifndef FEATURE_TRACKING_PIPELINES_FEATURE_PIPELINE_MATCHING_BASED_H_
#define FEATURE_TRACKING_PIPELINES_FEATURE_PIPELINE_MATCHING_BASED_H_

#include <string>
#include <vector>

#include <Eigen/Core>
#include <aslam/cameras/ncamera.h>
#include <glog/logging.h>
#include <opencv2/core/core.hpp>

#include "feature-tracking-pipelines/feature-describer-base.h"
#include "feature-tracking-pipelines/feature-detector-base.h"
#include "feature-tracking-pipelines/feature-pipeline-base.h"
#include "feature-tracking-pipelines/keyframe-features.h"
#include "feature-tracking-pipelines/pnp-ransac.h"

namespace feature_tracking_pipelines {
//
// struct MatchingBasedTrackingSettings {
//  size_t num_features_detected = 500u;
//};
//
// class FeaturePipelineMatchingBased : public FeatureTrackingPipelineBase {
// public:
//  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//  FeaturePipelineMatchingBased(
//      const MatchingBasedTrackingSettings& settings,
//      const RansacSettings& ransac_settings,
//      std::shared_ptr<FeatureDetectorBase> feature_detector,
//      std::shared_ptr<FeatureDescriberBase> feature_describer)
//      : FeatureTrackingPipelineBase(ransac_settings, feature_detector,
//                                    feature_describer), settings_(settings) {}
//  virtual ~FeaturePipelineMatchingBased() {}
//
//  virtual void processImages(
//      const aslam::NCamera& ncamera,
//      const aslam::Transformation& T_Icurr_Iprev,
//      const std::vector<cv::Mat>& curr_camera_images,
//      const std::vector<cv::Mat>& prev_camera_images,
//      const std::vector<KeyframeFeatures>& previous_keyframe,
//      std::vector<KeyframeFeatures>* current_keyframe_ptr,
//      FeaturePipelineDebugData* optional_debug_data) {
//    // optional_debug_data is optional and can be a nullptr.
//
//    CHECK_EQ(ncamera.numCameras(), camera_images.size());
//    CHECK_EQ(ncamera.numCameras(), previous_keyframe.size());
//    CHECK_NOTNULL(current_keyframe_ptr)->clear();
//
//    // Perform detection and description in current frame.
//    std::vector<KeyframeFeatures>& current_keyframe =  *current_keyframe_ptr;
//    current_keyframe.resize(ncamera.size());
//
//    for (size_t frame_idx = 0u; frame_idx < ncamera.numCameras(); ++frame_idx)
//    {
//      feature_detector_->detectFeatures(
//          image[frame_idx], settings_.num_features_detected,
//          image_masks[frame_idx],
//          &current_keyframe[frame_idx].keypoint_measurements,
//          &current_keyframe[frame_idx].keypoint_scales,
//          &current_keyframe[frame_idx].keypoint_orientations_rad,
//          &current_keyframe[frame_idx].keypoint_scores);
//
//      // Remove the non-describable features.
//      std::vector<size_t> non_describable_indices;
//      if (feature_describer_->hasNonDescribableFeatures(
//          current_keyframe[frame_idx].keypoint_measurements,
//          current_keyframe[frame_idx].keypoint_scales,
//          &non_describable_indices)) {
//        RemoveKeypoints(non_describable_indices, current_keyframe[frame_idx]);
//      }
//
//      feature_describer_->describeFeatures(
//          image[frame_idx],
//          current_keyframe[frame_idx].keypoint_measurements,
//          current_keyframe[frame_idx].keypoint_scales,
//          current_keyframe[frame_idx].keypoint_orientations_rad,
//          &current_keyframe[frame_idx].keypoint_descriptors);
//    }
//
//    // Perform rotation-aided matching.
//    LOG(FATAL) << "Impl.";
//
//
//
//    // Perform PNP-RANSAC to find and discard outliers.
//    LOG(FATAL) << "Impl.";
//
//
//    // Write results to output structure.
//    LOG(FATAL) << "Impl.";
//  }
//
//
//
// private:
//  const MatchingBasedTrackingSettings settings_;
//
//};

}  // namespace feature_tracking_pipelines

#endif  // FEATURE_TRACKING_PIPELINES_FEATURE_PIPELINE_MATCHING_BASED_H_
