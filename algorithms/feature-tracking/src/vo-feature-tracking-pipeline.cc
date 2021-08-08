#include "feature-tracking/vo-feature-tracking-pipeline.h"

#include <aslam/geometric-vision/match-outlier-rejection-twopt.h>
#include <aslam/matcher/match.h>
#include <aslam/tracker/feature-tracker-gyro.h>
#include <aslam/tracker/feature-tracker.h>
#include <aslam/visualization/basic-visualization.h>
#include <maplab-common/conversions.h>
#include <sensors/external-features.h>
#include <visualization/common-rviz-visualization.h>

DEFINE_double(
    feature_tracker_two_pt_ransac_threshold, 1.0 - cos(0.5 * kDegToRad),
    "Threshold for the 2-pt RANSAC used for feature tracking outlier "
    "removal. The error is defined as (1 - cos(alpha)) where alpha is "
    "the angle between the predicted and measured bearing vectors.");

DEFINE_double(
    feature_tracker_two_pt_ransac_max_iterations, 200,
    "Max iterations for the 2-pt RANSAC used for feature tracking "
    "outlier removal.");

DEFINE_bool(
    feature_tracker_deterministic, false,
    "If true, deterministic RANSAC outlier rejection is used.");
DEFINE_bool(
    detection_visualize_keypoints, false,
    "Visualize the raw keypoint detections to a ros topic.");

namespace feature_tracking {

VOFeatureTrackingPipeline::VOFeatureTrackingPipeline(
    const aslam::NCamera::ConstPtr& ncamera,
    const FeatureTrackingExtractorSettings& extractor_settings,
    const FeatureTrackingDetectorSettings& detector_settings)
    : first_nframe_initialized_(false),
      extractor_settings_(extractor_settings),
      detector_settings_(detector_settings) {
  initialize(ncamera);
}

VOFeatureTrackingPipeline::~VOFeatureTrackingPipeline() {
  if (thread_pool_) {
    thread_pool_->stop();
  }
}

void VOFeatureTrackingPipeline::initializeFirstNFrame(
    aslam::VisualNFrame* nframe_k) {
  CHECK_NOTNULL(nframe_k);
  CHECK(ncamera_.get() == nframe_k->getNCameraShared().get());

  const size_t num_cameras = nframe_k->getNumCameras();
  CHECK_EQ(num_cameras, detectors_extractors_.size());

  for (size_t camera_idx = 0u; camera_idx < num_cameras; ++camera_idx) {
    aslam::VisualFrame* frame_k = nframe_k->getFrameShared(camera_idx).get();
    detectors_extractors_[camera_idx]->detectAndExtractFeatures(frame_k);
  }

  first_nframe_initialized_ = true;
}

void VOFeatureTrackingPipeline::trackFeaturesNFrame(
    const aslam::Transformation& T_Bk_Bkp1, aslam::VisualNFrame* nframe_k,
    aslam::VisualNFrame* nframe_kp1) {
  CHECK_NOTNULL(nframe_kp1);
  CHECK_NOTNULL(nframe_k);
  aslam::FrameToFrameMatchesList inlier_matches_kp1_k;
  aslam::FrameToFrameMatchesList outlier_matches_kp1_k;
  trackFeaturesNFrame(
      T_Bk_Bkp1.getRotation().inverse(), nframe_kp1, nframe_k,
      &inlier_matches_kp1_k, &outlier_matches_kp1_k);
}

void VOFeatureTrackingPipeline::trackFeaturesNFrame(
    const aslam::Quaternion& q_Bkp1_Bk, aslam::VisualNFrame* nframe_kp1,
    aslam::VisualNFrame* nframe_k,
    aslam::FrameToFrameMatchesList* inlier_matches_kp1_k,
    aslam::FrameToFrameMatchesList* outlier_matches_kp1_k) {
  CHECK_NOTNULL(nframe_kp1);
  CHECK_NOTNULL(nframe_k);
  CHECK_NOTNULL(inlier_matches_kp1_k)->clear();
  CHECK_NOTNULL(outlier_matches_kp1_k)->clear();
  CHECK_GT(
      nframe_kp1->getMinTimestampNanoseconds(),
      nframe_k->getMinTimestampNanoseconds());
  CHECK(ncamera_.get() == nframe_kp1->getNCameraShared().get());

  const size_t num_cameras = nframe_kp1->getNumCameras();
  CHECK_EQ(num_cameras, trackers_.size());
  CHECK_EQ(num_cameras, track_managers_.size());
  CHECK_EQ(num_cameras, detectors_extractors_.size());

  // The first nframe has to be initialized when it is received so that
  // the standard binary features are already inserted before the nframe
  // is attached to a vertex. Otherwise the external features might be
  // over written for just this vertex.
  CHECK(first_nframe_initialized_)
      << "Feature tracking pipeline not initialized. Please call "
      << "initializeFirstNFrame on the very first nframe.";

  // Track features for each camera in its own thread.
  inlier_matches_kp1_k->resize(num_cameras);
  outlier_matches_kp1_k->resize(num_cameras);

  CHECK(thread_pool_);
  for (size_t camera_idx = 0u; camera_idx < num_cameras; ++camera_idx) {
    aslam::VisualFrame* frame_kp1 =
        nframe_kp1->getFrameShared(camera_idx).get();
    aslam::VisualFrame* frame_k = nframe_k->getFrameShared(camera_idx).get();
    thread_pool_->enqueue(
        &VOFeatureTrackingPipeline::trackFeaturesSingleCamera, this, q_Bkp1_Bk,
        camera_idx, frame_kp1, frame_k, &(*inlier_matches_kp1_k)[camera_idx],
        &(*outlier_matches_kp1_k)[camera_idx]);
  }
  thread_pool_->waitForEmptyQueue();
}

void VOFeatureTrackingPipeline::trackFeaturesSingleCamera(
    const aslam::Quaternion& q_Bkp1_Bk, const size_t camera_idx,
    aslam::VisualFrame* frame_kp1, aslam::VisualFrame* frame_k,
    aslam::FrameToFrameMatches* inlier_matches_kp1_k,
    aslam::FrameToFrameMatches* outlier_matches_kp1_k) {
  CHECK_LE(camera_idx, track_managers_.size());
  CHECK_NOTNULL(frame_k);
  CHECK_NOTNULL(frame_kp1);
  CHECK_NOTNULL(inlier_matches_kp1_k);
  CHECK_NOTNULL(outlier_matches_kp1_k);
  inlier_matches_kp1_k->clear();
  outlier_matches_kp1_k->clear();

  // Maintaining a consistent locking order (i.e. temporal) is very important
  // to avoid potential deadlocking with other trackers running in parallel
  frame_k->lock();
  frame_kp1->lock();

  // Initialize keypoints and descriptors in frame_kp1
  detectors_extractors_[camera_idx]->detectAndExtractFeatures(frame_kp1);

  if (FLAGS_detection_visualize_keypoints) {
    cv::Mat image;
    cv::cvtColor(frame_kp1->getRawImage(), image, cv::COLOR_GRAY2BGR);

    aslam_cv_visualization::drawKeypoints(*CHECK_NOTNULL(frame_kp1), &image);
    const std::string topic = feature_tracking_ros_base_topic_ +
                              "/keypoints_raw_cam" + std::to_string(camera_idx);
    visualization::RVizVisualizationSink::publish(topic, image);
  }

  // The default detector / tracker with always insert descriptors of type
  // kBinary = 0 for both BRISK and FREAK
  constexpr int descriptor_type =
      static_cast<int>(vi_map::FeatureType::kBinary);

  CHECK(frame_k->hasKeypointMeasurements());
  CHECK(frame_k->hasDescriptors());
  CHECK(frame_k->hasDescriptorType(descriptor_type));
  CHECK(frame_kp1->hasKeypointMeasurements());
  CHECK(frame_kp1->hasDescriptors());
  CHECK(frame_kp1->hasDescriptorType(descriptor_type));

  // Get the relative motion of the camera using the extrinsics of the camera
  // system.
  const aslam::Quaternion& q_C_B =
      ncamera_->get_T_C_B(camera_idx).getRotation();
  aslam::Quaternion q_Ckp1_Ck = q_C_B * q_Bkp1_Bk * q_C_B.inverse();

  aslam::FrameToFrameMatchesWithScore matches_with_score_kp1_k;
  trackers_[camera_idx]->track(
      q_Ckp1_Ck, *frame_k, frame_kp1, &matches_with_score_kp1_k);

  // The tracker will return the indices with respect to the tracked feature
  // block, so here we renormalize them so that the rest of the code can deal
  // with them agnostically, since the descriptors are no longer needed.
  size_t start_k, size_k, start_kp1, size_kp1;
  frame_k->getDescriptorBlockTypeStartAndSize(
      descriptor_type, &start_k, &size_k);
  frame_kp1->getDescriptorBlockTypeStartAndSize(
      descriptor_type, &start_kp1, &size_kp1);

  for (aslam::FrameToFrameMatchWithScore& match_kp1_k :
       matches_with_score_kp1_k) {
    match_kp1_k.setIndexApple(match_kp1_k.getIndexApple() + start_kp1);
    match_kp1_k.setIndexBanana(match_kp1_k.getIndexBanana() + start_k);
  }

  // Remove outlier matches.
  aslam::FrameToFrameMatchesWithScore inlier_matches_with_score_kp1_k;
  aslam::FrameToFrameMatchesWithScore outlier_matches_with_score_kp1_k;

  bool ransac_success = aslam::geometric_vision::
      rejectOutlierFeatureMatchesTranslationRotationSAC(
          *frame_kp1, *frame_k, q_Ckp1_Ck, matches_with_score_kp1_k,
          FLAGS_feature_tracker_deterministic,
          FLAGS_feature_tracker_two_pt_ransac_threshold,
          FLAGS_feature_tracker_two_pt_ransac_max_iterations,
          &inlier_matches_with_score_kp1_k, &outlier_matches_with_score_kp1_k);

  LOG_IF(WARNING, !ransac_success)
      << "Match outlier rejection RANSAC failed on camera " << camera_idx
      << ".";
  const size_t num_outliers = outlier_matches_with_score_kp1_k.size();
  VLOG_IF(5, num_outliers > 0) << "Removed " << num_outliers << " outliers of "
                               << matches_with_score_kp1_k.size()
                               << " matches on camera " << camera_idx << ".";

  // Assign track ids
  track_managers_[camera_idx]->applyMatchesToFrames(
      inlier_matches_with_score_kp1_k, frame_kp1, frame_k);

  aslam::convertMatchesWithScoreToMatches<
      aslam::FrameToFrameMatchWithScore, aslam::FrameToFrameMatch>(
      inlier_matches_with_score_kp1_k, inlier_matches_kp1_k);
  aslam::convertMatchesWithScoreToMatches<
      aslam::FrameToFrameMatchWithScore, aslam::FrameToFrameMatch>(
      outlier_matches_with_score_kp1_k, outlier_matches_kp1_k);

  if (visualize_keypoint_matches_) {
    cv::Mat image;
    aslam_cv_visualization::visualizeMatches(
        *frame_kp1, *frame_k, inlier_matches_with_score_kp1_k, &image);
    const std::string topic = feature_tracking_ros_base_topic_ +
                              "/keypoint_matches_camera_" +
                              std::to_string(camera_idx);
    visualization::RVizVisualizationSink::publish(topic, image);

    cv::Mat outlier_image;
    aslam_cv_visualization::visualizeMatches(
        *frame_kp1, *frame_k, outlier_matches_with_score_kp1_k, &outlier_image);
    const std::string outlier_topic = feature_tracking_ros_base_topic_ +
                                      "/keypoint_outlier_matches_camera_" +
                                      std::to_string(camera_idx);
    visualization::RVizVisualizationSink::publish(outlier_topic, outlier_image);
  }

  frame_kp1->unlock();
  frame_k->unlock();
}

void VOFeatureTrackingPipeline::initialize(
    const aslam::NCamera::ConstPtr& ncamera) {
  CHECK(ncamera);
  ncamera_ = ncamera;
  // Create a thread pool.
  const size_t num_cameras = ncamera_->numCameras();
  thread_pool_.reset(new aslam::ThreadPool(num_cameras));

  // Create a feature tracker.
  detectors_extractors_.reserve(num_cameras);
  trackers_.reserve(num_cameras);
  track_managers_.reserve(num_cameras);

  for (size_t cam_idx = 0u; cam_idx < num_cameras; ++cam_idx) {
    detectors_extractors_.emplace_back(new FeatureDetectorExtractor(
        ncamera_->getCamera(cam_idx), extractor_settings_, detector_settings_));
    trackers_.emplace_back(new aslam::GyroTracker(
        ncamera_->getCamera(cam_idx),
        detector_settings_.min_tracking_distance_to_image_border_px,
        detectors_extractors_.back()->getExtractorPtr()));
    track_managers_.emplace_back(new aslam::SimpleTrackManager);
  }
}
}  // namespace feature_tracking
