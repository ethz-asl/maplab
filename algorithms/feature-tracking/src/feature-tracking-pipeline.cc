#include "feature-tracking/feature-tracking-pipeline.h"

DEFINE_bool(
    feature_tracker_visualize_keypoint_detections, false,
    "Visualize the raw keypoint detections to a ros topic.");
DEFINE_bool(
    feature_tracker_visualize_keypoint_matches, false,
    "Visualize the keypoint matches and the outliers.");

namespace feature_tracking {

FeatureTrackingPipeline::FeatureTrackingPipeline()
    : feature_tracking_ros_base_topic_("tracking/"),
      visualize_keypoint_detections_(
          FLAGS_feature_tracker_visualize_keypoint_detections),
      visualize_keypoint_matches_(
          FLAGS_feature_tracker_visualize_keypoint_matches) {}

}  // namespace feature_tracking
