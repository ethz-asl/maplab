#ifndef FEATURE_TRACKING_FEATURE_TRACKING_PIPELINE_H_
#define FEATURE_TRACKING_FEATURE_TRACKING_PIPELINE_H_

#include <string>
#include <vector>

#include <aslam/cameras/ncamera.h>
#include <aslam/common/memory.h>
#include <aslam/frames/visual-nframe.h>
#include <maplab-common/macros.h>

namespace feature_tracking {

class FeatureTrackingPipeline {
 public:
  MAPLAB_POINTER_TYPEDEFS(FeatureTrackingPipeline);
  MAPLAB_DISALLOW_EVIL_CONSTRUCTORS(FeatureTrackingPipeline);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FeatureTrackingPipeline();
  virtual ~FeatureTrackingPipeline() = default;

 protected:
  const std::string feature_tracking_ros_base_topic_;
  const bool visualize_keypoint_detections_;
  const bool visualize_keypoint_matches_;
};

}  // namespace feature_tracking

#endif  // FEATURE_TRACKING_FEATURE_TRACKING_PIPELINE_H_
