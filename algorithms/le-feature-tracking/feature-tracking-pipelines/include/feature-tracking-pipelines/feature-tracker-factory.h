#ifndef FEATURE_TRACKING_PIPELINES_FEATURE_TRACKER_FACTORY_H_
#define FEATURE_TRACKING_PIPELINES_FEATURE_TRACKER_FACTORY_H_

#include <memory>
#include <vector>

#include <Eigen/Core>
#include <aslam/cameras/ncamera.h>
#include <gflags/gflags.h>
#include <glog/logging.h>

#include "feature-tracking-pipelines/feature-describer-brisk.h"
#include "feature-tracking-pipelines/feature-describer-freak.h"
#include "feature-tracking-pipelines/feature-describer-sift.h"
#include "feature-tracking-pipelines/feature-describer-surf.h"
#include "feature-tracking-pipelines/feature-detector-brisk.h"
#include "feature-tracking-pipelines/feature-detector-gftt.h"
#include "feature-tracking-pipelines/feature-detector-orb.h"
#include "feature-tracking-pipelines/feature-detector-sift.h"
#include "feature-tracking-pipelines/feature-detector-surf.h"
#include "feature-tracking-pipelines/feature-pipeline-lk-tracking-laser.h"
#include "feature-tracking-pipelines/feature-pipeline-matching-based.h"
#include "feature-tracking-pipelines/feature-tracker-gyro-aided.h"
#include "feature-tracking-pipelines/flags.h"


namespace feature_tracking_pipelines {

FeatureTrackingPipelineBase* CreateFeaturePipelineFromGFlags() {
  // Initialize the feature detector.
  std::shared_ptr<FeatureDetectorBase> feature_detector;

  if (FLAGS_NEW_feature_detector_type == "brisk") {
    FeatureDetectorBriskSettings brisk_settings =
        InitFeatureDetectorBriskSettingsFromGFlags();
    feature_detector.reset(new FeatureDetectorBrisk(brisk_settings));
  } else if (FLAGS_NEW_feature_detector_type == "gfft") {
    feature_detector.reset(new FeatureDetectorGFTT());
  } else if (FLAGS_NEW_feature_detector_type == "orb") {
    FeatureDetectorOrbSettings orb_settings =
        InitFeatureDetectorOrbSettingsFromGFlags();
    feature_detector.reset(new FeatureDetectorOrb(orb_settings));
  } else if (FLAGS_NEW_feature_detector_type == "sift") {
    FeatureDetectorSiftSettings sift_settings =
        InitFeatureDetectorSiftSettingsFromGFlags();
    feature_detector.reset(new FeatureDetectorSift(sift_settings));
  } else if (FLAGS_NEW_feature_detector_type == "surf") {
    FeatureDetectorSurfSettings surf_settings =
        InitFeatureDetectorSurfSettingsFromGFlags();
    feature_detector.reset(new FeatureDetectorSurf(surf_settings));
  } else {
    LOG(FATAL) << "Unknown feature detector set with --feature_tracker_type: "
               << FLAGS_NEW_feature_detector_type;
  }
  CHECK(feature_detector);

  // Initialize the feature describer.
  std::shared_ptr<FeatureDescriberBase> feature_describer;
  if (FLAGS_NEW_feature_descriptor_type == "brisk") {
    FeatureDescriberBriskSettings brisk_settings =
        InitFeatureDescriberBriskSettingsFromGFlags();
    feature_describer.reset(new FeatureDescriberBrisk(brisk_settings));
  } else if (FLAGS_NEW_feature_descriptor_type == "freak") {
    FeatureDescriberFreakSettings freak_settings =
        InitFeatureDescriberFreakSettingsFromGFlags();
    feature_describer.reset(new FeatureDescriberFreak(freak_settings));
  } else if (FLAGS_NEW_feature_descriptor_type == "sift") {
    FeatureDescriberSiftSettings sift_settings =
        InitFeatureDescriberSiftSettingsFromGFlags();
    feature_describer.reset(new FeatureDescriberSift(sift_settings));
  } else if (FLAGS_NEW_feature_descriptor_type == "surf") {
    FeatureDescriberSurfSettings surf_settings =
        InitFeatureDescriberSurfSettingsFromGFlags();
    feature_describer.reset(new FeatureDescriberSurf(surf_settings));
  } else {
    LOG(FATAL) << "Unknown feature pipeline set with --feature_tracker_type: "
               << FLAGS_NEW_feature_descriptor_type;
  }
  CHECK(feature_describer);

  // Initialize the method to establish temporal feature correspondances.
  if (FLAGS_NEW_feature_pipeline_type == "lk") {
    feature_tracking_pipelines::LkTrackingSettingsLaser lk_settings =
        InitLkLaserTrackingSettingsFromGFlags();
    feature_tracking_pipelines::RansacSettings ransac_settings;
    InitRansacSettingsFromGFlags();

    return new FeaturePipelineLkTrackingLaser(
        lk_settings, ransac_settings, feature_detector, feature_describer);
  } else if (FLAGS_NEW_feature_pipeline_type == "matching-based") {
    LOG(FATAL) << "TODO(schneith): impl.";
  } else {
    LOG(FATAL) << "Unknown feature pipeline set with --feature_tracker_type: "
               << FLAGS_NEW_feature_pipeline_type;
  }
  return nullptr;
}

}  // namespace feature_tracking_pipelines

#endif  // FEATURE_TRACKING_PIPELINES_FEATURE_TRACKER_FACTORY_H_
