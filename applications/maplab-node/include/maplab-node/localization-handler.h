#ifndef MAPLAB_NODE_LOCALIZATION_HANDLER_H_
#define MAPLAB_NODE_LOCALIZATION_HANDLER_H_

#include <memory>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <aslam/cameras/ncamera.h>
#include <aslam/common/pose-types.h>
#include <aslam/common/stl-helpers.h>
#include <aslam/common/time.h>
#include <aslam/common/unique-id.h>
#include <gflags/gflags.h>
#include <localization-fusion/localization-filter.h>
#include <localization-fusion/ukf.h>
#include <maplab-common/fixed-size-queue.h>
#include <maplab-common/geometry.h>
#include <maplab-common/localization-result.h>
#include <maplab-common/string-tools.h>
#include <message-flow/message-flow.h>
#include <vio-common/pose-lookup-buffer.h>
#include <vio-common/vio-types.h>

#include "maplab-node/flow-topics.h"

namespace maplab {

class LocalizationHandler {
 public:
  explicit LocalizationHandler(
      const vi_map::SensorManager& sensor_manager,
      const vio_common::PoseLookupBuffer& T_M_B_buffer);

  bool processLocalizationResult(
      const common::LocalizationResult::ConstPtr& localization_result,
      common::FusedLocalizationResult* fused_localization_result);

 private:
  bool initializeBaseframe(
      const common::LocalizationResult& localization_result);
  bool processVisualLocalizationResultAsUpdate(
      const aslam::NCamera& ncamera,
      const vio::LocalizationResult& localization_result);

  // Returns the ratio of successfully reprojected matches.
  double getVisualLocalizationReprojectionErrors(
      const aslam::NCamera& ncamera,
      const vio::LocalizationResult& localization_result,
      const aslam::Transformation& T_M_B_filter,
      std::vector<double>* lc_reprojection_errors,
      std::vector<double>* filter_reprojection_errors);

  const vi_map::SensorManager& sensor_manager_;

  const vio_common::PoseLookupBuffer& T_M_B_buffer_;

  common::LocalizationState localization_state_;

  common::FixedSizeQueue<aslam::Transformation> T_G_M_loc_buffer_;

  static constexpr size_t kInitializationMaxNumRansacIterations = 3u;
  static constexpr double kInitializationRansacPositionErrorThresholdMeters =
      5.0;
  static constexpr double
      kInitializationRansacOrientationErrorThresholdRadians = 20.0 * kDegToRad;
  static constexpr double kInitializationRansacInlierRatioThreshold = 0.75;

  aslam::TransformationCovariance T_St_Stp1_fixed_covariance_;

  // store the ID of the previously submitted odometry measurement to calculate
  // the uncertainty gain (i.e. covariance matrix) between the current
  // measurement and the previous one

  int64_t previous_odometry_measurement_id_;

  // Current solution of the localization.
  aslam::Transformation T_G_M_;

  LocalizationFilter<Ukf> localization_filter_;
};

}  //  namespace maplab

#endif  // MAPLAB_NODE_LOCALIZATION_HANDLER_H_
