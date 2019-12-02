#include "maplab-node/localization-handler.h"

#include <limits>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <aslam/common/pose-types.h>
#include <aslam/common/time.h>
#include <gflags/gflags.h>
#include <maplab-common/conversions.h>
#include <maplab-common/fixed-size-queue.h>
#include <maplab-common/geometry.h>
#include <maplab-common/localization-result.h>
#include <maplab-common/quaternion-math.h>
#include <vio-common/pose-lookup-buffer.h>
#include <vio-common/vio-types.h>

#include "maplab-node/flow-topics.h"

DEFINE_bool(
    use_6dof_localization, true,
    "Localize using 6dof constraints instead of structure constraints.");
DEFINE_uint64(
    min_num_baseframe_estimates_before_init, 2u,
    "Number of T_G_M measurements to collect before initializing T_G_M.");
DEFINE_double(
    baseframe_init_position_covariance_msq, 20.0,
    "Position covariance of the baseframe initialization [m^2].");
DEFINE_double(
    baseframe_init_rotation_covariance_radsq, 90.0 * kDegToRad,
    "Rotation covariance of the baseframe initialization [rad^2].");

DEFINE_double(
    max_mean_localization_reprojection_error_px, 100.0,
    "If mean reprojection error of the matches exceeds this value, "
    "reinitialize the baseframe.");

DEFINE_double(
    localization_max_gravity_misalignment_deg, 5.0,
    "Localization results are rejected if the angle between the gravity"
    "direction of the odometry and the localization exceeds this value.");

DEFINE_int32(
    min_number_of_structure_constraints, 5,
    "After we reject structure constraints based on their reprojection "
    "error, this is the minimum number of constraints required to accept a "
    "localization.");

namespace maplab {

namespace {
bool getReprojectionErrorForGlobalLandmark(
    const Eigen::Vector3d& p_G, const pose::Transformation& T_G_C,
    const aslam::Camera& camera, const Eigen::Vector2d& measurement,
    double* reprojection_error) {
  CHECK_NOTNULL(reprojection_error);

  const Eigen::Vector3d p_C = T_G_C.inverse() * p_G;
  Eigen::Vector2d reprojected_keypoint;
  const aslam::ProjectionResult result =
      camera.project3(p_C, &reprojected_keypoint);

  const aslam::ProjectionResult::Status projection_result =
      result.getDetailedStatus();
  CHECK_NE(projection_result, aslam::ProjectionResult::UNINITIALIZED);
  if (projection_result == aslam::ProjectionResult::POINT_BEHIND_CAMERA ||
      projection_result == aslam::ProjectionResult::PROJECTION_INVALID) {
    return false;
  }

  *reprojection_error = (reprojected_keypoint - measurement).norm();
  return true;
}
}  // namespace

LocalizationHandler::LocalizationHandler(
    const vi_map::SensorManager& sensor_manager,
    const vio_common::PoseLookupBuffer& T_M_B_buffer)
    : sensor_manager_(sensor_manager),
      T_M_B_buffer_(T_M_B_buffer),
      // 6dof constraint based localization does not need initialization.
      localization_state_(
          FLAGS_use_6dof_localization
              ? common::LocalizationState::kLocalized
              : common::LocalizationState::kUninitialized),
      T_G_M_loc_buffer_(FLAGS_min_num_baseframe_estimates_before_init),
      previous_odometry_measurement_id_(-1) {
  if (FLAGS_use_6dof_localization) {
    LOG(INFO) << "[MaplabNode-LocalizationHandler] Localization mode: 6DoF "
                 "constraints.";
    aslam::SensorIdSet odometry_sensors;
    sensor_manager_.getAllSensorIdsOfType(
        vi_map::kOdometry6DoF, &odometry_sensors);
    CHECK_GT(odometry_sensors.size(), 0)
        << "[MaplabNode-LocalizationHandler] There needs to be at least one "
        << "odometry source to perform localization fusion.";
    vi_map::Odometry6DoF odometry_sensor =
        sensor_manager_.getSensor<vi_map::Odometry6DoF>(
            *odometry_sensors.begin());
    odometry_sensor.get_T_St_Stp1_fixed_covariance(
        &T_St_Stp1_fixed_covariance_);
  } else {
    // TODO(mfehr): TODO(LBern): change once we have structure constraint mode.
    LOG(FATAL) << "[MaplabNode-LocalizationHandler] Currently only 6DoF "
               << "localization mode is implemented!";
  }
}

bool LocalizationHandler::processLocalizationResult(
    const common::LocalizationResult::ConstPtr& localization_result,
    common::FusedLocalizationResult* fused_localization_result) {
  CHECK(localization_result);
  CHECK_NOTNULL(fused_localization_result);

  // Checking if there is a sensor associated with this localization. It
  // might be ok if there is not, depending on the localization type.
  const bool localization_has_sensor = localization_result->sensor_id.isValid();

  bool sensor_manager_has_sensor = false;
  vi_map::SensorType sensor_type = vi_map::SensorType::kUnknown;
  if (localization_has_sensor) {
    sensor_manager_has_sensor =
        sensor_manager_.hasSensor(localization_result->sensor_id);
    if (sensor_manager_has_sensor) {
      sensor_type =
          sensor_manager_.getSensorType(localization_result->sensor_id);
    }
  }

  bool success = false;

  switch (localization_state_) {
    case common::LocalizationState::kUninitialized:
    // Fall-through intended.
    case common::LocalizationState::kNotLocalized: {
      success = initializeBaseframe(*localization_result);
      if (success) {
        LOG(INFO) << "[MaplabNode-LocalizationHandler] (Re-)initialized the "
                     "localization baseframe.";
        localization_state_ = common::LocalizationState::kLocalized;
      }
      break;
    }
    case common::LocalizationState::kLocalized: {
      switch (localization_result->localization_type) {
        case common::LocalizationType::kVisualFeatureBased: {
          CHECK(localization_has_sensor)
              << "[MaplabNode-LocalizationHandler] Received "
              << "visual-feature-based localization result, but there was no "
              << "sensor associated to it!";
          CHECK(sensor_manager_has_sensor)
              << "[MaplabNode-LocalizationHandler] Received "
              << "visual-feature-based localization result, but the associated "
              << "sensor (id: " << localization_result->sensor_id
              << " type: " << sensor_type << ") is "
              << "not in the sensor manager!";
          CHECK_EQ(sensor_type, vi_map::SensorType::kNCamera)
              << "[MaplabNode-LocalizationHandler] Received "
              << "visual-feature-based localization result, but the associated "
              << "sensor (id: " << localization_result->sensor_id
              << " type: " << sensor_type << ") is not a camera!";

          const vio::LocalizationResult* visual_localization_result =
              dynamic_cast<const vio::LocalizationResult*>(
                  localization_result.get());
          CHECK_NOTNULL(visual_localization_result);

          const aslam::NCamera& ncamera =
              sensor_manager_.getSensor<aslam::NCamera>(
                  localization_result->sensor_id);
          success = processVisualLocalizationResultAsUpdate(
              ncamera, *visual_localization_result);
        } break;
        default:
          LOG(FATAL) << "Unknown localization type: "
                     << static_cast<int>(
                            localization_result->localization_type);
          break;
      }
      break;
    }
    default:
      LOG(FATAL) << "Unknown localization state: "
                 << static_cast<int>(localization_state_);
      break;
  }
  if (success) {
    localization_filter_.getFusedLocalization(fused_localization_result);
  }
  return success;
}

bool LocalizationHandler::initializeBaseframe(
    const common::LocalizationResult& localization_result) {
  // Collect a certain number of localizations before performing the actual
  // initialization.
  vio::ViNodeState odometry_estimate, sequence_number_state;
  if (T_M_B_buffer_.interpolateViNodeStateAt(
          localization_result.timestamp_ns, &odometry_estimate) >
      vio_common::PoseLookupBuffer::ResultStatus::
          kSuccessImuForwardPropagation) {
    LOG(WARNING) << "[MaplabNode-LocalizationHandler] Could not get T_M_B for "
                    "baseframe initialization.";
    return false;
  }
  const aslam::Transformation T_G_M_loc_estimate =
      localization_result.T_G_B * odometry_estimate.get_T_M_I().inverse();

  T_G_M_loc_buffer_.insert(T_G_M_loc_estimate);
  if (T_G_M_loc_buffer_.size() <
      FLAGS_min_num_baseframe_estimates_before_init) {
    return false;
  }

  // Perform initialization with LSQ estimate of the baseframe transformation
  // in the buffer.
  const int kNumInliersThreshold = std::ceil(
      FLAGS_min_num_baseframe_estimates_before_init *
      kInitializationRansacInlierRatioThreshold);

  int num_inliers = 0;
  std::random_device device;
  const int ransac_seed = device();
  common::transformationRansac(
      T_G_M_loc_buffer_.buffer(), kInitializationMaxNumRansacIterations,
      kInitializationRansacOrientationErrorThresholdRadians,
      kInitializationRansacPositionErrorThresholdMeters, ransac_seed, &T_G_M_,
      &num_inliers);
  if (num_inliers < kNumInliersThreshold) {
    VLOG(1) << "Too few localization transformation inliers (" << num_inliers
            << "/" << T_G_M_loc_buffer_.size() << ").";
    return false;
  }

  LOG(INFO) << "[MaplabNode-LocalizationHandler] Initializing localization "
               "filter from ransac.";
  // Initialize the localization filter
  common::LocalizationResult initialization_result(localization_result);
  initialization_result.T_G_M = T_G_M_;
  localization_filter_.initialize(
      initialization_result, odometry_estimate.get_T_M_I());
  CHECK(T_M_B_buffer_.getValueAtOrAfterTime(
      localization_result.timestamp_ns, &sequence_number_state));
  previous_odometry_measurement_id_ = sequence_number_state.getSequenceNumber();
  T_G_M_loc_buffer_.clear();
  return true;
}

double getLocalizationResultGravityDisparityAngleDeg(
    const vio::LocalizationResult& localization_result,
    const pose::Transformation& T_G_I_filter) {
  const pose::Transformation& T_G_B = localization_result.T_G_B;

  const Eigen::Vector3d gravity_direction_filter =
      T_G_I_filter.getRotation().inverse().rotate(Eigen::Vector3d::UnitZ());
  const Eigen::Vector3d gravity_direction_localization_pnp =
      T_G_B.getRotation().inverse().rotate(Eigen::Vector3d::UnitZ());

  CHECK_NEAR(gravity_direction_filter.squaredNorm(), 1.0, 1e-8);
  CHECK_NEAR(gravity_direction_localization_pnp.squaredNorm(), 1.0, 1e-8);

  const double error_cosine =
      gravity_direction_filter.dot(gravity_direction_localization_pnp);

  double error_angle_degrees = 180.;
  if (error_cosine <= -1) {
    error_angle_degrees = 180.;
  } else if (error_cosine >= 1) {
    error_angle_degrees = 0.;
  } else {
    // Cosine is in the valid range.
    error_angle_degrees = std::acos(error_cosine) * kRadToDeg;
  }

  return error_angle_degrees;
}

bool LocalizationHandler::processVisualLocalizationResultAsUpdate(
    const aslam::NCamera& /*ncamera*/,
    const vio::LocalizationResult& localization_result) {
  vio::ViNodeState odometry_estimate;
  const vio_common::PoseLookupBuffer::ResultStatus lookup_result =
      T_M_B_buffer_.interpolateViNodeStateAt(
          localization_result.timestamp_ns, &odometry_estimate);

  CHECK(
      lookup_result !=
      vio_common::PoseLookupBuffer::ResultStatus::kFailedNotYetAvailable);
  CHECK(
      lookup_result !=
      vio_common::PoseLookupBuffer::ResultStatus::kFailedWillNeverSucceed);

  const double gravity_error_angle_deg =
      getLocalizationResultGravityDisparityAngleDeg(
          localization_result, odometry_estimate.get_T_M_I());
  if (gravity_error_angle_deg >
      FLAGS_localization_max_gravity_misalignment_deg) {
    LOG(WARNING)
        << "[MaplabNode-LocalizationHandler] The gravity direction of the "
        << "localization is not "
        << "consistent with the odometry estimate. The disparity angle "
        << "is " << gravity_error_angle_deg
        << "deg (threshold: " << FLAGS_localization_max_gravity_misalignment_deg
        << "). Rejected the localization result.";
    return false;
  }

  bool measurement_accepted = false;
  if (FLAGS_use_6dof_localization) {
    // To receive a conservative estimate of the odometry covariance, the first
    // available sequence number >= the current timestamp is used
    vio::ViNodeState sequence_number_state;
    CHECK(T_M_B_buffer_.getValueAtOrAfterTime(
        localization_result.timestamp_ns, &sequence_number_state));
    int64_t sequence_number = sequence_number_state.getSequenceNumber();

    if (localization_filter_.isInitialized()) {
      // Calculate the odometry covariance as nSteps * covariance_per_step
      double nsteps = static_cast<double>(
          sequence_number - previous_odometry_measurement_id_);
      if (nsteps < 0) {
        // Per the current covariance calculation method, this would lead to a
        // negative odometry covariance. Therefore ignoring the measurement.
        LOG(WARNING) << "[MaplabNode-LocalizationHandler]: A localization "
                     << "arrived out of order. "
                     << "It will be ignored by the filter.";
        return false /*measurement_accepted = false*/;
      }
      aslam::TransformationCovariance odometry_covariance;
      odometry_covariance = nsteps * T_St_Stp1_fixed_covariance_;
      odometry_estimate.setPoseCovariance(odometry_covariance);
      measurement_accepted = localization_filter_.localizationCallback(
          localization_result, odometry_estimate);
      if (measurement_accepted) {
        previous_odometry_measurement_id_ = sequence_number;
      }
    } else {
      LOG(INFO) << "[MaplabNode-LocalizationHandler] Using the first "
                   "localization result to initialize the localization filter.";
      localization_filter_.initialize(
          localization_result, odometry_estimate.get_T_M_I());
      previous_odometry_measurement_id_ = sequence_number;
      measurement_accepted = true;
    }
  } else {
    // TODO(mfehr): TODO(LBern): do proper fusion using structure constraints.
  }

  LOG_IF(WARNING, !measurement_accepted)
      << "[MaplabNode-LocalizationHandler] Localization update rejected at "
      << "time = " << localization_result.timestamp_ns << "ns.";
  return measurement_accepted;
}

double LocalizationHandler::getVisualLocalizationReprojectionErrors(
    const aslam::NCamera& ncamera,
    const vio::LocalizationResult& localization_result,
    const aslam::Transformation& T_G_I_filter,
    std::vector<double>* lc_reprojection_errors,
    std::vector<double>* filter_reprojection_errors) {
  CHECK_NOTNULL(lc_reprojection_errors)->clear();
  CHECK_NOTNULL(filter_reprojection_errors)->clear();

  CHECK_EQ(
      localization_result.G_landmarks_per_camera.size(),
      localization_result.keypoint_measurements_per_camera.size());

  size_t num_matches_processed = 0;

  const size_t num_cameras = localization_result.G_landmarks_per_camera.size();
  CHECK_EQ(num_cameras, ncamera.numCameras());
  for (size_t cam_idx = 0u; cam_idx < num_cameras; ++cam_idx) {
    CHECK_EQ(
        localization_result.G_landmarks_per_camera[cam_idx].cols(),
        localization_result.keypoint_measurements_per_camera[cam_idx].cols());

    const size_t num_matches =
        localization_result.G_landmarks_per_camera[cam_idx].cols();

    if (num_matches == 0u) {
      continue;
    }

    const pose::Transformation& T_C_B = ncamera.get_T_C_B(cam_idx);
    const pose::Transformation T_G_C_filter =
        (T_C_B * T_G_I_filter.inverse()).inverse();
    const pose::Transformation T_G_C_loc =
        (T_C_B * localization_result.T_G_B.inverse()).inverse();

    for (size_t i = 0u; i < num_matches; ++i) {
      const Eigen::Vector2d& keypoint =
          localization_result.keypoint_measurements_per_camera[cam_idx].col(i);
      const Eigen::Vector3d& p_G =
          localization_result.G_landmarks_per_camera[cam_idx].col(i);

      double reproj_error_sq_filter;
      double reproj_error_sq_lc;
      const aslam::Camera& camera = ncamera.getCamera(cam_idx);
      bool projection_successful = getReprojectionErrorForGlobalLandmark(
          p_G, T_G_C_filter, camera, keypoint, &reproj_error_sq_filter);
      if (projection_successful) {
        projection_successful &= getReprojectionErrorForGlobalLandmark(
            p_G, T_G_C_loc, camera, keypoint, &reproj_error_sq_lc);
      }

      ++num_matches_processed;

      if (projection_successful) {
        lc_reprojection_errors->push_back(reproj_error_sq_filter);
        filter_reprojection_errors->push_back(reproj_error_sq_lc);
      }
    }
  }

  CHECK_EQ(lc_reprojection_errors->size(), filter_reprojection_errors->size());
  CHECK_GT(num_matches_processed, 0u);
  return static_cast<double>(lc_reprojection_errors->size()) /
         num_matches_processed;
}

}  //  namespace maplab
