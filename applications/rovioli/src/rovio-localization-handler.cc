#include "rovioli/rovio-localization-handler.h"

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
#include <maplab-common/quaternion-math.h>
#include <vio-common/pose-lookup-buffer.h>
#include <vio-common/vio-types.h>

#include "rovioli/flow-topics.h"
#include "rovioli/rovio-factory.h"
#include "rovioli/rovio-maplab-timetranslation.h"

DEFINE_bool(
    rovioli_use_6dof_localization, false,
    "Localize using 6dof constraints instead of structure constraints.");
DEFINE_uint64(
    rovioli_min_num_baseframe_estimates_before_init, 2u,
    "Number of T_G_M measurements to collect before initializing T_G_M.");
DEFINE_double(
    rovioli_baseframe_init_position_covariance_msq, 20.0,
    "Position covariance of the baseframe initialization [m^2].");
DEFINE_double(
    rovioli_baseframe_init_rotation_covariance_radsq, 90.0 * kDegToRad,
    "Rotation covariance of the baseframe initialization [rad^2].");

DEFINE_double(
    rovioli_max_mean_localization_reprojection_error_px, 100.0,
    "If mean reprojection error of the matches exceeds this value, "
    "reinitialize the baseframe.");

DEFINE_double(
    rovioli_localization_max_gravity_misalignment_deg, 5.0,
    "Localization results are rejected if the angle between the gravity"
    "direction of the odometry and the localization exceeds this value.");

DEFINE_bool(
    rovioli_use_6dof_localization_for_inactive_cameras, false,
    "ROVIO is set to always run in monocular mode, but the maplab part of "
    "ROVIOLI will build a map and localize based on all cameras. If there is a "
    "localization result for the active ROVIO camera, it will update the "
    "filter using either 2D-3D correspondences (structure constraints) or 6DoF "
    "constraints.  In structure constraints mode (default) it will ignore the "
    "results of the inactive cameras. If this option is enabled however, it "
    "will use the localization results of the inactive camera as 6DoF update "
    "in case the active camera didn't localize at all.");

DEFINE_int32(
    rovioli_min_number_of_structure_constraints, 5,
    "After ROVIOLI rejects structure constraints based on their reprojection "
    "error, this is the minimum number of constraints required to accept a "
    "localization.");

namespace rovioli {

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

  CHECK_NE(result.getDetailedStatus(), aslam::ProjectionResult::UNINITIALIZED);
  if (result.getDetailedStatus() ==
          aslam::ProjectionResult::POINT_BEHIND_CAMERA ||
      result.getDetailedStatus() ==
          aslam::ProjectionResult::PROJECTION_INVALID) {
    return false;
  }

  *reprojection_error = (reprojected_keypoint - measurement).norm();
  return true;
}
}  // namespace

RovioLocalizationHandler::RovioLocalizationHandler(
    rovio::RovioInterface* rovio_interface,
    RovioMaplabTimeTranslation* time_translator,
    const aslam::NCamera& camera_calibration,
    const common::BidirectionalMap<size_t, size_t>&
        maplab_to_rovio_cam_indices_mapping)
    : rovio_interface_(CHECK_NOTNULL(rovio_interface)),
      time_translator_(CHECK_NOTNULL(time_translator)),
      // 6dof constraint based localization does not need initialization.
      localization_state_(
          FLAGS_rovioli_use_6dof_localization
              ? LocalizationState::kLocalized
              : LocalizationState::kUninitialized),
      T_M_I_buffer_(kBufferPoseHistoryNs, kBufferMaxPropagationNs),
      T_G_M_filter_buffer_(kFilterBaseframeBufferSize),
      T_G_M_lc_buffer_(FLAGS_rovioli_min_num_baseframe_estimates_before_init),
      camera_calibration_(camera_calibration),
      maplab_to_rovio_cam_indices_mapping_(
          maplab_to_rovio_cam_indices_mapping) {
  if (FLAGS_rovioli_use_6dof_localization) {
    LOG(INFO) << "Localization mode: 6dof constraints.";
  } else {
    LOG(INFO) << "Localization mode: structure constraints.";
  }
}

void RovioLocalizationHandler::processLocalizationResult(
    const vio::LocalizationResult::ConstPtr& localization_result) {
  CHECK(localization_result);
  switch (localization_state_) {
    // Fall-through intended.
    case LocalizationState::kUninitialized:
    case LocalizationState::kNotLocalized: {
      const bool success = initializeBaseframe(localization_result);
      if (success) {
        LOG(INFO) << "(Re-)initialized the localization baseframe.";
        localization_state_ = LocalizationState::kLocalized;
      }
      break;
    }

    case LocalizationState::kLocalized: {
      processAsUpdate(localization_result);
      break;
    }
  }
}

bool RovioLocalizationHandler::initializeBaseframe(
    const vio::LocalizationResult::ConstPtr& localization_result) {
  CHECK(localization_result);

  // Collect a certain number of localizations before performing the actual
  // initialization.
  aslam::Transformation T_M_I;
  if (T_M_I_buffer_.getPoseAt(localization_result->timestamp_ns, &T_M_I) ==
      vio_common::PoseLookupBuffer::ResultStatus::kFailed) {
    LOG(WARNING) << "Could not get T_M_I for baseframe initialization.";
    return false;
  }
  const aslam::Transformation T_G_M_lc_estimate =
      localization_result->T_G_I_lc_pnp * T_M_I.inverse();

  T_G_M_lc_buffer_.insert(T_G_M_lc_estimate);
  if (T_G_M_lc_buffer_.size() <
      FLAGS_rovioli_min_num_baseframe_estimates_before_init) {
    return false;
  }

  // Perform initialization with LSQ estimate of the baseframe transformation
  // in the buffer.
  const int kNumInliersThreshold = std::ceil(
      FLAGS_rovioli_min_num_baseframe_estimates_before_init *
      kInitializationRansacInlierRatioThreshold);

  int num_inliers = 0;
  std::random_device device;
  const int ransac_seed = device();
  aslam::Transformation T_G_M_lsq;
  common::transformationRansac(
      T_G_M_lc_buffer_.buffer(), kInitializationMaxNumRansacIterations,
      kInitializationRansacOrientationErrorThresholdRadians,
      kInitializationRansacPositionErrorThresholdMeters, ransac_seed,
      &T_G_M_lsq, &num_inliers);
  if (num_inliers < kNumInliersThreshold) {
    VLOG(1) << "Too few localization transformation inliers (" << num_inliers
            << "/" << T_G_M_lc_buffer_.size() << ").";
    return false;
  }

  const aslam::Transformation T_M_G_lsq = T_G_M_lsq.inverse();
  const Eigen::Vector3d WrWG = T_M_G_lsq.getPosition();
  const kindr::RotationQuaternionPD qWG(
      T_M_G_lsq.getRotation().toImplementation());

  rovio_interface_->resetLocalizationMapBaseframeAndCovariance(
      WrWG, qWG, FLAGS_rovioli_baseframe_init_position_covariance_msq,
      FLAGS_rovioli_baseframe_init_rotation_covariance_radsq);
  return true;
}

double getLocalizationResultGravityDisparityAngleDeg(
    const vio::LocalizationResult::ConstPtr& localization_result,
    const pose::Transformation& T_G_I_vio) {
  const pose::Transformation& T_G_I_lc_pnp = localization_result->T_G_I_lc_pnp;

  const Eigen::Vector3d gravity_direction_vio =
      T_G_I_vio.getRotation().inverse().rotate(Eigen::Vector3d::UnitZ());
  const Eigen::Vector3d gravity_direction_localization_pnp =
      T_G_I_lc_pnp.getRotation().inverse().rotate(Eigen::Vector3d::UnitZ());

  CHECK_NEAR(gravity_direction_vio.squaredNorm(), 1.0, 1e-8);
  CHECK_NEAR(gravity_direction_localization_pnp.squaredNorm(), 1.0, 1e-8);

  const double error_cosine =
      gravity_direction_vio.dot(gravity_direction_localization_pnp);

  double error_angle_degrees = 180.;
  if (error_cosine <= -1) {
    error_angle_degrees = 180.;
  } else if (error_cosine <= -1) {
    error_angle_degrees = 0.;
  } else {
    // Cosine is in the valid range.
    error_angle_degrees = std::acos(error_cosine) * kRadToDeg;
  }

  return error_angle_degrees;
}

bool RovioLocalizationHandler::processAsUpdate(
    const vio::LocalizationResult::ConstPtr& localization_result) {
  CHECK(localization_result != nullptr);

  const double rovio_timestamp_sec =
      time_translator_->convertMaplabToRovioTimestamp(
          localization_result->timestamp_ns);

  aslam::Transformation T_M_I_filter;
  const vio_common::PoseLookupBuffer::ResultStatus lookup_result =
      T_M_I_buffer_.getPoseAt(localization_result->timestamp_ns, &T_M_I_filter);
  CHECK(lookup_result != vio_common::PoseLookupBuffer::ResultStatus::kFailed);

  pose::Transformation T_G_M_filter;
  {
    std::lock_guard<std::mutex> lock(m_T_G_M_filter_buffer_);
    // Buffer cannot be empty as we must have received at least one filter
    // update.
    CHECK(!T_G_M_filter_buffer_.buffer().empty());
    T_G_M_filter = T_G_M_filter_buffer_.buffer().back();
  }

  const pose::Transformation T_G_I_filter = T_G_M_filter * T_M_I_filter;

  const double gravity_error_angle_deg =
      getLocalizationResultGravityDisparityAngleDeg(
          localization_result, T_M_I_filter);
  if (gravity_error_angle_deg >
      FLAGS_rovioli_localization_max_gravity_misalignment_deg) {
    LOG(WARNING) << "The gravity direction of the localization is not "
                 << "consistent with the VIO estimate. The disparity angle "
                 << "is " << gravity_error_angle_deg << "deg (threshold: "
                 << FLAGS_rovioli_localization_max_gravity_misalignment_deg
                 << "). Rejected the localization result.";
    return false;
  }

  bool measurement_accepted = true;
  if (FLAGS_rovioli_use_6dof_localization) {
    // ROVIO coordinate frames:
    //  - J: Inertial frame of pose update
    //  - V: Body frame of pose update sensor
    const Eigen::Vector3d JrJV =
        localization_result->T_G_I_lc_pnp.getPosition();
    const kindr::RotationQuaternionPD qJV(
        localization_result->T_G_I_lc_pnp.getRotation().toImplementation());
    measurement_accepted = rovio_interface_->processGroundTruthUpdate(
        JrJV, qJV, rovio_timestamp_sec);
  } else {
    // Check if there are any matches to be processed in the camera frames that
    // are used by ROVIO for estimation (inactive).
    const size_t num_cameras =
        localization_result->G_landmarks_per_camera.size();
    size_t num_valid_matches = 0u;
    for (size_t maplab_cam_idx = 0u; maplab_cam_idx < num_cameras;
         ++maplab_cam_idx) {
      const size_t* rovio_cam_idx =
          maplab_to_rovio_cam_indices_mapping_.getRight(maplab_cam_idx);
      if (rovio_cam_idx == nullptr) {
        continue;
      }
      num_valid_matches +=
          localization_result->G_landmarks_per_camera[maplab_cam_idx].cols();
    }
    if (num_valid_matches == 0u) {
      // There are no valid localization matches for the cameras used by ROVIO.
      // Use the localization result of the inactive cameras but integrate them
      // using the 6dof localization mode, since we cannot currently pass 2D-3D
      // correspondences to ROVIO for a camera it's not using.
      if (FLAGS_rovioli_use_6dof_localization_for_inactive_cameras) {
        // ROVIO coordinate frames:
        //  - J: Inertial frame of pose update
        //  - V: Body frame of pose update sensor
        const Eigen::Vector3d JrJV =
            localization_result->T_G_I_lc_pnp.getPosition();
        const kindr::RotationQuaternionPD qJV(
            localization_result->T_G_I_lc_pnp.getRotation().toImplementation());
        measurement_accepted = rovio_interface_->processGroundTruthUpdate(
            JrJV, qJV, rovio_timestamp_sec);

        VLOG_IF(1, measurement_accepted)
            << "No localization found for active camera, successfully updated "
            << "ROVIO using 6DoF constraints based on localization from "
            << "inactive cameras.";

        LOG_IF(
            WARNING, !measurement_accepted && rovio_interface_->isInitialized())
            << "No localization found for active camera, failed to update "
            << "ROVIO using 6DoF constraints based on localization from "
            << "inactive cameras, because ROVIO rejected the localization "
            << "update at time = " << localization_result->timestamp_ns
            << "ns. The latency was too large; consider reducing the "
            << "localization rate.";

        return measurement_accepted;
      }
      return false;
    }

    std::vector<double> lc_reprojection_errors;
    std::vector<double> filter_reprojection_errors;
    const double reprojection_success_rate = getLocalizationReprojectionErrors(
        *localization_result, T_G_I_filter, &lc_reprojection_errors,
        &filter_reprojection_errors);

    double mean_reprojection_error_diff = std::numeric_limits<double>::max();
    const double kMinReprojectionSuccessRate = 0.5;
    const int num_accepted_localization_constraints =
        lc_reprojection_errors.size();
    const bool reprojection_success =
        reprojection_success_rate > kMinReprojectionSuccessRate &&
        num_accepted_localization_constraints >
            FLAGS_rovioli_min_number_of_structure_constraints;

    if (reprojection_success) {
      const double lc_reproj_mean = aslam::common::mean(
          lc_reprojection_errors.begin(), lc_reprojection_errors.end());
      const double filter_reproj_mean = aslam::common::mean(
          filter_reprojection_errors.begin(), filter_reprojection_errors.end());
      mean_reprojection_error_diff =
          std::abs(filter_reproj_mean - lc_reproj_mean);
      VLOG(3) << "Localization reprojection error [px]: "
              << mean_reprojection_error_diff;
    }

    if (!reprojection_success ||
        mean_reprojection_error_diff >
            FLAGS_rovioli_max_mean_localization_reprojection_error_px) {
      if (reprojection_success) {
        LOG(WARNING)
            << "Mean reprojection error of localization matches, "
            << mean_reprojection_error_diff
            << ", is larger than the threshold ("
            << FLAGS_rovioli_max_mean_localization_reprojection_error_px
            << "). Will reset the localization.";
      } else {
        LOG(WARNING)
            << "Most of the localization matches cannot be reprojected into "
            << "the image plane. Will reset the localization.";
      }
    }

    for (size_t maplab_cam_idx = 0u; maplab_cam_idx < num_cameras;
         ++maplab_cam_idx) {
      const size_t* rovio_cam_idx =
          maplab_to_rovio_cam_indices_mapping_.getRight(maplab_cam_idx);

      if (rovio_cam_idx == nullptr) {
        // Skip this localization result, as the camera was marked as inactive.
        continue;
      }
      measurement_accepted &=
          rovio_interface_->processLocalizationLandmarkUpdates(
              *rovio_cam_idx,
              localization_result
                  ->keypoint_measurements_per_camera[maplab_cam_idx],
              localization_result->G_landmarks_per_camera[maplab_cam_idx],
              rovio_timestamp_sec);
    }
  }

  LOG_IF(WARNING, !measurement_accepted && rovio_interface_->isInitialized())
      << "ROVIO rejected localization update at time = "
      << localization_result->timestamp_ns << "ns. The latency was too large; "
      << "consider reducing the localization rate.";
  return measurement_accepted;
}

double RovioLocalizationHandler::getLocalizationReprojectionErrors(
    const vio::LocalizationResult& localization_result,
    const aslam::Transformation& T_G_I_filter,
    std::vector<double>* lc_reprojection_errors,
    std::vector<double>* filter_reprojection_errors) {
  CHECK_NOTNULL(lc_reprojection_errors)->clear();
  CHECK_NOTNULL(filter_reprojection_errors)->clear();

  CHECK_EQ(
      localization_result.G_landmarks_per_camera.size(),
      localization_result.keypoint_measurements_per_camera.size());

  int num_matches_processed = 0;

  const int num_cameras = localization_result.G_landmarks_per_camera.size();
  CHECK_EQ(num_cameras, static_cast<int>(camera_calibration_.numCameras()));
  for (int maplab_cam_idx = 0; maplab_cam_idx < num_cameras; ++maplab_cam_idx) {
    CHECK_EQ(
        localization_result.G_landmarks_per_camera[maplab_cam_idx].cols(),
        localization_result.keypoint_measurements_per_camera[maplab_cam_idx]
            .cols());

    const size_t* rovio_cam_idx =
        maplab_to_rovio_cam_indices_mapping_.getRight(maplab_cam_idx);

    const int num_matches =
        localization_result.G_landmarks_per_camera[maplab_cam_idx].cols();

    if (num_matches == 0) {
      continue;
    }

    if (rovio_cam_idx == nullptr) {
      // Skip this localization result, as the camera was marked as inactive.
      continue;
    }

    const pose::Transformation& T_C_B =
        camera_calibration_.get_T_C_B(maplab_cam_idx);
    const pose::Transformation T_G_C_filter =
        (T_C_B * T_G_I_filter.inverse()).inverse();
    const pose::Transformation T_G_C_lc =
        (T_C_B * localization_result.T_G_I_lc_pnp.inverse()).inverse();

    for (int i = 0; i < num_matches; ++i) {
      const Eigen::Vector2d& keypoint =
          localization_result.keypoint_measurements_per_camera[maplab_cam_idx]
              .col(i);
      const Eigen::Vector3d& p_G =
          localization_result.G_landmarks_per_camera[maplab_cam_idx].col(i);

      double reproj_error_sq_filter;
      double reproj_error_sq_lc;
      bool projection_successful = true;
      projection_successful &= getReprojectionErrorForGlobalLandmark(
          p_G, T_G_C_filter, camera_calibration_.getCamera(maplab_cam_idx),
          keypoint, &reproj_error_sq_filter);
      projection_successful &= getReprojectionErrorForGlobalLandmark(
          p_G, T_G_C_lc, camera_calibration_.getCamera(maplab_cam_idx),
          keypoint, &reproj_error_sq_lc);

      ++num_matches_processed;

      if (projection_successful) {
        lc_reprojection_errors->push_back(reproj_error_sq_filter);
        filter_reprojection_errors->push_back(reproj_error_sq_lc);
      }
    }
  }

  CHECK_EQ(lc_reprojection_errors->size(), filter_reprojection_errors->size());
  CHECK_GT(num_matches_processed, 0);
  return static_cast<double>(lc_reprojection_errors->size()) /
         num_matches_processed;
}

bool extractLocalizationFromRovioState(
    const rovio::RovioState& state, aslam::Transformation* T_G_M) {
  CHECK_NOTNULL(T_G_M);

  bool has_T_G_M = false;
  if (FLAGS_rovioli_use_6dof_localization) {
    if (state.getHasInertialPose()) {
      *T_G_M = aslam::Transformation(
          state.get_qWI().inverted().toImplementation(), state.get_IrIW());
      common::ensurePositiveQuaternion(&T_G_M->getRotation());
    }
    has_T_G_M = true;
  } else {
    if (state.getHasMapLocalizationPose()) {
      aslam::Transformation T_M_G = aslam::Transformation(
          state.get_qWG().toImplementation(), state.get_WrWG());
      *T_G_M = T_M_G.inverse();
      common::ensurePositiveQuaternion(&T_G_M->getRotation());
    }
    has_T_G_M = true;
  }
  return has_T_G_M;
}

}  //  namespace rovioli
