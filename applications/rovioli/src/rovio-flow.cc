#include "rovioli/rovio-flow.h"

#include <memory>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <aslam/cameras/ncamera.h>
#include <aslam/common/pose-types.h>
#include <aslam/common/time.h>
#include <gflags/gflags.h>
#include <maplab-common/fixed-size-queue.h>
#include <maplab-common/geometry.h>
#include <maplab-common/string-tools.h>
#include <maplab-common/unique-id.h>
#include <message-flow/message-flow.h>
#include <vio-common/pose-lookup-buffer.h>
#include <vio-common/vio-types.h>

#include "rovioli/flow-topics.h"
#include "rovioli/rovio-factory.h"
#include "rovioli/rovio-health-monitor.h"
#include "rovioli/rovio-localization-handler.h"
#include "rovioli/rovio-maplab-timetranslation.h"

DEFINE_bool(
    rovio_update_filter_on_imu, true,
    "Update the filter state for IMU measurement; if false the IMU measurements"
    " are queued and the state is only forward propagated before the next "
    "update.");
DEFINE_string(
    rovio_active_camera_indices, "0",
    "Comma separated indices of cameras to use for motion tracking.");
DEFINE_bool(
    rovioli_enable_health_checking, false,
    "Perform health checking on the estimator output and reset if necessary.");

namespace rovioli {
RovioFlow::RovioFlow(
    const aslam::NCamera& camera_calibration,
    const vi_map::ImuSigmas& imu_sigmas) {
  // Multi-camera support in ROVIO is still experimental. Therefore, only a
  // single camera will be used for motion tracking per default.
  const size_t num_cameras = camera_calibration.getNumCameras();
  LOG_IF(WARNING, num_cameras > 1u)
      << "Multi-camera support of ROVIO is still experimental. Per default "
      << "only the first camera will be used for motion tracking. However, all "
      << "cameras will be used for mapping and localization. You can override  "
      << "the active used for motion tracking with the flag: "
      << "--rovio_active_camera_indices";

  CHECK(!FLAGS_rovio_active_camera_indices.empty());
  constexpr char kDelimiter = ',';
  constexpr bool kRemoveEmpty = true;
  std::vector<std::string> tokens;
  common::tokenizeString(
      FLAGS_rovio_active_camera_indices, kDelimiter, kRemoveEmpty, &tokens);
  LOG_IF(WARNING, tokens.size() > 1u)
      << "Selected more than one camera for motion tracking. Consider only "
      << "using a single camera if latency issues develop.";

  // Build NCamera of active cameras.
  int rovio_camera_index = 0;
  std::vector<aslam::Camera::Ptr> active_cameras;
  aslam::TransformationVector active_T_C_Bs;
  for (const std::string& camera_id_str : tokens) {
    const int maplab_camera_idx = std::stoi(camera_id_str);
    CHECK_GE(maplab_camera_idx, 0);
    CHECK_LT(maplab_camera_idx, static_cast<int>(num_cameras));
    CHECK(
        maplab_to_rovio_cam_indices_mapping_.insert(
            maplab_camera_idx, rovio_camera_index))
        << "--rovio_active_camera_indices contains duplicates.";

    active_cameras.emplace_back(
        camera_calibration.getCameraShared(maplab_camera_idx)->clone());
    active_T_C_Bs.emplace_back(camera_calibration.get_T_C_B(maplab_camera_idx));
    ++rovio_camera_index;
  }
  CHECK_EQ(static_cast<int>(active_cameras.size()), rovio_camera_index);
  CHECK_EQ(static_cast<int>(active_T_C_Bs.size()), rovio_camera_index);

  aslam::NCameraId id;
  common::generateId<aslam::NCameraId>(&id);
  aslam::NCamera motion_tracking_ncamera(
      id, active_T_C_Bs, active_cameras, "Cameras active for motion tracking.");

  // Construct ROVIO interface using only the active cameras.
  rovio_interface_.reset(
      constructAndConfigureRovio(motion_tracking_ncamera, imu_sigmas));
  rovio_interface_->setEnablePatchUpdateOutput(false);
  rovio_interface_->setEnableFeatureUpdateOutput(true);  // For health checking.
  localization_handler_.reset(new RovioLocalizationHandler(
      rovio_interface_.get(), &time_translation_, camera_calibration,
      maplab_to_rovio_cam_indices_mapping_));
}

RovioFlow::~RovioFlow() {}

void RovioFlow::attachToMessageFlow(message_flow::MessageFlow* flow) {
  CHECK_NOTNULL(flow);
  static constexpr char kSubscriberNodeName[] = "RovioFlow";

  // All data input subscribers are put in an exclusivity group such that the
  // delivery ordering for all messages (cam, imu, localization) are
  // corresponding to the publishing order and no sensor can be left behind.
  message_flow::DeliveryOptions rovio_subscriber_options;
  rovio_subscriber_options.exclusivity_group_id =
      kExclusivityGroupIdRovioSensorSubscribers;

  // Input IMU.
  flow->registerSubscriber<message_flow_topics::IMU_MEASUREMENTS>(
      kSubscriberNodeName, rovio_subscriber_options,
      [this](const vio::ImuMeasurement::ConstPtr& imu) {
        // Do not apply the predictions but only queue them. They will be
        // applied before the next update.
        const double rovio_timestamp_sec =
            time_translation_.convertMaplabToRovioTimestamp(imu->timestamp);
        const bool measurement_accepted =
            this->rovio_interface_->processImuUpdate(
                imu->imu_data.head<3>(), imu->imu_data.tail<3>(),
                rovio_timestamp_sec, FLAGS_rovio_update_filter_on_imu);
        LOG_IF(
            WARNING, !measurement_accepted && rovio_interface_->isInitialized())
            << "ROVIO rejected IMU measurement. Latency is too large.";

        localization_handler_->T_M_I_buffer_mutable()->bufferImuMeasurement(
            *imu);
      });
  // Input camera.
  flow->registerSubscriber<message_flow_topics::IMAGE_MEASUREMENTS>(
      kSubscriberNodeName, rovio_subscriber_options,
      [this](const vio::ImageMeasurement::ConstPtr& image) {
        const size_t maplab_cam_idx = image->camera_index;
        const size_t* rovio_cam_index =
            maplab_to_rovio_cam_indices_mapping_.getRight(maplab_cam_idx);
        if (rovio_cam_index == nullptr) {
          // Skip this image, as the camera was marked as inactive.
          return;
        }
        const double rovio_timestamp_sec =
            time_translation_.convertMaplabToRovioTimestamp(image->timestamp);
        const bool measurement_accepted =
            this->rovio_interface_->processImageUpdate(
                *rovio_cam_index, image->image, rovio_timestamp_sec);
        LOG_IF(
            WARNING, !measurement_accepted && rovio_interface_->isInitialized())
            << "ROVIO rejected image measurement. Latency is too large.";
      });
  // Input localization updates.
  flow->registerSubscriber<message_flow_topics::LOCALIZATION_RESULT>(
      kSubscriberNodeName, rovio_subscriber_options,
      std::bind(
          &RovioLocalizationHandler::processLocalizationResult,
          localization_handler_.get(), std::placeholders::_1));

  // Output ROVIO estimates.
  publish_rovio_estimates_ =
      flow->registerPublisher<message_flow_topics::ROVIO_ESTIMATES>();
  CHECK(rovio_interface_);
  rovio_interface_->registerStateUpdateCallback(
      std::bind(
          &RovioFlow::processAndPublishRovioUpdate, this,
          std::placeholders::_1));
}

void RovioFlow::processAndPublishRovioUpdate(const rovio::RovioState& state) {
  if (!state.getIsInitialized()) {
    LOG(WARNING) << "ROVIO not yet initialized. Discarding state update.";
    return;
  }

  if (FLAGS_rovioli_enable_health_checking &&
      health_monitor_.shouldResetEstimator(state)) {
    health_monitor_.resetRovioToLastHealthyPose(rovio_interface_.get());
    return;
  }

  // ROVIO coordinate frames:
  //  - I: Inertial frame of pose update
  //  - V: Body frame of pose update sensor
  //  - W: Inertial frame of odometry
  //  - B: IMU-coordinate frame
  // ROVIO and maplab both use passive Hamilton quaternion convention; no
  // conversion is necessary.
  aslam::Transformation T_M_I(
      state.get_qBW().inverted().toImplementation(), state.get_WrWB());
  common::ensurePositiveQuaternion(&T_M_I.getRotation());
  const Eigen::Vector3d v_M = T_M_I.getRotation().rotate(state.get_BvB());

  RovioEstimate::Ptr rovio_estimate(new RovioEstimate);
  // VIO states.
  const int64_t timestamp_ns =
      time_translation_.convertRovioToMaplabTimestamp(state.getTimestamp());
  rovio_estimate->timestamp_ns = timestamp_ns;
  rovio_estimate->vinode.setTimestamp(timestamp_ns);
  rovio_estimate->vinode.set_T_M_I(T_M_I);
  rovio_estimate->vinode.set_v_M_I(v_M);
  rovio_estimate->vinode.setAccBias(state.getAcb());
  rovio_estimate->vinode.setGyroBias(state.getGyb());

  // Camera extrinsics.
  for (size_t rovio_cam_idx = 0u; rovio_cam_idx < state.numCameras();
       ++rovio_cam_idx) {
    const size_t* maplab_cam_idx =
        maplab_to_rovio_cam_indices_mapping_.getLeft(rovio_cam_idx);
    CHECK_NOTNULL(maplab_cam_idx);

    aslam::Transformation T_B_C(
        state.get_qCM(rovio_cam_idx).inverted().toImplementation(),
        state.get_MrMC(rovio_cam_idx));
    common::ensurePositiveQuaternion(&T_B_C.getRotation());
    CHECK(
        rovio_estimate->maplab_camera_index_to_T_C_B
            .emplace(*maplab_cam_idx, T_B_C.inverse())
            .second);
  }

  // Optional localizations.
  rovio_estimate->has_T_G_M =
      extractLocalizationFromRovioState(state, &rovio_estimate->T_G_M);
  localization_handler_->T_M_I_buffer_mutable()->bufferRovioEstimate(
      rovio_estimate->vinode);
  if (rovio_estimate->has_T_G_M) {
    localization_handler_->buffer_T_G_M(rovio_estimate->T_G_M);
  }

  publish_rovio_estimates_(rovio_estimate);
}
}  // namespace rovioli
