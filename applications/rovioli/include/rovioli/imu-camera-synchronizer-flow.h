#ifndef ROVIOLI_IMU_CAMERA_SYNCHRONIZER_FLOW_H_
#define ROVIOLI_IMU_CAMERA_SYNCHRONIZER_FLOW_H_

#include <aslam/cameras/ncamera.h>
#include <message-flow/message-flow.h>
#include <sensors/imu.h>
#include <vio-common/vio-types.h>

#include "rovioli/flow-topics.h"
#include "rovioli/imu-camera-synchronizer.h"

namespace rovioli {

class ImuCameraSynchronizerFlow {
 public:
  explicit ImuCameraSynchronizerFlow(const aslam::NCamera::Ptr& camera_system)
      : synchronizing_pipeline_(camera_system) {
    CHECK(camera_system);
  }

  ~ImuCameraSynchronizerFlow() {
    shutdown();
  }

  void attachToMessageFlow(message_flow::MessageFlow* flow) {
    CHECK_NOTNULL(flow);
    static constexpr char kSubscriberNodeName[] = "ImuCameraSynchronizerFlow";

    // Image input.
    flow->registerSubscriber<message_flow_topics::IMAGE_MEASUREMENTS>(
        kSubscriberNodeName, message_flow::DeliveryOptions(),
        [this](const vio::ImageMeasurement::Ptr& image) {
          CHECK(image);
          this->synchronizing_pipeline_.addCameraImage(
              image->camera_index, image->image, image->timestamp);
        });
    // IMU input.
    flow->registerSubscriber<message_flow_topics::IMU_MEASUREMENTS>(
        kSubscriberNodeName, message_flow::DeliveryOptions(),
        [this](const vio::ImuMeasurement::Ptr& imu) {
          CHECK(imu);
          // TODO(schneith): This seems inefficient. Should we batch IMU
          // measurements on the datasource side?
          this->synchronizing_pipeline_.addImuMeasurements(
              (Eigen::Matrix<int64_t, 1, 1>() << imu->timestamp).finished(),
              imu->imu_data);
        });

    // Tracked nframes and IMU output.
    synchronizing_pipeline_.registerSynchronizedNFrameImuCallback(
        flow->registerPublisher<message_flow_topics::SYNCED_NFRAMES_AND_IMU>());
  }

  void shutdown() {
    synchronizing_pipeline_.shutdown();
  }

 private:
  ImuCameraSynchronizer synchronizing_pipeline_;
};

}  // namespace rovioli

#endif  // ROVIOLI_IMU_CAMERA_SYNCHRONIZER_FLOW_H_
