#ifndef ROVIOLI_FEATURE_TRACKING_FLOW_H_
#define ROVIOLI_FEATURE_TRACKING_FLOW_H_

#include <aslam/cameras/ncamera.h>
#include <feature-tracking-pipelines/feature-tracker-factory.h>
#include <feature-tracking-pipelines/feature-tracker-gyro-aided.h>
#include <memory>
#include <message-flow/message-flow.h>
#include <sensors/imu.h>
#include <vio-common/vio-types.h>

#include "rovioli/feature-tracking.h"
#include "rovioli/flow-topics.h"

namespace rovioli {

class FeatureTrackingFlow {
 public:
  FeatureTrackingFlow(
      const aslam::NCamera::Ptr& camera_system, const vi_map::Imu& imu_sensor)
      : tracking_pipeline_(camera_system, imu_sensor) {
    CHECK(camera_system);
    tracking_pipeline_new_.reset(
        new feature_tracking_pipelines::FeatureTrackerGyroAided(
            *camera_system,
            feature_tracking_pipelines::CreateFeaturePipelineFromGFlags()));
  }

  void attachToMessageFlow(message_flow::MessageFlow* flow) {
    CHECK_NOTNULL(flow);
    static constexpr char kSubscriberNodeName[] = "FeatureTrackingFlow";

    std::function<void(vio::SynchronizedNFrameImu::ConstPtr)> publish_result =
        flow->registerPublisher<message_flow_topics::TRACKED_NFRAMES_AND_IMU>();

    // NOTE: the publisher function pointer is copied intentionally; otherwise
    // we would capture a reference to a temporary.
    flow->registerSubscriber<message_flow_topics::SYNCED_NFRAMES_AND_IMU>(
        kSubscriberNodeName, message_flow::DeliveryOptions(),
        [publish_result,
         this](const vio::SynchronizedNFrameImu::Ptr& nframe_imu) {
          CHECK(nframe_imu);
          /*
          const bool success =
              this->tracking_pipeline_.trackSynchronizedNFrameImuCallback(
                  nframe_imu);
              */
          const bool success = tracking_pipeline_new_->processNFrame(
              nframe_imu->imu_timestamps, nframe_imu->imu_measurements,
              nframe_imu->nframe.get());
          if (success) {
            // This will only fail for the first frame.
            publish_result(nframe_imu);
          }
        });

    flow->registerSubscriber<message_flow_topics::ROVIO_ESTIMATES>(
        kSubscriberNodeName, message_flow::DeliveryOptions(),
        [this](const RovioEstimate::ConstPtr& estimate) {
          CHECK(estimate);
          // this->tracking_pipeline_.setCurrentImuBias(estimate);
          tracking_pipeline_new_->setCurrentImuBias(
              estimate->timestamp_ns, estimate->vinode.getImuBias());
        });
  }

 private:
  FeatureTracking tracking_pipeline_;
  std::unique_ptr<feature_tracking_pipelines::FeatureTrackerGyroAided>
      tracking_pipeline_new_;
};

}  // namespace rovioli

#endif  // ROVIOLI_FEATURE_TRACKING_FLOW_H_
