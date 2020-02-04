#ifndef MAPLAB_NODE_FEATURE_TRACKING_FLOW_H_
#define MAPLAB_NODE_FEATURE_TRACKING_FLOW_H_

#include <aslam/cameras/ncamera.h>
#include <message-flow/message-flow.h>
#include <sensors/imu.h>
#include <vio-common/pose-lookup-buffer.h>
#include <vio-common/vio-types.h>

#include "maplab-node/feature-tracking.h"
#include "maplab-node/flow-topics.h"
#include "maplab-node/odometry-estimate.h"

namespace maplab {

class FeatureTrackingFlow {
 public:
  FeatureTrackingFlow(
      const aslam::NCamera::Ptr& camera_system,
      const vio_common::PoseLookupBuffer& T_M_B_buffer)
      : tracking_pipeline_(camera_system, T_M_B_buffer) {
    CHECK(camera_system);
  }

  void attachToMessageFlow(message_flow::MessageFlow* flow) {
    VLOG(3) << "FeatureTrackingFlow::attachToMessageFlow";
    CHECK_NOTNULL(flow);
    static constexpr char kSubscriberNodeName[] = "FeatureTracking";

    std::function<void(vio::SynchronizedNFrame::ConstPtr)> publish_result =
        flow->registerPublisher<message_flow_topics::TRACKED_NFRAMES>();

    // NOTE: the publisher function pointer is copied intentionally; otherwise
    // we would capture a reference to a temporary.
    flow->registerSubscriber<message_flow_topics::SYNCED_NFRAMES>(
        kSubscriberNodeName, message_flow::DeliveryOptions(),
        [publish_result, this](const vio::SynchronizedNFrame::Ptr& nframe_imu) {
          CHECK(nframe_imu);
          const bool success =
              this->tracking_pipeline_.trackSynchronizedNFrameCallback(
                  nframe_imu);
          if (success) {
            // This will only fail for the first frame.
            publish_result(nframe_imu);
          }
        });
  }

 private:
  FeatureTracking tracking_pipeline_;
};

}  // namespace maplab

#endif  // MAPLAB_NODE_FEATURE_TRACKING_FLOW_H_
