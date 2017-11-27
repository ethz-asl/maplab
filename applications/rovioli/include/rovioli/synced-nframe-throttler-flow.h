#ifndef ROVIOLI_SYNCED_NFRAME_THROTTLER_FLOW_H_
#define ROVIOLI_SYNCED_NFRAME_THROTTLER_FLOW_H_

#include <aslam/cameras/ncamera.h>
#include <message-flow/message-flow.h>
#include <vio-common/vio-types.h>

#include "rovioli/flow-topics.h"
#include "rovioli/synced-nframe-throttler.h"

namespace rovioli {

class SyncedNFrameThrottlerFlow {
 public:
  void attachToMessageFlow(message_flow::MessageFlow* flow) {
    CHECK_NOTNULL(flow);
    static constexpr char kSubscriberNodeName[] = "SyncedNFrameThrottlerFlow";

    std::function<void(vio::SynchronizedNFrameImu::ConstPtr)> publish_result =
        flow->registerPublisher<
            message_flow_topics::THROTTLED_TRACKED_NFRAMES_AND_IMU>();

    // NOTE: the publisher function pointer is copied intentionally; otherwise
    // we would capture a reference to a temporary.
    flow->registerSubscriber<message_flow_topics::TRACKED_NFRAMES_AND_IMU>(
        kSubscriberNodeName, message_flow::DeliveryOptions(),
        [publish_result,
         this](const vio::SynchronizedNFrameImu::ConstPtr& nframe_imu) {
          CHECK(nframe_imu);
          const bool should_publish =
              this->throttler_.shouldPublishNFrame(nframe_imu);
          if (should_publish) {
            publish_result(nframe_imu);
          }
        });
  }

 private:
  SyncedNFrameThrottler throttler_;
};

}  // namespace rovioli

#endif  // ROVIOLI_SYNCED_NFRAME_THROTTLER_FLOW_H_
