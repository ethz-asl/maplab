#ifndef ROVIOLI_VIO_UPDATE_BUILDER_H_
#define ROVIOLI_VIO_UPDATE_BUILDER_H_

#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <utility>

#include <aslam/common/memory.h>
#include <vio-common/vio-types.h>
#include <vio-common/vio-update.h>

#include "rovioli/rovio-estimate.h"

namespace rovioli {

class VioUpdateBuilder {
 public:
  typedef std::function<void(const vio::VioUpdate::ConstPtr&)>
      VioUpdatePublishFunction;

  VioUpdateBuilder();

  void registerVioUpdatePublishFunction(
      const VioUpdatePublishFunction& vio_update_publish_function) {
    vio_update_publish_function_ = vio_update_publish_function;
  }

  void processSynchronizedNFrameImu(
      const vio::SynchronizedNFrameImu::ConstPtr& synced_nframe_imu);
  void processRovioEstimate(const RovioEstimate::ConstPtr& rovio_estimate);
  void processLocalizationResult(
      const vio::LocalizationResult::ConstPtr& localization_result);

  void clearSynchronizedNFrameImuQueue() {
    // For unit tests.
    SynchronizedNFrameImuQueue empty_queue;
    synced_nframe_imu_queue_.swap(empty_queue);
  }

 private:
  typedef std::queue<vio::SynchronizedNFrameImu::ConstPtr>
      SynchronizedNFrameImuQueue;
  typedef Aligned<std::deque, RovioEstimate::ConstPtr> RovioEstimateQueue;

  void findMatchAndPublish();
  void interpolateViNodeState(
      const int64_t timestamp_ns_a, const vio::ViNodeState& vi_node_a,
      const int64_t timestamp_ns_b, const vio::ViNodeState& vi_node_b,
      const int64_t timestamp_ns_interpolated,
      vio::ViNodeState* vi_node_interpolated);

  std::recursive_mutex queue_mutex_;
  SynchronizedNFrameImuQueue synced_nframe_imu_queue_;
  RovioEstimateQueue rovio_estimate_queue_;
  // These values indicate the timestamp of the last message in the given topic
  // so that we can enforce that the timestamps are strictly monotonically
  // increasing
  int64_t last_received_timestamp_synced_nframe_queue_;
  double last_received_timestamp_rovio_estimate_queue;

  VioUpdatePublishFunction vio_update_publish_function_;

  std::mutex mutex_last_localization_state_;
  vio::LocalizationState last_localization_state_;
};

}  // namespace rovioli

#endif  // ROVIOLI_VIO_UPDATE_BUILDER_H_
