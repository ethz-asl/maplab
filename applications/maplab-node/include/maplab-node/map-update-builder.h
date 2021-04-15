#ifndef MAPLAB_NODE_MAP_UPDATE_BUILDER_H_
#define MAPLAB_NODE_MAP_UPDATE_BUILDER_H_

#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <utility>

#include <aslam/common/memory.h>
#include <maplab-common/localization-result.h>
#include <vio-common/map-update.h>
#include <vio-common/pose-lookup-buffer.h>
#include <vio-common/vio-types.h>
#include <vio-common/vio-update.h>
#include "maplab-node/odometry-estimate.h"

namespace maplab {

class MapUpdateBuilder {
 public:
  typedef std::function<void(const vio::MapUpdate::ConstPtr&)>
      MapUpdatePublishFunction;

  explicit MapUpdateBuilder(const vio_common::PoseLookupBuffer& T_M_B_buffer);

  void registerMapUpdatePublishFunction(
      const MapUpdatePublishFunction& map_update_publish_function) {
    map_update_publish_function_ = map_update_publish_function;
  }

  void processTrackedNFrame(
      const vio::SynchronizedNFrame::ConstPtr& tracked_nframe);
  void processLocalizationResult(
      const common::LocalizationResult::ConstPtr& localization_result);

  void clearSynchronizedNFrameImuQueue() {
    // For unit tests.
    TrackedNFrameQueue empty_queue;
    tracked_nframe_queue_.swap(empty_queue);
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  typedef std::queue<vio::SynchronizedNFrame::ConstPtr> TrackedNFrameQueue;
  typedef Aligned<std::deque, OdometryEstimate::ConstPtr> OdometryEstimateQueue;

  void findMatchAndPublish();
  void interpolateViNodeState(
      const int64_t timestamp_ns_a, const vio::ViNodeState& vi_node_a,
      const int64_t timestamp_ns_b, const vio::ViNodeState& vi_node_b,
      const int64_t timestamp_ns_interpolated,
      vio::ViNodeState* vi_node_interpolated);

  // Already threadsafe.
  const vio_common::PoseLookupBuffer& T_M_B_buffer_;

  std::mutex localization_buffer_mutex_;
  common::TemporalBuffer<common::FusedLocalizationResult> localization_buffer_;

  std::recursive_mutex queue_mutex_;
  TrackedNFrameQueue tracked_nframe_queue_;
  // These values indicate the timestamp of the last message in the given topic
  // so that we can enforce that the timestamps are strictly monotonically
  // increasing
  std::atomic<int64_t> last_received_timestamp_tracked_nframe_queue_;

  MapUpdatePublishFunction map_update_publish_function_;

  std::mutex mutex_last_localization_state_;
  common::LocalizationState last_localization_state_;
};

}  // namespace maplab

#endif  // MAPLAB_NODE_MAP_UPDATE_BUILDER_H_
