#ifndef MAPLAB_NODE_MAP_UPDATE_BUILDER_H_
#define MAPLAB_NODE_MAP_UPDATE_BUILDER_H_

#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <utility>

#include <aslam/common/memory.h>
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

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  // Already threadsafe.
  const vio_common::PoseLookupBuffer& T_M_B_buffer_;

  std::mutex process_nframe_mutex_;
  // Enforce that the timestamps are strictly monotonically increasing
  std::atomic<int64_t> last_received_timestamp_tracked_nframe_;

  MapUpdatePublishFunction map_update_publish_function_;
};

}  // namespace maplab

#endif  // MAPLAB_NODE_MAP_UPDATE_BUILDER_H_
