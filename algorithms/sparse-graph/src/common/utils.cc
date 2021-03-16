#include "sparse-graph/common/utils.h"

#include <glog/logging.h>

namespace spg {

ros::Time Utils::CreateRosTimestamp(const int64_t ts_ns) {
  CHECK_GE(ts_ns, 0);
  static constexpr uint32_t kNanosecondsPerSecond = 1e9;
  const uint64_t timestamp_u64 = static_cast<uint64_t>(ts_ns);
  const uint32_t ros_timestamp_sec = timestamp_u64 / kNanosecondsPerSecond;
  const uint32_t ros_timestamp_nsec =
      timestamp_u64 - (ros_timestamp_sec * kNanosecondsPerSecond);
  return ros::Time(ros_timestamp_sec, ros_timestamp_nsec);
}

}  // namespace spg
