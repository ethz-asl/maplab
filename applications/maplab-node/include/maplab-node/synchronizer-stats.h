#ifndef MAPLAB_NODE_SYNCHRONIZER_STATS_H_
#define MAPLAB_NODE_SYNCHRONIZER_STATS_H_

#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <aslam/common/statistics/statistics.h>

namespace maplab {

struct SynchronizerStatistics {
  SynchronizerStatistics()
      : imu_latency_stats("IMU latency"),
        imu_num_measurements_per_msg_stats("IMU meas. per msg"),
        lidar_latency_stats("Lidar latency"),
        odom_latency_stats("Odom latency") {}

  inline void initializeCameraStats(const uint32_t num_cams) {
    for (uint32_t cam_idx = 0u; cam_idx < num_cams; ++cam_idx) {
      cam_latency_stats.emplace_back(
          "CAM" + std::to_string(cam_idx) + " latency");
    }
  }

  inline std::string printSensorMsgStats(
      const std::string& sensor_name,
      const statistics::StatsCollectorImpl& stats_handle,
      const int64_t latency_shift_ns) const {
    CHECK_GE(latency_shift_ns, 0);
    CHECK(!sensor_name.empty());
    if (statistics::Statistics::GetNumSamples(stats_handle.GetHandle()) > 0) {
      std::stringstream ss;

      const int64_t latency_ns =
          statistics::Statistics::GetMean(stats_handle.GetHandle()) -
          latency_shift_ns;
      const double hz = statistics::Statistics::GetHz(stats_handle.GetHandle());
      const double mean_delta_time_s =
          statistics::Statistics::GetMeanDeltaTime(stats_handle.GetHandle());
      const double min_delta_time_s =
          statistics::Statistics::GetMinDeltaTime(stats_handle.GetHandle());
      const double max_delta_time_s =
          statistics::Statistics::GetMaxDeltaTime(stats_handle.GetHandle());
      const double delta_time_variance_s =
          statistics::Statistics::GetVarianceDeltaTime(
              stats_handle.GetHandle());

      ss << " - " << sensor_name << "\t\t"
         << aslam::time::timeNanosecondsToString(latency_ns) << "\t" << hz
         << "Hz\t" << mean_delta_time_s << "s"
         << " (" << min_delta_time_s << "s"
         << " - " << max_delta_time_s << "s"
         << " - " << delta_time_variance_s << "s"
         << ")"
         << "\n";
      return ss.str();
    } else {
      return "";
    }
  }

  inline void getMinLatency(
      int64_t* min_latency_ns, std::string* min_latency_sensor) const {
    CHECK_NOTNULL(min_latency_ns);
    CHECK_NOTNULL(min_latency_sensor);

    *min_latency_ns = std::numeric_limits<int64_t>::max();
    *min_latency_sensor = "";

    if (statistics::Statistics::GetMin(imu_latency_stats.GetHandle()) <
        *min_latency_ns) {
      *min_latency_ns =
          statistics::Statistics::GetMin(imu_latency_stats.GetHandle());
      *min_latency_sensor = "IMU";
    }
    for (uint32_t cam_idx = 0u; cam_idx < cam_latency_stats.size(); ++cam_idx) {
      const statistics::StatsCollectorImpl& cam_latency_stats_elem =
          cam_latency_stats[cam_idx];
      if (statistics::Statistics::GetMin(cam_latency_stats_elem.GetHandle()) <
          *min_latency_ns) {
        *min_latency_ns =
            statistics::Statistics::GetMin(cam_latency_stats_elem.GetHandle());
        *min_latency_sensor = "CAM" + std::to_string(cam_idx);
      }
      ++cam_idx;
    }
    if (statistics::Statistics::GetMin(lidar_latency_stats.GetHandle()) <
        *min_latency_ns) {
      *min_latency_ns =
          statistics::Statistics::GetMin(lidar_latency_stats.GetHandle());
      *min_latency_sensor = "LIDAR";
    }
    if (statistics::Statistics::GetMin(odom_latency_stats.GetHandle()) <
        *min_latency_ns) {
      *min_latency_ns =
          statistics::Statistics::GetMin(odom_latency_stats.GetHandle());
      *min_latency_sensor = "ODOM";
    }
  }

  inline std::string print() const {
    std::stringstream ss;

    // Print general statistics
    ss << "\n[MaplabNode-Synchronizer] Statistics:\n"
       << statistics::Statistics::Print() << "\n";

    // Print min-shifted latency statistics.
    ss << "Message Timing/Sync Metrics:\n";
    ss << "============================";
    ss << "(mean latencies shifted by min latency of ";
    int64_t min_latency_ns;
    std::string min_latency_sensor;
    getMinLatency(&min_latency_ns, &min_latency_sensor);

    ss << min_latency_sensor << " -> "
       << aslam::time::timeNanosecondsToString(min_latency_ns) << ")\n";

    ss << "sensor\t\t| mean latency\t| Hz \t\t| mean delta t (min - max - "
          "var)\t|\n";
    for (uint32_t cam_idx = 0u; cam_idx < cam_latency_stats.size(); ++cam_idx) {
      const statistics::StatsCollectorImpl& cam_latency_stats_elem =
          cam_latency_stats[cam_idx];
      ss << printSensorMsgStats(
          "CAM" + std::to_string(cam_idx), cam_latency_stats_elem,
          min_latency_ns);
    }
    ss << printSensorMsgStats("IMU", imu_latency_stats, min_latency_ns);
    ss << printSensorMsgStats("LIDAR", lidar_latency_stats, min_latency_ns);
    ss << printSensorMsgStats("ODOM", odom_latency_stats, min_latency_ns);

    return ss.str();
  }

  statistics::StatsCollectorImpl imu_latency_stats;
  statistics::StatsCollectorImpl imu_num_measurements_per_msg_stats;
  statistics::StatsCollectorImpl lidar_latency_stats;
  std::vector<statistics::StatsCollectorImpl> cam_latency_stats;
  statistics::StatsCollectorImpl odom_latency_stats;
};

}  // namespace maplab

#endif  // MAPLAB_NODE_SYNCHRONIZER_STATS_H_
