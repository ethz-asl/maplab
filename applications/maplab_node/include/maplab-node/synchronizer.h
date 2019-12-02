#ifndef MAPLAB_NODE_SYNCHRONIZER_H_
#define MAPLAB_NODE_SYNCHRONIZER_H_

#include <atomic>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <aslam/cameras/ncamera.h>
#include <aslam/common/statistics/statistics.h>
#include <aslam/pipeline/visual-npipeline.h>
#include <opencv2/core/core.hpp>
#include <sensors/absolute-6dof-pose.h>
#include <sensors/odometry-6dof-pose.h>
#include <sensors/pointcloud-map-sensor.h>
#include <sensors/wheel-odometry-sensor.h>
#include <vi-map/sensor-manager.h>
#include <vio-common/imu-measurements-buffer.h>
#include <vio-common/pose-lookup-buffer.h>
#include <vio-common/vio-types.h>
#include <vio-common/vio-update.h>

#include "maplab-node/synchronizer-stats.h"

namespace maplab {

class Synchronizer {
 public:
  MAPLAB_POINTER_TYPEDEFS(Synchronizer);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Synchronizer() = delete;

  explicit Synchronizer(const vi_map::SensorManager& sensor_manager);

  ~Synchronizer();

  void initializeNCameraSynchronization(
      const aslam::NCamera::Ptr& camera_system);

  void processImuMeasurements(
      const Eigen::Matrix<int64_t, 1, Eigen::Dynamic>& timestamps_nanoseconds,
      const Eigen::Matrix<double, 6, Eigen::Dynamic>& imu_measurements);
  void processCameraImage(
      const size_t camera_index, const cv::Mat& image, const int64_t timestamp);
  void processLidarMeasurement(
      const vi_map::RosLidarMeasurement::ConstPtr& lidar_measurement);
  void processOdometryMeasurement(const vio::ViNodeState& odometry);
  void processAbsolute6DoFMeasurement(
      const vi_map::Absolute6DoFMeasurement::Ptr& absolute_6dof_measurement);
  void processLoopClosureMeasurement(
      const vi_map::LoopClosureMeasurement::ConstPtr&
          loop_closure_measurement);
  void processWheelOdometryMeasurement(
      const vi_map::WheelOdometryMeasurement::ConstPtr&
          wheel_odometry_measurement);
  void processPointCloudMapMeasurement(
      const vi_map::RosPointCloudMapSensorMeasurement::ConstPtr&
          pointcloud_map);

  // These functions make sure the synchronizer is aware that data of this type
  // is incomming and it will raise a warning if no data is received for more
  // than 5s.
  void expectOdometryData();
  void expectLidarData();
  void expectVisualData();
  void expectImuData();
  void expectAbsolute6DoFData();
  void expectWheelOdometryData();
  void expectLoopClosureData();
  void expectPointCloudMapData();

  // Release the buffered data based on the availability of the odometry pose.
  void releaseData();
  void releaseNFrameData(
      const int64_t oldest_timestamp_ns, const int64_t newest_timestamp_ns);
  void releaseLidarData(
      const int64_t oldest_timestamp_ns, const int64_t newest_timestamp_ns);
  void releaseAbsolute6DoFData(
      const int64_t oldest_timestamp_ns, const int64_t newest_timestamp_ns);
  void releaseWheelOdometryData(
      const int64_t oldest_timestamp_ns, const int64_t newest_timestamp_ns);
  void releaseLoopClosureData(
      const int64_t oldest_timestamp_ns, const int64_t newest_timestamp_ns);
  void releasePointCloudMapData(
      const int64_t oldest_timestamp_ns, const int64_t newest_timestamp_ns);

  void registerSynchronizedNFrameCallback(
      const std::function<void(const vio::SynchronizedNFrame::Ptr&)>& callback);

  void registerLidarMeasurementCallback(
      const std::function<void(const vi_map::RosLidarMeasurement::ConstPtr&)>&
          callback);

  void registerAbsolute6DoFMeasurementCallback(
      const std::function<void(const vi_map::Absolute6DoFMeasurement::Ptr&)>&
          callback);

  void registerLoopClosureMeasurementCallback(
      const std::function<
          void(const vi_map::LoopClosureMeasurement::ConstPtr&)>& callback);

  void registerWheelOdometryMeasurementCallback(
      const std::function<
          void(const vi_map::WheelOdometryMeasurement::ConstPtr&)>& callback);

  void registerLocalizationResultMeasurementCallback(
      const std::function<void(const common::LocalizationResult::ConstPtr&)>&
          callback);

  void registerPointCloudMapSensorMeasurementCallback(
      const std::function<
          void(const vi_map::RosPointCloudMapSensorMeasurement::ConstPtr&)>&
          callback);

  void start();
  void shutdown();

  vio_common::PoseLookupBuffer& T_M_B_buffer() {
    return T_M_B_buffer_;
  }

  inline void registerEndOfDataCallback(const std::function<void()>& cb) {
    CHECK(cb);
    end_of_data_callbacks_.emplace_back(cb);
  }

  inline void invokeEndOfDataCallbacks() const {
    for (const std::function<void()>& cb : end_of_data_callbacks_) {
      cb();
    }
  }

 private:
  void checkIfMessagesAreIncomingWorker();

  std::vector<std::function<void()>> end_of_data_callbacks_;

  const vi_map::SensorManager& sensor_manager_;

  // Buffer to store incomming odometry estimates, imu biases and raw imu
  // measurements. This Buffer is directly linked and modified in several other
  // components of the maplab node.
  vio_common::PoseLookupBuffer T_M_B_buffer_;

  // Pipeline to combine images into NFrames.
  aslam::VisualNPipeline::UniquePtr visual_pipeline_;
  // Buffer to store the assembled NFrames before they are released.
  mutable std::mutex nframe_buffer_mutex_;
  common::TemporalBuffer<aslam::VisualNFrame::Ptr> nframe_buffer_;

  // Buffer to store the lidar measurements before they are released.
  mutable std::mutex lidar_buffer_mutex_;
  common::TemporalBuffer<vi_map::RosLidarMeasurement::ConstPtr> lidar_buffer_;

  // Buffer to store the absolute 6DoF measurements before they are released.
  mutable std::mutex absolute_6dof_buffer_mutex_;
  common::TemporalBuffer<vi_map::Absolute6DoFMeasurement::Ptr>
      absolute_6dof_buffer_;

  // Buffer to store the loop measurements before they are released.
  mutable std::mutex loop_closure_buffer_mutex_;
  vi_map::LoopClosureTemporalMap loop_closure_buffer_;

  // Buffer to store the wheel odometry measurements before they are released.
  mutable std::mutex wheel_odometry_buffer_mutex_;
  common::TemporalBuffer<vi_map::WheelOdometryMeasurement::ConstPtr>
      wheel_odometry_buffer_;

  // Buffer to store the point cloud map measurements before they are released.
  mutable std::mutex pointcloud_map_buffer_mutex_;
  common::TemporalBuffer<vi_map::RosPointCloudMapSensorMeasurement::ConstPtr>
      pointcloud_map_buffer_;

  // Number of already skipped frames.
  size_t frame_skip_counter_;
  // Timestamp of previously released NFrame, used to throttle the NFrames.
  int64_t previous_nframe_timestamp_ns_;
  // Threshold used to throttle the consecutively published NFrames.
  const int64_t min_nframe_timestamp_diff_ns_;

  // Number of received odometry measurements.
  int64_t odometry_measurement_counter_;

  // Number of already skipped lidar measurements.
  size_t lidar_skip_counter_;
  // Number of already skipped absolute 6DoF measurements.
  size_t absolute_6dof_skip_counter_;
  // Number of already skipped loop closure measurements.
  size_t loop_closure_skip_counter_;
  // Number of already skipped pointcloud map measurements.
  size_t pointcloud_map_skip_counter_;

  std::atomic<bool> shutdown_;
  std::condition_variable cv_shutdown_;
  std::thread check_if_messages_are_incoming_thread_;
  std::mutex mutex_check_if_messages_are_incoming_;

  // Indicates the timestamp when either the last message was received or the
  // check that messages are (still) incoming was performed last (whichever
  // happend most recently). The timers are initialized to -1, in case we do not
  // subscribe to that topic, no warnings will be shown.
  std::atomic<int64_t> time_last_imu_message_received_or_checked_ns_;
  std::atomic<int64_t> time_last_lidar_message_received_or_checked_ns_;
  std::atomic<int64_t> time_last_odometry_message_received_or_checked_ns_;
  std::atomic<bool> received_first_odometry_message_;
  std::atomic<int64_t> time_last_absolute_6dof_message_received_or_checked_ns_;
  std::atomic<int64_t> time_last_wheel_odometry_message_received_or_checked_ns_;
  std::atomic<int64_t> time_last_loop_closure_message_received_or_checked_ns_;
  std::atomic<int64_t> time_last_pointcloud_map_message_received_or_checked_ns_;

  // For the ncamera we want to keep track for each individual image since they
  // need to be bundled for release
  std::mutex mutex_times_last_camera_messages_received_or_checked_ns_;
  std::vector<int64_t> times_last_camera_messages_received_or_checked_ns_;

  std::vector<std::function<void(const vio::SynchronizedNFrame::Ptr&)>>
      nframe_callbacks_;
  std::mutex nframe_callback_mutex_;

  std::vector<std::function<void(const vi_map::RosLidarMeasurement::ConstPtr&)>>
      lidar_callbacks_;
  std::mutex lidar_callback_mutex_;

  std::vector<std::function<void(const vi_map::Absolute6DoFMeasurement::Ptr&)>>
      absolute_6dof_callbacks_;
  std::mutex absolute_6dof_callback_mutex_;

  std::vector<
      std::function<void(const vi_map::WheelOdometryMeasurement::ConstPtr&)>>
      wheel_odometry_callbacks_;
  std::mutex wheel_odometry_callback_mutex_;

  std::vector<std::function<void(const common::LocalizationResult::ConstPtr&)>>
      localization_result_callbacks_;
  std::mutex localization_result_callback_mutex_;

  std::vector<
      std::function<void(const vi_map::LoopClosureMeasurement::ConstPtr&)>>
      loop_closure_callbacks_;
  std::mutex loop_closure_callback_mutex_;

  std::vector<std::function<void(
      const vi_map::RosPointCloudMapSensorMeasurement::ConstPtr&)>>
      pointcloud_map_callbacks_;
  std::mutex pointcloud_map_callback_mutex_;

  std::unique_ptr<SynchronizerStatistics> statistics_;
};

}  // namespace maplab

#endif  // MAPLAB_NODE_SYNCHRONIZER_H_
