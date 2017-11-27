#ifndef ROVIOLI_IMU_CAMERA_SYNCHRONIZER_H_
#define ROVIOLI_IMU_CAMERA_SYNCHRONIZER_H_

#include <atomic>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <aslam/cameras/ncamera.h>
#include <aslam/pipeline/visual-npipeline.h>
#include <opencv2/core/core.hpp>
#include <vio-common/imu-measurements-buffer.h>
#include <vio-common/vio-types.h>
#include <vio-common/vio-update.h>

namespace rovioli {

class ImuCameraSynchronizer {
 public:
  MAPLAB_POINTER_TYPEDEFS(ImuCameraSynchronizer);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ImuCameraSynchronizer() = delete;

  explicit ImuCameraSynchronizer(const aslam::NCamera::Ptr& camera_system);

  ~ImuCameraSynchronizer();

  void addImuMeasurements(
      const Eigen::Matrix<int64_t, 1, Eigen::Dynamic>& timestamps_nanoseconds,
      const Eigen::Matrix<double, 6, Eigen::Dynamic>& imu_measurements);
  void addCameraImage(
      size_t camera_index, const cv::Mat& image, int64_t timestamp);

  void registerSynchronizedNFrameImuCallback(
      const std::function<void(const vio::SynchronizedNFrameImu::Ptr&)>& cb);

  void shutdown();

  static constexpr size_t kFramesToSkipAtInit = 1u;

 private:
  void checkIfMessagesAreIncomingWorker();
  void processDataThreadWorker();

  const aslam::NCamera::Ptr camera_system_;

  aslam::VisualNPipeline::UniquePtr visual_pipeline_;
  vio_common::ImuMeasurementBuffer::UniquePtr imu_buffer_;
  const int64_t kImuBufferLengthNanoseconds;

  // Number of already skipped frames.
  size_t frame_skip_counter_;

  int64_t previous_nframe_timestamp_ns_;
  std::mutex m_previous_nframe_timestamp_ns_;

  // Minimum interval of the output, as configured by the maximum frequency
  // flag.
  int64_t min_nframe_timestamp_diff_ns_;

  std::vector<std::function<void(const vio::SynchronizedNFrameImu::Ptr&)>>
      nframe_callbacks_;
  std::mutex m_nframe_callbacks_;
  std::atomic<bool> initial_sync_succeeded_;

  std::atomic<bool> shutdown_;
  std::condition_variable cv_shutdown_;

  std::thread check_if_messages_are_incomfing_thread_;
  std::thread process_thread_;
  std::mutex mutex_check_if_messages_are_incoming_;

  // Indicates the timestamp when either the last message was received or the
  // check that messages are (still) incoming was performed last (whichever
  // happend most recently).
  int64_t time_last_imu_message_received_or_checked_ns_;
  int64_t time_last_camera_message_received_or_checked_ns_;
};

}  // namespace rovioli

#endif  // ROVIOLI_IMU_CAMERA_SYNCHRONIZER_H_
