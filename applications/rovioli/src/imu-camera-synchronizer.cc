#include "rovioli/imu-camera-synchronizer.h"

#include <aslam/pipeline/visual-pipeline-null.h>
#include <maplab-common/conversions.h>

DEFINE_int64(
    vio_nframe_sync_tolerance_ns, 500000,
    "Tolerance of the timestamps of two images to consider them as "
    "part of a single n-frame [ns].");
DEFINE_double(
    vio_nframe_sync_max_output_frequency_hz, 10.0,
    "Maximum output frequency of the synchronized IMU-NFrame structures "
    "from the synchronizer.");

namespace rovioli {

ImuCameraSynchronizer::ImuCameraSynchronizer(
    const aslam::NCamera::Ptr& camera_system)
    : camera_system_(camera_system),
      kImuBufferLengthNanoseconds(aslam::time::seconds(30u)),
      frame_skip_counter_(0u),
      previous_nframe_timestamp_ns_(-1),
      min_nframe_timestamp_diff_ns_(
          kSecondsToNanoSeconds /
          FLAGS_vio_nframe_sync_max_output_frequency_hz),
      initial_sync_succeeded_(false),
      shutdown_(false),
      time_last_imu_message_received_or_checked_ns_(
          aslam::time::nanoSecondsSinceEpoch()),
      time_last_camera_message_received_or_checked_ns_(
          aslam::time::nanoSecondsSinceEpoch()) {
  CHECK(camera_system_ != nullptr);
  CHECK_GT(FLAGS_vio_nframe_sync_max_output_frequency_hz, 0.);

  // Initialize the pipeline.
  static constexpr bool kCopyImages = false;
  std::vector<aslam::VisualPipeline::Ptr> mono_pipelines;
  for (size_t camera_idx = 0; camera_idx < camera_system_->getNumCameras();
       ++camera_idx) {
    mono_pipelines.emplace_back(
        new aslam::NullVisualPipeline(
            camera_system_->getCameraShared(camera_idx), kCopyImages));
  }

  const int kNFrameToleranceNs = FLAGS_vio_nframe_sync_tolerance_ns;
  constexpr size_t kNumThreads = 1u;
  visual_pipeline_.reset(
      new aslam::VisualNPipeline(
          kNumThreads, mono_pipelines, camera_system_, camera_system_,
          kNFrameToleranceNs));

  imu_buffer_.reset(
      new vio_common::ImuMeasurementBuffer(kImuBufferLengthNanoseconds));

  check_if_messages_are_incomfing_thread_ = std::thread(
      &ImuCameraSynchronizer::checkIfMessagesAreIncomingWorker, this);
  process_thread_ =
      std::thread(&ImuCameraSynchronizer::processDataThreadWorker, this);
}

ImuCameraSynchronizer::~ImuCameraSynchronizer() {
  shutdown();
}

void ImuCameraSynchronizer::addCameraImage(
    size_t camera_index, const cv::Mat& image, int64_t timestamp) {
  constexpr int kMaxNFrameQueueSize = 50;
  CHECK(visual_pipeline_ != nullptr);
  time_last_camera_message_received_or_checked_ns_ =
      aslam::time::nanoSecondsSinceEpoch();
  if (!visual_pipeline_->processImageBlockingIfFull(
          camera_index, image, timestamp, kMaxNFrameQueueSize)) {
    shutdown();
  }
}

void ImuCameraSynchronizer::addImuMeasurements(
    const Eigen::Matrix<int64_t, 1, Eigen::Dynamic>& timestamps_nanoseconds,
    const Eigen::Matrix<double, 6, Eigen::Dynamic>& imu_measurements) {
  CHECK(imu_buffer_ != nullptr);
  time_last_imu_message_received_or_checked_ns_ =
      aslam::time::nanoSecondsSinceEpoch();
  imu_buffer_->addMeasurements(timestamps_nanoseconds, imu_measurements);
}

void ImuCameraSynchronizer::checkIfMessagesAreIncomingWorker() {
  constexpr int kMaxTimeBeforeWarningS = 5;
  const int64_t kMaxTimeBeforeWarningNs =
      aslam::time::secondsToNanoSeconds(kMaxTimeBeforeWarningS);
  while (true) {
    std::unique_lock<std::mutex> lock(mutex_check_if_messages_are_incoming_);
    const bool shutdown_requested = cv_shutdown_.wait_for(
        lock, std::chrono::seconds(kMaxTimeBeforeWarningS),
        [this]() { return shutdown_.load(); });
    if (shutdown_requested) {
      return;
    }

    const int64_t current_time_ns = aslam::time::nanoSecondsSinceEpoch();
    LOG_IF(
        WARNING,
        current_time_ns - time_last_imu_message_received_or_checked_ns_ >
            kMaxTimeBeforeWarningNs)
        << "No IMU messages have been received in the last "
        << kMaxTimeBeforeWarningS
        << " seconds. Check for measurement drops or if the topic is properly "
        << "set in the maplab IMU configuration file";
    LOG_IF(
        WARNING,
        current_time_ns - time_last_camera_message_received_or_checked_ns_ >
            kMaxTimeBeforeWarningNs)
        << "No camera messages have been received in the last "
        << kMaxTimeBeforeWarningS
        << " seconds. Check for measurement drops or if the topic is properly "
        << "set in the camera configuration file";
    time_last_imu_message_received_or_checked_ns_ = current_time_ns;
    time_last_camera_message_received_or_checked_ns_ = current_time_ns;
  }
}

void ImuCameraSynchronizer::processDataThreadWorker() {
  while (!shutdown_) {
    aslam::VisualNFrame::Ptr new_nframe;
    if (!visual_pipeline_->getNextBlocking(&new_nframe)) {
      // Shutdown.
      return;
    }

    // Block the previous nframe timestamp so that no other thread can use it.
    // It should wait till this iteration is done.
    std::unique_lock<std::mutex> lock(m_previous_nframe_timestamp_ns_);

    // Drop few first nframes as there might have incomplete IMU data.
    const int64_t current_frame_timestamp_ns =
        new_nframe->getMinTimestampNanoseconds();
    if (frame_skip_counter_ < kFramesToSkipAtInit) {
      ++frame_skip_counter_;
      previous_nframe_timestamp_ns_ = current_frame_timestamp_ns;
      continue;
    }

    // Throttle the output rate of VisualNFrames to reduce the rate of which
    // the following nodes are running (e.g. tracker).
    CHECK_GE(previous_nframe_timestamp_ns_, 0);
    if (new_nframe->getMinTimestampNanoseconds() -
            previous_nframe_timestamp_ns_ <
        min_nframe_timestamp_diff_ns_) {
      continue;
    }

    vio::SynchronizedNFrameImu::Ptr new_imu_nframe_measurement(
        new vio::SynchronizedNFrameImu);
    new_imu_nframe_measurement->nframe = new_nframe;

    // Wait for the required IMU data.
    CHECK(aslam::time::isValidTime(previous_nframe_timestamp_ns_));
    const int64_t kWaitTimeoutNanoseconds = aslam::time::milliseconds(50);
    vio_common::ImuMeasurementBuffer::QueryResult result;
    bool skip_frame = false;
    while ((result = imu_buffer_->getImuDataInterpolatedBordersBlocking(
                previous_nframe_timestamp_ns_, current_frame_timestamp_ns,
                kWaitTimeoutNanoseconds,
                &new_imu_nframe_measurement->imu_timestamps,
                &new_imu_nframe_measurement->imu_measurements)) !=
           vio_common::ImuMeasurementBuffer::QueryResult::kDataAvailable) {
      if (result ==
          vio_common::ImuMeasurementBuffer::QueryResult::kQueueShutdown) {
        // Shutdown.
        return;
      }
      if (result ==
          vio_common::ImuMeasurementBuffer::QueryResult::kDataNeverAvailable) {
        LOG(ERROR) << "Camera/IMU data out-of-order. This might be okay during "
                      "initialization.";
        CHECK(!initial_sync_succeeded_)
            << "Some synced IMU-camera frames were"
            << "already published. This will lead to map inconsistency.";

        // Skip this frame, but also advanced the previous frame timestamp.
        previous_nframe_timestamp_ns_ = current_frame_timestamp_ns;
        skip_frame = true;
        break;
      }

      if (result ==
          vio_common::ImuMeasurementBuffer::QueryResult::kDataNotYetAvailable) {
        LOG(WARNING) << "NFrame-IMU synchronization timeout. IMU measurements "
                     << "lag behind. Dropping this nframe.";
        // Skip this frame.
        skip_frame = true;
        break;
      }

      if (result == vio_common::ImuMeasurementBuffer::QueryResult::
                        kTooFewMeasurementsAvailable) {
        LOG(WARNING) << "NFrame-IMU synchronization: Too few IMU measurements "
                     << "available between the previous and current nframe. "
                     << "Dropping this nframe.";
        // Skip this frame.
        skip_frame = true;
        break;
      }
    }

    if (skip_frame) {
      continue;
    }

    previous_nframe_timestamp_ns_ = current_frame_timestamp_ns;
    // Manually unlock the mutex as the previous nframe timestamp can be
    // consumed by the next iteration.
    lock.unlock();

    // All the synchronization succeeded so let's mark we will publish
    // the frames now. Any IMU data drops after this point mean that the map
    // is inconsistent.
    initial_sync_succeeded_ = true;

    std::lock_guard<std::mutex> callback_lock(m_nframe_callbacks_);
    for (const std::function<void(const vio::SynchronizedNFrameImu::Ptr&)>&
             callback : nframe_callbacks_) {
      callback(new_imu_nframe_measurement);
    }
  }
}

void ImuCameraSynchronizer::registerSynchronizedNFrameImuCallback(
    const std::function<void(const vio::SynchronizedNFrameImu::Ptr&)>&
        callback) {
  std::lock_guard<std::mutex> lock(m_nframe_callbacks_);
  CHECK(callback);
  nframe_callbacks_.push_back(callback);
}

void ImuCameraSynchronizer::shutdown() {
  shutdown_ = true;
  visual_pipeline_->shutdown();
  imu_buffer_->shutdown();
  if (process_thread_.joinable()) {
    process_thread_.join();
  }
  cv_shutdown_.notify_all();
  if (check_if_messages_are_incomfing_thread_.joinable()) {
    check_if_messages_are_incomfing_thread_.join();
  }
}

}  // namespace rovioli
