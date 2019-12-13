#include "maplab-node/synchronizer.h"

#include <aslam/common/covariance-helpers.h>
#include <aslam/pipeline/visual-pipeline-null.h>
#include <maplab-common/conversions.h>

DEFINE_int64(
    vio_nframe_sync_tolerance_ns, 500000,
    "Tolerance of the timestamps of two images to consider them as "
    "part of a single n-frame [ns].");
DEFINE_double(
    vio_nframe_sync_max_output_frequency_hz, 10.0,
    "Maximum output frequency of the synchronized NFrame structures "
    "from the synchronizer.");
DEFINE_double(
    vio_nframe_sync_max_output_frequency_tolerance_factor_, 0.95,
    "Tolerance on the minimum timestamp differance required by throttler above "
    "which an nframe is released. This is helpful when the desired throttling "
    "frequency is close to the actual frame rate and the latter has slight "
    "variations.");
DEFINE_int32(
    vio_nframe_sync_max_queue_size, 50,
    "Maximum queue size of the synchronization pipeline trying to match images "
    "into NFrames.");
DEFINE_int64(
    odometry_buffer_history_ns, aslam::time::seconds(30u),
    "History length of the buffered external 6DOF odometry measurements.");
DEFINE_int64(
    odometry_buffer_max_forward_propagation_ns, aslam::time::milliseconds(500),
    "Determines the maximum duration the odometry buffer can "
    "forward-propagate using the IMU.");
DEFINE_bool(
    enable_synchronizer_statistics, false,
    "If enable, the synchronizer will keep data about the latency and other "
    "key properties of the data it synchronizes.");

namespace maplab {

Synchronizer::Synchronizer(const vi_map::SensorManager& sensor_manager)
    : sensor_manager_(sensor_manager),
      T_M_B_buffer_(
          FLAGS_odometry_buffer_history_ns,
          FLAGS_odometry_buffer_max_forward_propagation_ns),
      frame_skip_counter_(0u),
      previous_nframe_timestamp_ns_(aslam::time::getInvalidTime()),
      min_nframe_timestamp_diff_ns_(
          kSecondsToNanoSeconds /
          FLAGS_vio_nframe_sync_max_output_frequency_hz),
      odometry_measurement_counter_(0),
      lidar_skip_counter_(0u),
      shutdown_(false),
      time_last_imu_message_received_or_checked_ns_(
          aslam::time::getInvalidTime()),
      time_last_lidar_message_received_or_checked_ns_(
          aslam::time::getInvalidTime()),
      time_last_odometry_message_received_or_checked_ns_(
          aslam::time::getInvalidTime()),
      received_first_odometry_message_(false),
      time_last_absolute_6dof_message_received_or_checked_ns_(
          aslam::time::getInvalidTime()),
      time_last_wheel_odometry_message_received_or_checked_ns_(
          aslam::time::getInvalidTime()),
      time_last_loop_closure_message_received_or_checked_ns_(
          aslam::time::getInvalidTime()),
      time_last_pointcloud_map_message_received_or_checked_ns_(
          aslam::time::getInvalidTime()),
      min_nframe_timestamp_diff_tolerance_factor_(FLAGS_vio_nframe_sync_max_output_frequency_tolerance_factor_) {
  CHECK_GT(FLAGS_vio_nframe_sync_max_output_frequency_hz, 0.);

  if (FLAGS_enable_synchronizer_statistics) {
    statistics_.reset(new SynchronizerStatistics());
  }
}

Synchronizer::~Synchronizer() {
  if (statistics_) {
    LOG(INFO) << statistics_->print();
  }

  if (!shutdown_.load()) {
    shutdown();
  }
}

void Synchronizer::start() {
  check_if_messages_are_incoming_thread_ =
      std::thread(&Synchronizer::checkIfMessagesAreIncomingWorker, this);
}

void Synchronizer::initializeNCameraSynchronization(
    const aslam::NCamera::Ptr& camera_system) {
  CHECK(camera_system);
  CHECK(!visual_pipeline_) << "[MaplabNode-Synchronizer] NCamera "
                           << "synchronization already initialized!";

  // Initialize the pipeline.
  static constexpr bool kCopyImages = false;
  std::vector<aslam::VisualPipeline::Ptr> mono_pipelines;
  for (size_t camera_idx = 0u; camera_idx < camera_system->getNumCameras();
       ++camera_idx) {
    mono_pipelines.emplace_back(new aslam::NullVisualPipeline(
        camera_system->getCameraShared(camera_idx), kCopyImages));
  }
  {
    std::lock_guard<std::mutex> lock(
        mutex_times_last_camera_messages_received_or_checked_ns_);
    times_last_camera_messages_received_or_checked_ns_.assign(
        camera_system->getNumCameras(), aslam::time::getInvalidTime());
  }
  const int kNFrameToleranceNs = FLAGS_vio_nframe_sync_tolerance_ns;
  constexpr size_t kNumThreads = 1u;
  visual_pipeline_.reset(new aslam::VisualNPipeline(
      kNumThreads, mono_pipelines, camera_system, camera_system,
      kNFrameToleranceNs));

  if (statistics_) {
    statistics_->initializeCameraStats(camera_system->getNumCameras());
  }
}

void Synchronizer::processCameraImage(
    const size_t camera_index, const cv::Mat& image, const int64_t timestamp) {
  CHECK(visual_pipeline_) << "[MaplabNode-Synchronizer] The visual pipeline, "
                             "which turns individual images "
                          << "into NFrames, has not been initialized yet!";

  const int64_t current_time_ns = aslam::time::nanoSecondsSinceEpoch();

  {
    std::lock_guard<std::mutex> lock(
        mutex_times_last_camera_messages_received_or_checked_ns_);
    CHECK_LT(
        camera_index,
        times_last_camera_messages_received_or_checked_ns_.size());
    times_last_camera_messages_received_or_checked_ns_[camera_index] =
        current_time_ns;
  }

  if (statistics_) {
    CHECK_LT(camera_index, statistics_->cam_latency_stats.size());

    const int64_t latency_ns = current_time_ns - timestamp;
    LOG_IF(WARNING, latency_ns < 0)
        << "[MaplabNode-Synchronizer] Received image message (cam "
        << camera_index << ") from the "
        << "future! msg time "
        << aslam::time::timeNanosecondsToString(timestamp) << " current time: "
        << aslam::time::timeNanosecondsToString(current_time_ns)
        << " latency: " << latency_ns << "ns";

    statistics_->cam_latency_stats[camera_index].AddSample(
        static_cast<double>(latency_ns));
  }

  if (!visual_pipeline_->processImageBlockingIfFull(
          camera_index, image, timestamp,
          FLAGS_vio_nframe_sync_max_queue_size)) {
    LOG(ERROR)
        << "[MaplabNode-Synchronizer] Failed to process an image of camera "
        << camera_index << " into an NFrame at time " << timestamp << "ns!";
    shutdown();
  }

  // Put all visual frames that are ready into the buffer.
  {
    std::lock_guard<std::mutex> lock(nframe_buffer_mutex_);
    aslam::VisualNFrame::Ptr next_nframe = visual_pipeline_->getNext();
    while (next_nframe) {
      nframe_buffer_.addValue(
          next_nframe->getMinTimestampNanoseconds(), next_nframe);
      next_nframe = visual_pipeline_->getNext();
    }
  }
}

void Synchronizer::processLidarMeasurement(
    const vi_map::RosLidarMeasurement::ConstPtr& lidar_measurement) {
  CHECK(lidar_measurement);

  const int64_t current_time_ns = aslam::time::nanoSecondsSinceEpoch();

  time_last_lidar_message_received_or_checked_ns_.store(current_time_ns);

  if (statistics_) {
    const int64_t latency_ns =
        current_time_ns - lidar_measurement->getTimestampNanoseconds();
    LOG_IF(WARNING, latency_ns < 0)
        << "[MaplabNode-Synchronizer] Received Lidar message from the "
        << "future! msg time "
        << aslam::time::timeNanosecondsToString(
               lidar_measurement->getTimestampNanoseconds())
        << " current time: "
        << aslam::time::timeNanosecondsToString(current_time_ns)
        << " latency: " << latency_ns << "ns";
    statistics_->lidar_latency_stats.AddSample(static_cast<double>(latency_ns));
  }

  {
    std::lock_guard<std::mutex> lock(lidar_buffer_mutex_);
    lidar_buffer_.addValue(
        lidar_measurement->getTimestampNanoseconds(), lidar_measurement);
  }
}

void Synchronizer::processAbsolute6DoFMeasurement(
    const vi_map::Absolute6DoFMeasurement::Ptr& absolute_6dof_measurement) {
  CHECK(absolute_6dof_measurement);
  time_last_absolute_6dof_message_received_or_checked_ns_.store(
      aslam::time::nanoSecondsSinceEpoch());

  {
    std::lock_guard<std::mutex> lock(absolute_6dof_buffer_mutex_);
    absolute_6dof_buffer_.addValue(
        absolute_6dof_measurement->getTimestampNanoseconds(),
        absolute_6dof_measurement);
  }
}

void Synchronizer::processLoopClosureMeasurement(
    const vi_map::LoopClosureMeasurement::ConstPtr&
        loop_closure_measurement) {
  CHECK(loop_closure_measurement);
  time_last_loop_closure_message_received_or_checked_ns_.store(
      aslam::time::nanoSecondsSinceEpoch());

  {
    std::lock_guard<std::mutex> lock(loop_closure_buffer_mutex_);
    std::pair<int64_t, int64_t> timestamp_pair(
        loop_closure_measurement->getTimestampNanosecondsA(),
        loop_closure_measurement->getTimestampNanosecondsB());

    loop_closure_buffer_[timestamp_pair] = loop_closure_measurement;
  }
}

void Synchronizer::processWheelOdometryMeasurement(
    const vi_map::WheelOdometryMeasurement::ConstPtr&
        wheel_odometry_measurement) {
  CHECK(wheel_odometry_measurement);
  time_last_wheel_odometry_message_received_or_checked_ns_.store(
      aslam::time::nanoSecondsSinceEpoch());

  {
    std::lock_guard<std::mutex> lock(wheel_odometry_buffer_mutex_);
    wheel_odometry_buffer_.addValue(
        wheel_odometry_measurement->getTimestampNanoseconds(),
        wheel_odometry_measurement);
  }
}

void Synchronizer::processPointCloudMapMeasurement(
    const vi_map::RosPointCloudMapSensorMeasurement::ConstPtr& pointcloud_map) {
  CHECK(pointcloud_map);
  time_last_pointcloud_map_message_received_or_checked_ns_.store(
      aslam::time::nanoSecondsSinceEpoch());

  {
    std::lock_guard<std::mutex> lock(pointcloud_map_buffer_mutex_);

    pointcloud_map_buffer_.addValue(
        pointcloud_map->getTimestampNanoseconds(), pointcloud_map);
  }
}

void Synchronizer::processImuMeasurements(
    const Eigen::Matrix<int64_t, 1, Eigen::Dynamic>& timestamps_nanoseconds,
    const Eigen::Matrix<double, 6, Eigen::Dynamic>& imu_measurements) {
  const int64_t current_time_ns = aslam::time::nanoSecondsSinceEpoch();

  time_last_imu_message_received_or_checked_ns_.store(current_time_ns);

  if (statistics_) {
    CHECK_GT(imu_measurements.cols(), 0);
    const int last_idx = imu_measurements.cols() - 1;
    const int64_t latency_ns =
        current_time_ns - timestamps_nanoseconds[last_idx];
    LOG_IF(WARNING, latency_ns < 0)
        << "[MaplabNode-Synchronizer] Received IMU message from the future! "
        << "msg time "
        << aslam::time::timeNanosecondsToString(
               timestamps_nanoseconds[last_idx])
        << " current time: "
        << aslam::time::timeNanosecondsToString(current_time_ns)
        << " latency: " << latency_ns << "ns";
    statistics_->imu_latency_stats.AddSample(static_cast<double>(latency_ns));
    statistics_->imu_num_measurements_per_msg_stats.AddSample(
        static_cast<double>(imu_measurements.cols()));
  }

  T_M_B_buffer_.imu_buffer_mutable().addMeasurements(
      timestamps_nanoseconds, imu_measurements);
}

void Synchronizer::processOdometryMeasurement(
    const vio::ViNodeState& odometry) {
  const int64_t current_time_ns = aslam::time::nanoSecondsSinceEpoch();

  time_last_odometry_message_received_or_checked_ns_.store(current_time_ns);

  if (statistics_) {
    const int64_t latency_ns = current_time_ns - odometry.getTimestamp();
    LOG_IF(WARNING, latency_ns < 0)
        << "[MaplabNode-Synchronizer] Received Odometry message from the "
        << "future! msg time "
        << aslam::time::timeNanosecondsToString(odometry.getTimestamp())
        << " current time: "
        << aslam::time::timeNanosecondsToString(current_time_ns)
        << " latency: " << latency_ns << "ns";
    statistics_->odom_latency_stats.AddSample(static_cast<double>(latency_ns));
  }

  VLOG_IF(1, !received_first_odometry_message_.exchange(true))
      << "[MaplabNode-Synchronizer] Received first odometry message!";

  // To compute the odometry covariance for later filtering, it is necessary to
  // keep track of the number of odometry measurements that were received over
  // time.
  ++odometry_measurement_counter_;
  vio::ViNodeState odometry_copy = odometry;
  odometry_copy.setSequenceNumber(odometry_measurement_counter_);
  T_M_B_buffer_.bufferOdometryEstimate(odometry_copy);

  releaseData();
}

void Synchronizer::releaseNFrameData(
    const int64_t oldest_timestamp_ns, const int64_t newest_timestamp_ns) {
  CHECK_GE(newest_timestamp_ns, oldest_timestamp_ns);
  std::vector<aslam::VisualNFrame::Ptr> nframes;
  {
    std::lock_guard<std::mutex> lock(nframe_buffer_mutex_);

    // Drop these frames, since there is no odometry data anymore for them.
    const size_t dropped_nframes =
        nframe_buffer_.removeItemsBefore(oldest_timestamp_ns);
    LOG_IF(WARNING, dropped_nframes != 0u)
        << "[MaplabNode-Synchronizer] Could not find an odometry "
        << "transformation for " << dropped_nframes << " nframes "
        << "because it was already dropped from the buffer! "
        << "This might be okay during initialization.";

    frame_skip_counter_ += dropped_nframes;

    nframe_buffer_.extractItemsBeforeIncluding(newest_timestamp_ns, &nframes);
  }

  for (const aslam::VisualNFrame::Ptr nframe : nframes) {
    CHECK(nframe);
    vio::SynchronizedNFrame::Ptr new_nframe_measurement(
        new vio::SynchronizedNFrame);
    new_nframe_measurement->nframe = nframe;
    const int64_t current_frame_timestamp_ns =
        nframe->getMinTimestampNanoseconds();

    // Throttle the output rate of VisualNFrames to reduce the rate of which
    // the following nodes are running (e.g. tracker).
    if (aslam::time::isValidTime(previous_nframe_timestamp_ns_)) {
      if (current_frame_timestamp_ns - previous_nframe_timestamp_ns_ <
          min_nframe_timestamp_diff_ns_ *
              min_nframe_timestamp_diff_tolerance_factor_) {
        ++frame_skip_counter_;
        continue;
      }
    }
    previous_nframe_timestamp_ns_ = current_frame_timestamp_ns;

    {
      std::lock_guard<std::mutex> callback_lock(nframe_callback_mutex_);
      for (const std::function<void(const vio::SynchronizedNFrame::Ptr&)>&
               callback : nframe_callbacks_) {
        callback(new_nframe_measurement);
      }
    }
  }
}

void Synchronizer::releaseLidarData(
    const int64_t oldest_timestamp_ns, const int64_t newest_timestamp_ns) {
  CHECK_GE(newest_timestamp_ns, oldest_timestamp_ns);
  std::vector<vi_map::RosLidarMeasurement::ConstPtr> lidar_measurements;
  {
    std::lock_guard<std::mutex> lock(lidar_buffer_mutex_);

    // Drop these lidar measurements, since there is no odometry data anymore
    // for them.
    const size_t dropped_lidar_measurements =
        lidar_buffer_.removeItemsBefore(oldest_timestamp_ns);
    LOG_IF(WARNING, dropped_lidar_measurements != 0u)
        << "[MaplabNode-Synchronizer] Could not find an odometry "
        << "transformation for " << dropped_lidar_measurements
        << " lidar measurements because it was already dropped from the "
        << "buffer! This might be okay during initialization.";

    lidar_skip_counter_ += dropped_lidar_measurements;

    lidar_buffer_.extractItemsBeforeIncluding(
        newest_timestamp_ns, &lidar_measurements);
  }

  for (const vi_map::RosLidarMeasurement::ConstPtr lidar_measurement :
       lidar_measurements) {
    CHECK(lidar_measurement);
    std::lock_guard<std::mutex> callback_lock(lidar_callback_mutex_);
    for (const std::function<void(
             const vi_map::RosLidarMeasurement::ConstPtr&)>& callback :
         lidar_callbacks_) {
      callback(lidar_measurement);
    }
  }
}

void Synchronizer::releaseAbsolute6DoFData(
    const int64_t /*oldest_timestamp_ns*/, const int64_t newest_timestamp_ns) {
  std::vector<vi_map::Absolute6DoFMeasurement::Ptr> absolute_6dof_measurements;
  {
    std::lock_guard<std::mutex> lock(absolute_6dof_buffer_mutex_);

    absolute_6dof_buffer_.extractItemsBeforeIncluding(
        newest_timestamp_ns, &absolute_6dof_measurements);
  }

  for (const vi_map::Absolute6DoFMeasurement::Ptr& absolute_6dof_measurement :
       absolute_6dof_measurements) {
    CHECK(absolute_6dof_measurement);

    vi_map::Absolute6DoFMeasurement::Ptr absolute_6dof_measurement_copy(
        new vi_map::Absolute6DoFMeasurement(*absolute_6dof_measurement));
    aslam::Transformation T_M_B_cached;
    if (T_M_B_buffer_.getPoseAt(
            absolute_6dof_measurement->getTimestampNanoseconds(),
            &T_M_B_cached) <= vio_common::PoseLookupBuffer::ResultStatus::
                                  kSuccessImuForwardPropagation) {
      absolute_6dof_measurement_copy->set_T_M_B_cached(T_M_B_cached);

      // The poses for which an odometry estimate is available are also released
      // as localization results and subsequently fused with the localizations

      common::LocalizationResult::Ptr absolute_6dof_localization =
          std::make_shared<common::LocalizationResult>(
              common::LocalizationType::kAbsolutePose);

      absolute_6dof_localization->localization_mode =
          common::LocalizationMode::kGlobal;
      absolute_6dof_localization->timestamp_ns =
          absolute_6dof_measurement_copy->getTimestampNanoseconds();
      absolute_6dof_localization->sensor_id =
          absolute_6dof_measurement_copy->getSensorId();

      aslam::Transformation T_G_S = absolute_6dof_measurement_copy->get_T_G_S();
      aslam::Transformation T_S_B =
          sensor_manager_
              .getSensor_T_B_S(absolute_6dof_measurement_copy->getSensorId())
              .inverse();
      absolute_6dof_localization->T_G_B = T_G_S * T_S_B;
      absolute_6dof_localization->is_T_G_B_set = true;
      const aslam::TransformationCovariance& T_G_S_covariance =
          absolute_6dof_measurement_copy->get_T_G_S_covariance();
      aslam::common::rotateCovariance(
          T_S_B.inverse(), T_G_S_covariance,
          &absolute_6dof_localization->T_G_B_covariance);

      absolute_6dof_localization->is_T_G_M_set = false;

      std::lock_guard<std::mutex> callback_lock(
          localization_result_callback_mutex_);
      for (const std::function<void(
               const common::LocalizationResult::ConstPtr&)>& callback :
           localization_result_callbacks_) {
        callback(absolute_6dof_localization);
      }
    }

    std::lock_guard<std::mutex> callback_lock(absolute_6dof_callback_mutex_);
    for (const std::function<void(const vi_map::Absolute6DoFMeasurement::Ptr&)>&
             callback : absolute_6dof_callbacks_) {
      callback(absolute_6dof_measurement_copy);
    }
  }
}

void Synchronizer::releaseLoopClosureData(
    const int64_t /*oldest_timestamp_ns*/, const int64_t newest_timestamp_ns) {
  std::vector<vi_map::LoopClosureMeasurement::ConstPtr>
      loop_closure_measurements;
  {
    std::lock_guard<std::mutex> lock(loop_closure_buffer_mutex_);
    vi_map::LoopClosureTemporalMap::iterator it = loop_closure_buffer_.begin();
    while (it != loop_closure_buffer_.end()) {
      const int64_t newer_timestamp_ns =
          std::max(it->first.first, it->first.second);
      if (newer_timestamp_ns <= newest_timestamp_ns) {
        loop_closure_measurements.push_back(it->second);
        it = loop_closure_buffer_.erase(it);
      } else {
        ++it;
      }
    }
  }

  for (const vi_map::LoopClosureMeasurement::ConstPtr
           loop_closure_measurement : loop_closure_measurements) {
    CHECK(loop_closure_measurement);
    std::lock_guard<std::mutex> callback_lock(loop_closure_callback_mutex_);
    for (const std::function<void(
             const vi_map::LoopClosureMeasurement::ConstPtr&)>& callback :
         loop_closure_callbacks_) {
      callback(loop_closure_measurement);
    }
  }
}

void Synchronizer::releaseWheelOdometryData(
    const int64_t /*oldest_timestamp_ns*/, const int64_t newest_timestamp_ns) {
  std::vector<vi_map::WheelOdometryMeasurement::ConstPtr>
      wheel_odometry_measurements;
  {
    std::lock_guard<std::mutex> lock(wheel_odometry_buffer_mutex_);

    wheel_odometry_buffer_.extractItemsBeforeIncludingKeepMostRecent(
        newest_timestamp_ns, &wheel_odometry_measurements);
  }

  for (const vi_map::WheelOdometryMeasurement::ConstPtr
           wheel_odometry_measurement : wheel_odometry_measurements) {
    CHECK(wheel_odometry_measurement);
    std::lock_guard<std::mutex> callback_lock(wheel_odometry_callback_mutex_);
    for (const std::function<void(
             const vi_map::WheelOdometryMeasurement::ConstPtr&)>& callback :
         wheel_odometry_callbacks_) {
      callback(wheel_odometry_measurement);
    }
  }
}

void Synchronizer::releasePointCloudMapData(
    const int64_t /*oldest_timestamp_ns*/, const int64_t newest_timestamp_ns) {
  std::vector<vi_map::RosPointCloudMapSensorMeasurement::ConstPtr>
      pointcloud_map_measurements;
  {
    std::lock_guard<std::mutex> lock(pointcloud_map_buffer_mutex_);

    pointcloud_map_buffer_.extractItemsBeforeIncluding(
        newest_timestamp_ns, &pointcloud_map_measurements);
  }

  CHECK(!pointcloud_map_callbacks_.empty());

  for (const vi_map::RosPointCloudMapSensorMeasurement::ConstPtr
           pointcloud_map_measurement : pointcloud_map_measurements) {
    CHECK(pointcloud_map_measurement);
    std::lock_guard<std::mutex> callback_lock(pointcloud_map_callback_mutex_);
    for (const std::function<void(
             const vi_map::RosPointCloudMapSensorMeasurement::ConstPtr&)>&
             callback : pointcloud_map_callbacks_) {
      callback(pointcloud_map_measurement);
    }
  }
}

void Synchronizer::releaseData() {
  int64_t oldest_timestamp_ns;
  int64_t newest_timestamp_ns;
  if (!T_M_B_buffer_.getTimeRangeOfAvailablePoses(
          &oldest_timestamp_ns, &newest_timestamp_ns)) {
    return;
  }

  // Release (or drop) NFrames.
  {
    std::lock_guard<std::mutex> lock(
        mutex_times_last_camera_messages_received_or_checked_ns_);
    const bool all_times_valid = std::all_of(
        times_last_camera_messages_received_or_checked_ns_.begin(),
        times_last_camera_messages_received_or_checked_ns_.end(),
        [](int64_t time_ns) { return aslam::time::isValidTime(time_ns); });
    if (all_times_valid) {
      releaseNFrameData(oldest_timestamp_ns, newest_timestamp_ns);
    }
  }

  // Release (or drop) lidar measurements.
  if (aslam::time::isValidTime(
          time_last_lidar_message_received_or_checked_ns_.load())) {
    releaseLidarData(oldest_timestamp_ns, newest_timestamp_ns);
  }

  // Release (or drop) absolute 6DoF data.
  if (aslam::time::isValidTime(
          time_last_absolute_6dof_message_received_or_checked_ns_.load())) {
    releaseAbsolute6DoFData(oldest_timestamp_ns, newest_timestamp_ns);
  }

  // Release (or drop) wheel odometry data.
  if (aslam::time::isValidTime(
          time_last_wheel_odometry_message_received_or_checked_ns_.load())) {
    releaseWheelOdometryData(oldest_timestamp_ns, newest_timestamp_ns);
  }

  // Release (or drop) loop closure data.
  if (aslam::time::isValidTime(
          time_last_loop_closure_message_received_or_checked_ns_.load())) {
    releaseLoopClosureData(oldest_timestamp_ns, newest_timestamp_ns);
  }

  // Release (or drop) pointcloud map data.
  if (aslam::time::isValidTime(
          time_last_pointcloud_map_message_received_or_checked_ns_.load())) {
    releasePointCloudMapData(oldest_timestamp_ns, newest_timestamp_ns);
  }
}

void Synchronizer::registerSynchronizedNFrameCallback(
    const std::function<void(const vio::SynchronizedNFrame::Ptr&)>& callback) {
  std::lock_guard<std::mutex> lock(nframe_callback_mutex_);
  CHECK(callback);
  nframe_callbacks_.emplace_back(callback);
}

void Synchronizer::registerLidarMeasurementCallback(
    const std::function<void(const vi_map::RosLidarMeasurement::ConstPtr&)>&
        callback) {
  std::lock_guard<std::mutex> lock(lidar_callback_mutex_);
  CHECK(callback);
  lidar_callbacks_.emplace_back(callback);
}

void Synchronizer::registerAbsolute6DoFMeasurementCallback(
    const std::function<void(const vi_map::Absolute6DoFMeasurement::Ptr&)>&
        callback) {
  std::lock_guard<std::mutex> lock(absolute_6dof_callback_mutex_);
  CHECK(callback);
  absolute_6dof_callbacks_.push_back(callback);
}

void Synchronizer::registerWheelOdometryMeasurementCallback(
    const std::function<
        void(const vi_map::WheelOdometryMeasurement::ConstPtr&)>& callback) {
  std::lock_guard<std::mutex> lock(wheel_odometry_callback_mutex_);
  CHECK(callback);
  wheel_odometry_callbacks_.push_back(callback);
}

void Synchronizer::registerLoopClosureMeasurementCallback(
    const std::function<void(const vi_map::LoopClosureMeasurement::ConstPtr&)>&
        callback) {
  std::lock_guard<std::mutex> lock(loop_closure_callback_mutex_);
  CHECK(callback);
  loop_closure_callbacks_.push_back(callback);
}

void Synchronizer::registerLocalizationResultMeasurementCallback(
    const std::function<void(const common::LocalizationResult::ConstPtr&)>&
        callback) {
  std::lock_guard<std::mutex> lock(localization_result_callback_mutex_);
  CHECK(callback);
  localization_result_callbacks_.push_back(callback);
}

void Synchronizer::registerPointCloudMapSensorMeasurementCallback(
    const std::function<
        void(const vi_map::RosPointCloudMapSensorMeasurement::ConstPtr&)>&
        callback) {
  std::lock_guard<std::mutex> lock(pointcloud_map_callback_mutex_);
  CHECK(callback);
  pointcloud_map_callbacks_.push_back(callback);
}

void Synchronizer::shutdown() {
  shutdown_.store(true);

  if (visual_pipeline_) {
    visual_pipeline_->shutdown();
  }

  T_M_B_buffer_.imu_buffer_mutable().shutdown();

  cv_shutdown_.notify_all();
  if (check_if_messages_are_incoming_thread_.joinable()) {
    check_if_messages_are_incoming_thread_.join();
  }

  LOG(INFO)
      << "[MaplabNode-Synchronizer] Shutting down. Skipped visual frames: "
      << frame_skip_counter_
      << " Skipped lidar measurements: " << lidar_skip_counter_;
}

void Synchronizer::expectOdometryData() {
  time_last_odometry_message_received_or_checked_ns_.store(
      aslam::time::nanoSecondsSinceEpoch());
}

void Synchronizer::expectLidarData() {
  time_last_lidar_message_received_or_checked_ns_.store(
      aslam::time::nanoSecondsSinceEpoch());
}

void Synchronizer::expectVisualData() {
  {
    std::lock_guard<std::mutex> lock(
        mutex_times_last_camera_messages_received_or_checked_ns_);
    const int64_t time_since_last_epoch = aslam::time::nanoSecondsSinceEpoch();
    std::fill(
        times_last_camera_messages_received_or_checked_ns_.begin(),
        times_last_camera_messages_received_or_checked_ns_.end(),
        time_since_last_epoch);
  }
}

void Synchronizer::expectImuData() {
  time_last_imu_message_received_or_checked_ns_.store(
      aslam::time::nanoSecondsSinceEpoch());
}

void Synchronizer::expectAbsolute6DoFData() {
  time_last_absolute_6dof_message_received_or_checked_ns_.store(
      aslam::time::nanoSecondsSinceEpoch());
}

void Synchronizer::expectLoopClosureData() {
  time_last_loop_closure_message_received_or_checked_ns_.store(
      aslam::time::nanoSecondsSinceEpoch());
}

void Synchronizer::expectWheelOdometryData() {
  time_last_wheel_odometry_message_received_or_checked_ns_.store(
      aslam::time::nanoSecondsSinceEpoch());
}

void Synchronizer::expectPointCloudMapData() {
  time_last_pointcloud_map_message_received_or_checked_ns_.store(
      aslam::time::nanoSecondsSinceEpoch());
}

void Synchronizer::checkIfMessagesAreIncomingWorker() {
  constexpr int kMaxTimeBeforeWarningS = 5;
  const int64_t kMaxTimeBeforeWarningNs =
      aslam::time::secondsToNanoSeconds(kMaxTimeBeforeWarningS);
  constexpr int kLongMaxTimeBeforeWarningS = 10;
  const int64_t kLongMaxTimeBeforeWarningNs =
      aslam::time::secondsToNanoSeconds(kLongMaxTimeBeforeWarningS);

  while (true) {
    std::unique_lock<std::mutex> lock(mutex_check_if_messages_are_incoming_);
    const bool shutdown_requested = cv_shutdown_.wait_for(
        lock, std::chrono::seconds(kMaxTimeBeforeWarningS),
        [this]() { return shutdown_.load(); });
    if (shutdown_requested) {
      return;
    }

    const int64_t current_time_ns = aslam::time::nanoSecondsSinceEpoch();

    if (time_last_odometry_message_received_or_checked_ns_.load() !=
        aslam::time::getInvalidTime()) {
      const int64_t last_time_ns =
          time_last_odometry_message_received_or_checked_ns_.exchange(
              current_time_ns);
      if (current_time_ns - last_time_ns > kMaxTimeBeforeWarningNs) {
        LOG(WARNING)
            << "[MaplabNode-Synchronizer] No odometry messages have been "
            << "received in the last " << kMaxTimeBeforeWarningS << " seconds.";

        if (received_first_odometry_message_.load()) {
          LOG(WARNING) << "[MaplabNode-Synchronizer] Either the data sources "
                       << "has stopped publishing odometry estimates or there "
                          "has been a data drop. Triggering shutdown!";
          invokeEndOfDataCallbacks();
          continue;
        } else {
          LOG(WARNING) << "[MaplabNode-Synchronizer] Check if the topic is "
                       << "properly set in the sensor calibration file.";
        }
      }
    }

    if (time_last_imu_message_received_or_checked_ns_.load() !=
        aslam::time::getInvalidTime()) {
      const int64_t last_time_ns =
          time_last_imu_message_received_or_checked_ns_.exchange(
              current_time_ns);
      LOG_IF(WARNING, current_time_ns - last_time_ns > kMaxTimeBeforeWarningNs)
          << "[MaplabNode-Synchronizer] No IMU messages have been received "
          << "in the last " << kMaxTimeBeforeWarningS
          << " seconds. Check for measurement drops or if the topic is "
          << "properly set in the sensor calibration file";
    }

    {
      std::lock_guard<std::mutex> lock(
          mutex_times_last_camera_messages_received_or_checked_ns_);
      for (size_t i = 0;
           i < times_last_camera_messages_received_or_checked_ns_.size(); ++i) {
        const int64_t last_time_ns =
            times_last_camera_messages_received_or_checked_ns_[i];
        if (last_time_ns != aslam::time::getInvalidTime()) {
          times_last_camera_messages_received_or_checked_ns_[i] =
              current_time_ns;
          LOG_IF(
              WARNING, current_time_ns - last_time_ns > kMaxTimeBeforeWarningNs)
              << "[MaplabNode-Synchronizer] No camera messages from camera "
              << i << " have been received "
              << "in the last " << kMaxTimeBeforeWarningS
              << " seconds. Check for measurement drops or if the topic is "
              << "properly set in the sensor calibration file";
        }
      }
    }

    if (time_last_lidar_message_received_or_checked_ns_.load() !=
        aslam::time::getInvalidTime()) {
      const int64_t last_time_ns =
          time_last_lidar_message_received_or_checked_ns_.exchange(
              current_time_ns);
      LOG_IF(WARNING, current_time_ns - last_time_ns > kMaxTimeBeforeWarningNs)
          << "[MaplabNode-Synchronizer] No lidar messages have been "
          << "received in the last " << kMaxTimeBeforeWarningS
          << " seconds. Check for measurement drops or if the topic is "
          << "properly set in the sensor calibration file";
    }

    if (time_last_absolute_6dof_message_received_or_checked_ns_.load() !=
        aslam::time::getInvalidTime()) {
      const int64_t last_time_ns =
          time_last_absolute_6dof_message_received_or_checked_ns_.exchange(
              current_time_ns);
      LOG_IF(WARNING, current_time_ns - last_time_ns > kMaxTimeBeforeWarningNs)
          << "[MaplabNode-Synchronizer] No absolute 6DoF messages have been "
          << "received in the last " << kMaxTimeBeforeWarningS << " seconds.";
    }

    if (time_last_loop_closure_message_received_or_checked_ns_.load() !=
        aslam::time::getInvalidTime()) {
      const int64_t last_time_ns =
          time_last_loop_closure_message_received_or_checked_ns_.exchange(
              current_time_ns);
      LOG_IF(
          WARNING, current_time_ns - last_time_ns > kLongMaxTimeBeforeWarningNs)
          << "[MaplabNode-Synchronizer] No loop closure messages have been "
          << "received in the last " << kLongMaxTimeBeforeWarningS
          << " seconds. Check for measurement drops or if the topic is "
          << "properly set in the sensor calibration file";
    }

    if (time_last_wheel_odometry_message_received_or_checked_ns_.load() !=
        aslam::time::getInvalidTime()) {
      const int64_t last_time_ns =
          time_last_wheel_odometry_message_received_or_checked_ns_.exchange(
              current_time_ns);
      LOG_IF(
          WARNING, current_time_ns - last_time_ns > kLongMaxTimeBeforeWarningNs)
          << "[MaplabNode-Synchronizer] No wheel odometry messages have been "
          << "received in the last " << kLongMaxTimeBeforeWarningS
          << " seconds. Check for measurement drops or if the topic is "
          << "properly set in the sensor calibration file";
    }

    if (time_last_pointcloud_map_message_received_or_checked_ns_.load() !=
        aslam::time::getInvalidTime()) {
      const int64_t last_time_ns =
          time_last_pointcloud_map_message_received_or_checked_ns_.exchange(
              current_time_ns);
      LOG_IF(
          WARNING, current_time_ns - last_time_ns > kLongMaxTimeBeforeWarningNs)
          << "[MaplabNode-Synchronizer] No point cloud map messages have been "
          << "received in the last " << kLongMaxTimeBeforeWarningS
          << " seconds. Check for measurement drops or if the topic is "
          << "properly set in the sensor calibration file";
    }
  }
}
}  // namespace maplab
