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
      image_skip_counter_(0u),
      frame_skip_counter_(0u),
      previous_nframe_timestamp_ns_(aslam::time::getInvalidTime()),
      min_nframe_timestamp_diff_ns_(
          kSecondsToNanoSeconds /
          FLAGS_vio_nframe_sync_max_output_frequency_hz),
      min_nframe_timestamp_diff_tolerance_factor_(
          FLAGS_vio_nframe_sync_max_output_frequency_tolerance_factor_),
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
          aslam::time::getInvalidTime()) {
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

  // Initialize temporal buffer per frame.
  {
    std::lock_guard<std::mutex> lock(image_buffer_mutex_);
    image_buffer_.resize(camera_system->getNumCameras());
  }

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

void Synchronizer::initializeExternalFeaturesSynchronization(
    const aslam::SensorIdSet& external_feature_sensor_ids) {
  // Initialize temporal buffer per external feature sensor.
  {
    std::lock_guard<std::mutex> lock(external_features_buffer_mutex_);

    for (const aslam::SensorId sensor_id : external_feature_sensor_ids) {
      CHECK(sensor_id.isValid());
      const size_t external_feature_sensor_index =
          external_features_id_to_index_map_.size();
      external_features_id_to_index_map_.emplace(
          sensor_id, external_feature_sensor_index);
    }

    external_features_buffer_.resize(external_feature_sensor_ids.size());
  }

  {
    std::lock_guard<std::mutex> lock(
        mutex_times_last_external_feature_messages_received_or_checked_ns_);
    times_last_external_feature_messages_received_or_checked_ns_.assign(
        external_feature_sensor_ids.size(), aslam::time::getInvalidTime());
  }

  if (statistics_) {
    // TODO(smauq): Initialize statistics
    // statistics_->initializeExternalFeaturesStats(
    //     external_feature_sensor_ids.size());
  }
}

void Synchronizer::processCameraImage(
    const vio::ImageMeasurement::ConstPtr& image_measurement) {
  CHECK(image_measurement);

  const int64_t current_time_ns = aslam::time::nanoSecondsSinceEpoch();

  VLOG(5) << "[MaplabNode-Synchronizer] processCameraImage "
          << aslam::time::timeNanosecondsToString(image_measurement->timestamp);

  {
    std::lock_guard<std::mutex> lock(
        mutex_times_last_camera_messages_received_or_checked_ns_);
    CHECK_LT(
        image_measurement->camera_index,
        static_cast<int>(
            times_last_camera_messages_received_or_checked_ns_.size()));
    times_last_camera_messages_received_or_checked_ns_[image_measurement
                                                           ->camera_index] =
        current_time_ns;
  }

  if (statistics_) {
    CHECK_LT(
        image_measurement->camera_index,
        static_cast<int>(statistics_->cam_latency_stats.size()));

    const int64_t latency_ns = current_time_ns - image_measurement->timestamp;
    LOG_IF(WARNING, latency_ns < 0)
        << "[MaplabNode-Synchronizer] Received image message (cam "
        << image_measurement->camera_index << ") from the "
        << "future! msg time "
        << aslam::time::timeNanosecondsToString(image_measurement->timestamp)
        << " current time: "
        << aslam::time::timeNanosecondsToString(current_time_ns)
        << " latency: " << latency_ns << "ns";

    statistics_->cam_latency_stats[image_measurement->camera_index].AddSample(
        static_cast<double>(latency_ns));
  }

  {
    std::lock_guard<std::mutex> lock(image_buffer_mutex_);
    CHECK_LT(
        image_measurement->camera_index,
        static_cast<int>(image_buffer_.size()));
    image_buffer_[image_measurement->camera_index].addValue(
        image_measurement->timestamp, image_measurement);
  }
}

void Synchronizer::processLidarMeasurement(
    const vi_map::RosLidarMeasurement::ConstPtr& lidar_measurement) {
  CHECK(lidar_measurement);

  const int64_t current_time_ns = aslam::time::nanoSecondsSinceEpoch();

  time_last_lidar_message_received_or_checked_ns_.store(current_time_ns);

  const int64_t timestamp_ns = lidar_measurement->getTimestampNanoseconds();

  VLOG(5) << "[MaplabNode-Synchronizer] processLidarMeasurement "
          << aslam::time::timeNanosecondsToString(timestamp_ns);

  if (statistics_) {
    const int64_t latency_ns = current_time_ns - timestamp_ns;
    LOG_IF(WARNING, latency_ns < 0)
        << "[MaplabNode-Synchronizer] Received Lidar message from the "
        << "future! msg time "
        << aslam::time::timeNanosecondsToString(timestamp_ns)
        << " current time: "
        << aslam::time::timeNanosecondsToString(current_time_ns)
        << " latency: " << latency_ns << "ns";
    statistics_->lidar_latency_stats.AddSample(static_cast<double>(latency_ns));
  }

  {
    std::lock_guard<std::mutex> lock(lidar_buffer_mutex_);
    lidar_buffer_.addValue(timestamp_ns, lidar_measurement);
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
    const vi_map::LoopClosureMeasurement::ConstPtr& loop_closure_measurement) {
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

  CHECK_GT(imu_measurements.cols(), 0);
  const int last_idx = imu_measurements.cols() - 1;
  const int64_t timestamp_ns = timestamps_nanoseconds[last_idx];

  VLOG(5) << "[MaplabNode-Synchronizer] processImuMeasurements "
          << aslam::time::timeNanosecondsToString(timestamp_ns);

  if (statistics_) {
    const int64_t latency_ns = current_time_ns - timestamp_ns;
    LOG_IF(WARNING, latency_ns < 0)
        << "[MaplabNode-Synchronizer] Received IMU message from the future! "
        << "msg time " << aslam::time::timeNanosecondsToString(timestamp_ns)
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

  const int64_t timestamp_ns = odometry.getTimestamp();
  VLOG(5) << "[MaplabNode-Synchronizer] processOdometryMeasurement "
          << aslam::time::timeNanosecondsToString(timestamp_ns);

  if (statistics_) {
    const int64_t latency_ns = current_time_ns - timestamp_ns;
    LOG_IF(WARNING, latency_ns < 0)
        << "[MaplabNode-Synchronizer] Received Odometry message from the "
        << "future! msg time "
        << aslam::time::timeNanosecondsToString(timestamp_ns)
        << " current time: "
        << aslam::time::timeNanosecondsToString(current_time_ns)
        << " latency: " << latency_ns << "ns";
    statistics_->odom_latency_stats.AddSample(static_cast<double>(latency_ns));
  }

  VLOG_IF(1, !received_first_odometry_message_.exchange(true))
      << "[MaplabNode-Synchronizer] Received first odometry message!";

  T_M_B_buffer_.bufferOdometryEstimate(odometry);

  releaseData();
}

void Synchronizer::processExternalFeatureMeasurement(
    const vi_map::ExternalFeaturesMeasurement::ConstPtr&
        external_features_measurement) {
  CHECK(external_features_measurement);

  const int64_t timestamp_ns =
      external_features_measurement->getTimestampNanoseconds();
  VLOG(5) << "[MaplabNode-Synchronizer] processExternalFeatureMeasurement "
          << aslam::time::timeNanosecondsToString(timestamp_ns);

  size_t buffer_index;
  {
    std::lock_guard<std::mutex> lock(external_features_buffer_mutex_);
    buffer_index = common::getChecked(
        external_features_id_to_index_map_,
        external_features_measurement->getSensorId());
  }

  const int64_t current_time_ns = aslam::time::nanoSecondsSinceEpoch();

  {
    std::lock_guard<std::mutex> lock(
        mutex_times_last_external_feature_messages_received_or_checked_ns_);
    times_last_external_feature_messages_received_or_checked_ns_[buffer_index] =
        current_time_ns;
  }

  if (statistics_) {
    // TODO(smauq): Finish statistics
    /*CHECK_LT(
        image_measurement->camera_index,
        static_cast<int>(statistics_->cam_latency_stats.size()));

    const int64_t latency_ns = current_time_ns - image_measurement->timestamp;
    LOG_IF(WARNING, latency_ns < 0)
        << "[MaplabNode-Synchronizer] Received image message (cam "
        << image_measurement->camera_index << ") from the "
        << "future! msg time "
        << aslam::time::timeNanosecondsToString(image_measurement->timestamp)
        << " current time: "
        << aslam::time::timeNanosecondsToString(current_time_ns)
        << " latency: " << latency_ns << "ns";

    statistics_->cam_latency_stats[image_measurement->camera_index].AddSample(
        static_cast<double>(latency_ns));*/
  }

  {
    std::lock_guard<std::mutex> lock(external_features_buffer_mutex_);
    external_features_buffer_[buffer_index].addValue(
        timestamp_ns, external_features_measurement);
  }
}

void Synchronizer::releaseCameraImages(
    const int64_t oldest_timestamp_ns, const int64_t newest_timestamp_ns) {
  CHECK_GE(newest_timestamp_ns, oldest_timestamp_ns);
  Aligned<std::vector, vio::ImageMeasurement::ConstPtr> all_extracted_images;
  {
    std::lock_guard<std::mutex> lock(image_buffer_mutex_);
    const size_t n_image_buffer = image_buffer_.size();
    for (size_t frame_idx = 0u; frame_idx < n_image_buffer; ++frame_idx) {
      common::TemporalBuffer<vio::ImageMeasurement::ConstPtr>& image_buffer =
          image_buffer_[frame_idx];

      // Drop these images, since there is no odometry data anymore for them.
      const size_t dropped_images =
          image_buffer.removeItemsBefore(oldest_timestamp_ns);
      LOG_IF(WARNING, dropped_images != 0u)
          << "[MaplabNode-Synchronizer] Could not find an odometry "
          << "transformation for " << dropped_images << " images "
          << "because it was already dropped from the buffer! "
          << "This might be okay during initialization.";

      image_skip_counter_ += dropped_images;
      Aligned<std::vector, vio::ImageMeasurement::ConstPtr> extracted_images;
      image_buffer.extractItemsBeforeIncluding(
          newest_timestamp_ns, &extracted_images);
      all_extracted_images.insert(
          all_extracted_images.end(), extracted_images.begin(),
          extracted_images.end());
    }
  }

  CHECK(visual_pipeline_) << "[MaplabNode-Synchronizer] The visual pipeline, "
                          << "which turns individual images "
                          << "into NFrames, has not been initialized yet!";

  VLOG(5) << "[MaplabNode-Synchronizer] releasing "
          << all_extracted_images.size()
          << " camera images into the visual processing pipeline for "
          << "synchronization and tracking.";

  // Insert all images that are released into the visual pipeline to synchronize
  // and track features.
  for (const vio::ImageMeasurement::ConstPtr& image_measurement :
       all_extracted_images) {
    CHECK_NOTNULL(image_measurement);
    CHECK_LE(image_measurement->timestamp, newest_timestamp_ns);
    CHECK_GE(image_measurement->timestamp, oldest_timestamp_ns);

    VLOG(5)
        << "[MaplabNode-Synchronizer] release camera image at "
        << aslam::time::timeNanosecondsToString(image_measurement->timestamp)
        << " into visual processing pipeline... (blocking if pipeline is full)";
    if (!visual_pipeline_->processImageBlockingIfFull(
            image_measurement->camera_index, image_measurement->image,
            image_measurement->timestamp,
            FLAGS_vio_nframe_sync_max_queue_size)) {
      LOG(ERROR)
          << "[MaplabNode-Synchronizer] Failed to process an image of camera "
          << image_measurement->camera_index << " into an NFrame at time "
          << image_measurement->timestamp << "ns!";
      shutdown();
    }
    VLOG(5) << "[MaplabNode-Synchronizer] done.";
  }

  // Release all visual frames that are ready.
  Aligned<std::vector, aslam::VisualNFrame::Ptr> finished_nframes;
  aslam::VisualNFrame::Ptr next_nframe = visual_pipeline_->getNext();
  size_t num_nframes_released = 0u;
  while (next_nframe) {
    VLOG(5) << "[MaplabNode-Synchronizer] retrieved finished VisualNFrame at "
            << aslam::time::timeNanosecondsToString(
                   next_nframe->getMinTimestampNanoseconds())
            << " from visual processing pipeline.";

    finished_nframes.emplace_back(next_nframe);
    next_nframe = visual_pipeline_->getNext();
    ++num_nframes_released;
  }

  VLOG(5) << "[MaplabNode-Synchronizer] retreived " << num_nframes_released
          << " finished VisualNFrames in total from the visual processing "
             "pipeline.";

  for (const aslam::VisualNFrame::Ptr& finished_nframe : finished_nframes) {
    const int64_t current_frame_timestamp_ns =
        finished_nframe->getMinTimestampNanoseconds();

    // Throttle the output rate of VisualNFrames to reduce the rate of which
    // the following nodes are running (e.g. tracker).
    if (aslam::time::isValidTime(previous_nframe_timestamp_ns_)) {
      if (current_frame_timestamp_ns - previous_nframe_timestamp_ns_ <
          min_nframe_timestamp_diff_ns_ *
              min_nframe_timestamp_diff_tolerance_factor_) {
        ++frame_skip_counter_;

        // Release the VisualNFrames to the callbacks.
        VLOG(5) << "[MaplabNode-Synchronizer] skipped finished VisualNFrame at "
                << aslam::time::timeNanosecondsToString(
                       current_frame_timestamp_ns)
                << " due to VIO frame throttling.";

        continue;
      }
    }
    previous_nframe_timestamp_ns_ = current_frame_timestamp_ns;

    // Release the VisualNFrames to the callbacks.
    VLOG(5) << "[MaplabNode-Synchronizer] release finished VisualNFrame at "
            << aslam::time::timeNanosecondsToString(current_frame_timestamp_ns)
            << " to callbacks...";
    vio::SynchronizedNFrame::Ptr new_nframe_measurement(
        new vio::SynchronizedNFrame);
    new_nframe_measurement->nframe = finished_nframe;
    {
      std::lock_guard<std::mutex> callback_lock(nframe_callback_mutex_);
      for (const std::function<void(const vio::SynchronizedNFrame::Ptr&)>&
               callback : nframe_callbacks_) {
        callback(new_nframe_measurement);
      }
    }
    VLOG(5) << "[MaplabNode-Synchronizer] done.";
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
    std::lock_guard<std::mutex> callback_lock(absolute_6dof_callback_mutex_);
    for (const std::function<void(const vi_map::Absolute6DoFMeasurement::Ptr&)>&
             callback : absolute_6dof_callbacks_) {
      callback(absolute_6dof_measurement);
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

  for (const vi_map::LoopClosureMeasurement::ConstPtr loop_closure_measurement :
       loop_closure_measurements) {
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

void Synchronizer::releaseExternalFeatures(
    const int64_t oldest_timestamp_ns, const int64_t newest_timestamp_ns) {
  CHECK_GE(newest_timestamp_ns, oldest_timestamp_ns);
  Aligned<std::vector, vi_map::ExternalFeaturesMeasurement::ConstPtr>
      all_external_features_measurements;
  {
    std::lock_guard<std::mutex> lock(external_features_buffer_mutex_);
    for (auto& buffer : external_features_buffer_) {
      // Drop these external feature messages, since there is no odometry data
      // anymore for them.
      // TODO(smauq): enable this only when tracking is needed, otherwise we are
      // fine, since we don't need odometry to attach the messages to the graph
      /*const size_t dropped_external_features =
          buffer.removeItemsBefore(oldest_timestamp_ns);
      LOG_IF(WARNING, dropped_external_features != 0u)
          << "[MaplabNode-Synchronizer] Could not find an odometry "
          << "transformation for " << dropped_external_features
          << " external features because it was already dropped from the "
          << "buffer! This might be okay during initialization.";
      external_features_skip_counter_ += dropped_external_features;*/

      Aligned<std::vector, vi_map::ExternalFeaturesMeasurement::ConstPtr>
          extracted_external_features_measurements;
      buffer.extractItemsBeforeIncluding(
          newest_timestamp_ns, &extracted_external_features_measurements);
      all_external_features_measurements.insert(
          all_external_features_measurements.end(),
          extracted_external_features_measurements.begin(),
          extracted_external_features_measurements.end());
    }
  }

  for (const vi_map::ExternalFeaturesMeasurement::ConstPtr
           external_features_measurement : all_external_features_measurements) {
    CHECK(external_features_measurement);
    std::lock_guard<std::mutex> callback_lock(
        external_features_callback_mutex_);
    for (const std::function<void(
             const vi_map::ExternalFeaturesMeasurement::ConstPtr&)>& callback :
         external_features_callbacks_) {
      callback(external_features_measurement);
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

  VLOG(5) << "[MaplabNode-Synchronizer] release data between "
          << aslam::time::timeNanosecondsToString(oldest_timestamp_ns)
          << " and "
          << aslam::time::timeNanosecondsToString(newest_timestamp_ns);

  // Release (or drop) camera images.
  {
    std::lock_guard<std::mutex> lock(
        mutex_times_last_camera_messages_received_or_checked_ns_);
    const bool all_times_valid = std::all_of(
        times_last_camera_messages_received_or_checked_ns_.begin(),
        times_last_camera_messages_received_or_checked_ns_.end(),
        [](int64_t time_ns) { return aslam::time::isValidTime(time_ns); });
    if (all_times_valid) {
      releaseCameraImages(oldest_timestamp_ns, newest_timestamp_ns);
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

  // Release (or drop) external feature measurements
  {
    std::lock_guard<std::mutex> lock(
        mutex_times_last_external_feature_messages_received_or_checked_ns_);
    const bool any_times_valid = std::any_of(
        times_last_external_feature_messages_received_or_checked_ns_.begin(),
        times_last_external_feature_messages_received_or_checked_ns_.end(),
        [](int64_t time_ns) { return aslam::time::isValidTime(time_ns); });
    if (any_times_valid) {
      releaseExternalFeatures(oldest_timestamp_ns, newest_timestamp_ns);
    }
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

void Synchronizer::registerPointCloudMapSensorMeasurementCallback(
    const std::function<
        void(const vi_map::RosPointCloudMapSensorMeasurement::ConstPtr&)>&
        callback) {
  std::lock_guard<std::mutex> lock(pointcloud_map_callback_mutex_);
  CHECK(callback);
  pointcloud_map_callbacks_.push_back(callback);
}

void Synchronizer::registerExternalFeaturesMeasurementCallback(
    const std::function<
        void(const vi_map::ExternalFeaturesMeasurement::ConstPtr&)>& callback) {
  std::lock_guard<std::mutex> lock(external_features_callback_mutex_);
  CHECK(callback);
  external_features_callbacks_.emplace_back(callback);
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

  LOG(INFO) << "[MaplabNode-Synchronizer] Shutting down. Skipped images: "
            << image_skip_counter_
            << ", skipped VisualNFrames: " << frame_skip_counter_
            << ", skipped lidar measurements: " << lidar_skip_counter_;
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

void Synchronizer::expectExternalFeaturesData() {
  {
    std::lock_guard<std::mutex> lock(
        mutex_times_last_external_feature_messages_received_or_checked_ns_);
    const int64_t time_since_last_epoch = aslam::time::nanoSecondsSinceEpoch();
    std::fill(
        times_last_external_feature_messages_received_or_checked_ns_.begin(),
        times_last_external_feature_messages_received_or_checked_ns_.end(),
        time_since_last_epoch);
  }
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
