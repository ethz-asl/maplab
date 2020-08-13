#include <aslam/matcher/match-helpers.h>
#include <aslam/matcher/match-visualization.h>
#include <aslam/visualization/basic-visualization.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <maplab-common/data-synchronizer.h>
#include <maplab-common/sigint-breaker.h>
#include <maplab-common/threading-helpers.h>
#include <message-flow/message-dispatcher-fifo.h>
#include <message-flow/message-flow.h>
#include <ros/ros.h>
#include <rovioli/datasource-flow.h>
#include <rovioli/imu-camera-synchronizer-flow.h>
#include <sensors/imu.h>
#include <signal.h>

#include <atomic>
#include <memory>
#include <utility>
#include <vector>

#include "feature-tracking-pipelines/feature-tracker-factory.h"
#include "feature-tracking-pipelines/feature-tracker-gyro-aided.h"

DEFINE_string(ncamera_calibration, "ncamera.yaml", "Camera calibration yaml.");
DEFINE_string(camera_topic, "/cam0/image_raw", "Image topic.");
DEFINE_string(imu_topic, "/imu0", "Image topic.");

typedef feature_tracking_pipelines::FeaturePipelineDebugData
    FeaturePipelineDebugData;

void VisualizeTracking(
    const std::pair<aslam::VisualNFrame::Ptr, aslam::VisualNFrame::Ptr>&
        nframe_k_kp1,
    const std::shared_ptr<FeaturePipelineDebugData>& debug_data) {
  CHECK(nframe_k_kp1.second);
  CHECK(debug_data);

  // Skip the first nframe.
  if (nframe_k_kp1.first == nullptr) {
    return;
  }
  LOG(WARNING) << nframe_k_kp1.first->getMinTimestampNanoseconds();
  LOG(WARNING) << nframe_k_kp1.second->getMinTimestampNanoseconds();
  LOG(WARNING) << debug_data->timestamp_nframe_k;
  LOG(WARNING) << debug_data->timestamp_nframe_kp1;

  CHECK_EQ(
      nframe_k_kp1.first->getNumFrames(), nframe_k_kp1.second->getNumFrames());
  CHECK_EQ(
      nframe_k_kp1.first->getMinTimestampNanoseconds(),
      debug_data->timestamp_nframe_k);
  CHECK_EQ(
      nframe_k_kp1.second->getMinTimestampNanoseconds(),
      debug_data->timestamp_nframe_kp1);

  // Frame-to-frame matches visualization.
  for (size_t frame_idx = 0; frame_idx < nframe_k_kp1.second->getNumFrames();
       ++frame_idx) {
    aslam::FrameToFrameMatches matches_kp1_k;
    aslam::extractMatchesFromTrackIdChannel(
        nframe_k_kp1.second->getFrame(frame_idx),
        nframe_k_kp1.first->getFrame(frame_idx), &matches_kp1_k);

    cv::Mat matches_image;
    aslam::drawVisualFrameKeyPointsAndMatches(
        nframe_k_kp1.second->getFrame(frame_idx),
        nframe_k_kp1.first->getFrame(frame_idx),
        aslam::FeatureVisualizationType::kHorizontal, matches_kp1_k,
        &matches_image);

    // Single frame tracking visualization.
    cv::Mat tracking_image;
    cv::cvtColor(
        nframe_k_kp1.second->getFrame(frame_idx).getRawImage(), tracking_image,
        CV_GRAY2BGR);

    aslam::Matches raw_matches_kp1_k;
    aslam::convertFrameToFrameMatchesToMatches(
        matches_kp1_k, &raw_matches_kp1_k);
    aslam_cv_visualization::drawKeypointMatches(
        nframe_k_kp1.second->getFrame(frame_idx),
        nframe_k_kp1.first->getFrame(frame_idx), raw_matches_kp1_k,
        aslam_cv_visualization::kGreen, aslam_cv_visualization::kGreen,
        &tracking_image);

    aslam::Matches raw_outliers_matches_kp1_k;
    aslam::convertFrameToFrameMatchesToMatches(
        debug_data->outlier_matches_kp1_k[frame_idx],
        &raw_outliers_matches_kp1_k);

    // TODO(schneith): Returning of outlier tracks does not work anymore.
    // Earlier we set the TrackId to -1 but now we delete it, so the indices
    // change....

    //    aslam_cv_visualization::drawKeypointMatches(
    //        nframe_k_kp1.second->getFrame(frame_idx),
    //        nframe_k_kp1.first->getFrame(frame_idx),
    //        raw_outliers_matches_kp1_k,
    //        aslam_cv_visualization::kRed, aslam_cv_visualization::kRed,
    //        &tracking_image);

    // Display the images.
    std::string window_name = "matches - frame " + std::to_string(frame_idx);
    cv::namedWindow(window_name);
    cv::imshow(window_name, matches_image);

    window_name = "tracking - frame " + std::to_string(frame_idx);
    cv::namedWindow(window_name);
    cv::imshow(window_name, tracking_image);
    cv::waitKey(1);
  }
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  ros::init(argc, argv, "rovioli");
  ros::NodeHandle nh;

  // Load camera calibration.
  aslam::NCamera::Ptr camera_system(new aslam::NCamera());
  camera_system->deserializeFromFile(FLAGS_ncamera_calibration);
  CHECK(camera_system) << "Could not load the camera calibration from: \'"
                       << FLAGS_ncamera_calibration << "\'";

  // Construct the application.
  ros::AsyncSpinner ros_spinner(common::getNumHardwareThreads());
  std::unique_ptr<message_flow::MessageFlow> message_flow(
      message_flow::MessageFlow::create<message_flow::MessageDispatcherFifo>(
          common::getNumHardwareThreads()));

  // TODO(schneith): Adapt interface to take a rostopic directly instead of an
  // IMU structure.
  vi_map::Imu imu(aslam::createRandomId<aslam::SensorId>(), FLAGS_imu_topic);
  rovioli::DataSourceFlow datasource_flow(*camera_system, imu);
  datasource_flow.attachToMessageFlow(message_flow.get());
  rovioli::ImuCameraSynchronizerFlow synchronizer_flow(camera_system);
  synchronizer_flow.attachToMessageFlow(message_flow.get());

  std::atomic<bool> is_datasource_exhausted;
  datasource_flow.registerEndOfDataCallback(
      [&]() { is_datasource_exhausted.store(true); });

  // Create a data synchronizer for visualization of outlier and tracking data.
  typedef std::pair<aslam::VisualNFrame::Ptr /*nframe_k*/,
                    aslam::VisualNFrame::Ptr /*nframe_kp1*/>
      NFramePair;

  struct NFramePairTimestampExtractor {
    int64_t operator()(const NFramePair& nframe_k_kp1) {
      return nframe_k_kp1.second->getMinTimestampNanoseconds();
    }
  };
  struct FeaturePipelineDebugDataTimestampExtractor {
    int64_t operator()(const std::shared_ptr<FeaturePipelineDebugData>& data) {
      CHECK(data);
      return data->timestamp_nframe_kp1;
    }
  };
  typedef common::DataSynchronizer<
      NFramePair, std::shared_ptr<FeaturePipelineDebugData>,
      NFramePairTimestampExtractor, FeaturePipelineDebugDataTimestampExtractor>
      DataSynchronizer;
  DataSynchronizer data_synchronizer;
  data_synchronizer.registerCallback(std::bind(
      &VisualizeTracking, std::placeholders::_1, std::placeholders::_2));

  // Create and hook-up the feature trackers.
  feature_tracking_pipelines::FeatureTrackerGyroAided feature_tracker(
      *camera_system,
      feature_tracking_pipelines::CreateFeaturePipelineFromGFlags());

  feature_tracker.setOptionalDebugDataCallback(std::bind(
      &DataSynchronizer::processTypeB, &data_synchronizer,
      std::placeholders::_1));

  std::function<void(const NFramePair&)> sync_tracked_nframe_for_viz =
      std::bind(
          &DataSynchronizer::processTypeA, &data_synchronizer,
          std::placeholders::_1);

  message_flow->registerSubscriber<message_flow_topics::SYNCED_NFRAMES_AND_IMU>(
      "FeatureTrackingFlow", message_flow::DeliveryOptions(),
      [sync_tracked_nframe_for_viz,
       &feature_tracker](const vio::SynchronizedNFrameImu::Ptr& nframe_imu) {
        CHECK(nframe_imu);
        static aslam::VisualNFrame::Ptr last_nframe_ = nullptr;

        const bool success = feature_tracker.processNFrame(
            nframe_imu->imu_timestamps, nframe_imu->imu_measurements,
            nframe_imu->nframe.get());
        CHECK(success);

        NFramePair nframe_k_kp1;
        nframe_k_kp1.first = last_nframe_;
        nframe_k_kp1.second = nframe_imu->nframe;
        sync_tracked_nframe_for_viz(nframe_k_kp1);

        last_nframe_ = nframe_imu->nframe;
      });

  // Start the pipeline. The ROS spinner will handle SIGINT for us and abort
  // the application on CTRL+C.
  ros_spinner.start();

  CHECK(!is_datasource_exhausted.load());
  datasource_flow.startStreaming();
  VLOG(1) << "Starting data source...";

  while (ros::ok() && !is_datasource_exhausted.load()) {
    VLOG_EVERY_N(1, 10) << "\n" << message_flow->printDeliveryQueueStatistics();
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  datasource_flow.shutdown();
  message_flow->shutdown();
  message_flow->waitUntilIdle();

  return 0;
}
