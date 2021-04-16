#ifndef MAPLAB_NODE_LIDAR_FEATURE_FLOW_H_
#define MAPLAB_NODE_LIDAR_FEATURE_FLOW_H_

#include <chrono>
#include <memory>

#include <aslam/cameras/ncamera.h>
#include <aslam/pipeline/visual-npipeline.h>
#include <lidar-feature-extraction/image-projection.h>
#include <message-flow/message-flow.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <sensors/imu.h>
#include <vio-common/pose-lookup-buffer.h>
#include <vio-common/vio-types.h>

#include "lidar-feature-extraction/image-projection.h"
#include "maplab-node/flow-topics.h"
#include "maplab-node/lidar-tracking.h"
#include "maplab-node/odometry-estimate.h"

namespace maplab {

class LidarFeatureFlow {
 public:
  explicit LidarFeatureFlow(
      aslam::NCamera::Ptr ncamera,
      const vio_common::PoseLookupBuffer& T_M_B_buffer)
      : projector_(T_M_B_buffer), lidar_tracking_(ncamera, T_M_B_buffer) {}

  void attachToMessageFlow(message_flow::MessageFlow* flow) {
    CHECK_NOTNULL(flow);
    static constexpr char kSubscriberNodeName[] = "LidarFeatures";

    std::function<void(vio::SynchronizedNFrame::ConstPtr)> publish_result =
        flow->registerPublisher<message_flow_topics::TRACKED_NFRAMES>();

    // NOTE: the publisher function pointer is copied intentionally; otherwise
    // we would capture a reference to a temporary.
    flow->registerSubscriber<message_flow_topics::SYNCED_LIDAR_MEASUREMENTS>(
        kSubscriberNodeName, message_flow::DeliveryOptions(),
        [publish_result,
         this](const vi_map::RosLidarMeasurement::ConstPtr& lidar_measurement) {
          CHECK(lidar_measurement);
          auto tic = std::chrono::steady_clock::now();
          bool success = this->projector_.projectToImage(lidar_measurement);
          auto tac = std::chrono::steady_clock::now();
          auto dur_projection = tac - tic;

          if (!success) {
            return;
          }

          // TODO(lbern) temporary solution to get the LK tracking done
          const cv::Mat& feature_image = this->projector_.getFeatureImage();
          cv::Mat resized_image;
          cv::resize(
              feature_image, resized_image, cv::Size(4096, 256), 0, 0,
              cv::INTER_NEAREST);

          tic = std::chrono::steady_clock::now();
          success = lidar_tracking_.trackSynchronizedLidarMeasurementCallback(
              this->projector_.getPointcloud(), resized_image,
              lidar_measurement->getTimestampNanoseconds());
          tac = std::chrono::steady_clock::now();
          auto dur_tracking = tac - tic;
          if (!success) {
            return;
          }

          LOG(INFO) << "=====================================";
          LOG(INFO) << "LiDAR projection takes: "
                    << std::chrono::duration_cast<std::chrono::milliseconds>(
                           dur_projection)
                           .count()
                    << "ms";
          LOG(INFO) << "LiDAR tracking takes: "
                    << std::chrono::duration_cast<std::chrono::milliseconds>(
                           dur_tracking)
                           .count()
                    << "ms";
          LOG(INFO) << "=====================================";

          // This will only fail for the first frame.
          aslam::VisualNFrame::Ptr tracked_nframe =
              lidar_tracking_.getLastTrackedLidarNFrame();
          vio::SynchronizedNFrame::Ptr synced_nframe =
              std::make_shared<vio::SynchronizedNFrame>(
                  tracked_nframe, vio::MotionType::kGeneralMotion);
          publish_result(synced_nframe);
        });
  }

 private:
  LidarFeatureExtraction::ImageProjection projector_;
  LidarTracking lidar_tracking_;
};

}  // namespace maplab

#endif  // MAPLAB_NODE_FEATURE_TRACKING_FLOW_H_
