#ifndef ROVIOLI_DATASOURCE_ROSBAG_H_
#define ROVIOLI_DATASOURCE_ROSBAG_H_

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <aslam/common/time.h>
#include <vio-common/rostopic-settings.h>
#include <vio-common/vio-types.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/image_encodings.h>
#pragma GCC diagnostic pop

#include "rovioli/datasource.h"

namespace rovioli {

class DataSourceRosbag : public DataSource {
 public:
  DataSourceRosbag(
      const std::string& rosbag_path_filename,
      const vio_common::RosTopicSettings& settings);
  virtual ~DataSourceRosbag();
  void initialize();

  virtual void startStreaming();
  virtual void shutdown();
  virtual bool allDataStreamed() const {
    return all_data_streamed_;
  }
  virtual std::string getDatasetName() const;

 private:
  void streamingWorker();

  std::unique_ptr<std::thread> streaming_thread_;
  std::atomic<bool> shutdown_requested_;
  std::atomic<bool> all_data_streamed_;
  std::string rosbag_path_filename_;
  vio_common::RosTopicSettings ros_topics_;

  std::unique_ptr<rosbag::Bag> bag_;
  std::unique_ptr<rosbag::View> bag_view_;
};

}  // namespace rovioli

#endif  // ROVIOLI_DATASOURCE_ROSBAG_H_
