#ifndef MAPLAB_NODE_MAPLAB_NODE_H_
#define MAPLAB_NODE_MAPLAB_NODE_H_

#include <atomic>
#include <memory>
#include <string>

#include <message-flow/message-flow.h>
#include <sensors/imu.h>
#include <sensors/lidar.h>
#include <vio-common/pose-lookup-buffer.h>

#include "maplab-node/data-publisher-flow.h"
#include "maplab-node/datasource-flow.h"
#include "maplab-node/feature-tracking-flow.h"
#include "maplab-node/map-builder-flow.h"
#include "maplab-node/synchronizer-flow.h"

namespace maplab {

class MaplabNode final {
 public:
  MaplabNode(
      const std::string& sensor_calibration_file,
      const std::string& save_map_folder,
      message_flow::MessageFlow* const flow);

  ~MaplabNode();

  // Once the node is started, the configuration cannot be changed anymore.
  void start();
  void shutdown();

  // Save the map to disk. Optionally keyframe, optimize and summarize the map.
  bool saveMapAndOptionallyOptimize(
      const std::string& path, const bool overwrite_existing_map,
      const bool process_to_localization_map, const bool stop_mapping);

  bool isDataSourceExhausted();

 private:
  // Map constraints.
  void initializeVisualMapping();
  void initializeExternalFeatures();
  void initializeLidarMapping();
  void initializeOdometrySource();
  void initializeInertialMapping();
  void initializeAbsolute6DoFSource();
  void initializeWheelOdometrySource();
  void initializeLoopClosureSource();
  void initializePointCloudMapSource();

  message_flow::MessageFlow* const message_flow_;

  std::unique_ptr<DataSourceFlow> datasource_flow_;
  std::unique_ptr<DataPublisherFlow> data_publisher_flow_;

  std::unique_ptr<MapBuilderFlow> map_builder_flow_;

  std::unique_ptr<SynchronizerFlow> synchronizer_flow_;
  std::unique_ptr<FeatureTrackingFlow> tracker_flow_;

  // Set to true once the data-source has played back all its data. Will never
  // be true for infinite data-sources (live-data).
  std::atomic<bool> is_datasource_exhausted_;

  bool is_running_;

  // The MaplabNode is the owner of the sensor manager and passes it to the
  // other blocks as const reference or raw pointer depending on their use-case.
  std::unique_ptr<vi_map::SensorManager> sensor_manager_;

  mutable std::mutex mutex_;
};

}  // namespace maplab

#endif  // MAPLAB_NODE_MAPLAB_NODE_H_
