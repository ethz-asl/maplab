#ifndef MAPLAB_NODE_MAP_BUILDER_FLOW_H_
#define MAPLAB_NODE_MAP_BUILDER_FLOW_H_

#include <atomic>
#include <functional>
#include <memory>
#include <string>

#include <aslam/cameras/ncamera.h>
#include <message-flow/message-flow.h>
#include <online-map-builders/stream-map-builder.h>
#include <sensors/imu.h>
#include <vi-map/vi-map.h>
#include <vio-common/vio-types.h>

#include "maplab-node/flow-topics.h"
#include "maplab-node/map-update-builder.h"
#include "maplab-node/vi-map-with-mutex.h"

namespace maplab {

// Note that the VisualNFrames are not deep-copied and the passed shared-pointer
// is direcly added to the map.
class MapBuilderFlow {
 public:
  MapBuilderFlow(
      const vi_map::SensorManager& sensor_manager,
      const std::string& save_map_folder,
      const vio_common::PoseLookupBuffer& T_M_B_buffer);
  void attachToMessageFlow(message_flow::MessageFlow* flow);

  bool saveMapAndOptionallyOptimize(
      const std::string& path, const bool overwrite_existing_map,
      const bool process_to_localization_map, const bool stop_mapping);

 private:
  const vi_map::SensorManager& sensor_manager_;

  VIMapWithMutex::Ptr map_with_mutex_;
  pose_graph::VertexId last_vertex_of_previous_map_saving_;

  // If set then all incoming callbacks that cause operations on the map will be
  // rejected. This is used during shutdown.
  std::atomic<bool> mapping_terminated_;

  MapUpdateBuilder map_update_builder_;
  std::string external_resource_folder_;
  online_map_builders::StreamMapBuilder stream_map_builder_;
};

}  // namespace maplab

#endif  // MAPLAB_NODE_MAP_BUILDER_FLOW_H_
