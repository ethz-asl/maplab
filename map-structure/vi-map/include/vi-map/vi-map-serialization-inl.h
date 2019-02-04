#ifndef VI_MAP_VI_MAP_SERIALIZATION_INL_H_
#define VI_MAP_VI_MAP_SERIALIZATION_INL_H_

#include <algorithm>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <glog/logging.h>
#include <maplab-common/map-manager-config.h>
#include <maplab-common/multi-threaded-progress-bar.h>
#include <maplab-common/parallel-process.h>

#include "vi-map/vi-map-metadata.h"
#include "vi-map/vi-map-serialization.h"
#include "vi-map/vi-map.h"
#include "vi-map/vi_map.pb.h"

namespace vi_map {
namespace serialization {

template <typename ProtoProcessFunction>
void serializeToProtoAndCallFunction(
    const vi_map::VIMap& map, const VIMapMetadata& metadata,
    const ProtoProcessFunction& function) {
  common::MultiThreadedProgressBar progress_bar;
  std::vector<VIMapMetadata::value_type> files_list(
      metadata.begin(), metadata.end());
  std::atomic_size_t counter_vertex_files_atomic(0u);
  std::atomic_size_t counter_edges_files_atomic(0u);
  auto serialize_function = [&](const std::vector<size_t>& range) {
    size_t num_processed_tasks = 0u;
    progress_bar.setNumElements(range.size());
    proto::VIMap proto;
    for (size_t task_idx : range) {
      proto.Clear();
      const VIMapMetadata::value_type& entry = files_list[task_idx];
      switch (entry.first) {
        case VIMapFileType::kMissions:
          serializeMissionsAndBaseframes(map, &proto);
          break;
        case VIMapFileType::kVertices: {
          const size_t vertex_index = counter_vertex_files_atomic++;
          serializeVertices(
              map, vertex_index * backend::SaveConfig::kVerticesPerProtoFile,
              backend::SaveConfig::kVerticesPerProtoFile, &proto);
          break;
        }
        case VIMapFileType::kEdges: {
          const size_t edge_index = counter_edges_files_atomic++;
          serializeEdges(
              map, edge_index * backend::SaveConfig::kEdgesPerProtoFile,
              backend::SaveConfig::kEdgesPerProtoFile, &proto);
          break;
        }
        case VIMapFileType::kLandmarkIndex:
          serializeLandmarkIndex(map, &proto);
          break;
        case VIMapFileType::kOptionalSensorData:
          serializeOptionalSensorData(map, &proto);
          break;
      }

      CHECK(function(entry.second, proto));
      progress_bar.update(++num_processed_tasks);
    }
  };

  constexpr bool kAlwaysParallelize = true;
  constexpr size_t kMaxNumberOfThreads = 8u;
  const size_t num_threads =
      std::min(common::getNumHardwareThreads(), kMaxNumberOfThreads);
  common::ParallelProcess(
      files_list.size(), serialize_function, kAlwaysParallelize, num_threads);
}

template <typename ProtoSourceFunction>
void deserializeFromProtoFromFunction(
    const VIMapMetadata& metadata, const ProtoSourceFunction& function,
    VIMap* map) {
  CHECK(!metadata.empty());
  CHECK_NOTNULL(map);

  const size_t num_vertices_files = metadata.count(VIMapFileType::kVertices);
  std::vector<VIMapMetadata::value_type> metadata_vector(
      metadata.begin(), metadata.end());

  // Deserialize missions first.
  VIMapMetadataConstRange missions_range =
      metadata.equal_range(VIMapFileType::kMissions);
  for (VIMapMetadata::const_iterator it = missions_range.first;
       it != missions_range.second; ++it) {
    proto::VIMap proto;
    function(it->second, &proto);
    deserializeMissionsAndBaseframes(proto, map);
  }

  std::mutex map_mutex;
  auto deserialize_one_element = [&](const VIMapMetadata::value_type& entry) {
    proto::VIMap proto;
    function(entry.second, &proto);
    switch (entry.first) {
      case VIMapFileType::kMissions:
        LOG(FATAL)
            << "Missions need to be deserialized outside of this function!";
        break;
      case VIMapFileType::kVertices: {
        std::unique_lock<std::mutex> lock(map_mutex);
        deserializeVertices(proto, map);
        break;
      }
      case VIMapFileType::kEdges: {
        std::unique_lock<std::mutex> lock(map_mutex);
        deserializeEdges(proto, map);
        break;
      }
      case VIMapFileType::kLandmarkIndex: {
        std::unique_lock<std::mutex> lock(map_mutex);
        deserializeLandmarkIndex(proto, map);
        break;
      }
      case VIMapFileType::kOptionalSensorData: {
        std::unique_lock<std::mutex> lock(map_mutex);
        deserializeOptionalSensorData(proto, map);
        break;
      }
    }
  };

  common::MultiThreadedProgressBar progress_bar;
  std::size_t num_parsed_vertices_files = 0u;
  std::mutex cv_mutex;
  std::condition_variable cv_vertices_finished;
  auto deserialize_function = [&](const std::vector<size_t>& range) {
    size_t num_processed_tasks = 0u;
    progress_bar.setNumElements(range.size());
    std::vector<size_t> heldback_indices;
    for (const size_t index : range) {
      const VIMapMetadata::value_type& entry = metadata_vector[index];
      if (entry.first == VIMapFileType::kMissions) {
        // Missions will be treated separately and done before everything else.
        progress_bar.update(++num_processed_tasks);
        continue;
      }

      // Handle vertices first.
      if (entry.first == VIMapFileType::kVertices) {
        deserialize_one_element(entry);
        progress_bar.update(++num_processed_tasks);

        {
          std::unique_lock<std::mutex> lock(cv_mutex);
          ++num_parsed_vertices_files;
        }
        cv_vertices_finished.notify_all();
      } else {
        heldback_indices.emplace_back(index);
      }
    }

    // Wait for all vertices protos to be parsed.
    {
      std::unique_lock<std::mutex> lock(cv_mutex);
      while (num_parsed_vertices_files != num_vertices_files) {
        cv_vertices_finished.wait_for(lock, std::chrono::milliseconds(50));
      }
      CHECK_EQ(num_vertices_files, num_parsed_vertices_files);
    }

    // Parse other files.
    for (const size_t index : heldback_indices) {
      const VIMapMetadata::value_type& entry = metadata_vector[index];
      CHECK(
          entry.first != VIMapFileType::kMissions &&
          entry.first != VIMapFileType::kVertices);
      deserialize_one_element(entry);
      progress_bar.update(++num_processed_tasks);
    }
  };

  constexpr bool kAlwaysParallelize = true;
  constexpr size_t kMaxNumberOfThreads = 8;
  const size_t num_threads =
      std::min(common::getNumHardwareThreads(), kMaxNumberOfThreads);
  common::ParallelProcess(
      metadata.size(), deserialize_function, kAlwaysParallelize, num_threads);
}

}  // namespace serialization
}  // namespace vi_map

#endif  // VI_MAP_VI_MAP_SERIALIZATION_INL_H_
