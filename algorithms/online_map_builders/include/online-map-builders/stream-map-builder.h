#ifndef ONLINE_MAP_BUILDERS_STREAM_MAP_BUILDER_H_
#define ONLINE_MAP_BUILDERS_STREAM_MAP_BUILDER_H_

#include <memory>

#include <Eigen/Dense>
#include <posegraph/unique-id.h>
#include <sensors/imu.h>
#include <sensors/sensor.h>
#include <vi-map-helpers/vi-map-manipulation.h>
#include <vi-map/unique-id.h>
#include <vi-map/vi-mission.h>

namespace aslam {
class NCamera;
class VisualNFrame;
}
namespace vio {
class VioUpdate;
class ViNodeState;
}
namespace vi_map {
class VIMap;
}

namespace online_map_builders {
class StreamMapBuilder {
 public:
  StreamMapBuilder(
      const std::shared_ptr<aslam::NCamera>& camera_rig, vi_map::VIMap* map);
  StreamMapBuilder(
      const std::shared_ptr<aslam::NCamera>& camera_rig,
      vi_map::Imu::UniquePtr imu, vi_map::VIMap* map);

  // Deep copies the nframe.
  void apply(const vio::VioUpdate& update);
  void apply(const vio::VioUpdate& update, bool deep_copy_nframe);

  vi_map::MissionId getMissionId() const {
    return mission_id_;
  }

  pose_graph::VertexId getRootVertexId() const;
  pose_graph::VertexId getLastVertexId() const;

  void removeAllVerticesAfterVertexId(
      const pose_graph::VertexId& vertiex_id_from,
      pose_graph::VertexIdList* removed_vertex_ids);

  bool checkConsistency() const;

 private:
  void addRootViwlsVertex(
      const std::shared_ptr<aslam::VisualNFrame>& nframe,
      const vio::ViNodeState& vinode_state);

  void addViwlsVertexAndEdge(
      const std::shared_ptr<aslam::VisualNFrame>& nframe,
      const vio::ViNodeState& vinode_state,
      const Eigen::Matrix<int64_t, 1, Eigen::Dynamic>& imu_timestamps,
      const Eigen::Matrix<double, 6, Eigen::Dynamic>& imu_data);

  pose_graph::VertexId addViwlsVertex(
      const std::shared_ptr<aslam::VisualNFrame>& nframe,
      const vio::ViNodeState& vinode_state);

  void addImuEdge(
      pose_graph::VertexId target_vertex_id,
      const Eigen::Matrix<int64_t, 1, Eigen::Dynamic>& imu_timestamps,
      const Eigen::Matrix<double, 6, Eigen::Dynamic>& imu_measurements);

  inline const vi_map::VIMap* constMap() const;

  vi_map::VIMap* const map_;
  vi_map_helpers::VIMapManipulation manipulation_;
  const vi_map::MissionId mission_id_;
  pose_graph::VertexId last_vertex_;
  const std::shared_ptr<aslam::NCamera> camera_rig_;

  static constexpr size_t kKeepNMostRecentImages = 10u;
};

}  // namespace online_map_builders

#endif  // ONLINE_MAP_BUILDERS_STREAM_MAP_BUILDER_H_
