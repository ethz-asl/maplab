#ifndef VISUALIZATION_SEQUENTIAL_PLOTTER_H_
#define VISUALIZATION_SEQUENTIAL_PLOTTER_H_

#include <string>
#include <unordered_set>

#include <aslam/common/memory.h>

#include "visualization/constant-velocity-smoother.h"

namespace pose_graph {
class VertexId;
}  // namespace pose_graph

namespace vi_map {
class MissionId;
class VIMap;
}  // namespace vi_map

namespace visualization {
class ViwlsGraphRvizPlotter;

class SequentialPlotter final {
 public:
  SequentialPlotter() = default;
  ~SequentialPlotter() = default;

  void publishMissionsSequentially(
      const vi_map::VIMap& map,
      const std::unordered_set<vi_map::MissionId>& missions);

 private:
  void publishVertexPoseAsTFSmoothed(
      const vi_map::VIMap& map, const pose_graph::VertexId& vertex_id) const;

  inline void resetSmoothers() {
    smoothers_.clear();
  }

  mutable AlignedMap<std::string, ConstantVelocitySmoother> smoothers_;
};

}  // namespace visualization

#endif  // VISUALIZATION_SEQUENTIAL_PLOTTER_H_
