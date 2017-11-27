#ifndef VISUALIZATION_VERTEX_LANDMARK_VISIBILITY_PLOTTER_H_
#define VISUALIZATION_VERTEX_LANDMARK_VISIBILITY_PLOTTER_H_

#include <memory>
#include <string>

#include <posegraph/edge.h>
#include <vi-map-helpers/vi-map-queries.h>

#include "visualization/color-palette.h"

namespace visualization {
class ViwlsGraphRvizPlotter;
}  // namespace visualization

namespace vi_map {
class Vertex;
class VIMap;
}  // namespace vi_map

namespace visualization {

class VertexLandmarkVisibilityPlotter {
 public:
  VertexLandmarkVisibilityPlotter(
      const std::shared_ptr<visualization::ViwlsGraphRvizPlotter>& plotter,
      const std::string& mission_id_string, const vi_map::VIMap& map);

  void publishVertexRaysAndIterate(const bool absolute_time_scale);

  inline void reset() {
    current_vertex_id_ = root_vertex_id_;
  }

 private:
  void iterateVertexAlongGraph();
  void getKeypointObservationTimeRange(
      const vi_map::Vertex& viwls_vertex, const bool absolute_time_scale,
      int64_t* timestamp_min, int64_t* timestamp_max) const;
  void forEachLandmarkSeenByVertex(
      const vi_map::Vertex& vertex,
      const std::function<void(
          const size_t frame_idx, const vi_map::LandmarkId& landmark_id,
          const vi_map::Vertex& storing_vertex)>& action) const;

  static constexpr unsigned int kFirstFrame = 0;
  std::shared_ptr<visualization::ViwlsGraphRvizPlotter> plotter_;
  const vi_map::VIMap& map_;
  const vi_map_helpers::VIMapQueries map_queries_;

  pose_graph::VertexId root_vertex_id_;
  int64_t root_vertex_timestamp_;
  pose_graph::VertexId current_vertex_id_;
  pose_graph::Edge::EdgeType graph_traversal_edge_type_;
  visualization::Palette color_palette_;

  const std::string vertex_rays_topic_;
};

}  // namespace visualization

#endif  // VISUALIZATION_VERTEX_LANDMARK_VISIBILITY_PLOTTER_H_
