#include "visualization/sliding-window-plotter.h"

#include "visualization/common-rviz-visualization.h"

namespace visualization {

void SlidingWindowPlotter::plotMissionWithSlidingWindow(
    const vi_map::VIMap& map, const pose_graph::VertexId& last_vertex_id) {
  CHECK(last_vertex_id.isValid());

  const vi_map::Vertex& last_vertex = map.getVertex(last_vertex_id);
  pose_graph::VertexId current_vertex_id;

  // Verify if the last is really the last. This check can be skipped
  // if performance is a concern.
  CHECK(
      !map.getNextVertex(
          last_vertex_id,
          map.getGraphTraversalEdgeType(last_vertex.getMissionId()),
          &current_vertex_id));

  pose_graph::VertexIdSet sliding_window_vertices;
  current_vertex_id = last_vertex_id;
  for (size_t i = 0u; i < kSlidingWindowSize; ++i) {
    sliding_window_vertices.insert(current_vertex_id);

    if (!map.getPreviousVertex(
            current_vertex_id,
            map.getGraphTraversalEdgeType(last_vertex.getMissionId()),
            &current_vertex_id)) {
      break;
    }
  }

  plotMissionWithSlidingWindow(map, sliding_window_vertices);
}

void SlidingWindowPlotter::plotMissionWithSlidingWindow(
    const vi_map::VIMap& map,
    const pose_graph::VertexIdSet& sliding_window_vertices) {
  CHECK(plotter_ != nullptr) << "You need to initialize the plotter first.";

  pose_graph::VertexIdSet vertices_out_of_window;
  for (const pose_graph::VertexId& last_vertex_id :
       last_sliding_window_vertices_) {
    if (sliding_window_vertices.count(last_vertex_id) == 0u) {
      vertices_out_of_window.insert(last_vertex_id);
    }
  }

  CHECK_LE(vertices_out_of_window.size(), 1u)
      << "Currently only the shift-by-one sliding window is supported.";

  pose_graph::VertexIdList vertices_out_of_window_list;
  if (!vertices_out_of_window.empty()) {
    vertices_out_of_window_list.push_back(*vertices_out_of_window.begin());
  }

  // Plot sliding window.
  last_sliding_window_vertices_.clear();
  last_sliding_window_vertices_.insert(
      last_sliding_window_vertices_.end(), sliding_window_vertices.begin(),
      sliding_window_vertices.end());
  plotter_->publishVertices(map, last_sliding_window_vertices_);

  CHECK(sliding_window_landmarks_.empty());
  plotter_->appendLandmarksToSphereVector(
      map, last_sliding_window_vertices_, kSlidingWindowColor,
      &sliding_window_landmarks_);

  // Plot out of window vertices, they will note move anymore.
  plotter_->publishVertices(map, vertices_out_of_window_list);
  plotter_->appendLandmarksToSphereVector(
      map, vertices_out_of_window_list, kOutOfWindowColor,
      &out_of_window_landmarks_);

  visualization::SphereVector common_sphere_vector(
      out_of_window_landmarks_.begin(), out_of_window_landmarks_.end());
  common_sphere_vector.insert(
      common_sphere_vector.end(), sliding_window_landmarks_.begin(),
      sliding_window_landmarks_.end());

  visualization::publishSpheresAsPointCloud(
      common_sphere_vector, visualization::kDefaultMapFrame, landmarks_topic_);

  sliding_window_landmarks_.clear();
}

}  // namespace visualization
