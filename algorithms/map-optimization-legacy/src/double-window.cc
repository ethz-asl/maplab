#include "map-optimization-legacy/double-window.h"

#include <glog/logging.h>

namespace map_optimization_legacy {

void DoubleWindow::getAllWindowVertices(
    pose_graph::VertexIdSet* all_window_vertex_ids) const {
  CHECK_NOTNULL(all_window_vertex_ids)->clear();
  *all_window_vertex_ids = vertices_outer_window_;
  all_window_vertex_ids->insert(
      vertices_inner_window_.begin(), vertices_inner_window_.end());
}

bool DoubleWindow::isVertexInDoubleWindow(
    const pose_graph::VertexId& query_verex_id) const {
  if (vertices_inner_window_.count(query_verex_id) > 0u) {
    return true;
  }
  if (vertices_outer_window_.count(query_verex_id) > 0u) {
    return true;
  }
  return false;
}

}  // namespace map_optimization_legacy
