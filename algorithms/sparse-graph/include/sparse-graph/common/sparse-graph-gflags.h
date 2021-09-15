#ifndef SPARSE_GRAPH_COMMON_SPARSE_GRAPH_GFLAGS_H_
#define SPARSE_GRAPH_COMMON_SPARSE_GRAPH_GFLAGS_H_

#include <gflags/gflags.h>
#include <glog/logging.h>

namespace spg {

DECLARE_bool(sparse_graph_include_co_observability_weight);
DECLARE_bool(sparse_graph_include_lc_edge_weight);
DECLARE_bool(sparse_graph_add_every_node);
DECLARE_double(sparse_graph_min_distance_to_last_node_m);
DECLARE_double(sparse_graph_min_rotation_to_last_node_rad);

}  // namespace spg

#endif  // SPARSE_GRAPH_COMMON_SPARSE_GRAPH_GFLAGS_H_
