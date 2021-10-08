#include "sparse-graph/common/sparse-graph-gflags.h"

namespace spg {

DEFINE_bool(
    sparse_graph_include_rotational_weight, true,
    "If true, the adjacency edge weights include co-observability weights.");
DEFINE_bool(
    sparse_graph_include_co_observability_weight, false,
    "If true, the adjacency edge weights include co-observability weights.");
DEFINE_bool(
    sparse_graph_include_lc_edge_weight, false,
    "If true, the adjacency edge weights include loop closure edge weights.");
DEFINE_double(
    sparse_graph_min_distance_to_last_node_m, 1.0,
    "Minimum distance to the last node.");
DEFINE_double(
    sparse_graph_min_rotation_to_last_node_rad, 0.2,
    "Minimum rotation to the last node.");

}  // namespace spg
