#include "sparse-graph/common/sparse-graph-gflags.h"

namespace spg {

DEFINE_bool(
    sparse_graph_include_co_observability_weight, false,
    "If true, the adjacency edge weights include co-observability weights.");
DEFINE_bool(
    sparse_graph_include_lc_edge_weight, false,
    "If true, the adjacency edge weights include loop closure edge weights.");

}  // namespace spg
