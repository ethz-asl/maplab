#ifndef SPARSE_GRAPH_COMMON_SPARSE_GRAPH_GFLAGS_H_
#define SPARSE_GRAPH_COMMON_SPARSE_GRAPH_GFLAGS_H_

#include <gflags/gflags.h>
#include <glog/logging.h>

namespace spg {

DECLARE_bool(sparse_graph_include_co_observability_weight);
DECLARE_bool(sparse_graph_include_lc_edge_weight);

}  // namespace spg

#endif  // SPARSE_GRAPH_COMMON_SPARSE_GRAPH_GFLAGS_H_
