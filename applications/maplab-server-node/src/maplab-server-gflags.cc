#include "maplab-server-node/maplab-server-gflags.h"

namespace maplab {

DEFINE_bool(
    maplab_server_enable_sparse_graph_computation, false,
    "If enabled, the mapping server will build and publish the sparse graph.");

}  // namespace maplab
