#ifndef MAPLAB_NODE_ODOMETRY_ESTIMATE_H_
#define MAPLAB_NODE_ODOMETRY_ESTIMATE_H_

#include <Eigen/Core>
#include <aslam/common/memory.h>
#include <aslam/common/pose-types.h>
#include <maplab-common/macros.h>
#include <vio-common/vio-types.h>

// TODO(smauq): Fix this up, this is a completely useless duplication
namespace maplab {
struct OdometryEstimate {
  MAPLAB_POINTER_TYPEDEFS(OdometryEstimate);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  int64_t timestamp_ns;
  vio::ViNodeState vinode;
};
}  // namespace maplab
#endif  // MAPLAB_NODE_ODOMETRY_ESTIMATE_H_
