#ifndef ROVIOLI_ROVIO_ESTIMATE_H_
#define ROVIOLI_ROVIO_ESTIMATE_H_

#include <Eigen/Core>
#include <aslam/common/pose-types.h>
#include <maplab-common/macros.h>
#include <vio-common/vio-types.h>

namespace rovioli {
struct RovioEstimate {
  MAPLAB_POINTER_TYPEDEFS(RovioEstimate);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double timestamp_s;
  vio::ViNodeState vinode;

  aslam::Transformation T_G_M;
  bool has_T_G_M;
};
}  // namespace rovioli
#endif  // ROVIOLI_ROVIO_ESTIMATE_H_
