#ifndef ROVIOLI_ROVIO_ESTIMATE_H_
#define ROVIOLI_ROVIO_ESTIMATE_H_

#include <Eigen/Core>
#include <aslam/common/memory.h>
#include <aslam/common/pose-types.h>
#include <maplab-common/macros.h>
#include <vio-common/vio-types.h>

namespace rovioli {
struct RovioEstimate {
  MAPLAB_POINTER_TYPEDEFS(RovioEstimate);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  int64_t timestamp_ns;
  vio::ViNodeState vinode;

  aslam::Transformation T_G_M;
  bool has_T_G_M;

  // Mapping maplab camera index to the estimated camera extrinsics.
  AlignedUnorderedMap<size_t, aslam::Transformation>
      maplab_camera_index_to_T_C_B;
};
}  // namespace rovioli
#endif  // ROVIOLI_ROVIO_ESTIMATE_H_
