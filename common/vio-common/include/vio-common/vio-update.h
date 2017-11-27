#ifndef VIO_COMMON_VIO_UPDATE_H_
#define VIO_COMMON_VIO_UPDATE_H_

#include <aslam/common/pose-types.h>
#include <maplab-common/macros.h>

#include "vio-common/vio-types.h"

namespace pose_graph {
class VertexId;
}

namespace vi_map {
class MissionId;
}

namespace vi_map {
class VIMap;
}

namespace vio {
struct SynchronizedNFrameImu;

enum class UpdateType { kInvalid, kNormalUpdate, kZeroVelocityUpdate };

struct VioUpdate {
  MAPLAB_POINTER_TYPEDEFS(VioUpdate);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Vio update states.
  int64_t timestamp_ns;
  EstimatorState vio_state;
  UpdateType vio_update_type;
  std::shared_ptr<const SynchronizedNFrameImu> keyframe_and_imudata;
  ViNodeState vinode;
  ViNodeCovariance vinode_covariance;

  // Localization update states.
  LocalizationState localization_state;
  aslam::Transformation T_G_M;

  inline bool check() const {
    return static_cast<bool>(keyframe_and_imudata);
  }
};

}  // namespace vio

#endif  // VIO_COMMON_VIO_UPDATE_H_
