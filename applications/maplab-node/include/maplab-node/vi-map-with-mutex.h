#ifndef MAPLAB_NODE_VI_MAP_WITH_MUTEX_H_
#define MAPLAB_NODE_VI_MAP_WITH_MUTEX_H_

#include <memory>
#include <mutex>

#include <Eigen/Core>
#include <maplab-common/macros.h>
#include <vi-map/vi-map.h>

namespace maplab {

struct VIMapWithMutex {
  MAPLAB_POINTER_TYPEDEFS(VIMapWithMutex);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  vi_map::VIMap vi_map;
  mutable std::mutex mutex;
};

}  // namespace maplab

#endif  // MAPLAB_NODE_VI_MAP_WITH_MUTEX_H_
