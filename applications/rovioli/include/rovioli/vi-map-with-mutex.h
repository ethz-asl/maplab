#ifndef ROVIOLI_VI_MAP_WITH_MUTEX_H_
#define ROVIOLI_VI_MAP_WITH_MUTEX_H_

#include <memory>
#include <mutex>

#include <Eigen/Core>
#include <maplab-common/macros.h>
#include <vi-map/vi-map.h>

namespace rovioli {

struct VIMapWithMutex {
  MAPLAB_POINTER_TYPEDEFS(VIMapWithMutex);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  vi_map::VIMap vi_map;
  mutable std::mutex mutex;
};

}  // namespace rovioli

#endif  // ROVIOLI_VI_MAP_WITH_MUTEX_H_
