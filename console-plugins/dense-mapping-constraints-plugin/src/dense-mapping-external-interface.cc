#include "dense-mapping/dense-mapping-external-interface.h"

#include <glog/logging.h>

namespace dense_mapping {

ExternalInterface::ExternalInterface(
    const bool enable_intra_mission_global_search,
    const bool enable_inter_mission_global_search)
    : enable_intra_missions_(enable_intra_mission_global_search),
      enable_inter_missions_(enable_inter_mission_global_search) {
  if (enable_intra_missions_ || enable_inter_missions_) {
    LOG(INFO) << "[DenseMapping] Global LLC search is enabled!";
  }
}

}  // namespace dense_mapping
