#include "dense-mapping/dense-mapping-external-interface.h"

#include <maplab_msgs/PlaceLookup.h>

#include <glog/logging.h>
#include <ros/ros.h>

#include "dense-mapping/dense-mapping-gflags.h"

namespace dense_mapping {

ExternalInterface::ExternalInterface(
    const bool enable_intra_mission_global_search,
    const bool enable_inter_mission_global_search)
    : enable_intra_missions_(enable_intra_mission_global_search),
      enable_inter_missions_(enable_inter_mission_global_search) {
  if (!enable_intra_missions_ && !enable_inter_missions_) {
    LOG(ERROR) << "[DenseMapping] Global LLC search is disabled!";
    return;
  }
  LOG(INFO) << "[DenseMapping] Global LLC search is enabled!";
}

void ExternalInterface::poseLookupRequest() {
  CHECK(!FLAGS_dm_candidate_search_external_global_service.empty());
  maplab_msgs::PlaceLookup lookup;
  lookup.request.n_neighbors =
      FLAGS_dm_candidate_search_external_n_nearest_neighbors;
  lookup.request.confidence_threshold =
      FLAGS_dm_candidate_search_external_min_confidence;
  lookup.request.enable_intra_mission_lookup = enable_intra_missions_;
  lookup.request.enable_inter_mission_lookup = enable_inter_missions_;

  if (ros::service::call(
          FLAGS_dm_candidate_search_external_global_service, lookup)) {
    LOG(ERROR) << "CALL TO SERVICE WAS SUCCESSFULL!!!!!!!!!!!!!!!!!!!!!";
    const std::vector<std::string>& mission_ids_a =
        lookup.response.mission_ids_a;
    const std::vector<std::string>& mission_ids_b =
        lookup.response.mission_ids_b;
    LOG(ERROR) << "[DenseMapping] size of mission ids: " << mission_ids_a.size()
               << " and " << mission_ids_b.size();
  } else {
    LOG(ERROR) << "[DenseMapping - ExternalInterface] Call to service "
               << FLAGS_dm_candidate_search_external_global_service
               << " failed.";
  }
}

}  // namespace dense_mapping
