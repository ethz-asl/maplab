#ifndef DENSE_MAPPING_DENSE_MAPPING_EXTERNAL_INTERFACE_H_
#define DENSE_MAPPING_DENSE_MAPPING_EXTERNAL_INTERFACE_H_

#include <vector>

#include <maplab_msgs/PlaceLookupResponse.h>

namespace dense_mapping {

class ExternalInterface {
public:
  ExternalInterface(const bool enable_intra_mission_global_search,
      const bool enable_inter_mission_global_search);

private:
  const bool enable_intra_missions_;
  const bool enable_inter_missions_;

  std::vector<maplab_msgs::PlaceLookupResponse> latest_responses_;
};

} // namespace dense_mapping

#endif // DENSE_MAPPING_DENSE_MAPPING_EXTERNAL_INTERFACE_H_
