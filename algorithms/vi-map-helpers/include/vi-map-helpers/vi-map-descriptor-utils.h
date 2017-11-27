#ifndef VI_MAP_HELPERS_VI_MAP_DESCRIPTOR_UTILS_H_
#define VI_MAP_HELPERS_VI_MAP_DESCRIPTOR_UTILS_H_

#include <Eigen/Core>
#include <vi-map/vi-map.h>

namespace vi_map_helpers {
// Retrieves the descriptor of the given observations that has the smallest
// accumulated descriptor-distance to all other observations.
void getDescriptorClosestToMedian(
    const vi_map::KeypointIdentifierList& observations,
    const vi_map::VIMap& map,
    Eigen::Matrix<unsigned char, Eigen::Dynamic, 1>* descriptor,
    size_t* descriptor_index);

}  // namespace vi_map_helpers

#endif  // VI_MAP_HELPERS_VI_MAP_DESCRIPTOR_UTILS_H_
