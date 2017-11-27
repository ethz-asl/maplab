#ifndef DESCRIPTOR_PROJECTION_MAP_TRACK_EXTRACTOR_H_
#define DESCRIPTOR_PROJECTION_MAP_TRACK_EXTRACTOR_H_
#include <string>
#include <vector>

#include <Eigen/Core>
#include <descriptor-projection/flags.h>
#include <loopclosure-common/types.h>

namespace vi_map {
class VIMap;
class MissionId;
}  // namespace vi_map

namespace descriptor_projection {
void CollectAndConvertDescriptors(
    const vi_map::VIMap& map, const vi_map::MissionId& mission_id,
    unsigned int descriptor_size, unsigned int matching_threshold,
    loop_closure::DescriptorContainer* all_descriptors,
    std::vector<Track>* tracks);

void CollectAndConvertDescriptors(
    const vi_map::VIMap& map, const vi_map::MissionId& mission_id,
    unsigned int descriptor_size, unsigned int matching_threshold,
    Eigen::MatrixXf* all_descriptors, std::vector<Track>* tracks);
}  // namespace descriptor_projection
#endif  // DESCRIPTOR_PROJECTION_MAP_TRACK_EXTRACTOR_H_
