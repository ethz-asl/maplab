#ifndef LOCALIZATION_SUMMARY_MAP_LOCALIZATION_SUMMARY_MAP_CACHE_H_
#define LOCALIZATION_SUMMARY_MAP_LOCALIZATION_SUMMARY_MAP_CACHE_H_

#include <unordered_map>
#include <utility>

#include <Eigen/Core>
#include <maplab-common/accessors.h>
#include <posegraph/unique-id.h>
#include <vi-map/unique-id.h>

namespace summary_map {

// This class stores projected descriptors to speed up the summary map creation.
class LocalizationSummaryMapCache {
 public:
  // Gets the projected descriptor if it's stored. Returns true if a descriptor
  // is stored in the cache, returns false otherwise.
  bool getProjectedDescriptorForLandmark(
      const vi_map::KeypointIdentifier& keypoint_identifier,
      const vi_map::LandmarkId& landmark_id,
      const Eigen::MatrixXf::ColXpr& projected_descriptor_const) {
    const DescriptorMap::const_iterator descriptor_iterator =
        descriptor_map_.find(landmark_id);
    if (descriptor_iterator != descriptor_map_.end()) {
      const DescriptorMapInner& inner_map = descriptor_iterator->second;
      const DescriptorMapInner::const_iterator inner_iterator =
          inner_map.find(keypoint_identifier);
      if (inner_iterator != inner_map.end()) {
        Eigen::MatrixXf::ColXpr& projected_descriptor =
            const_cast<Eigen::MatrixXf::ColXpr&>(projected_descriptor_const);
        projected_descriptor = inner_iterator->second;
        return true;
      }
    }
    return false;
  }

  // Adds a projected descriptor to the cache.
  void addProjectedDescriptor(
      const vi_map::KeypointIdentifier& keypoint_identifier,
      const vi_map::LandmarkId& landmark_id,
      const Eigen::VectorXf& projected_descriptor) {
    DescriptorMap::iterator descriptor_iterator =
        descriptor_map_.find(landmark_id);
    if (descriptor_iterator == descriptor_map_.end()) {
      std::pair<DescriptorMap::iterator, bool> insertion_result =
          descriptor_map_.insert(
              std::make_pair(landmark_id, DescriptorMapInner()));
      CHECK(insertion_result.second);
      descriptor_iterator = insertion_result.first;
    }

    descriptor_iterator->second.insert(
        std::make_pair(keypoint_identifier, projected_descriptor));
  }

 private:
  typedef std::unordered_map<vi_map::KeypointIdentifier, const Eigen::VectorXf>
      DescriptorMapInner;
  typedef std::unordered_map<vi_map::LandmarkId, DescriptorMapInner>
      DescriptorMap;
  DescriptorMap descriptor_map_;
};

}  // namespace summary_map

#endif  // LOCALIZATION_SUMMARY_MAP_LOCALIZATION_SUMMARY_MAP_CACHE_H_
