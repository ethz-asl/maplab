#ifndef MAP_SPARSIFICATION_HEURISTIC_SCORING_DESCRIPTOR_VARIANCE_SCORING_H_
#define MAP_SPARSIFICATION_HEURISTIC_SCORING_DESCRIPTOR_VARIANCE_SCORING_H_

#include <algorithm>

#include <aslam/common/descriptor-utils.h>
#include <map-sparsification/heuristic/scoring/scoring-function.h>
#include <vi-map/unique-id.h>
#include <vi-map/vi-map.h>

namespace map_sparsification {
namespace scoring {

class DescriptorVarianceScoring : public ScoringFunction {
 public:
  MAPLAB_POINTER_TYPEDEFS(DescriptorVarianceScoring);
  explicit DescriptorVarianceScoring(double descriptor_dev_scoring_threshold)
      : descriptor_dev_scoring_threshold_(descriptor_dev_scoring_threshold) {}
  virtual ~DescriptorVarianceScoring() {}

 private:
  virtual inline double scoreImpl(
      const vi_map::LandmarkId& landmark_id, const vi_map::VIMap& map) const {
    CHECK(map.hasLandmark(landmark_id));
    vi_map::VIMap::DescriptorsType descriptors;
    map.getLandmarkDescriptors(landmark_id, &descriptors);
    const double descriptor_std_dev =
        aslam::common::descriptor_utils::descriptorMeanStandardDeviation(
            descriptors);
    return std::max(
        descriptor_dev_scoring_threshold_ - descriptor_std_dev, 0.0);
  }

  double descriptor_dev_scoring_threshold_;
};

}  // namespace scoring
}  // namespace map_sparsification
#endif  // MAP_SPARSIFICATION_HEURISTIC_SCORING_DESCRIPTOR_VARIANCE_SCORING_H_
