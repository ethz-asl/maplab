#ifndef VI_MAP_HELPERS_VI_MAP_STATS_H_
#define VI_MAP_HELPERS_VI_MAP_STATS_H_

#include <string>
#include <unordered_map>
#include <vector>

#include <aslam/common/statistics/accumulator.h>

#include "vi-map-helpers/vi-map-queries.h"

namespace vi_map {
class MissionId;
class VIMap;
}  // namespace vi_map

namespace vi_map_helpers {

class VIMapStats {
 public:
  explicit VIMapStats(const vi_map::VIMap& map);

  std::string toString() const;

  void getLandmarkAssociatedKeypointsRatio(
      statistics::Accumulatord* accumulator) const;

  void getLandmarkObserverCount(statistics::Accumulatord* accumulator) const;

  // Returns true if observations associated with a landmark are contiguous
  // over the vertex sequence. In other words, returns false if a vertex sees
  // a landmark that has been seen before, but not in the preceding vertex.
  void areLandmarkAssociationsTrackScoped(
      std::unordered_map<vi_map::MissionId, bool>* result) const;
  bool areLandmarkAssociationsTrackScoped(
      const vi_map::MissionId& mission) const;

  void getNumLandmarksObservedByOtherMissionsForEachVertexAlongGraph(
      const vi_map::MissionId& mission_id, std::vector<size_t>* result) const;

 private:
  const vi_map::VIMap& map_;
  const VIMapQueries map_queries_;
};

}  // namespace vi_map_helpers

#endif  // VI_MAP_HELPERS_VI_MAP_STATS_H_
