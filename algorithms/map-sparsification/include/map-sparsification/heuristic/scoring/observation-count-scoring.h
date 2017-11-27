#ifndef MAP_SPARSIFICATION_HEURISTIC_SCORING_OBSERVATION_COUNT_SCORING_H_
#define MAP_SPARSIFICATION_HEURISTIC_SCORING_OBSERVATION_COUNT_SCORING_H_

#include <map-sparsification/heuristic/scoring/scoring-function.h>
#include <vi-map/unique-id.h>
#include <vi-map/vi-map.h>

namespace map_sparsification {
namespace scoring {

class ObservationCountScoringFunction : public ScoringFunction {
 public:
  MAPLAB_POINTER_TYPEDEFS(ScoringFunction);

  virtual ~ObservationCountScoringFunction() {}

 private:
  virtual inline double scoreImpl(
      const vi_map::LandmarkId& landmark_id, const vi_map::VIMap& map) const {
    CHECK(map.hasLandmark(landmark_id));
    return map.getLandmark(landmark_id).numberOfObserverVertices();
  }
};

}  // namespace scoring
}  // namespace map_sparsification
#endif  // MAP_SPARSIFICATION_HEURISTIC_SCORING_OBSERVATION_COUNT_SCORING_H_
