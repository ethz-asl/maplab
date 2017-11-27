#ifndef MAP_SPARSIFICATION_HEURISTIC_SCORING_SCORING_FUNCTION_H_
#define MAP_SPARSIFICATION_HEURISTIC_SCORING_SCORING_FUNCTION_H_

#include <maplab-common/macros.h>
#include <vi-map/unique-id.h>
#include <vi-map/vi-map.h>

namespace map_sparsification {
namespace scoring {

class ScoringFunction {
 public:
  MAPLAB_POINTER_TYPEDEFS(ScoringFunction);

  ScoringFunction() : weight_(1.0) {}
  virtual ~ScoringFunction() {}

  double operator()(
      const vi_map::LandmarkId& store_landmark_id,
      const vi_map::VIMap& map) const {
    double raw_score = scoreImpl(store_landmark_id, map);
    return weight_ * raw_score;
  }

  void setWeight(double weight) {
    weight_ = weight;
  }

 private:
  virtual double scoreImpl(
      const vi_map::LandmarkId& store_landmark_id,
      const vi_map::VIMap& map) const = 0;
  double weight_;
};

}  // namespace scoring
}  // namespace map_sparsification
#endif  // MAP_SPARSIFICATION_HEURISTIC_SCORING_SCORING_FUNCTION_H_
