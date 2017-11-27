#ifndef MAP_OPTIMIZATION_LEGACY_LANDMARK_COVARIANCE_ESTIMATION_H_
#define MAP_OPTIMIZATION_LEGACY_LANDMARK_COVARIANCE_ESTIMATION_H_

#include <map-optimization-legacy/graph-ba-optimizer.h>

namespace map_optimization_legacy {

class LandmarkCovarianceEstimation : public GraphBaOptimizer {
 public:
  explicit LandmarkCovarianceEstimation(vi_map::VIMap* map);
  virtual ~LandmarkCovarianceEstimation();

  void assignCovarianceToLandmarks(
      const pose_graph::VertexIdSet& fixed_vertices);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  void addErrorTerms(const pose_graph::VertexIdSet& fixed_vertices);
  void calculateCovariance(ceres::Problem* problem);
};

}  // namespace map_optimization_legacy

#endif  // MAP_OPTIMIZATION_LEGACY_LANDMARK_COVARIANCE_ESTIMATION_H_
