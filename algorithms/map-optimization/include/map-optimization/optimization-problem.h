#ifndef MAP_OPTIMIZATION_OPTIMIZATION_PROBLEM_H_
#define MAP_OPTIMIZATION_OPTIMIZATION_PROBLEM_H_

#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <ceres-error-terms/parameterization/pose-param-jpl.h>
#include <ceres-error-terms/parameterization/quaternion-param-jpl.h>
#include <ceres-error-terms/problem-information.h>
#include <maplab-common/macros.h>
#include <vi-map-helpers/mission-clustering-coobservation.h>
#include <vi-map/vi-map.h>

#include "map-optimization/mission-cluster-gauge-fixes.h"
#include "map-optimization/optimization-state-buffer.h"

namespace map_optimization {

class OptimizationProblem {
 public:
  MAPLAB_POINTER_TYPEDEFS(OptimizationProblem);

  OptimizationProblem(
      vi_map::VIMap* vi_map, const vi_map::MissionIdSet& mission_ids);

  // Fix the open degrees-of-freedom for a single initial vertex of each
  // cluster.
  void applyGaugeFixesForInitialVertices(
      const std::vector<MissionClusterGaugeFixes>& new_cluster_fixes);
  // Returns nullptr if none set.
  const std::vector<MissionClusterGaugeFixes>*
  getAppliedGaugeFixesForInitialVertices() const {
    return problem_books_.cluster_gauge_fixes_initial_vertex.get();
  }

  const std::vector<vi_map::MissionIdSet>& getMissionCoobservationClusters()
      const {
    return mission_coobservation_clusters_;
  }

  ceres_error_terms::ProblemInformation* getProblemInformationMutable() {
    return &problem_information_;
  }
  OptimizationStateBuffer* getOptimizationStateBufferMutable() {
    return &state_buffer_;
  }
  vi_map::VIMap* getMapMutable() const {
    return map_;
  }
  const vi_map::MissionIdSet& getMissionIds() const {
    return missions_ids_;
  }

  struct LocalParameterizations {
    std::shared_ptr<ceres::LocalParameterization> pose_parameterization;
    std::shared_ptr<ceres::LocalParameterization> baseframe_parameterization;
    std::shared_ptr<ceres::LocalParameterization> quaternion_parameterization;
  };

  struct ProblemBookkeeping {
    // Information of the degrees-of-freedom to fix for the first mission in
    // each cluster. Not set if nullptr.
    std::unique_ptr<std::vector<MissionClusterGaugeFixes>>
        cluster_gauge_fixes_initial_vertex;

    std::unordered_set<pose_graph::VertexId> keyframes_in_problem;
    std::unordered_multimap<vi_map::LandmarkId, ceres::CostFunction*>
        landmarks_in_problem;
  };

  const LocalParameterizations& getLocalParameterizations() const {
    return local_parameterizations_;
  }

  ProblemBookkeeping* getProblemBookkeepingMutable() {
    return &problem_books_;
  }

 private:
  // Map from which the problem has been built.
  vi_map::VIMap* const map_;
  // Missions included in this problem.
  const vi_map::MissionIdSet missions_ids_;

  // Clustering of missions according to landmark coobservations; no landmark
  // coobervations between clusters.
  const std::vector<vi_map::MissionIdSet> mission_coobservation_clusters_;

  // Gauge fixes that have been applied to the problem.
  ceres_error_terms::ProblemInformation problem_information_;
  // Additional bookkeeping information to incrementally update things. This
  // has to be kept in sync with the ProblemInformation.
  ProblemBookkeeping problem_books_;

  // Copies of the map states on which the optimization will operate.
  OptimizationStateBuffer state_buffer_;

  // Local parameterization that are used when adding cost terms.
  LocalParameterizations local_parameterizations_;
};

}  // namespace map_optimization

#endif  // MAP_OPTIMIZATION_OPTIMIZATION_PROBLEM_H_
