#ifndef LOCALIZATION_EVALUATOR_LOCALIZATION_EVALUATOR_H_
#define LOCALIZATION_EVALUATOR_LOCALIZATION_EVALUATOR_H_

#include <limits>
#include <vector>

#include <Eigen/Core>
#include <aslam/common/memory.h>
#include <loop-closure-handler/loop-detector-node.h>
#include <vi-map/vi-map.h>

namespace localization_evaluator {

struct MissionEvaluationStats {
  unsigned int num_vertices;
  unsigned int successful_localizations;

  Aligned<std::vector, Eigen::Vector3d> localization_p_G_I;
  Aligned<std::vector, Eigen::Vector3d> bad_localization_p_G_I;
  std::vector<unsigned int> lc_matches_counts;
  std::vector<unsigned int> inliers_counts;
  std::vector<double> localization_errors_meters;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class LocalizationEvaluator {
 public:
  LocalizationEvaluator(
      const vi_map::LandmarkIdSet& selected_landmarks, vi_map::VIMap* map)
      : map_(CHECK_NOTNULL(map)) {
    // Add selected landmarks to the loop detector database.
    loop_detector_node_.addLandmarkSetToDatabase(selected_landmarks, *map);
  }

  bool evaluateSingleKeyframe(
      const pose_graph::VertexId& vertex_id, Eigen::Vector3d* p_G_I,
      unsigned int* lc_matches_count, unsigned int* inliers_count,
      double* error_meters, bool* ransac_ok);
  void evaluateMission(
      const vi_map::MissionId& mission_id, MissionEvaluationStats* statistics);

 private:
  vi_map::VIMap* map_;
  loop_detector_node::LoopDetectorNode loop_detector_node_;

  // TODO(dymczykm) Fix for visual n frame needed here.
  static constexpr unsigned int kFrameIndex = 0;
};

}  // namespace localization_evaluator

#endif  // LOCALIZATION_EVALUATOR_LOCALIZATION_EVALUATOR_H_
