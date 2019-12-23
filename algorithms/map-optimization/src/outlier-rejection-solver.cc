#include "map-optimization/outlier-rejection-solver.h"

#include <aslam/common/timer.h>
#include <ceres/ceres.h>
#include <gflags/gflags.h>
#include <maplab-common/parallel-process.h>

DEFINE_int32(
    ba_outlier_rejection_reject_every_n_iters, 3,
    "Reject outliers every n iterations of the optimizer.");

DEFINE_bool(
    ba_outlier_rejection_reject_using_reprojection_error, false,
    "Use reprojection error to reject outliers.");
DEFINE_double(
    ba_outlier_rejection_reprojection_error_same_mission_px, 50,
    "Reprojection error threshold in pixels for observations from a mission"
    "the landmark is stored in.");
DEFINE_double(
    ba_outlier_rejection_reprojection_error_other_mission_px, 400,
    "Reprojection error threshold in pixels for observations from missions"
    "the landmark is NOT stored in.");
DEFINE_bool(
    ba_hide_iterations_console_output, false,
    "Define whether iterations should have output on verbosity level 0 or "
    "not.");

namespace map_optimization {

namespace {

double computeSquaredReprojectionError(
    const vi_map::Vertex& vertex, const int frame_idx, const int keypoint_idx,
    const Eigen::Vector3d& landmark_p_C) {
  Eigen::Vector2d reprojected_point;
  aslam::ProjectionResult projection_result =
      vertex.getCamera(frame_idx)->project3(landmark_p_C, &reprojected_point);

  if (projection_result == aslam::ProjectionResult::KEYPOINT_VISIBLE ||
      projection_result ==
          aslam::ProjectionResult::KEYPOINT_OUTSIDE_IMAGE_BOX) {
    return (reprojected_point -
            vertex.getVisualFrame(frame_idx).getKeypointMeasurement(
                keypoint_idx))
        .squaredNorm();
  }
  return std::numeric_limits<double>::max();
}

void findOutlierLandmarks(
    const vi_map::VIMap& map, const vi_map::LandmarkIdSet& landmark_ids_set,
    const bool use_reprojection_error,
    const double same_mission_reprojection_error_px,
    const double other_mission_reprojection_error_px,
    vi_map::LandmarkIdList* outlier_landmarks) {
  CHECK_NOTNULL(outlier_landmarks)->clear();

  const double same_mission_reproj_error_px_sq =
      same_mission_reprojection_error_px * same_mission_reprojection_error_px;
  const double other_mission_reproj_error_px_sq =
      other_mission_reprojection_error_px * other_mission_reprojection_error_px;

  vi_map::LandmarkIdList landmark_ids(
      landmark_ids_set.begin(), landmark_ids_set.end());

  std::function<void(const std::vector<size_t>&)> process_landmark =
      [&](const std::vector<size_t>& batch) {
        for (size_t item : batch) {
          const vi_map::LandmarkId& landmark_id = landmark_ids[item];
          const vi_map::Landmark& landmark = map.getLandmark(landmark_id);

          const vi_map::MissionId& landmark_store_mission_id =
              map.getLandmarkStoreVertex(landmark_id).getMissionId();

          // Iterate over all the observations
          const vi_map::KeypointIdentifierList& keypoint_ids =
              landmark.getObservations();
          for (const vi_map::KeypointIdentifier& keypoint_id : keypoint_ids) {
            const vi_map::Vertex& observer_vertex =
                map.getVertex(keypoint_id.frame_id.vertex_id);

            const int frame_idx = keypoint_id.frame_id.frame_index;
            CHECK(observer_vertex.isVisualFrameSet(frame_idx));
            CHECK(observer_vertex.isVisualFrameValid(frame_idx));

            CHECK_LT(
                keypoint_id.keypoint_index,
                observer_vertex.getVisualFrame(frame_idx)
                    .getNumKeypointMeasurements());

            const Eigen::Vector3d& p_C_fi =
                map.getLandmark_p_C_fi(landmark_id, observer_vertex, frame_idx);

            if (p_C_fi[2] <= 0.0) {
              outlier_landmarks->emplace_back(landmark_id);
              break;
            }

            if (use_reprojection_error) {
              const double reprojection_error_sq =
                  computeSquaredReprojectionError(
                      observer_vertex, frame_idx, keypoint_id.keypoint_index,
                      p_C_fi);
              if (observer_vertex.getMissionId() == landmark_store_mission_id &&
                  reprojection_error_sq > same_mission_reproj_error_px_sq) {
                // The landmarks is in the same mission so we use the same
                // mission reprojection error threshold.
                outlier_landmarks->emplace_back(landmark_id);
              } else if (
                  reprojection_error_sq > other_mission_reproj_error_px_sq) {
                // The observation is coming from a different mission than the
                // one where the landmark is stored.
                outlier_landmarks->emplace_back(landmark_id);
              }
            }
          }
        }
      };

  constexpr bool kAlwaysParallelize = true;
  const size_t num_threads = common::getNumHardwareThreads();
  common::ParallelProcess(
      landmark_ids.size(), process_landmark, kAlwaysParallelize, num_threads);
}

ceres::TerminationType solveStep(
    const ceres::Solver::Options& solver_options, int num_iters,
    OptimizationProblem* optimization_problem,
    OutlierRejectionCallback* callback) {
  CHECK_NOTNULL(optimization_problem);
  CHECK_NOTNULL(callback);

  ceres::Problem problem(ceres_error_terms::getDefaultProblemOptions());
  ceres_error_terms::buildCeresProblemFromProblemInformation(
      optimization_problem->getProblemInformationMutable(), &problem);

  ceres::Solver::Options local_options = solver_options;
  local_options.callbacks.push_back(callback);
  // Reusing the trust region size from the last iteration.
  local_options.initial_trust_region_radius =
      callback->initial_trust_region_radius_;

  // Disable the default Ceres output.
  local_options.minimizer_progress_to_stdout = false;
  local_options.max_num_iterations = num_iters;
  local_options.update_state_every_iteration = true;

  ceres::Solver::Summary summary;
  ceres::Solve(local_options, &problem, &summary);

  return summary.termination_type;
}

void rejectOutliers(
    const OutlierRejectionSolverOptions& rejection_options,
    OptimizationProblem* optimization_problem) {
  CHECK_NOTNULL(optimization_problem);

  vi_map::VIMap& map = *optimization_problem->getMapMutable();

  typedef std::unordered_multimap<vi_map::LandmarkId, ceres::CostFunction*>
      ProblemLandmarksMap;
  ProblemLandmarksMap& landmarks_in_problem =
      optimization_problem->getProblemBookkeepingMutable()
          ->landmarks_in_problem;

  vi_map::LandmarkIdSet present_landmarks;
  std::transform(
      landmarks_in_problem.begin(), landmarks_in_problem.end(),
      std::inserter(present_landmarks, present_landmarks.end()),
      [](const ProblemLandmarksMap::value_type& key_elem) {
        return key_elem.first;
      });

  vi_map::LandmarkIdList outlier_landmarks;
  findOutlierLandmarks(
      map, present_landmarks,
      rejection_options.reject_landmarks_based_on_reprojection_errors,
      rejection_options.reprojection_error_same_mission_px,
      rejection_options.reprojection_error_other_mission_px,
      &outlier_landmarks);

  for (const vi_map::LandmarkId& landmark_id : outlier_landmarks) {
    const auto range = landmarks_in_problem.equal_range(landmark_id);
    // Deactivate all observation constraints of this landmark.
    for (auto it = range.first; it != range.second; ++it) {
      optimization_problem->getProblemInformationMutable()
          ->deactivateCostFunction(it->second);
    }
    landmarks_in_problem.erase(landmark_id);
    map.getLandmark(landmark_id).setQuality(vi_map::Landmark::Quality::kBad);
  }

  LOG_IF(INFO, !outlier_landmarks.empty())
      << "Removed " << outlier_landmarks.size() << " outlier landmark(s) of "
      << present_landmarks.size() << " present in the problem.";
}
}  // namespace

OutlierRejectionSolverOptions OutlierRejectionSolverOptions::initFromFlags() {
  OutlierRejectionSolverOptions options;
  options.reject_outliers_every_n_iters =
      FLAGS_ba_outlier_rejection_reject_every_n_iters;

  options.reject_landmarks_based_on_reprojection_errors =
      FLAGS_ba_outlier_rejection_reject_using_reprojection_error;
  options.reprojection_error_same_mission_px =
      FLAGS_ba_outlier_rejection_reprojection_error_same_mission_px;
  options.reprojection_error_other_mission_px =
      FLAGS_ba_outlier_rejection_reprojection_error_other_mission_px;
  return options;
}

ceres::TerminationType solveWithOutlierRejection(
    const ceres::Solver::Options& solver_options,
    const OutlierRejectionSolverOptions& rejection_options,
    OptimizationProblem* optimization_problem) {
  CHECK_NOTNULL(optimization_problem);

  if (rejection_options.reject_outliers_every_n_iters == 0 ||
      solver_options.max_num_iterations == 0) {
    LOG(WARNING) << "Please specify a non-zero number of iterations.";
    return ceres::TerminationType::FAILURE;
  }

  OutlierRejectionCallback callback(solver_options.initial_trust_region_radius);

  ceres::TerminationType termination_type =
      ceres::TerminationType::NO_CONVERGENCE;
  int num_iters_remaining = solver_options.max_num_iterations;
  while (num_iters_remaining > 0) {
    const int step_iters = std::min(
        num_iters_remaining, rejection_options.reject_outliers_every_n_iters);

    timing::Timer timer_solve("BA: Solve");
    termination_type =
        solveStep(solver_options, step_iters, optimization_problem, &callback);
    timer_solve.Stop();

    timing::Timer timer_copy("BA: CopyDataToMap");
    optimization_problem->getOptimizationStateBufferMutable()
        ->copyAllStatesBackToMap(optimization_problem->getMapMutable());
    timer_copy.Stop();

    timing::Timer timer_reject("BA: Outlier rejection");
    rejectOutliers(rejection_options, optimization_problem);
    timer_reject.Stop();

    if (termination_type != ceres::TerminationType::NO_CONVERGENCE) {
      break;
    }

    num_iters_remaining -= step_iters;
  }

  return termination_type;
}

}  // namespace map_optimization
