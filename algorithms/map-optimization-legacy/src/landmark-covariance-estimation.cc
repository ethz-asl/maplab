#include <map-optimization-legacy/landmark-covariance-estimation.h>

#include <Eigen/Core>
#include <ceres/ceres.h>
#include <gflags/gflags.h>
#include <maplab-common/threading-helpers.h>
#include <vi-map/landmark-quality-metrics.h>

DEFINE_uint64(
    cov_estimation_min_landmark_observer_count, 4,
    "Minimum required number of observers to estimate landmark"
    " covariance.");
DEFINE_uint64(
    cov_estimation_min_observations_per_frame, 5,
    "Minimum required number of observations per frame.");

namespace map_optimization_legacy {

LandmarkCovarianceEstimation::LandmarkCovarianceEstimation(vi_map::VIMap* map)
    : GraphBaOptimizer(CHECK_NOTNULL(map)) {}

LandmarkCovarianceEstimation::~LandmarkCovarianceEstimation() {}

void LandmarkCovarianceEstimation::assignCovarianceToLandmarks(
    const std::unordered_set<pose_graph::VertexId>& fixed_vertices) {
  addErrorTerms(fixed_vertices);

  ceres::Solver::Options options = getDefaultSolverOptions();
  // We don't need to iterate, but just build the problem.
  options.max_num_iterations = 0;
  static constexpr bool kCopyDataFromSolverBackToMap = true;

  ceres::Problem problem(ceres_error_terms::getDefaultProblemOptions());
  buildProblem(&problem);
  ceres::Solver::Summary summary;
  solve(kCopyDataFromSolverBackToMap, options, &problem, &summary);
  calculateCovariance(&problem);
}

void LandmarkCovarianceEstimation::addErrorTerms(
    const pose_graph::VertexIdSet& fixed_vertices) {
  bool kFixIntrinsics = true;
  bool kFixExtrinsicsRotation = true;
  bool kFixExtrinsicsTranslation = true;
  bool kFixLandmarkPosition = false;
  bool kFixAccelBias = false;
  bool kFixGyroBias = false;
  bool kFixVelocity = false;
  bool kUseGivenEdges = false;
  bool kStoreCachedImuCovariances = false;
  bool kIncludeOnlyMergedLandmarks = false;
  pose_graph::EdgeIdList edges;

  removeLandmarksBehindCamera();
  const size_t kMinNumOfObserverMissions = 0u;
  addVisualResidualBlocks(
      kFixIntrinsics, kFixExtrinsicsRotation, kFixExtrinsicsTranslation,
      kFixLandmarkPosition, kIncludeOnlyMergedLandmarks,
      FLAGS_cov_estimation_min_landmark_observer_count,
      FLAGS_cov_estimation_min_observations_per_frame,
      kMinNumOfObserverMissions);
  BaOptimizationOptions options;
  addInertialResidualBlocks(
      kFixGyroBias, kFixAccelBias, kFixVelocity, kUseGivenEdges, edges,
      kStoreCachedImuCovariances, options.gravity_magnitude, nullptr);

  addPosePriorResidualBlocks(
      fixed_vertices, options.prior_position_std_dev_meters,
      options.prior_orientation_std_dev_radians);
}

void LandmarkCovarianceEstimation::calculateCovariance(
    ceres::Problem* problem) {
  ceres::Covariance::Options covariance_options;
  covariance_options.algorithm_type = ceres::SUITE_SPARSE_QR;
  covariance_options.num_threads = common::getNumHardwareThreads();
  covariance_options.min_reciprocal_condition_number = 1e-32;
  covariance_options.apply_loss_function = true;
  ceres::Covariance covariance(covariance_options);

  std::vector<std::pair<const double*, const double*> > covariance_blocks;

  pose_graph::VertexIdList all_vertices;
  const_map_.getAllVertexIds(&all_vertices);

  vi_map::LandmarkIdSet added_landmarks;

  LOG(INFO) << "Adding covariance blocks";
  for (const pose_graph::VertexId& vertex_id : all_vertices) {
    vi_map::Vertex& ba_vertex = map_.getVertex(vertex_id);

    const unsigned int num_frames = ba_vertex.numFrames();
    for (unsigned int frame_idx = 0; frame_idx < num_frames; ++frame_idx) {
      const Eigen::Matrix2Xd& image_points_distorted =
          ba_vertex.getVisualFrame(frame_idx).getKeypointMeasurements();
      for (int i = 0; i < image_points_distorted.cols(); ++i) {
        vi_map::LandmarkId landmark_id =
            ba_vertex.getObservedLandmarkId(frame_idx, i);
        if (landmark_id.isValid()) {
          vi_map::Landmark& landmark = map_.getLandmark(landmark_id);
          CHECK_EQ(landmark.id(), landmark_id);

          if (added_landmarks.count(landmark.id()) == 0) {
            if (landmark.numberOfObservations() >
                    FLAGS_cov_estimation_min_landmark_observer_count &&
                vi_map::isLandmarkWellConstrained(map_, landmark)) {
              // Check if the position block was really added to the problem.
              // If not, there must be some inconsistency in choosing the
              // well-constrained landmarks.
              CHECK(problem->HasParameterBlock(landmark.get_p_B_Mutable()));
              covariance_blocks.push_back(
                  std::make_pair(
                      landmark.get_p_B_Mutable(), landmark.get_p_B_Mutable()));

              added_landmarks.insert(landmark.id());
            } else {
              // Not enough landmarks, let's put something big to covariance
              // diagonal.
              Eigen::Matrix3d some_large_covariance =
                  Eigen::Matrix3d::Identity();
              some_large_covariance *= 100;
              landmark.set_p_B_Covariance(some_large_covariance);
            }
          }
        }
      }
    }  // Loop over all frames in a vertex.
  }    // Loop over all vertices.

  LOG(INFO) << "Calculating covariance.";
  CHECK(covariance.Compute(covariance_blocks, problem));

  LOG(INFO) << "Storing covariance in landmark objects.";
  for (const vi_map::LandmarkId& landmark_id : added_landmarks) {
    vi_map::Landmark& landmark = map_.getLandmark(landmark_id);
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> position_covariance;

    covariance.GetCovarianceBlock(
        landmark.get_p_B_Mutable(), landmark.get_p_B_Mutable(),
        position_covariance.data());
    landmark.set_p_B_Covariance(position_covariance);
  }
}

}  // namespace map_optimization_legacy
