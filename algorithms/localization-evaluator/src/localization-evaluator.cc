#include "localization-evaluator/localization-evaluator.h"

#include <Eigen/Core>
#include <aslam/common/statistics/statistics.h>
#include <loop-closure-handler/loop-closure-handler.h>
#include <maplab-common/parallel-process.h>

DEFINE_double(
    benchmark_position_error_threshold, 0.05,
    "Position error threshold in meters to count vertex "
    "as correctly localized.");

namespace localization_evaluator {

bool LocalizationEvaluator::evaluateSingleKeyframe(
    const pose_graph::VertexId& query_vertex_id, Eigen::Vector3d* pnp_p_G_I,
    unsigned int* lc_matches_count, unsigned int* inliers_count,
    double* error_meters, bool* ransac_ok) {
  CHECK_NOTNULL(pnp_p_G_I);
  CHECK_NOTNULL(lc_matches_count);
  CHECK_NOTNULL(inliers_count);
  CHECK_NOTNULL(error_meters);
  CHECK_NOTNULL(ransac_ok);

  statistics::StatsCollector stats_collector_error_successes(
      "LocalizationEvaluator -- successes");
  statistics::StatsCollector stats_collector_error_norm(
      "LocalizationEvaluator -- error norm");
  statistics::StatsCollector stats_collector_matches(
      "LocalizationEvaluator -- matches");
  statistics::StatsCollector stats_collector_ransac_inliers(
      "LocalizationEvaluator -- RANSAC inliers");

  CHECK(map_->hasVertex(query_vertex_id))
      << "Couldn't find map vertex with ID: " << query_vertex_id.hexString();
  const vi_map::Vertex& query_vertex = map_->getVertex(query_vertex_id);

  const bool kMergeLandmarks = false;
  const bool kAddLoopclosureEdges = false;
  vi_map::LoopClosureConstraint inlier_constraints;
  pose::Transformation pnp_T_G_I;
  *ransac_ok = loop_detector_node_.findVertexInDatabase(
      query_vertex, kMergeLandmarks, kAddLoopclosureEdges, map_, &pnp_T_G_I,
      lc_matches_count, &inlier_constraints);
  *inliers_count = inlier_constraints.structure_matches.size();
  *pnp_p_G_I = pnp_T_G_I.getPosition();

  Eigen::Vector3d p_G_I = map_->getVertex_G_p_I(query_vertex_id);
  double position_error = (p_G_I - pnp_T_G_I.getPosition()).norm();

  stats_collector_matches.AddSample(*lc_matches_count);
  stats_collector_ransac_inliers.AddSample(*inliers_count);

  if (*ransac_ok) {
    VLOG(1) << "\t" << pnp_T_G_I.getPosition().transpose() << " vs "
            << p_G_I.transpose() << " --> norm: " << position_error << " [m]";
    stats_collector_error_norm.AddSample(position_error);
    *error_meters = position_error;
  } else {
    LOG(WARNING) << "\tRansac failed:" << *lc_matches_count << " matches, "
                 << *inliers_count << " inliers.";
    *error_meters = std::numeric_limits<double>::infinity();
  }
  if (*ransac_ok && position_error < FLAGS_benchmark_position_error_threshold) {
    stats_collector_error_successes.AddSample(1.0);
    return true;
  } else {
    LOG(WARNING) << "\tCouldn't localize " << query_vertex_id;
    stats_collector_error_successes.AddSample(0.0);
    return false;
  }
}

void LocalizationEvaluator::evaluateMission(
    const vi_map::MissionId& mission_id, MissionEvaluationStats* statistics) {
  CHECK_NOTNULL(statistics);

  pose_graph::VertexIdList vertices;
  map_->getAllVertexIdsInMission(mission_id, &vertices);

  std::vector<char> is_correct;
  std::vector<char> ransac_ok;
  std::vector<unsigned int> inlier_counts;
  std::vector<unsigned int> lc_matches_counts;
  Aligned<std::vector, Eigen::Vector3d> localization_p_G_I;
  is_correct.resize(vertices.size(), false);
  ransac_ok.resize(vertices.size(), false);
  inlier_counts.resize(vertices.size(), 0u);
  lc_matches_counts.resize(vertices.size(), 0u);
  localization_p_G_I.resize(vertices.size(), Eigen::Vector3d::Zero());
  std::vector<double> errors_meters(
      vertices.size(), std::numeric_limits<double>::infinity());

  std::function<void(const std::vector<size_t>&)> pose_query =
      [this, &vertices, &is_correct, &inlier_counts, &lc_matches_counts,
       &localization_p_G_I, &errors_meters,
       &ransac_ok](const std::vector<size_t>& batch) {
        for (size_t item : batch) {
          const pose_graph::VertexId& vertex_id = vertices[item];
          Eigen::Vector3d& pnp_p_G_I = localization_p_G_I[item];
          unsigned int& lc_matches_count = lc_matches_counts[item];
          unsigned int& inliers_count = inlier_counts[item];
          double& error_meters = errors_meters[item];
          bool ransac_ok_item;
          if (evaluateSingleKeyframe(
                  vertex_id, &pnp_p_G_I, &lc_matches_count, &inliers_count,
                  &error_meters, &ransac_ok_item)) {
            CHECK_GE(lc_matches_count, inliers_count);
            is_correct[item] = true;
          }
          ransac_ok[item] = ransac_ok_item;
        }
      };

  constexpr bool kAlwaysParallelize = true;
  const size_t num_threads = common::getNumHardwareThreads();
  common::ParallelProcess(
      is_correct.size(), pose_query, kAlwaysParallelize, num_threads);

  // Copy back all valid results.
  statistics->num_vertices = 0u;
  statistics->successful_localizations = 0u;
  statistics->inliers_counts.reserve(vertices.size());
  statistics->lc_matches_counts.reserve(vertices.size());
  statistics->localization_p_G_I.reserve(vertices.size());
  statistics->num_vertices = 0u;
  statistics->successful_localizations = 0u;
  for (size_t i = 0; i < is_correct.size(); ++i) {
    if (is_correct[i]) {
      statistics->localization_p_G_I.emplace_back(localization_p_G_I[i]);
      ++statistics->successful_localizations;
    } else if (ransac_ok[i]) {
      statistics->bad_localization_p_G_I.emplace_back(localization_p_G_I[i]);
    }
    statistics->inliers_counts.emplace_back(inlier_counts[i]);
    statistics->lc_matches_counts.emplace_back(lc_matches_counts[i]);
    statistics->localization_errors_meters.emplace_back(errors_meters[i]);
    ++statistics->num_vertices;
  }

  if (statistics->num_vertices > 0) {
    VLOG(3) << "Ratio "
            << static_cast<double>(statistics->successful_localizations) /
                   statistics->num_vertices
            << " (" << statistics->successful_localizations << "/"
            << statistics->num_vertices << ")";
  } else {
    LOG(WARNING) << "No vertices in mission: " << mission_id;
  }
}

}  // namespace localization_evaluator
