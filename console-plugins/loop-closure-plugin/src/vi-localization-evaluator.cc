#include "loop-closure-plugin/vi-localization-evaluator.h"

#include <localization-evaluator/localization-evaluator.h>
#include <localization-evaluator/mission-aligner.h>
#include <maplab-common/file-system-tools.h>
#include <vi-map/unique-id.h>
#include <vi-map/vi-map.h>

namespace loop_closure_plugin {

VILocalizationEvaluator::VILocalizationEvaluator(
    vi_map::VIMap* map, visualization::ViwlsGraphRvizPlotter* plotter)
    : map_(map), plotter_(plotter) {
  CHECK_NOTNULL(map_);
}

void VILocalizationEvaluator::alignMissionsForEvaluation(
    const vi_map::MissionId& query_mission_id) {
  vi_map::MissionIdList all_mission_ids;
  map_->getAllMissionIds(&all_mission_ids);

  vi_map::MissionIdSet db_mission_ids(
      all_mission_ids.begin(), all_mission_ids.end());
  CHECK_GT(db_mission_ids.count(query_mission_id), 0u);
  db_mission_ids.erase(query_mission_id);

  constexpr bool kAlignMapMissions = true;
  constexpr bool kOptimizeOnlyQueryMission = false;
  localization_evaluator::alignAndCooptimizeMissionsWithoutLandmarkMerge(
      query_mission_id, db_mission_ids, kAlignMapMissions,
      kOptimizeOnlyQueryMission, map_);
}

void VILocalizationEvaluator::evaluateLocalizationPerformance(
    const vi_map::MissionId& query_mission_id) {
  vi_map::MissionIdList all_mission_ids;
  map_->getAllMissionIds(&all_mission_ids);

  vi_map::MissionIdSet db_mission_ids(
      all_mission_ids.begin(), all_mission_ids.end());
  CHECK_GT(db_mission_ids.count(query_mission_id), 0u);
  db_mission_ids.erase(query_mission_id);

  // Collect all database landmarks.
  vi_map::LandmarkIdSet selected_landmarks;
  for (const vi_map::MissionId& mission_id : db_mission_ids) {
    vi_map::LandmarkIdList mission_landmarks;
    map_->getAllLandmarkIdsInMission(mission_id, &mission_landmarks);
    selected_landmarks.insert(
        mission_landmarks.begin(), mission_landmarks.end());
  }
  LOG(INFO) << "Will query against " << selected_landmarks.size()
            << " landmarks.";

  localization_evaluator::MissionEvaluationStats mission_statistics;
  localization_evaluator::LocalizationEvaluator benchmark(
      selected_landmarks, map_);
  LOG(INFO) << "Evaluating the localizations.";
  benchmark.evaluateMission(query_mission_id, &mission_statistics);

  if (mission_statistics.num_vertices > 0u) {
    LOG(INFO) << "Recall: "
              << (static_cast<float>(
                      mission_statistics.successful_localizations) /
                  mission_statistics.num_vertices);
  } else {
    LOG(WARNING) << "No vertices evaluated!";
  }
}

}  // namespace loop_closure_plugin
