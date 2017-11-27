#ifndef LOCALIZATION_EVALUATOR_MISSION_ALIGNER_H_
#define LOCALIZATION_EVALUATOR_MISSION_ALIGNER_H_

#include <vi-map/unique-id.h>

namespace vi_map {
class VIMap;
}  // namespace vi_map

namespace localization_evaluator {

void alignAndCooptimizeMissionsWithoutLandmarkMerge(
    const vi_map::MissionId& query_mission_id,
    const vi_map::MissionIdSet& map_missions_ids,
    bool should_align_map_missions, bool optimize_only_query_mission,
    vi_map::VIMap* map);

}  // namespace localization_evaluator

#endif  // LOCALIZATION_EVALUATOR_MISSION_ALIGNER_H_
