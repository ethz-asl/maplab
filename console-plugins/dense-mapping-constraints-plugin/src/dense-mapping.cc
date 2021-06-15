#include "dense-mapping/dense-mapping.h"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <vi-map/vi-map.h>

#include "dense-mapping/dense-mapping-alignment.h"
#include "dense-mapping/dense-mapping-common.h"
#include "dense-mapping/dense-mapping-constraints.h"
#include "dense-mapping/dense-mapping-gflags.h"
#include "dense-mapping/dense-mapping-search.h"
#include "dense-mapping/dense-mapping-selection.h"

namespace dense_mapping {

Config Config::fromGflags() {
  Config config;

  config.search_config = SearchConfig::fromGflags();
  config.selection_config = SelectionConfig::fromGflags();
  config.alignment_config = AlignmentConfig::fromGflags();
  config.constraints_config = ConstraintsConfig::fromGflags();

  return config;
}

Config Config::forIncrementalSubmapAlignment() {
  Config config;

  config.search_config = SearchConfig::fromGflags();
  config.search_config.enable_incremental_submap_search = true;
  config.selection_config = SelectionConfig::fromGflags();
  config.alignment_config = AlignmentConfig::fromGflags();
  config.constraints_config = ConstraintsConfig::fromGflags();

  return config;
}

bool addDenseMappingConstraintsToMap(
    const Config& config, const vi_map::MissionIdList& mission_ids,
    vi_map::VIMap* vi_map_ptr) {
  CHECK_NOTNULL(vi_map_ptr);
  CHECK(!mission_ids.empty());

  AlignmentCandidatePairs candidates;

  if (!searchForAlignmentCandidatePairs(
          config.search_config, *vi_map_ptr, mission_ids, &candidates)) {
    LOG(ERROR) << "The search for alignment candidate pairs failed!";
    return false;
  }

  if (!config.search_config.enable_incremental_submap_search) {
    if (!selectAlignmentCandidatePairs(
            config.selection_config, vi_map_ptr, &candidates)) {
      LOG(ERROR)
          << "The selection and filtering of the alignment candidate pairs "
          << "failed!";
      return false;
    }
  }
  AlignmentCandidatePairs aligned_candidates;
  if (config.search_config.enable_incremental_submap_search) {
    if (!computeAlignmentForIncrementalSubmapCandidatePairs(
            config.alignment_config, *vi_map_ptr, candidates,
            &aligned_candidates)) {
      LOG(ERROR) << "Computing the alignment of the candidates failed!";
      return false;
    }
  } else if (!computeAlignmentForCandidatePairs(
                 config.alignment_config, *vi_map_ptr, candidates,
                 &aligned_candidates)) {
    LOG(ERROR) << "Computing the alignment of the candidates failed!";
    return false;
  }

  if (!applyConstraintsToMap(
          config.constraints_config, aligned_candidates, vi_map_ptr)) {
    LOG(ERROR)
        << "Applying the successful alignments as constraints to the map "
        << "failed.";
    return false;
  }

  return true;
}

}  // namespace dense_mapping
