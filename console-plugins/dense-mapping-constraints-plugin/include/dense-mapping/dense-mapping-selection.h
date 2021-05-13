#ifndef DENSE_MAPPING_DENSE_MAPPING_SELECTION_H_
#define DENSE_MAPPING_DENSE_MAPPING_SELECTION_H_

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <vi-map/vi-map.h>

#include "dense-mapping/dense-mapping-common.h"
#include "dense-mapping/dense-mapping-gflags.h"

namespace dense_mapping {

struct SelectionConfig {
  static SelectionConfig fromGflags();

  bool recompute_all_constraints;
  bool recompute_invalid_constraints;

  double constraint_min_switch_variable_value;

  std::size_t max_number_of_candidates;
  std::string filter_strategy;
};

bool selectAlignmentCandidatePairs(
    const SelectionConfig& config, vi_map::VIMap* map_ptr,
    AlignmentCandidatePairs* candidate_pairs_ptr);

}  // namespace dense_mapping

#endif  // DENSE_MAPPING_DENSE_MAPPING_SELECTION_H_
