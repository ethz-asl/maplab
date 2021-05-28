#ifndef DENSE_MAPPING_DENSE_MAPPING_CONSTRAINTS_H_
#define DENSE_MAPPING_DENSE_MAPPING_CONSTRAINTS_H_

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <vi-map/vi-map.h>

#include "dense-mapping/dense-mapping-common.h"
#include "dense-mapping/dense-mapping-gflags.h"

namespace dense_mapping {

struct ConstraintsConfig {
  static ConstraintsConfig fromGflags();

  double constraint_switch_variable_value;
  double constraint_switch_variable_sigma;
};

bool applyConstraintsToMap(
    const ConstraintsConfig& config,
    const AlignmentCandidatePairs& candidate_pairs, vi_map::VIMap* map_ptr);

bool removeAllConstraintsFromVertices(
    const pose_graph::VertexIdList& vertex_ids, vi_map::VIMap* map_ptr);

}  // namespace dense_mapping

#endif  // DENSE_MAPPING_DENSE_MAPPING_CONSTRAINTS_H_
