#ifndef DENSE_MAPPING_DENSE_MAPPING_H_
#define DENSE_MAPPING_DENSE_MAPPING_H_

#include <vi-map/vi-map.h>

#include "dense-mapping/dense-mapping-alignment.h"
#include "dense-mapping/dense-mapping-common.h"
#include "dense-mapping/dense-mapping-constraints.h"
#include "dense-mapping/dense-mapping-gflags.h"
#include "dense-mapping/dense-mapping-search.h"
#include "dense-mapping/dense-mapping-selection.h"

namespace dense_mapping {

struct Config {
  static Config fromGflags();
  static Config forIncrementalSubmapAlignment();

  SearchConfig search_config;
  SelectionConfig selection_config;
  AlignmentConfig alignment_config;
  ConstraintsConfig constraints_config;
};

bool addDenseMappingConstraintsToMap(
    const Config& config, const vi_map::MissionIdList& mission_ids,
    vi_map::VIMap* vi_map_ptr);

bool verifyDenseMappingConstraintsFromSubmap(
    const Config& config, const vi_map::MissionIdList& mission_ids,
    const pose_graph::VertexIdList& vertex_ids, vi_map::VIMap* vi_map_ptr);

}  // namespace dense_mapping

#endif  // DENSE_MAPPING_DENSE_MAPPING_H_
