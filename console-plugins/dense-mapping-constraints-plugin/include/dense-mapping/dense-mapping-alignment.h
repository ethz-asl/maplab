#ifndef DENSE_MAPPING_DENSE_MAPPING_ALIGNMENT_H_
#define DENSE_MAPPING_DENSE_MAPPING_ALIGNMENT_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <registration-toolbox/common/base-controller.h>
#include <registration-toolbox/mock-controller.h>
#include <registration-toolbox/model/registration-result.h>
#include <registration-toolbox/pcl-icp-controller.h>
#include <vi-map/vi-map.h>

#include "dense-mapping/dense-mapping-common.h"
#include "dense-mapping/dense-mapping-gflags.h"

namespace dense_mapping {

struct AlignmentConfig {
  static AlignmentConfig fromGflags();

  double maximum_deviation_from_initial_guess_delta_position_m;
  double maximum_deviation_from_initial_guess_delta_rotation_deg;
};

bool computeAlignmentForCandidatePairs(
    const AlignmentConfig& config, const vi_map::VIMap& map,
    const AlignmentCandidatePairs& candidate_pairs,
    AlignmentCandidatePairs* aligned_candidate_pairs);

template <typename ResourceDataType>
bool computeAlignmentForCandidatePairsImpl(
    const vi_map::VIMap& map, const AlignmentCandidatePair& pair,
    std::shared_ptr<regbox::BaseController>* aligner_ptr,
    AlignmentCandidatePair* aligned_pair);

bool computeAlignmentForIncrementalSubmapCandidatePairs(
    const AlignmentConfig& config,
    const AlignmentCandidatePairs& candidate_pairs, vi_map::VIMap* vi_map_ptr,
    AlignmentCandidatePairs* aligned_candidate_pairs);

}  // namespace dense_mapping

#endif  // DENSE_MAPPING_DENSE_MAPPING_ALIGNMENT_H_
