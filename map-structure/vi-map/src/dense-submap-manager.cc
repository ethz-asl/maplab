#include "vi-map/dense-submap-manager.h"

namespace vi_map {

void DenseSubmap::addTransformationToSubmap(
    const int64_t timestamp_ns, const aslam::Transformation& T_S0_S) {
  T_S0_S_map_[timestamp_ns] = T_S0_S;
}

bool DenseSubmap::getStampedTransformsToResourceFrameAtTimestamp(
    const int64_t timestamp_ns,
    StampedTransformationMap* stamped_transforms) const {
  int64_t min_difference_ns = std::numeric_limits<int64_t>::max();
  int64_t closest_timestamp_ns;
  for (const auto& stamped_transform : T_S0_S_map_) {
    const int64_t difference_ns =
        std::abs(stamped_transform.first - timestamp_ns);
    if (difference_ns < min_difference_ns) {
      closest_timestamp_ns = stamped_transform.first;
      min_difference_ns = difference_ns;
    }
  }
  if (T_S0_S_map_.find(timestamp_ns) == T_S0_S_map_.end()) {
    return false;
  }
  const aslam::Transformation& T_S0_S_base =
      T_S0_S_map_.at(timestamp_ns);
  std::vector<resources::PointCloud> clouds_T_S_base(T_S0_S_map_.size());
  for (auto it = T_S0_S_map_.begin(); it != T_S0_S_map_.end(); ++it) {
    stamped_transforms->insert(
        std::make_pair(it->first, T_S0_S_base.inverse() * it->second));
  }
  return true;
}

bool DenseSubmap::getMinAndMaxTimestampNs(
    int64_t* min_timestamp_ns, int64_t* max_timestamp_ns) const {
  if (T_S0_S_map_.empty()) {
    return false;
  }
  *min_timestamp_ns = T_S0_S_map_.begin()->first;
  *max_timestamp_ns = T_S0_S_map_.rbegin()->first;
  return true;
}

void DenseSubmapManager::addDenseSubmap(const DenseSubmap& submap) {
  const DenseSubmapId& submap_id = submap.id();
  submap_id_to_idx_[submap_id] = submaps_.size();
  submaps_.push_back(submap);
  mission_id_to_submap_ids_[submap.getMissionId()].push_back(submap_id);
}

bool DenseSubmapManager::getStampedTransformsToResourceFrameAtTimestamp(
    const MissionId& mission_id, const int64_t timestamp_ns,
    StampedTransformationMap* stamped_transforms) const {
  for (const DenseSubmapId& submap_id :
       mission_id_to_submap_ids_.at(mission_id)) {
    int64_t min_timestamp_ns;
    int64_t max_timestamp_ns;
    const DenseSubmap& dense_submap = submaps_[submap_id_to_idx_.at(submap_id)];
    if (!dense_submap.getMinAndMaxTimestampNs(
            &min_timestamp_ns, &max_timestamp_ns)) {
      continue;
    }
    if (min_timestamp_ns <= timestamp_ns && max_timestamp_ns >= timestamp_ns) {
      return dense_submap.getStampedTransformsToResourceFrameAtTimestamp(
          timestamp_ns, stamped_transforms);
    }
  }
  return false;
}

bool DenseSubmapManager::getClosestDenseSubmap(
    const MissionId& mission_id, const int64_t timestamp_ns,
    DenseSubmap* submap) const {
  DenseSubmapId closest_id;
  if (!getClosestDenseSubmapId(mission_id, timestamp_ns, &closest_id)) {
    return false;
  }
  *submap = submaps_[submap_id_to_idx_.at(closest_id)];
  return true;
}

bool DenseSubmapManager::getClosestDenseSubmapId(
    const MissionId& mission_id, const int64_t timestamp_ns,
    DenseSubmapId* submap_id) const {
  CHECK_NOTNULL(submap_id);
  for (const DenseSubmapId& id : mission_id_to_submap_ids_.at(mission_id)) {
    int64_t min_timestamp_ns;
    int64_t max_timestamp_ns;
    const DenseSubmap& dense_submap = submaps_[submap_id_to_idx_.at(id)];
    if (!dense_submap.getMinAndMaxTimestampNs(
            &min_timestamp_ns, &max_timestamp_ns)) {
      continue;
    }
    if (min_timestamp_ns <= timestamp_ns && max_timestamp_ns >= timestamp_ns) {
      *submap_id = id;
      return true;
    }
  }
  return false;
}

}  // namespace vi_map
