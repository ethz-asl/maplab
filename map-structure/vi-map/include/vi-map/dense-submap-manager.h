#ifndef VI_MAP_DENSE_SUBMAP_MANAGER_H_
#define VI_MAP_DENSE_SUBMAP_MANAGER_H_

#include <map>
#include <unordered_map>
#include <vector>

#include <aslam/common/pose-types.h>
#include <aslam/common/unique-id.h>
#include <map-resources/temporal-resource-id-buffer.h>
#include "vi-map/unique-id.h"

namespace vi_map {
typedef std::map<int64_t, aslam::Transformation> StampedTransformationMap;
class VIMap;
class DenseSubmap {
 public:
  DenseSubmap(const MissionId& mission_id, const aslam::SensorId& sensor_id)
      : submap_id_(aslam::createRandomId<DenseSubmapId>()),
        mission_id_(mission_id),
        sensor_id_(sensor_id) {}
  DenseSubmap() {}

  void addTransformationToSubmap(
      const int64_t timestamp_ns, const aslam::Transformation& T_S0_S);

  DenseSubmapId id() const {
    return submap_id_;
  }

  MissionId getMissionId() const {
    return mission_id_;
  }

  aslam::SensorId getSensorId() const {
    return sensor_id_;
  }

  void getStampedTransformsToClosestFrame(
      const int64_t timestamp_ns,
      StampedTransformationMap* stamped_transforms) const;

  bool getMinAndMaxTimestampNs(
      int64_t* min_timestamp_ns, int64_t* max_timestamp_ns) const;

  size_t size() const {
    return T_S0_S_map_.size();
  }

 private:
  DenseSubmapId submap_id_;
  MissionId mission_id_;
  aslam::SensorId sensor_id_;
  StampedTransformationMap T_S0_S_map_;
};

class DenseSubmapManager {
 public:
  DenseSubmapManager() {}
  void addDenseSubmap(const DenseSubmap& submap);
  bool getClosestDenseSubmapStampedTransforms(
      const MissionId& mission_id, const int64_t timestamp_ns,
      StampedTransformationMap* stamped_transforms) const;
  void merge(const DenseSubmapManager& other) {
    for (const DenseSubmap& submap : other.submaps_) {
      const DenseSubmapId& submap_id = submap.id();
      submap_id_to_idx_[submap_id] = submaps_.size();
      submaps_.push_back(submap);
      mission_id_to_submap_ids_[submap.getMissionId()].push_back(submap_id);
    }
  }
  bool getClosestDenseSubmapId(
      const MissionId& mission_id, const int64_t timestamp_ns,
      DenseSubmapId* submap_id) const;
  bool getClosestDenseSubmap(
      const MissionId& mission_id, const int64_t timestamp_ns,
      DenseSubmap* submap) const;

 private:
  std::unordered_map<MissionId, std::vector<DenseSubmapId>>
      mission_id_to_submap_ids_;
  std::unordered_map<DenseSubmapId, size_t> submap_id_to_idx_;
  std::vector<DenseSubmap> submaps_;
};

}  // namespace vi_map
#endif  // VI_MAP_DENSE_SUBMAP_MANAGER_H_
