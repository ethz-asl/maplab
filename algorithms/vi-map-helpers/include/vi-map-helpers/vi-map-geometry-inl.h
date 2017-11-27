#ifndef VI_MAP_HELPERS_VI_MAP_GEOMETRY_INL_H_
#define VI_MAP_HELPERS_VI_MAP_GEOMETRY_INL_H_

#include <vector>

namespace vi_map_helpers {

template <typename ObjectIdType>
void VIMapGeometry::getBoundingBox(
    Eigen::Vector3d* p_G_min, Eigen::Vector3d* p_G_max) const {
  CHECK_NOTNULL(p_G_min);
  CHECK_NOTNULL(p_G_max);

  std::vector<ObjectIdType> all_object_ids;
  map_.getAllIds(&all_object_ids);
  return getBoundingBox(all_object_ids, p_G_min, p_G_max);
}

template <typename ObjectIdType>
void VIMapGeometry::getBoundingBox(
    const vi_map::MissionId& mission_id, Eigen::Vector3d* p_G_min,
    Eigen::Vector3d* p_G_max) const {
  CHECK_NOTNULL(p_G_min);
  CHECK_NOTNULL(p_G_max);

  std::vector<ObjectIdType> mission_object_ids;
  map_.getAllIdsInMission(mission_id, &mission_object_ids);
  return getBoundingBox(mission_object_ids, p_G_min, p_G_max);
}

template <typename ObjectIdContainerType>
void VIMapGeometry::getBoundingBox(
    const ObjectIdContainerType& object_ids, Eigen::Vector3d* p_G_min,
    Eigen::Vector3d* p_G_max) const {
  CHECK_NOTNULL(p_G_min);
  CHECK_NOTNULL(p_G_max);
  CHECK(!object_ids.empty());
  Eigen::AlignedBox3d box;

  for (const typename ObjectIdContainerType::value_type& object_id :
       object_ids) {
    box.extend(map_.get_p_G(object_id));
  }

  *p_G_min = box.min();
  *p_G_max = box.max();
}

}  // namespace vi_map_helpers

#endif  // VI_MAP_HELPERS_VI_MAP_GEOMETRY_INL_H_
