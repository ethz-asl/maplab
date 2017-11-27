#ifndef VI_MAP_HELPERS_SPATIAL_DATABASE_H_
#define VI_MAP_HELPERS_SPATIAL_DATABASE_H_

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Eigen/Core>
#include <aslam/common/memory.h>
#include <gtest/gtest_prod.h>
#include <vi-map/unique-id.h>
#include <vi-map/vi-map.h>

#include "vi-map-helpers/vi-map-geometry.h"

namespace std {
template <>
struct hash<Eigen::Vector3i> {
  typedef Eigen::Vector3i argument_type;
  typedef std::size_t value_type;
  value_type operator()(const argument_type& vector) const {
    static constexpr size_t kHashPrime = 805306457;
    return vector[0] * kHashPrime + vector[1] * kHashPrime * kHashPrime +
           vector[2] * kHashPrime * kHashPrime * kHashPrime;
  }
};
}  // namespace std

namespace vi_map {
class VIMap;
}  // namespace vi_map

namespace vi_map_helpers {

template <typename ObjectIdType>
class SpatialDatabase {
 public:
  typedef std::unordered_map<Eigen::Vector3i, std::vector<ObjectIdType>>
      SpatialMap;

  typedef AlignedUnorderedSet<Eigen::Vector3i> GridCellSet;

  typedef AlignedUnorderedMap<vi_map::MissionId, GridCellSet>
      MissionGridCellsMap;

  typedef typename SpatialMap::value_type value_type;

  typedef typename SpatialMap::const_iterator const_iterator;

  SpatialDatabase(
      const vi_map::VIMap& map, const Eigen::Vector3i& grid_resolution);

  SpatialDatabase(
      const vi_map::VIMap& map, const Eigen::Vector3d& grid_cell_size);

  const_iterator begin() const;

  const_iterator end() const;

  void getGridResolution(Eigen::Vector3i* resolution) const;

  void getObjectIdsInRadius(
      const Eigen::Vector3d& p_G_center, double radius_meters,
      std::unordered_set<ObjectIdType>* neighbors) const;

  void getObjectIdsOfMissionInRadius(
      const vi_map::MissionId& mission_id, const Eigen::Vector3d& p_G_center,
      double radius, std::unordered_set<ObjectIdType>* neighbors) const;

  bool isCellEmpty(const Eigen::Vector3i& grid_index_3d) const;

  double getDistanceToMission(
      const Eigen::Vector3d& p_G, const vi_map::MissionId& mission_id) const;

  double getSquaredDistanceToGridCell(
      const Eigen::Vector3d& p_G, const Eigen::Vector3i& grid_index) const;

  void getObjectIdsInCuboid(
      const Eigen::Vector3d& p_G_min, const Eigen::Vector3d& p_G_max,
      std::vector<ObjectIdType>* output_vertex_ids) const;

  void getObjectIdsOfMissionInCuboid(
      const vi_map::MissionId& mission_id, const Eigen::Vector3d& p_G_min,
      const Eigen::Vector3d& p_G_max,
      std::vector<ObjectIdType>* output_object_ids) const;

  void getObjectIdsOfGridIndex(
      const Eigen::Vector3i& grid_index,
      std::vector<ObjectIdType>* grid_unit_vertex_ids) const;

  void getGridIndexForPosition(
      const Eigen::Vector3d& p_G_I, Eigen::Vector3i* grid_index_3d) const;
  Eigen::Vector3d getGridCellCenterPositionForGridIndex(
      const Eigen::Vector3i& grid_index_3d) const;

  Eigen::Vector3d getGridCellSize() const;

  const vi_map::VIMap& getMap() const;

  inline size_t size() const {
    return spatial_map_.size();
  }

  inline size_t empty() const { return size() == 0u; }

 private:
  Eigen::Vector3d grid_cell_size_;
  Eigen::Vector3i grid_resolution_;
  Eigen::Vector3d p_G_min_;
  Eigen::Vector3d p_G_max_;

  SpatialMap spatial_map_;
  const VIMapGeometry map_geometry_;
  MissionGridCellsMap mission_map_;

 protected:
  const vi_map::VIMap& map_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace vi_map_helpers

#include "vi-map-helpers/spatial-database-inl.h"

#endif  // VI_MAP_HELPERS_SPATIAL_DATABASE_H_
