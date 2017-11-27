#ifndef VI_MAP_HELPERS_SPATIAL_DATABASE_INL_H_
#define VI_MAP_HELPERS_SPATIAL_DATABASE_INL_H_

#include <algorithm>
#include <limits>
#include <vector>

namespace vi_map_helpers {

template <typename ObjectIdType>
SpatialDatabase<ObjectIdType>::SpatialDatabase(
    const vi_map::VIMap& map, const Eigen::Vector3i& grid_resolution)
    : grid_resolution_(grid_resolution), map_geometry_(map), map_(map) {
  CHECK_GT(grid_resolution.minCoeff(), 0);
  std::vector<ObjectIdType> all_map_object_ids;
  map.getAllIds(&all_map_object_ids);
  CHECK(!all_map_object_ids.empty());
  map_geometry_.getBoundingBox<ObjectIdType>(&p_G_min_, &p_G_max_);
  const Eigen::Vector3d box_size(p_G_max_ - p_G_min_);
  CHECK_GT(box_size.maxCoeff(), 0)
      << "The spatial database can not be created with only one vertex.";
  grid_cell_size_ = box_size.cwiseQuotient(grid_resolution.cast<double>());

  for (const ObjectIdType& object_id : all_map_object_ids) {
    Eigen::Vector3d p_G_I = map.get_p_G(object_id);
    Eigen::Vector3i grid_index_3d;
    getGridIndexForPosition(p_G_I, &grid_index_3d);
    spatial_map_[grid_index_3d].push_back(object_id);
    vi_map::MissionIdSet mission_ids;
    // For Vertices always only one mission, for StoreLandmarkIds multiple.
    map_.getMissionIds(object_id, &mission_ids);
    for (const vi_map::MissionId& mission_id : mission_ids) {
      auto found = mission_map_.find(mission_id);
      if (found == mission_map_.end()) {
        GridCellSet cell_set({grid_index_3d});
        mission_map_.insert(std::make_pair(mission_id, cell_set));
      } else {
        found->second.insert(grid_index_3d);
      }
    }
  }
}

template <typename ObjectIdType>
SpatialDatabase<ObjectIdType>::SpatialDatabase(
    const vi_map::VIMap& map, const Eigen::Vector3d& grid_cell_size)
    : grid_cell_size_(grid_cell_size), map_geometry_(map), map_(map) {
  CHECK_GT(grid_cell_size.minCoeff(), 0);
  std::vector<ObjectIdType> all_map_object_ids;
  map.getAllIds(&all_map_object_ids);
  CHECK(!all_map_object_ids.empty());
  map_geometry_.getBoundingBox<ObjectIdType>(&p_G_min_, &p_G_max_);
  const Eigen::Vector3d box_size(p_G_max_ - p_G_min_);
  CHECK_GT(box_size.maxCoeff(), 0)
      << "The spatial database can not be created with only one vertex.";
  Eigen::Vector3d grid_temp_resolution(box_size.cwiseQuotient(grid_cell_size));
  grid_resolution_[0] = static_cast<int>(ceil(grid_temp_resolution[0]));
  grid_resolution_[1] = static_cast<int>(ceil(grid_temp_resolution[1]));
  grid_resolution_[2] = static_cast<int>(ceil(grid_temp_resolution[2]));

  // Redefine p_G_max_ since we defined the grid cells by passing the grid cell
  // size and not by passing the grid resolution (splitting factor).
  p_G_max_ =
      grid_cell_size_.cwiseProduct(grid_resolution_.cast<double>()) + p_G_min_;

  for (const ObjectIdType& object_id : all_map_object_ids) {
    Eigen::Vector3d p_G_I = map.get_p_G(object_id);
    Eigen::Vector3i grid_index_3d;
    getGridIndexForPosition(p_G_I, &grid_index_3d);
    spatial_map_[grid_index_3d].push_back(object_id);
    vi_map::MissionIdSet mission_ids;
    // For Vertices always only one mission, for StoreLandmarkIds multiple.
    map_.getMissionIds(object_id, &mission_ids);
    for (const vi_map::MissionId& mission_id : mission_ids) {
      auto found = mission_map_.find(mission_id);
      if (found == mission_map_.end()) {
        GridCellSet cell_set({grid_index_3d});
        mission_map_.insert(std::make_pair(mission_id, cell_set));
      } else {
        found->second.insert(grid_index_3d);
      }
    }
  }
}

template <typename ObjectIdType>
void SpatialDatabase<ObjectIdType>::getGridResolution(
    Eigen::Vector3i* resolution) const {
  CHECK_NOTNULL(resolution);
  *resolution = grid_resolution_;
}

template <typename ObjectIdType>
typename SpatialDatabase<ObjectIdType>::const_iterator
SpatialDatabase<ObjectIdType>::begin() const {
  return spatial_map_.begin();
}

template <typename ObjectIdType>
typename SpatialDatabase<ObjectIdType>::const_iterator
SpatialDatabase<ObjectIdType>::end() const {
  return spatial_map_.end();
}

template <typename ObjectIdType>
void SpatialDatabase<ObjectIdType>::getObjectIdsInRadius(
    const Eigen::Vector3d& p_G_center, double radius,
    std::unordered_set<ObjectIdType>* neighbors) const {
  CHECK_NOTNULL(neighbors)->clear();
  CHECK_GT(radius, 0.0);
  Eigen::Vector3i grid_index_3d;

  getGridIndexForPosition(p_G_center, &grid_index_3d);

  // Check if entire grid is inside radius by checking if center of the grid
  // has more distance to the sphere border than the diagonal of the grid.
  if (radius - ((p_G_min_ - p_G_max_) / 2 - p_G_center).norm() >
      (p_G_min_ - p_G_max_).norm()) {
    std::vector<ObjectIdType> all_objects;
    map_.getAllIds(&all_objects);
    neighbors->insert(all_objects.begin(), all_objects.end());
    return;
  }

  // Calculate number of grid units in x, y, and z directions
  // that are maximally needed to cover the sphere of given radius in space.
  Eigen::Vector3d radius_in_grid_units, unit_vector(1, 1, 1);
  radius_in_grid_units = radius * unit_vector;
  radius_in_grid_units.cwiseQuotient(grid_cell_size_);
  // Add 1 in order to get ceiling after casting int.
  radius_in_grid_units += unit_vector;
  Eigen::Vector3i num_of_grid_units_in_radius =
      radius_in_grid_units.cast<int>();
  for (int x_index = -num_of_grid_units_in_radius[0];
       x_index <= num_of_grid_units_in_radius[0]; ++x_index) {
    for (int y_index = -num_of_grid_units_in_radius[1];
         y_index <= num_of_grid_units_in_radius[1]; ++y_index) {
      for (int z_index = -num_of_grid_units_in_radius[2];
           z_index <= num_of_grid_units_in_radius[2]; ++z_index) {
        Eigen::Vector3i local_index_3d(x_index, y_index, z_index);
        local_index_3d += grid_index_3d;
        if (isCellEmpty(local_index_3d)) {
          continue;
        }
        std::vector<ObjectIdType> local_objects;
        getObjectIdsOfGridIndex(local_index_3d, &local_objects);
        for (const ObjectIdType& object_id : local_objects) {
          const Eigen::Vector3d distance = p_G_center - map_.get_p_G(object_id);
          if (distance.norm() <= radius) {
            neighbors->insert(object_id);
          }
        }
      }
    }
  }
}

template <typename ObjectIdType>
void SpatialDatabase<ObjectIdType>::getObjectIdsOfMissionInRadius(
    const vi_map::MissionId& mission_id, const Eigen::Vector3d& p_G_center,
    double radius, std::unordered_set<ObjectIdType>* neighbors) const {
  CHECK_NOTNULL(neighbors)->clear();
  std::unordered_set<ObjectIdType> all_neighbors;
  getObjectIdsInRadius(p_G_center, radius, &all_neighbors);
  for (const ObjectIdType& object_id : all_neighbors) {
    if (map_.getMissionId(object_id) == mission_id) {
      neighbors->insert(object_id);
    }
  }
}

template <typename ObjectIdType>
bool SpatialDatabase<ObjectIdType>::isCellEmpty(
    const Eigen::Vector3i& grid_index_3d) const {
  typename std::unordered_map<Eigen::Vector3i,
                              std::vector<ObjectIdType>>::const_iterator
      grid_iterator = spatial_map_.find(grid_index_3d);
  if (grid_iterator != spatial_map_.end()) {
    return grid_iterator->second.empty();
  }
  return true;
}

template <typename ObjectIdType>
double SpatialDatabase<ObjectIdType>::getDistanceToMission(
    const Eigen::Vector3d& p_G, const vi_map::MissionId& mission_id) const {
  auto found = mission_map_.find(mission_id);
  CHECK(found != mission_map_.end());

  Eigen::Vector3i grid_cell_of_p_G;
  getGridIndexForPosition(p_G, &grid_cell_of_p_G);
  double half_grid_cell_diagonal = grid_cell_size_.norm() / 2;

  double minimal_distance = std::numeric_limits<double>::max();
  GridCellSet grid_cell_set = found->second;
  for (const Eigen::Vector3i& grid_cell_index : grid_cell_set) {
    // Check if grid cell content is already out of reach of
    // current minimal distance.
    if (grid_cell_of_p_G != grid_cell_index) {
      double min_distance_to_cell_objects =
          (grid_cell_of_p_G -
           getGridCellCenterPositionForGridIndex(grid_cell_index))
              .norm() -
          half_grid_cell_diagonal;
      if (min_distance_to_cell_objects > minimal_distance) {
        continue;
      }
    }
    std::vector<ObjectIdType> cell_object_id_list;
    getObjectIdsOfGridIndex(grid_cell_index, &cell_object_id_list);
    for (const ObjectIdType& object_id : cell_object_id_list) {
      minimal_distance =
          std::min(minimal_distance, (map_.get_p_G(object_id) - p_G).norm());
    }
  }
  return minimal_distance;
}

template <typename ObjectIdType>
double SpatialDatabase<ObjectIdType>::getSquaredDistanceToGridCell(
    const Eigen::Vector3d& p_G, const Eigen::Vector3i& grid_index) const {
  Eigen::Vector3d grid_cell_origin =
      p_G_min_ + grid_cell_size_.cwiseProduct(grid_index.cast<double>());

  // Check if we are inside GridCell:
  Eigen::Vector3i grid_index_of_querying_p_G;
  getGridIndexForPosition(p_G, &grid_index_of_querying_p_G);
  if (grid_index_of_querying_p_G == grid_index) {
    return 0.;
  } else {
    for (int i = 0; i < 3; ++i) {
      int j = (i + 1) % 3;
      int k = (i + 2) % 3;

      // Check if position is perpendicularly in front of a cell face:
      if (grid_index_of_querying_p_G[i] == grid_index[i]) {
        if (grid_index_of_querying_p_G[j] == grid_index[j]) {
          if (grid_index_of_querying_p_G[k] < grid_index[k]) {
            return (grid_cell_origin[k] - p_G[k]) *
                   (grid_cell_origin[k] - p_G[k]);
          }
          if (grid_index_of_querying_p_G[k] > grid_index[k]) {
            return (p_G[k] - grid_cell_origin[k] - grid_cell_size_[k]) *
                   (p_G[k] - grid_cell_origin[k] - grid_cell_size_[k]);
          }
        }
      }
    }
  }
  double minimal_squared_distance = std::numeric_limits<double>::max();
  // If not perpendicularly in front of a face nor inside cell, minimal
  // distance is to one of the corners of GridCell.
  for (double x = grid_cell_origin[0];
       x <= grid_cell_origin[0] + grid_cell_size_[0]; x += grid_cell_size_[0]) {
    for (double y = grid_cell_origin[1];
         y <= grid_cell_origin[1] + grid_cell_size_[1];
         y += grid_cell_size_[1]) {
      for (double z = grid_cell_origin[2];
           z <= grid_cell_origin[2] + grid_cell_size_[2];
           z += grid_cell_size_[2]) {
        Eigen::Vector3d grid_cell_corner_position(x, y, z);
        minimal_squared_distance = std::min(
            minimal_squared_distance,
            (p_G - grid_cell_corner_position).squaredNorm());
      }
    }
  }
  return minimal_squared_distance;
}

template <typename ObjectIdType>
void SpatialDatabase<ObjectIdType>::getObjectIdsInCuboid(
    const Eigen::Vector3d& p_G_min, const Eigen::Vector3d& p_G_max,
    std::vector<ObjectIdType>* output_object_ids) const {
  CHECK_NOTNULL(output_object_ids)->clear();
  Eigen::Vector3i min_grid_index, max_grid_index;
  getGridIndexForPosition(p_G_max, &max_grid_index);
  getGridIndexForPosition(p_G_min, &min_grid_index);
  for (int x_index = min_grid_index[0]; x_index <= max_grid_index[0];
       ++x_index) {
    bool local_grid_unit_is_at_border_x =
        (x_index == min_grid_index[0]) || (x_index == max_grid_index[0]);

    for (int y_index = min_grid_index[1]; y_index <= max_grid_index[1];
         ++y_index) {
      bool local_grid_unit_is_at_border_xy = local_grid_unit_is_at_border_x ||
                                             (y_index == min_grid_index[1]) ||
                                             (y_index == max_grid_index[1]);
      for (int z_index = min_grid_index[2]; z_index <= max_grid_index[2];
           ++z_index) {
        bool local_grid_unit_is_at_border = local_grid_unit_is_at_border_xy ||
                                            (z_index == min_grid_index[2]) ||
                                            (z_index == max_grid_index[2]);
        Eigen::Vector3i local_index_3d(x_index, y_index, z_index);

        std::vector<ObjectIdType> local_objects;
        getObjectIdsOfGridIndex(local_index_3d, &local_objects);
        if (local_grid_unit_is_at_border) {
          for (const ObjectIdType& object_id : local_objects) {
            Eigen::Vector3d p_G_X = map_.get_p_G(object_id);
            bool object_is_inside_cuboid = true;
            for (int i = 0; i < 3; ++i) {
              object_is_inside_cuboid = object_is_inside_cuboid &&
                                        (p_G_X[i] <= p_G_max[i]) &&
                                        (p_G_X[i] >= p_G_min[i]);
            }
            if (object_is_inside_cuboid) {
              output_object_ids->push_back(object_id);
            }
          }
        } else {
          output_object_ids->insert(
              output_object_ids->end(), local_objects.begin(),
              local_objects.end());
        }
      }
    }
  }
}

template <typename ObjectIdType>
void SpatialDatabase<ObjectIdType>::getObjectIdsOfMissionInCuboid(
    const vi_map::MissionId& mission_id, const Eigen::Vector3d& p_G_min,
    const Eigen::Vector3d& p_G_max,
    std::vector<ObjectIdType>* output_object_ids) const {
  CHECK_NOTNULL(output_object_ids)->clear();
  std::vector<ObjectIdType> all_output_object_ids;
  getObjectIdsInCuboid(p_G_min, p_G_max, &all_output_object_ids);
  for (const ObjectIdType& object_id : all_output_object_ids) {
    if (map_.getMissionId(object_id) == mission_id) {
      output_object_ids->push_back(object_id);
    }
  }
}

template <typename ObjectIdType>
void SpatialDatabase<ObjectIdType>::getObjectIdsOfGridIndex(
    const Eigen::Vector3i& grid_index,
    std::vector<ObjectIdType>* grid_unit_vertex_ids) const {
  CHECK_NOTNULL(grid_unit_vertex_ids);
  grid_unit_vertex_ids->clear();
  auto it = spatial_map_.find(grid_index);
  if (it != spatial_map_.end()) {
    *grid_unit_vertex_ids = it->second;
  }
}

template <typename ObjectIdType>
void SpatialDatabase<ObjectIdType>::getGridIndexForPosition(
    const Eigen::Vector3d& p_G, Eigen::Vector3i* grid_index_3d) const {
  CHECK_NOTNULL(grid_index_3d);
  Eigen::Vector3d relative_position = p_G - p_G_min_;
  Eigen::Vector3d temp_position;
  temp_position = relative_position.cwiseQuotient(grid_cell_size_);
  (*grid_index_3d)[0] = static_cast<int>(floor(temp_position[0]));
  (*grid_index_3d)[1] = static_cast<int>(floor(temp_position[1]));
  (*grid_index_3d)[2] = static_cast<int>(floor(temp_position[2]));

  // Check if some objects are on the external border of a cell and could be
  // erroneously assigned to a cell outside of box.
  // Usually, the border facing to cells of higher index are not included in a
  // box. However, the values that define the p_G_max values are exceptions of
  // that rule.
  if (relative_position.cwiseEqual(p_G_max_ - p_G_min_).count() > 0) {
    for (int i = 0; i < 3; ++i) {
      if (relative_position[i] == (p_G_max_ - p_G_min_)[i]) {
        // grid_resolution_[i]-1 is equal to index of cell of highest index in
        // i:
        (*grid_index_3d)[i] = grid_resolution_[i] - 1;
      }
    }
  }
}

template <typename ObjectIdType>
Eigen::Vector3d
SpatialDatabase<ObjectIdType>::getGridCellCenterPositionForGridIndex(
    const Eigen::Vector3i& grid_index_3d) const {
  return p_G_min_ + grid_index_3d.cast<double>().cwiseProduct(grid_cell_size_) +
         grid_cell_size_ / 2;
}

template <typename ObjectIdType>
Eigen::Vector3d SpatialDatabase<ObjectIdType>::getGridCellSize() const {
  return grid_cell_size_;
}

template <typename ObjectIdType>
const vi_map::VIMap& SpatialDatabase<ObjectIdType>::getMap() const {
  return map_;
}

}  // namespace vi_map_helpers

#endif  // VI_MAP_HELPERS_SPATIAL_DATABASE_INL_H_
