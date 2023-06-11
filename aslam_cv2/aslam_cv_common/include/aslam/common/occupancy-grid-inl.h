#ifndef ASLAM_FEATURE_TRACKER_OCCUPANCY_GRID_INL_H_
#define ASLAM_FEATURE_TRACKER_OCCUPANCY_GRID_INL_H_

#include <algorithm>
#include <unordered_map>

#include <aslam/common/stl-helpers.h>
#include <opencv2/imgproc.hpp>

namespace aslam {
namespace common {

template<typename PointType>
WeightedOccupancyGrid<PointType>::WeightedOccupancyGrid(
    CoordinatesType max_input_coordinate_rows,
    CoordinatesType max_input_coordinate_cols,
    CoordinatesType cell_size_rows, CoordinatesType cell_size_cols)
 : max_input_coordinate_rows_(max_input_coordinate_rows),
   max_input_coordinate_cols_(max_input_coordinate_cols),
   cell_size_rows_(cell_size_rows),
   cell_size_cols_(cell_size_cols),
   current_num_points_(0u) {
  CHECK_GT(max_input_coordinate_rows, static_cast<CoordinatesType>(0.0));
  CHECK_GT(max_input_coordinate_cols, static_cast<CoordinatesType>(0.0));
  CHECK_GE(max_input_coordinate_rows, cell_size_rows);
  CHECK_GE(max_input_coordinate_cols, cell_size_cols);

  // Allocate the grid matrix.
  num_grid_rows_ = static_cast<size_t>(std::ceil(
      static_cast<double>(max_input_coordinate_rows) / cell_size_rows_));
  num_grid_cols_ = static_cast<size_t>(std::ceil(
      static_cast<double>(max_input_coordinate_cols) / cell_size_cols_));
  CHECK_GT(num_grid_rows_, 0u);
  CHECK_GT(num_grid_cols_, 0u);

  initializeGrid();
};

template<typename PointType>
void WeightedOccupancyGrid<PointType>::initializeGrid() {
  grid_.resize(num_grid_rows_);
  for (std::vector<PointList>& col : grid_) {
    col.resize(num_grid_cols_);
  }
}

template<typename PointType>
void WeightedOccupancyGrid<PointType>::reset() {
  for (std::vector<PointList>& col : grid_) {
    for (PointList& cell : col) {
      cell.clear();
    }
  }
  current_num_points_ = 0u;
}

template<typename PointType>
void WeightedOccupancyGrid<PointType>::addPointUnconditional(const PointType& point) {
  GridCoordinates grid_coordinates = inputToGridCoordinates(point.u_rows, point.v_cols);
  PointList& cell = getGridCell(grid_coordinates);
  cell.emplace_back(point);
  ++current_num_points_;
}

template<typename PointType>
bool WeightedOccupancyGrid<PointType>::addPointOrReplaceWeakestIfCellFull(
    const PointType& point, size_t max_points_per_cell) {
  CHECK_GT(max_points_per_cell, 0u);

  GridCoordinates grid_coordinates = inputToGridCoordinates(point.u_rows, point.v_cols);
  PointList& cell = getGridCell(grid_coordinates);

  // Add point if the cell point count is below the minimum.
  if (cell.size() < max_points_per_cell) {
    cell.emplace_back(point);
    ++current_num_points_;
    return true;
  }

  // Replace the point with the lowest weight if the cell is full and the
  // added point has a higher weight.
  if (cell.size() >= max_points_per_cell) {
    typename PointList::iterator it_min_weight = std::min_element(cell.begin(), cell.end());
    if (it_min_weight->weight < point.weight) {
      *it_min_weight = point;
      return true;
    }
  }
  return false;
}

template<typename PointType>
void WeightedOccupancyGrid<PointType>::addPointOrReplaceWeakestNearestPoints(
    const PointType& point_to_insert, CoordinatesType min_distance) {
  CHECK_GT(min_distance, static_cast<CoordinatesType>(0.0));
  CHECK_LE(min_distance, static_cast<CoordinatesType>(std::min(cell_size_cols_, cell_size_rows_)))
    << "Distances are only checked in the closest neighboring cells, therefore the min. distance "
    << "has to be smaller or equal to the cell size.";

  // As the min. distance has to be smaller than the grid size, we only need to check points in the
  // cells that are direct neighbors.
  GridCoordinates cell_input_point = inputToGridCoordinates(point_to_insert.u_rows,
                                                            point_to_insert.v_cols);

  size_t i_row_min = std::max(static_cast<int>(cell_input_point.i_rows) - 1, 0);
  size_t j_col_min = std::max(static_cast<int>(cell_input_point.j_cols) - 1, 0);
  size_t i_row_max = std::min(static_cast<int>(cell_input_point.i_rows) + 1,
                              static_cast<int>(num_grid_rows_ - 1));
  size_t j_col_max = std::min(static_cast<int>(cell_input_point.j_cols) + 1,
                              static_cast<int>(num_grid_cols_ - 1));

  // Collect all points from the neighboring cells that violate the min. distance to the point
  // that should be inserted.
  const CoordinatesType min_distance_sq = min_distance * min_distance;

  typedef std::unordered_map<PointList* /*cell*/,
                             std::vector<size_t> /*point index*/> PointIdentifierMap;
  PointIdentifierMap distance_violating_points;
  for (size_t i_row = i_row_min; i_row <= i_row_max; ++i_row) {
    for (size_t j_col = j_col_min; j_col <= j_col_max; ++j_col) {
      PointList& cell = getGridCell(GridCoordinates(i_row, j_col));

      // Add all points of this cell to the violation list if it is too close to the input point.
      for (size_t point_idx = 0u; point_idx < cell.size(); ++point_idx) {
        PointType& point_to_test = cell[point_idx];
        Eigen::Matrix<CoordinatesType, 2, 1> delta(
            point_to_insert.u_rows - point_to_test.u_rows,
            point_to_insert.v_cols - point_to_test.v_cols);

        if (delta.squaredNorm() < min_distance_sq) {
          distance_violating_points[&cell].emplace_back(point_idx);
        }
      }
    }
  }

  // Add the point to the grid cell.
  PointList& input_cell = getGridCell(cell_input_point);
  input_cell.emplace_back(point_to_insert);
  ++current_num_points_;

  // Return if there are no distance violations to other points.
  if (distance_violating_points.empty()) {
    return;
  }

  // Otherwise only keep the point with the highest score from the set
  // (distance_violating_points, point_to_insert) and remove the others.
  distance_violating_points[&input_cell].emplace_back(input_cell.size() - 1);

  // Find the point with the highest score among all conflicting points.
  WeightType highest_weight = std::numeric_limits<WeightType>::lowest();
  PointList* highest_weight_cell = nullptr;
  int highest_weight_violator_idx = -1;

  for (const typename PointIdentifierMap::value_type& cell_indices_pair :
        distance_violating_points) {
    PointList& cell = *CHECK_NOTNULL(cell_indices_pair.first);
    const std::vector<size_t>& point_indices = cell_indices_pair.second;

    for (size_t violator_index = 0u; violator_index < point_indices.size();
         ++violator_index) {
      const WeightType point_weight =
          cell[point_indices[violator_index]].weight;

      if (point_weight > highest_weight) {
        highest_weight = point_weight;
        highest_weight_cell = CHECK_NOTNULL(&cell);
        highest_weight_violator_idx = violator_index;
      }
    }
  }
  CHECK_GT(highest_weight, std::numeric_limits<WeightType>::lowest());
  CHECK_NOTNULL(highest_weight_cell);
  CHECK_GE(highest_weight_violator_idx, 0);
  CHECK_LT(highest_weight_violator_idx, distance_violating_points[highest_weight_cell].size());

  // Exclude the point with the highest weight from the list of points to remove.
  distance_violating_points[highest_weight_cell].erase(
      distance_violating_points[highest_weight_cell].begin() + highest_weight_violator_idx);

  // Now remove all the violating points from the grid.
  for (const typename PointIdentifierMap::value_type& cell_indices_pair :
      distance_violating_points) {
    PointList& cell = *CHECK_NOTNULL(cell_indices_pair.first);
    const std::vector<size_t>& indices_to_remove_from_cell = cell_indices_pair.second;
    PointList reduced_cell =
        aslam::common::eraseIndicesFromVector(cell, indices_to_remove_from_cell);
    cell.swap(reduced_cell);
    current_num_points_ -= indices_to_remove_from_cell.size();
  }
}

template<typename PointType>
size_t WeightedOccupancyGrid<PointType>::getAllPointsInGrid(
    PointList* points) const {
  CHECK_NOTNULL(points)->clear();

  // Go over all points and add to vector.
  for (size_t i_row = 0u; i_row < num_grid_rows_;  ++i_row) {
    for (size_t j_col = 0u; j_col < num_grid_cols_;  ++j_col) {
      const PointList& cell = getGridCell(GridCoordinates(i_row, j_col));
      points->insert(points->end(), cell.begin(), cell.end());
    }
  }

  // We just use this intermediate result to verify the internal state.
  CHECK_EQ(points->size(), getNumPoints());
  return points->size();
}

template<typename PointType>
void
WeightedOccupancyGrid<PointType>::setConstantWeightForAllPointsInGrid(
    WeightType weight) {
  for (size_t i_row = 0u; i_row < num_grid_rows_; ++i_row) {
    for (size_t j_col = 0u; j_col < num_grid_cols_; ++j_col) {
      for (PointType& point : getGridCell(GridCoordinates(i_row, j_col))) {
        point.weight = weight;
      }
    }
  }
}

template<typename PointType>
size_t WeightedOccupancyGrid<PointType>::getNumPoints() const {
#ifdef DEBUG
    size_t num_pts = 0;
    for (const std::vector<PointList>& rows : grid_) {
      for (const PointList& cell : rows) {
        num_pts += cell.size();
      }
    }
    CHECK_EQ(current_num_points_, num_pts);
#endif

  return current_num_points_;
}

template<typename PointType>
typename WeightedOccupancyGrid<PointType>::GridCoordinates
WeightedOccupancyGrid<PointType>::getFullestGridCell() const {
  GridCoordinates coords(0, 0);
  size_t max_size = 0u;
  for (size_t r = 0u; r < num_grid_rows_; ++r) {
    for (size_t c = 0u; c < num_grid_cols_; ++c) {
      if (grid_[r][c].size() > max_size) {
        max_size = grid_[r][c].size();
        coords = GridCoordinates(r, c);
      }
    }
  }
  return coords;
}

template<typename PointType>
void WeightedOccupancyGrid<PointType>::removePointsFromFullestCellsUntilSize(
    size_t max_total_num_points) {
  CHECK_GT(max_total_num_points, 0u);
  while (getNumPoints() > max_total_num_points) {
    const GridCoordinates fullest_cell_coords = getFullestGridCell();
    const size_t max_cell_count = getGridCell(fullest_cell_coords).size();
    removeWeightedPointsFromOverfullCell(
        fullest_cell_coords, max_cell_count - 1);
  }
}

template<typename PointType>
size_t WeightedOccupancyGrid<PointType>::removeWeightedPointsFromOverfullCell(
    const GridCoordinates& grid_coords, size_t max_points_per_cell) {
  CHECK_GT(max_points_per_cell, 0u);

  PointList& cell = getGridCell(grid_coords);
  if (cell.size() <= max_points_per_cell) {
    return 0u;
  }

  // Remove the points with the lowest score.
  size_t num_removed = cell.size() - max_points_per_cell;
  std::partial_sort(cell.begin(), cell.begin() + max_points_per_cell + 1,
                    cell.end(), std::greater<PointType>());
  cell.resize(max_points_per_cell);
  current_num_points_ -= num_removed;
  return num_removed;
}

template<typename PointType>
size_t WeightedOccupancyGrid<PointType>::removeWeightedPointsFromOverfullCells(
    size_t max_points_per_cell) {
  CHECK_GT(max_points_per_cell, 0u);

  size_t num_removed = 0u;
  for (size_t i_row = 0u; i_row < num_grid_rows_; ++i_row) {
    for (size_t j_col = 0u; j_col < num_grid_cols_; ++j_col) {
      num_removed += removeWeightedPointsFromOverfullCell(
          GridCoordinates(i_row, j_col), max_points_per_cell);
    }
  }
  return num_removed;
}

template<typename PointType>
cv::Mat WeightedOccupancyGrid<PointType>::getOccupancyMask(
    CoordinatesType radius_mask_around_points, size_t max_points_per_cell) const {
  CHECK_GT(radius_mask_around_points, static_cast<CoordinatesType>(0.0));
  CHECK_GT(max_points_per_cell, 0u);

  // Go over all cells and mask either the entire cell if the max. point count has been reached
  // in this cell or otherwise mask out the point in this cell.
  cv::Mat mask(static_cast<int>(std::floor(max_input_coordinate_rows_)),
               static_cast<int>(std::floor(max_input_coordinate_cols_)),
               CV_8UC1, cv::Scalar(255));

  for (size_t i_row = 0u; i_row < num_grid_rows_; ++i_row) {
    for (size_t j_col = 0u; j_col < num_grid_cols_; ++j_col) {
      const PointList& cell = getGridCell(GridCoordinates(i_row, j_col));

      // Mask out the individual keypoints in the cell.
      for (const PointType& point : cell) {
        cv::circle(mask, cv::Point(point.v_cols, point.u_rows), radius_mask_around_points,
                   cv::Scalar(0), CV_FILLED);
      }

      // Mask the entire cell if the cell is full.
      if (cell.size() >= max_points_per_cell) {
        cv::Point top_left(j_col * cell_size_cols_, i_row * cell_size_rows_);
        cv::Point bottom_right((j_col + 1) * cell_size_cols_ - 1,
                               (i_row + 1) * cell_size_rows_ - 1);
        cv::rectangle(mask, top_left, bottom_right, cv::Scalar(0), CV_FILLED);
      }
    }
  }
  return mask;
}

template<typename PointType>
inline typename WeightedOccupancyGrid<PointType>::PointList&
WeightedOccupancyGrid<PointType>::getGridCell(
    CoordinatesType u_rows, CoordinatesType v_cols) {
  return getGridCell(inputToGridCoordinates(u_rows, v_cols));
}

template<typename PointType>
inline const typename WeightedOccupancyGrid<PointType>::PointList&
WeightedOccupancyGrid<PointType>::getGridCell(
    CoordinatesType u_rows, CoordinatesType v_cols) const {
  return getGridCell(inputToGridCoordinates(u_rows, v_cols));
}

template<typename PointType>
inline typename WeightedOccupancyGrid<PointType>::PointList&
WeightedOccupancyGrid<PointType>::getGridCell(
    const GridCoordinates& grid_coordinates) {
  CHECK(isValidGridCoordinates(grid_coordinates));
  return grid_[grid_coordinates.i_rows][grid_coordinates.j_cols];
}

template<typename PointType>
inline const typename WeightedOccupancyGrid<PointType>::PointList&
WeightedOccupancyGrid<PointType>::getGridCell(
    const GridCoordinates& grid_coordinates) const {
  CHECK(isValidGridCoordinates(grid_coordinates));
  return grid_[grid_coordinates.i_rows][grid_coordinates.j_cols];
}

template<typename PointType>
inline bool WeightedOccupancyGrid<PointType>::isValidInputCoordinate(
    CoordinatesType u_rows, CoordinatesType v_cols) const {
  return (u_rows >= static_cast<CoordinatesType>(0.0)) &&
         (v_cols >= static_cast<CoordinatesType>(0.0)) &&
         (u_rows < max_input_coordinate_rows_) &&
         (v_cols < max_input_coordinate_cols_);
}

template<typename PointType>
inline bool WeightedOccupancyGrid<PointType>::isValidGridCoordinates(
    const GridCoordinates& grid_coordinates) const {
  return (grid_.size() > grid_coordinates.i_rows) &&
         (grid_[grid_coordinates.i_rows].size() > grid_coordinates.j_cols);
}

template<typename PointType>
inline typename WeightedOccupancyGrid<PointType>::GridCoordinates
WeightedOccupancyGrid<PointType>::inputToGridCoordinates(
    CoordinatesType u_rows, CoordinatesType v_cols) {
  CHECK(isValidInputCoordinate(u_rows, v_cols))
      << "u_rows: " << u_rows << ", v_cols: " << v_cols;
  const size_t grid_row = static_cast<size_t>(std::floor(u_rows / cell_size_rows_));
  const size_t grid_col = static_cast<size_t>(std::floor(v_cols / cell_size_cols_));

  GridCoordinates grid_coordinates(grid_row, grid_col);
  return grid_coordinates;
}

}  // namespace common
}  // namespace aslam

#endif  // ASLAM_FEATURE_TRACKER_OCCUPANCY_GRID_INL_H_
