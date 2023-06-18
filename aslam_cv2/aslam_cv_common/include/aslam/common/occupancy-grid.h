#ifndef ASLAM_FEATURE_TRACKER_OCCUPANCY_GRID_H_
#define ASLAM_FEATURE_TRACKER_OCCUPANCY_GRID_H_

#include <vector>

#include <aslam/common/macros.h>
#include <Eigen/Dense>
#include <glog/logging.h>
#include <opencv2/core/core.hpp>

namespace aslam {
namespace common {

template<typename CoordinatesType = double, typename WeightType = double, typename PointId = int>
struct WeightedKeypoint {
  typedef CoordinatesType coordinate_type;
  typedef WeightType weight_type;
  typedef PointId id_type;

  WeightedKeypoint() {
    LOG(FATAL) << "Only needed to downsize vectors using resize.";
  }
  WeightedKeypoint(CoordinatesType u_rows, CoordinatesType v_cols, WeightType weight, PointId id)
    : u_rows(u_rows), v_cols(v_cols), weight(weight), id(id) {}

  CoordinatesType u_rows, v_cols;
  WeightType weight;
  PointId id;

  inline bool operator<(const WeightedKeypoint& other) const {
    return weight < other.weight;
  }
  inline bool operator>(const WeightedKeypoint& other) const {
    return weight > other.weight;
  }
};

template<typename PointType = WeightedKeypoint<double, double, int>>
class WeightedOccupancyGrid {
 public:
  typedef typename PointType::coordinate_type CoordinatesType;
  typedef typename PointType::weight_type WeightType;
  typedef typename PointType::id_type PointId;

  typedef PointType Point;
  typedef std::vector<PointType> PointList;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ASLAM_POINTER_TYPEDEFS(WeightedOccupancyGrid);

 private:
  struct GridCoordinates {
    GridCoordinates(size_t i, size_t j) : i_rows(i), j_cols(j) {}
    size_t i_rows, j_cols;
  };

  //////////////////////////////////////////////////////////////
  /// \name Constructors/destructors and operators
  /// @{
 public:
  WeightedOccupancyGrid(CoordinatesType max_input_coordinate_rows,
                        CoordinatesType max_input_coordinate_cols,
                        CoordinatesType cell_size_rows,
                        CoordinatesType cell_size_cols);
  void reset();

 private:
  void initializeGrid();
  /// @}

  //////////////////////////////////////////////////////////////
  /// \name Methods to add points to the grid.
  /// @{
 public:
  /// Unconditionally add a point to the grid.
  void addPointUnconditional(const PointType& point);

  /// Add a point to the grid and replace other points in the same cell if the cell is full
  /// and this point has a higher score.
  bool addPointOrReplaceWeakestIfCellFull(const PointType& point, size_t max_points_per_cell);

  /// Add point to the grid and replace other points if they are closer than the specified min.
  /// distance and this point has a higher score.
  void addPointOrReplaceWeakestNearestPoints(const PointType& point, CoordinatesType min_distance);
  /// @}

  //////////////////////////////////////////////////////////////
  /// \name Methods to query the grid.
  /// @{
 public:
  size_t getNumPoints() const;
  size_t getAllPointsInGrid(PointList* points) const;

  inline PointList& getGridCell(CoordinatesType u_rows, CoordinatesType v_cols);
  inline const PointList& getGridCell(CoordinatesType u_rows, CoordinatesType v_cols) const;

  inline GridCoordinates getFullestGridCell() const;

  /// Return a mask that covers a square of specified size around all points and masked cells
  /// if selected.
  cv::Mat getOccupancyMask(CoordinatesType radius_mask_around_points,
                           size_t max_points_per_cell) const;

 private:
  inline PointList& getGridCell(const GridCoordinates& grid_coordinates);
  inline const PointList& getGridCell(const GridCoordinates& grid_coordinates) const;
  /// @}

  //////////////////////////////////////////////////////////////
  /// \name Methods to manipulate existing points in the grid.
  /// @{
 public:
  /// Methods to filter/modify the points in the grid.
  void setConstantWeightForAllPointsInGrid(WeightType weight);

  /// Remove points from cells that contain more points than specified. Points with the lowest
  /// score will be removed first.
  size_t removeWeightedPointsFromOverfullCells(size_t max_points_per_cell);

  size_t removeWeightedPointsFromOverfullCell(
      const GridCoordinates& grid_coords, size_t max_points_per_cell);

  /// Remove the weakest point from the fullest cells until the total number of
  /// points in the grid is met.
  void removePointsFromFullestCellsUntilSize(size_t max_total_num_points);
  /// @}

 private:
  GridCoordinates inputToGridCoordinates(CoordinatesType u_rows, CoordinatesType v_cols);
  inline bool isValidInputCoordinate(CoordinatesType u_rows, CoordinatesType v_cols) const;
  inline bool isValidGridCoordinates(const GridCoordinates& grid_coordinates) const;

  /// Grid size definitions.
  const CoordinatesType max_input_coordinate_rows_;
  const CoordinatesType max_input_coordinate_cols_;
  const CoordinatesType cell_size_rows_;
  const CoordinatesType cell_size_cols_;

  size_t num_grid_rows_;
  size_t num_grid_cols_;
  size_t current_num_points_;

  /// The grid is indexed by i_rows and j_cols where as the original coordinates are u and v.
  std::vector<std::vector<PointList>> grid_;
};

}  // namespace common
}  // namespace aslam
#include "./occupancy-grid-inl.h"
#endif  // ASLAM_FEATURE_TRACKER_OCCUPANCY_GRID_H_
