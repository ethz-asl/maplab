#include "rovioli/localizer-helpers.h"

#include <aslam/common/occupancy-grid.h>
#include <localization-summary-map/localization-summary-map-queries.h>
#include <localization-summary-map/localization-summary-map.h>
#include <loop-closure-handler/loop-detector-node.h>
#include <maplab-common/macros.h>
#include <vio-common/vio-types.h>

namespace rovioli {
void convertVertexKeyPointToStructureMatchListToLocalizationResult(
    const summary_map::LocalizationSummaryMap& map,
    const aslam::VisualNFrame& query_nframe,
    const vi_map::VertexKeyPointToStructureMatchList& inlier_structure_matches,
    vio::LocalizationResult* localization_result) {
  CHECK_NOTNULL(localization_result);

  const size_t num_cameras = query_nframe.getNumCameras();
  localization_result->keypoint_measurements_per_camera.resize(num_cameras);
  localization_result->G_landmarks_per_camera.resize(num_cameras);

  const size_t num_matches = inlier_structure_matches.size();
  for (size_t cam_idx = 0u; cam_idx < num_cameras; ++cam_idx) {
    localization_result->keypoint_measurements_per_camera[cam_idx].resize(
        Eigen::NoChange, num_matches);
    localization_result->G_landmarks_per_camera[cam_idx].resize(
        Eigen::NoChange, num_matches);
  }

  std::vector<size_t> num_constraints_per_camera(num_cameras, 0u);
  for (const vi_map::VertexKeyPointToStructureMatch& structure_match :
       inlier_structure_matches) {
    CHECK(structure_match.landmark_result.isValid());

    const size_t camera_idx = structure_match.frame_index_query;
    CHECK_LT(camera_idx, num_cameras);

    Eigen::Matrix3Xd& G_landmarks =
        localization_result->G_landmarks_per_camera[camera_idx];
    Eigen::Matrix2Xd& keypoint_measurements =
        localization_result->keypoint_measurements_per_camera[camera_idx];
    const size_t constraint_idx = num_constraints_per_camera[camera_idx];
    CHECK_LT(constraint_idx, static_cast<size_t>(G_landmarks.cols()));
    CHECK_LT(constraint_idx, static_cast<size_t>(keypoint_measurements.cols()));

    G_landmarks.col(constraint_idx) =
        map.getGLandmarkPosition(structure_match.landmark_result);
    keypoint_measurements.col(constraint_idx) =
        query_nframe.getFrame(camera_idx)
            .getKeypointMeasurement(structure_match.keypoint_index_query);

    ++num_constraints_per_camera[camera_idx];
  }

  for (size_t cam_idx = 0u; cam_idx < num_cameras; ++cam_idx) {
    localization_result->keypoint_measurements_per_camera[cam_idx]
        .conservativeResize(
            Eigen::NoChange, num_constraints_per_camera[cam_idx]);
    localization_result->G_landmarks_per_camera[cam_idx].conservativeResize(
        Eigen::NoChange, num_constraints_per_camera[cam_idx]);
  }
  CHECK(localization_result->isValid());
}

void subselectStructureMatches(
    const summary_map::LocalizationSummaryMap& map,
    const summary_map::SummaryMapCachedLookups& map_cached_lookup,
    const aslam::VisualNFrame& nframe,
    size_t num_max_landmarks_to_keep_per_camera,
    vi_map::VertexKeyPointToStructureMatchList* structure_matches) {
  CHECK_NOTNULL(structure_matches);
  CHECK_GT(num_max_landmarks_to_keep_per_camera, 0u);

  // Early exit.
  if (structure_matches->size() <
      num_max_landmarks_to_keep_per_camera * nframe.getNumCameras()) {
    return;
  }

  // Create an occupancy grid for each camera.
  const size_t kNumOccupancyGridRows = 3u;
  const size_t kNumOccupancyGridCols = 5u;

  typedef aslam::common::WeightedKeypoint<double, double, size_t> PointType;
  typedef aslam::common::WeightedOccupancyGrid<PointType> OccupancyGrid;
  std::vector<OccupancyGrid> grid_per_camera;
  for (size_t cam_idx = 0u; cam_idx < nframe.getNumCameras(); ++cam_idx) {
    const size_t height = nframe.getCamera(cam_idx).imageHeight();
    const size_t width = nframe.getCamera(cam_idx).imageWidth();
    const size_t cell_size_rows = height / kNumOccupancyGridRows;
    const size_t cell_size_cols = width / kNumOccupancyGridCols;
    grid_per_camera.emplace_back(
        OccupancyGrid(height, width, cell_size_rows, cell_size_cols));
  }

  // Populate the grids with all the points.
  size_t match_idx = 0u;
  for (const vi_map::VertexKeyPointToStructureMatch& match :
       *structure_matches) {
    double disparity_angle_rad =
        summary_map::getMaxDisparityRadAngleOfLandmarkObservationBundle(
            map, map_cached_lookup, match.landmark_result);

    const Eigen::Block<Eigen::Matrix2Xd, 2, 1> keypoint =
        nframe.getFrame(match.frame_index_query)
            .getKeypointMeasurement(match.keypoint_index_query);
    CHECK_LT(match.frame_index_query, grid_per_camera.size());
    // Coordinates of the keypoint are swapped as the grid points use the
    // (row, column) convention.
    grid_per_camera[match.frame_index_query].addPointUnconditional(
        PointType(keypoint(1), keypoint(0), disparity_angle_rad, match_idx));
    ++match_idx;
  }

  // Prune landmarks from the grid weighted by disparity angle of the landmark
  // observation rays bundle while ensuring an even distribution.
  std::vector<char> indices_to_remove(structure_matches->size(), true);
  for (size_t cam_idx = 0u; cam_idx < nframe.getNumCameras(); ++cam_idx) {
    OccupancyGrid& grid = grid_per_camera[cam_idx];
    grid.removePointsFromFullestCellsUntilSize(
        num_max_landmarks_to_keep_per_camera);

    std::vector<PointType> subsampled_points;
    grid.getAllPointsInGrid(&subsampled_points);

    for (const PointType& point : subsampled_points) {
      indices_to_remove[point.id] = false;
    }
  }

  int idx = 0;
  structure_matches->erase(
      std::remove_if(
          structure_matches->begin(), structure_matches->end(),
          [&indices_to_remove,
           &idx](const vi_map::VertexKeyPointToStructureMatch&) {
            return indices_to_remove[idx++];
          }),
      structure_matches->end());

  CHECK_LE(
      structure_matches->size(),
      num_max_landmarks_to_keep_per_camera * nframe.getNumCameras());
  CHECK(!structure_matches->empty());
}
}  // namespace rovioli
