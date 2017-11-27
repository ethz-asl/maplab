#include "vi-map-helpers/near-camera-pose-sampling.h"

#include <cmath>

#include <vi-map/vi-map.h>

#include "vi-map-helpers/vi-map-geometry.h"

namespace vi_map_helpers {

NearCameraPoseSampling::NearCameraPoseSampling(
    const vi_map::VIMap& map, const Options& options)
    : options_(options), radius2_m2_(options.radius_m * options.radius_m) {
  CHECK_GT(options.x_y_spacing_m(0), 0.);
  CHECK_GT(options.x_y_spacing_m(1), 0.);
  CHECK_GT(options.num_yaw_samples, 0u);
  CHECK_GT(options.radius_m, 0.);

  Eigen::Vector2i grid_radius;
  grid_radius << static_cast<int>(
      ceil(options_.radius_m / options_.x_y_spacing_m(0))),
      static_cast<int>(ceil(options_.radius_m / options_.x_y_spacing_m(1)));
  grid_offsets_around_camera_pose_.resize(
      Eigen::NoChange, 4 * grid_radius.prod() + 2 * grid_radius.sum() + 1);
  int i = 0;
  for (int x = -grid_radius(0); x <= grid_radius(0); ++x) {
    for (int y = -grid_radius(1); y <= grid_radius(1); ++y) {
      grid_offsets_around_camera_pose_.col(i) << x, y;
      ++i;
    }
  }
  CHECK_EQ(i, grid_offsets_around_camera_pose_.cols());

  pose_graph::VertexIdList all_vertex_ids;
  map.getAllVertexIds(&all_vertex_ids);

  Eigen::Matrix3Xd p_G_C(3, all_vertex_ids.size());
  if (p_G_C.row(2).maxCoeff() - p_G_C.row(2).minCoeff() > 0.5) {
    LOG(WARNING) << "Planar pose sampling might not be representative for "
                 << "a dataset with a z variation > 0.5m!";
  }

  VIMapGeometry map_geometry(map);
  for (size_t i = 0u; i < all_vertex_ids.size(); ++i) {
    p_G_C.col(i) = map_geometry
                       .getVisualFrame_T_G_C(
                           vi_map::VisualFrameIdentifier(
                               all_vertex_ids[i], options.camera_index))
                       .getPosition();
  }

  PlanarGrid grid_occupation;
  gridOccupationFrom_p_G_C(p_G_C, &grid_occupation);
  populateFromGrid(grid_occupation, p_G_C.bottomRows<1>().mean());
}

void NearCameraPoseSampling::gridOccupationFrom_p_G_C(
    const Eigen::Matrix3Xd& p_G_C, PlanarGrid* result) const {
  CHECK_NOTNULL(result)->clear();

  const Eigen::Matrix2Xi corresponding_2d_grid_points =
      (options_.x_y_spacing_m.cwiseInverse().asDiagonal() * p_G_C.topRows<2>())
          .cast<int>();

  for (int i = 0; i < p_G_C.cols(); ++i) {
    for (int j = 0; j < grid_offsets_around_camera_pose_.cols(); ++j) {
      const Eigen::Vector2i grid_point_i =
          corresponding_2d_grid_points.col(i) +
          grid_offsets_around_camera_pose_.col(j);
      if ((grid_point_i.cast<double>().cwiseProduct(options_.x_y_spacing_m) -
           p_G_C.topRows<2>().col(i))
              .squaredNorm() < radius2_m2_) {
        result->emplace(grid_point_i);
      }
    }
  }
}

void NearCameraPoseSampling::populateFromGrid(
    const PlanarGrid& grid, const double mean_z_position_m) {
  // Default orientation of camera: z point in viewing direction, y to bottom.
  constexpr double roll_offset = M_PI / 2;
  constexpr double yaw_offset = -M_PI / 2;

  const double yaw_spacing = 2 * M_PI / options_.num_yaw_samples;
  for (const Eigen::Vector2i& grid_point : grid) {
    Eigen::Vector3d position;
    position << grid_point.cast<double>().cwiseProduct(options_.x_y_spacing_m),
        mean_z_position_m;
    for (size_t i = 0u; i < options_.num_yaw_samples; ++i) {
      Eigen::Vector3d roll_pitch_yaw;
      roll_pitch_yaw << roll_offset, 0., yaw_offset + i * yaw_spacing;
      T_G_C_samples_.emplace_back(
          aslam::Transformation::Rotation(
              common::RollPitchYawToRotationMatrix(roll_pitch_yaw)),
          position);
    }
  }
}

}  // namespace vi_map_helpers
