/*
 *    Filename: temperature-mapping.cc
 *  Created on: April 26, 2021
 *      Author: Florian Achermann (acfloria@ethz.ch)
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */
/*
#include "grid-map-amo/temperature-mapping.h"

#include <math.h>

#include <Eigen/Core>
#include <glog/logging.h>
#include <opencv2/imgcodecs.hpp>


namespace amo {

void update_temperature_layer(std::unique_ptr<grid_map::GridMap>& map,
    std::string temperature_layer,
    std::string observation_angle_layer,
    std::string elevation_layer,
    const std::unique_ptr<grid_map::Position3>& map_offset,
    const aslam::VisualNFrame::PtrVector& nframes_in,
    const swe::NFrameIdViNodeStateMap& vi_map,
    int thermal_cam_idx) {
  CHECK_NOTNULL(map);
  CHECK_NOTNULL(map_offset);
  CHECK_EQ(vi_map.size(), nframes_in.size());
  CHECK(map->exists(temperature_layer));
  CHECK(map->exists(observation_angle_layer));
  CHECK(map->exists(elevation_layer));
  CHECK_GT(nframes_in.size(), 0u);
  CHECK_LT(thermal_cam_idx, nframes_in[0]->getNumCameras());

  grid_map::Matrix& temp_layer = (*map)[temperature_layer];
  grid_map::Matrix& obs_angle_layer = (*map)[observation_angle_layer];
  const aslam::Camera& cam = nframes_in[0]->getCamera(thermal_cam_idx);

  for (grid_map::GridMapIterator it(*map); !it.isPastEnd(); ++it) {
    grid_map::Position3 position;
    map->getPosition3(elevation_layer, *it, position);
    position += *map_offset;
    const grid_map::Index index(*it);
    double x = index(0);
    double y = index(1);

    const aslam::Transformation T_C_B = nframes_in[0]->get_T_C_B(thermal_cam_idx);

    for (size_t frame_idx = 0u; frame_idx < nframes_in.size(); ++frame_idx) {
      const aslam::Transformation T_C_W = T_C_B * vi_map.at(nframes_in[frame_idx]->getId()).get_T_M_I().inverse();
      const Eigen::Vector3d position_transformed = T_C_W * position;

      Eigen::Vector2d kp;
      aslam::ProjectionResult result = cam.project3(position_transformed, &kp);

      if (result == aslam::ProjectionResult::KEYPOINT_VISIBLE) {

        const double observation_angle = asin(std::fabs(position_transformed[2]) / position_transformed.norm());
        CHECK(observation_angle > 0.0);

        if (std::fabs(observation_angle) > obs_angle_layer(x, y) || std::isnan(obs_angle_layer(x, y))) {

          const aslam::VisualFrame& frame = nframes_in[frame_idx]->getFrame(thermal_cam_idx);
          CHECK(frame.hasRawImage());

          const cv::Mat& im = frame.getRawImage();
          CHECK_EQ(im.depth(), CV_16U);
          CHECK_EQ(im.channels(), 1);

          const int kp_y = static_cast<int>(std::round(kp(1)));
          const int kp_x = static_cast<int>(std::round(kp(0)));

          // Temporary fix since due to the undistortion the camera image is is larger than the actual undistorted image size
          if ((kp_y < im.size().height) &&
              (kp_x < im.size().width)) {
            temp_layer(x, y) = im.at<uint16_t>(kp_y, kp_x) * 0.04 - 273.15;
            obs_angle_layer(x, y) = std::fabs(observation_angle);
          }
        }
      }
    }
  }
}


void update_temperature_layer_projection(std::unique_ptr<grid_map::GridMap>& map,
    std::string temperature_layer,
    std::string observation_angle_layer,
    std::string elevation_layer,
    const std::unique_ptr<grid_map::Position3>& map_offset,
    const aslam::VisualNFrame::PtrVector& nframes_in,
    const swe::NFrameIdViNodeStateMap& vi_map,
    int thermal_cam_idx) {
  CHECK_NOTNULL(map);
  CHECK_NOTNULL(map_offset);
  CHECK_EQ(vi_map.size(), nframes_in.size());
  CHECK(map->exists(temperature_layer));
  CHECK(map->exists(observation_angle_layer));
  CHECK(map->exists(elevation_layer));
  CHECK_GT(nframes_in.size(), 0u);
  CHECK_LT(thermal_cam_idx, nframes_in[0]->getNumCameras());

  grid_map::Matrix& temp_layer = (*map)[temperature_layer];
  grid_map::Matrix& obs_angle_layer = (*map)[observation_angle_layer];
  const aslam::Camera& cam = nframes_in[0]->getCamera(thermal_cam_idx);
  const aslam::Transformation T_C_B = nframes_in[0]->get_T_C_B(thermal_cam_idx);

  const double min_height = (*map)[elevation_layer].minCoeff();
  CHECK(!std::isnan(min_height));

  std::array<Eigen::Vector3d, 4> corner_ray{};
  cam.backProject3(Eigen::Vector2d(0, 0), &corner_ray[0]);
  cam.backProject3(Eigen::Vector2d(0, cam.imageHeight() - 1), &corner_ray[1]);
  cam.backProject3(Eigen::Vector2d(cam.imageWidth() - 1, cam.imageHeight() - 1), &corner_ray[2]);
  cam.backProject3(Eigen::Vector2d(cam.imageWidth() - 1, 0), &corner_ray[3]);

  std::array<Eigen::Vector3d, 4> position_corners{};
  for (size_t frame_idx = 0u; frame_idx < nframes_in.size(); ++frame_idx) {
    const aslam::VisualFrame& frame = nframes_in[frame_idx]->getFrame(thermal_cam_idx);
    CHECK(frame.hasRawImage());

    const cv::Mat& im = frame.getRawImage();
    CHECK_EQ(im.depth(), CV_16U);
    CHECK_EQ(im.channels(), 1);

    const aslam::Transformation T_C_W = T_C_B * vi_map.at(nframes_in[frame_idx]->getId()).get_T_M_I().inverse();
    const aslam::Transformation T_W_C = T_C_W.inverse();
    const aslam::Position3D p_C_map = T_C_W.inverse().getPosition() - *map_offset;

    for (size_t i = 0u; i < 4; i++) {
      const Eigen::Vector3d ray_rotated = T_W_C.getRotation().rotate(corner_ray[i]);
      const double multiplier = (min_height - p_C_map[2]) / ray_rotated[2];
      position_corners[i] = p_C_map + multiplier * ray_rotated;
    }

    grid_map::Polygon polygon;
    polygon.setFrameId(map->getFrameId());
    polygon.addVertex(grid_map::Position(position_corners[0][0], position_corners[0][1]));
    polygon.addVertex(grid_map::Position(position_corners[1][0], position_corners[1][1]));
    polygon.addVertex(grid_map::Position(position_corners[2][0], position_corners[2][1]));
    polygon.addVertex(grid_map::Position(position_corners[3][0], position_corners[3][1]));
    polygon.addVertex(grid_map::Position(position_corners[0][0], position_corners[0][1]));

    for (grid_map::PolygonIterator it(*map, polygon);
          !it.isPastEnd(); ++it) {
      grid_map::Position3 position;
      map->getPosition3(elevation_layer, *it, position);
      position += *map_offset;
      const grid_map::Index index(*it);
      const double x = index(0);
      const double y = index(1);

      const Eigen::Vector3d position_transformed = T_C_W * position;

      Eigen::Vector2d kp;
      const aslam::ProjectionResult result = cam.project3(position_transformed, &kp);

      if (result == aslam::ProjectionResult::KEYPOINT_VISIBLE) {
        const double observation_angle = asin(std::fabs(position_transformed[2]) / position_transformed.norm());
        CHECK(observation_angle > 0.0);

        if (std::fabs(observation_angle) > obs_angle_layer(x, y) || std::isnan(obs_angle_layer(x, y))) {
          const int kp_y = static_cast<int>(std::round(kp(1)));
          const int kp_x = static_cast<int>(std::round(kp(0)));

          // Temporary fix since due to the undistortion the camera image is is larger than the actual undistorted image size
          if ((kp_y < im.size().height) &&
              (kp_x < im.size().width)) {
            temp_layer(x, y) = im.at<uint16_t>(kp_y, kp_x) * 0.04 - 273.15;
            obs_angle_layer(x, y) = std::fabs(observation_angle);
          }
        }
      }
    }
  }
}

}  // namespace amo
*/