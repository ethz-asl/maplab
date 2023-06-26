/*
 *    Filename: temperature-mapping.cc
 *  Created on: April 26, 2021
 *      Author: Florian Achermann (acfloria@ethz.ch)
 *    Modified: Luka Dragomirovic (lukavuk01@sunrise.ch)
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid-map-amo/temperature-mapping.h"

#include <math.h>
#include <memory>
#include <Eigen/Core>
#include <glog/logging.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <vi-map/vi-map.h>


namespace grid_map_amo {

void update_temperature_layer(std::unique_ptr<grid_map::GridMap>& map,
    std::string temperature_layer,
    std::string observation_angle_layer,
    std::string elevation_layer,
    const vi_map::VIMap& vi_map,
    int thermal_cam_idx) {
  CHECK_NOTNULL(map);
  CHECK(map->exists(temperature_layer));
  CHECK(map->exists(observation_angle_layer));
  CHECK(map->exists(elevation_layer));

  grid_map::Matrix& temp_layer = (*map)[temperature_layer];
  grid_map::Matrix& obs_angle_layer = (*map)[observation_angle_layer];

  vi_map::MissionIdList all_missions;
  vi_map.getAllMissionIds(&all_missions);
  for (const vi_map::MissionId& mission_id : all_missions) {
    pose_graph::VertexIdList all_vertices_in_mission;
    vi_map.getAllVertexIdsInMissionAlongGraph(
        mission_id, &all_vertices_in_mission);

    for (grid_map::GridMapIterator it(*map); !it.isPastEnd(); ++it) {
      grid_map::Position3 position;
      map->getPosition3(elevation_layer, *it, position);
      const grid_map::Index index(*it);
      double x = index(0);
      double y = index(1);

      const aslam::NCamera& n_camera = vi_map.getMissionNCamera(mission_id);
      const aslam::Camera& cam = n_camera.getCamera(thermal_cam_idx);
      const aslam::Transformation T_C_B = n_camera.get_T_C_B(thermal_cam_idx);

      for (const pose_graph::VertexId& vertex_id : all_vertices_in_mission) {
        const vi_map::Vertex& vertex = vi_map.getVertex(vertex_id);
        const aslam::Transformation T_G_B = vi_map.getVertex_T_G_I(vertex_id);
        const aslam::Transformation T_G_C = T_G_B * T_C_B.inverse();

        const aslam::Position3D p_C_map = T_G_C.getPosition();
        const Eigen::Vector3d position_transformed = T_G_C.inverse() * position;
        
        Eigen::Vector2d kp;
        aslam::ProjectionResult result = cam.project3(position_transformed, &kp);

        if (result == aslam::ProjectionResult::KEYPOINT_VISIBLE) {

          const double observation_angle = asin(std::fabs(position_transformed[2]) / position_transformed.norm());
          CHECK(observation_angle > 0.0);

          if (std::fabs(observation_angle) > obs_angle_layer(x, y) || std::isnan(obs_angle_layer(x, y))) {

            cv::Mat im;
            //in case of thermal images (raw depth maps)
            if(vi_map.getFrameResource(
                vertex, thermal_cam_idx, backend::ResourceType::kRawDepthMap, &im)) {
                VLOG(5) << "Found raw depth map.";
            }

            //in case of thermal images (raw depth maps)
            else if(vi_map.getFrameResource(
                vertex, thermal_cam_idx, backend::ResourceType::kImageForDepthMap, &im)) {
                VLOG(5) << "Found raw image for depth map.";
            }

            //in case of no image resource
            else {
              LOG(ERROR) << "No images have been found.";
            }

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
}


void update_temperature_layer_projection(std::unique_ptr<grid_map::GridMap>& map,
    std::string temperature_layer,
    std::string observation_angle_layer,
    std::string elevation_layer,
    const vi_map::VIMap& vi_map,
    int thermal_cam_idx) {
  CHECK_NOTNULL(map);
  CHECK(map->exists(temperature_layer));
  CHECK(map->exists(observation_angle_layer));
  CHECK(map->exists(elevation_layer));

  grid_map::Matrix& temp_layer = (*map)[temperature_layer];
  grid_map::Matrix& obs_angle_layer = (*map)[observation_angle_layer];
  
  vi_map::MissionIdList all_missions;
  vi_map.getAllMissionIds(&all_missions);
  for (const vi_map::MissionId& mission_id : all_missions) {
    pose_graph::VertexIdList all_vertices_in_mission;
    vi_map.getAllVertexIdsInMissionAlongGraph(
        mission_id, &all_vertices_in_mission);

    const aslam::NCamera& n_camera = vi_map.getMissionNCamera(mission_id);
    const aslam::Camera& cam = n_camera.getCamera(thermal_cam_idx);
    const aslam::Transformation T_C_B = n_camera.get_T_C_B(thermal_cam_idx);

    const double min_height = (*map)[elevation_layer].minCoeff();
    CHECK(!std::isnan(min_height));

    std::array<Eigen::Vector3d, 4> corner_ray{};
    cam.backProject3(Eigen::Vector2d(0, 0), &corner_ray[0]);
    cam.backProject3(Eigen::Vector2d(0, cam.imageHeight() - 1), &corner_ray[1]);
    cam.backProject3(Eigen::Vector2d(cam.imageWidth() - 1, cam.imageHeight() - 1), &corner_ray[2]);
    cam.backProject3(Eigen::Vector2d(cam.imageWidth() - 1, 0), &corner_ray[3]);

    for(int r = 0;r < temp_layer.rows();r++) {
      for(int c = 0;c < temp_layer.cols();c++) {
        temp_layer(r,c) = 0;
      }
    }

    std::array<Eigen::Vector3d, 4> position_corners{};
    for (const pose_graph::VertexId& vertex_id : all_vertices_in_mission) {
      const vi_map::Vertex& vertex = vi_map.getVertex(vertex_id);
      const aslam::Transformation T_G_B = vi_map.getVertex_T_G_I(vertex_id);
      const aslam::Transformation T_G_C = T_G_B * T_C_B.inverse();

      const aslam::Position3D p_C_map = T_G_C.getPosition();

      cv::Mat im;
      //in case of thermal images (raw depth maps)
      if(vi_map.getFrameResource(
          vertex, thermal_cam_idx, backend::ResourceType::kRawDepthMap, &im)) {
          VLOG(5) << "Found raw depth map.";
      }

      //in case of thermal images (raw depth maps)
      else if(vi_map.getFrameResource(
          vertex, thermal_cam_idx, backend::ResourceType::kImageForDepthMap, &im)) {
          VLOG(5) << "Found raw image for depth map.";
      }

      //in case of no image resource
      else {
        LOG(ERROR) << "No images have been found.";
      }
      
      CHECK_EQ(im.depth(), CV_16U);
      CHECK_EQ(im.channels(), 1);

      for (size_t i = 0u; i < 4; i++) {
        const Eigen::Vector3d ray_rotated = T_G_C.getRotation().rotate(corner_ray[i]);
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
        const grid_map::Index index(*it);
        const double x = index(0);
        const double y = index(1);

        const Eigen::Vector3d position_transformed = T_G_C.inverse() * position;

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
}

}  // namespace grid_map_amo
