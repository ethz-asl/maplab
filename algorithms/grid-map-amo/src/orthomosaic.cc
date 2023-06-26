/*
 *    Filename: orthomosaic.cc
 *  Created on: May 19, 2021
 *      Author: Florian Achermann (acfloria@ethz.ch)
 *    Modified: Luka Dragomirovic (lukavuk01@sunrise.ch)
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid-map-amo/orthomosaic.h"

#include <math.h>
#include <memory>
#include <Eigen/Core>
#include <glog/logging.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <vi-map/vi-map.h>

namespace grid_map_amo {

void update_ortho_layer(std::unique_ptr<grid_map::GridMap>& map,
    std::string orthomosaic_layer,
    std::string observation_angle_layer,
    std::string elevation_layer,
    const vi_map::VIMap& vi_map,
    int optical_cam_idx) {

  grid_map::Matrix& ortho_layer = (*map)[orthomosaic_layer];
  grid_map::Matrix& obs_angle_layer = (*map)[observation_angle_layer];

  //change to the logic
  vi_map::MissionIdList all_missions;
  vi_map.getAllMissionIds(&all_missions);
  for (const vi_map::MissionId& mission_id : all_missions) {
    pose_graph::VertexIdList all_vertices_in_mission;
    vi_map.getAllVertexIdsInMissionAlongGraph(
        mission_id, &all_vertices_in_mission);

    const aslam::NCamera& n_camera = vi_map.getMissionNCamera(mission_id);
    const aslam::Camera& cam = n_camera.getCamera(optical_cam_idx);
    const aslam::Transformation T_C_B = n_camera.get_T_C_B(optical_cam_idx);

    const double min_height = (*map)[elevation_layer].minCoeff();
    CHECK(!std::isnan(min_height));

    std::array<Eigen::Vector3d, 4> corner_ray{};
    cam.backProject3(Eigen::Vector2d(0, 0), &corner_ray[0]);
    cam.backProject3(Eigen::Vector2d(0, cam.imageHeight() - 1), &corner_ray[1]);
    cam.backProject3(Eigen::Vector2d(cam.imageWidth() - 1, cam.imageHeight() - 1), &corner_ray[2]);
    cam.backProject3(Eigen::Vector2d(cam.imageWidth() - 1, 0), &corner_ray[3]);

    for(int r = 0;r < ortho_layer.rows();r++) {
      for(int c = 0;c < ortho_layer.cols();c++) {
        ortho_layer(r,c) = 0;
      }
    }
    //iterate over vertices instead of frames(as in amo)
    std::array<Eigen::Vector3d, 4> position_corners{};
    for (const pose_graph::VertexId& vertex_id : all_vertices_in_mission) {
      const vi_map::Vertex& vertex = vi_map.getVertex(vertex_id);
      const aslam::Transformation T_G_B = vi_map.getVertex_T_G_I(vertex_id);
      const aslam::Transformation T_G_C = T_G_B * T_C_B.inverse();

      const aslam::Position3D p_C_map = T_G_C.getPosition();

      //check if thermal or optical
      bool thermCam = false;
      //in case of grayscale images
      cv::Mat im;
      cv::Mat color_image;
      if(vi_map.getFrameResource(
          vertex, optical_cam_idx, backend::ResourceType::kRawImage, &im)) {
          VLOG(5) << "Found raw grayscale image.";
      }

      //in case of color images
      else if(vi_map.getFrameResource(
          vertex, optical_cam_idx, backend::ResourceType::kRawColorImage,
          &color_image)) {
          VLOG(5) << "Found raw color image.";
          cv::Mat im;
          cv::cvtColor(color_image, im, cv::COLOR_RGB2GRAY);
      }

      //in case of thermal images (raw depth maps)
      else if(vi_map.getFrameResource(
          vertex, optical_cam_idx, backend::ResourceType::kRawDepthMap, &im)) {
          VLOG(5) << "Found raw depth map.";
          thermCam = true;
      }

      //in case of thermal images (raw depth maps)
      else if(vi_map.getFrameResource(
          vertex, optical_cam_idx, backend::ResourceType::kImageForDepthMap, &im)) {
          VLOG(5) << "Found raw image for depth map.";
          thermCam = true;
      }

      //in case of no image resource
      else {
        LOG(ERROR) << "No images have been found.";
      }

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
        map->getPosition3(elevation_layer, *it, position);//make sure to pass infill layer
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

              if(thermCam) {
                //conversion from 16bit to temperature (°C)
                ortho_layer(x, y) = im.at<uint16_t>(kp_y, kp_x) * 0.04 - 273.15;
              }
              else {
                ortho_layer(x, y) = im.at<uint8_t>(kp_y, kp_x) / 255.0;
              }
              
              obs_angle_layer(x, y) = std::fabs(observation_angle);
            }
          }
        }
      }
    }
  }
}

}  // namespace grid_map_amo
