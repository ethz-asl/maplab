/*
 *    Filename: amo-combined.cc
 *  Created on: April 4, 2021
 *      Author: Florian Achermann (acfloria@ethz.ch)
 *              Timo Hinzmann (hitimo@ethz.ch)
 *    Modified: Luka Dragomirovic (lukavuk01@sunrise.ch)
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid-map-amo/amo-combined.h"

#include <glog/logging.h>

#include <iostream> // for debugging

#include <math.h>
#include <memory>
#include <Eigen/Core>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <vi-map/vi-map.h>

#include <grid_map_cv/grid_map_cv.hpp>

namespace grid_map_amo {

void update_whole_grid_map(std::unique_ptr<grid_map::GridMap>& map,
    const Eigen::Matrix<double, 3, Eigen::Dynamic>& landmarks,
    const Eigen::Matrix<double, 1, Eigen::Dynamic>& landmarks_uncertainty,
    std::string input_layer,
    std::string output_layer,
    double radius,
    std::string orthomosaic_layer,
    std::string observation_angle_layer,
    std::string elevation_layer,
    const vi_map::VIMap& vi_map,
    int optical_cam_idx) {

  //start of em
  CHECK_NOTNULL(map);
  CHECK_EQ(landmarks.cols(), landmarks_uncertainty.cols());

  for (size_t i = 0; i < landmarks.cols(); ++i) {
    const grid_map::Position position(landmarks.col(i)(0), landmarks.col(i)(1));
    grid_map::Index index;
    if (!map->getIndex(position, index)) {
      // the point lies outside the current map
      // TODO print warning
      //std::cout << "Point outside of map" << std::endl;
      continue;
    }

    if (!map->isValid(index)) {
      // directly insert the measurements in previously unseen cells
      map->at("elevation", index) = landmarks.col(i)(2);
      map->at("uncertainty", index) = landmarks_uncertainty.col(i)(0);
    } else {

      // ekf update
      auto height_prior = map->at("elevation", index);
      auto var_prior = map->at("uncertainty", index);
      const float var_meas = landmarks_uncertainty.col(i)(0);
      const float height_meas = landmarks.col(i)(2);
      map->at("elevation", index) =
          (height_prior * var_meas + height_meas * var_prior) /
          (var_meas + var_prior);
      map->at("uncertainty", index) =
          (var_meas * var_prior) / (var_meas + var_prior);
    }
  }
  //end of em
  //start of ef
  CHECK_NOTNULL(map);
  CHECK_GT(radius, 0.0);

  map->add("inpaint_mask", 0.0);

  for (grid_map::GridMapIterator iterator(*map); !iterator.isPastEnd(); ++iterator) {
    if (!map->isValid(*iterator, input_layer)) {
      map->at("inpaint_mask", *iterator) = 1.0;
    }
  }

  cv::Mat originalImage, mask, filledImage;
  const float minValue = map->get(input_layer).minCoeffOfFinites();
  const float maxValue = map->get(input_layer).maxCoeffOfFinites();

  grid_map::GridMapCvConverter::toImage<float, 1>(*map, input_layer, CV_32FC1, minValue, maxValue,
                                                          originalImage);
  grid_map::GridMapCvConverter::toImage<unsigned char, 1>(*map, "inpaint_mask", CV_8UC1, mask);

  const double radiusInPixels = radius / map->getResolution();
  cv::inpaint(originalImage, mask, filledImage, radiusInPixels, cv::INPAINT_NS);

  grid_map::GridMapCvConverter::addLayerFromImage<float, 1>(filledImage, output_layer, *map, minValue, maxValue);
  map->erase("inpaint_mask");
  //end of ef
  //start of orth
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

    int im_counter = 0;//debugging
    //LOG(INFO) << ortho_layer.rows() << " " << ortho_layer.cols();//debugging
    for(int r = 0;r < 825;r++) {
      for(int c = 0;c < 661;c++) {
        ortho_layer(r,c) = 0;
      }
    }
    //iterate over vertices instead of frames(as in amo)
    std::array<Eigen::Vector3d, 4> position_corners{};
    for (const pose_graph::VertexId& vertex_id : all_vertices_in_mission) {
      const vi_map::Vertex& vertex = vi_map.getVertex(vertex_id);
      const aslam::Transformation T_G_B = vi_map.getVertex_T_G_I(vertex_id);
      const aslam::Transformation T_G_C = T_G_B * T_C_B.inverse();

      const aslam::Position3D p_C_map = T_G_C/*.inverse()*/.getPosition();

      //LOG(INFO) << T_G_B;//debugging

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

      //in case of no image resource
      else {
        LOG(ERROR) << "No images have been found.";
      }

      LOG(INFO) << im;//debugging
      cv::imshow("1st image", im);//debugging
      cv::waitKey(0);//debugging
      //cv::imwrite("/saved_data/im1.bmp", im);//debugging
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

        //LOG(INFO) << "x: " << x << " y: " << y;//debugging

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
              ortho_layer(x, y) = im.at<uint8_t>(kp_y, kp_x) / 255.0;
              obs_angle_layer(x, y) = std::fabs(observation_angle);
            }
          }
        }
      } im_counter++; if(im_counter == 1){ break;}//only one image for debugging purposes
    }
  }
  //end of orth
}

}  // namespace grid_map_amo
