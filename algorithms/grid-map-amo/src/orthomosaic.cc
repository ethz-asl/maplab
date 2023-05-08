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
#include <vi-map/vi-map.h>

namespace grid_map_amo {

void update_ortho_layer(std::unique_ptr<grid_map::GridMap>& map,
    std::string orthomosaic_layer,
    std::string observation_angle_layer,
    std::string elevation_layer,
    //const std::unique_ptr<grid_map::Position3>& map_offset,
    const aslam::VisualNFrame::PtrVector& nframes_in,
    //const swe::NFrameIdViNodeStateMap& vi_map,
    const vi_map::VIMap& vi_map,
    int optical_cam_idx) {
  CHECK_NOTNULL(map);
  //CHECK_NOTNULL(map_offset);
  CHECK_EQ(vi_map.numVertices(), nframes_in.size());//had to change the size call for vi_map, might still give an error
  CHECK(map->exists(orthomosaic_layer));
  CHECK(map->exists(observation_angle_layer));
  CHECK(map->exists(elevation_layer));
  CHECK_GT(nframes_in.size(), 0u);
  CHECK_LT(optical_cam_idx, nframes_in[0]->getNumCameras());

  grid_map::Matrix& ortho_layer = (*map)[orthomosaic_layer];
  grid_map::Matrix& obs_angle_layer = (*map)[observation_angle_layer];
  const aslam::Camera& cam = nframes_in[0]->getCamera(optical_cam_idx);
  const aslam::Transformation T_C_B = nframes_in[0]->get_T_C_B(optical_cam_idx);

  const double min_height = (*map)[elevation_layer].minCoeff();
  CHECK(!std::isnan(min_height));

  std::array<Eigen::Vector3d, 4> corner_ray{};
  cam.backProject3(Eigen::Vector2d(0, 0), &corner_ray[0]);
  cam.backProject3(Eigen::Vector2d(0, cam.imageHeight() - 1), &corner_ray[1]);
  cam.backProject3(Eigen::Vector2d(cam.imageWidth() - 1, cam.imageHeight() - 1), &corner_ray[2]);
  cam.backProject3(Eigen::Vector2d(cam.imageWidth() - 1, 0), &corner_ray[3]);

  std::array<Eigen::Vector3d, 4> position_corners{};
  for (size_t frame_idx = 0u; frame_idx < nframes_in.size(); ++frame_idx) {
    const aslam::VisualFrame& frame = nframes_in[frame_idx]->getFrame(optical_cam_idx);
    CHECK(frame.hasRawImage());

    const cv::Mat& im_color = frame.getRawImage();
    cv::Mat im;
    if (im_color.channels() == 3) {
      cv::cvtColor(im_color, im, CV_BGR2GRAY);
    } else {
      im = im_color.clone();
    }
    CHECK_EQ(im.depth(), CV_8U);
    CHECK_EQ(im.channels(), 1);
    pose_graph::VertexId vert_idx;
    pose_graph::VertexIdList all_vertex_ids;
    vi_map.getAllVertexIds(&all_vertex_ids);
    for (const pose_graph::VertexId& vertex_id : all_vertex_ids) {
      const vi_map::Vertex& vertex_sample = vi_map.getVertex(vertex_id);
      if(vertex_sample.getVisualNFrame() == *nframes_in[frame_idx]) {
        vert_idx = vertex_id;
        break;
      }
    }

    const aslam::Transformation T_C_W = T_C_B * (vi_map.getVertex(vert_idx)).get_T_M_I().inverse();//also had to change vi_map call, might as well give an error still
    const aslam::Transformation T_W_C = T_C_W.inverse();
    const aslam::Position3D p_C_map = T_C_W.inverse().getPosition();// - *map_offset;
    //simplify for maplab
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
      map->getPosition3(elevation_layer, *it, position);//use infill layer
      //position += *map_offset;
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
            ortho_layer(x, y) = im.at<uint8_t>(kp_y, kp_x) / 255.0;
            obs_angle_layer(x, y) = std::fabs(observation_angle);
          }
        }
      }
    }
  }
}

}  // namespace grid_map_amo
