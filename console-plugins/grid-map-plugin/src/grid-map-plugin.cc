#include "grid-map-plugin/grid-map-plugin.h"

#include <iostream>  //NOLINT
#include <algorithm>
#include <limits>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <utility>

// Yet again, not sure about these.
#include <aslam/common/statistics/statistics.h>
#include <aslam/common/timer.h>
#include <console-common/console.h>

#include <vi-map/vi-map.h>
#include <map-manager/map-manager.h>

#include <string>
#include <vector>

#include <Eigen/Core>
#include <aslam/common/feature-descriptor-ref.h>
#include <loopclosure-common/types.h>
#include <vi-map/vi-map.h>
#include <vocabulary-tree/distance.h>
#include <visualization/rviz-visualization-sink.h>

#include <vi-map/landmark.h>

#include <grid-map-amo/elevation-mapping.h>
#include <grid-map-amo/grid-map-filtering.h>
#include <grid-map-amo/orthomosaic.h>

#include <grid_map_core/grid_map_core.hpp>
#include <Eigen/Core>

#include <glog/logging.h>

#include <grid_map_ros/grid_map_ros.hpp>

#include <aslam/common/memory.h>
#include <aslam/common/pose-types.h>
#include <eigen-checks/glog.h>
#include <map-resources/resource-common.h>
#include <map-resources/resource-map.h>
#include <map-resources/temporal-resource-id-buffer.h>

#include "vi-map/vertex.h"
#include "vi-map/vi-map.h"
#include "vi-map/vi_map.pb.h"

#include <vi-mapping-test-app/vi-mapping-test-app.h>

DEFINE_double(grid_map_res, 20.0, "The resolution of the grid map in meters.");

DEFINE_double(inp_rad, 20.0, "The radius of the grid maps elevation fill in meters.");

namespace grid_map_plugin {

GridMapPlugin::GridMapPlugin(common::Console* console)
    : common::ConsolePluginBase(console) {
  addCommand(
      {"grid_map_test", "gmt"},
      []() -> int {
        std::cout << "Test successful!" << std::endl;
        return common::kSuccess;
      },
      "Print test.", common::Processing::Sync);
  /*
  addCommand(
      {"grid_map", "gm"},
      [this]() -> int { return createGridMap(); },
      "Create grid map.", common::Processing::Sync);
  */
  addCommand(
      {"elevation_mapping", "em"},
      [this]() -> int { return createElevationMapping(); },
      "Create elevation mapping needed for grid map. Use --grid_map_res to set the grid maps resolution.",
      common::Processing::Sync);

  addCommand(
      {"elevation_fill", "ef"},
      [this]() -> int { return createElevationFill(); },
      "Fill the created elevation mapping needed for grid map. Use --inp_rad to set the fill radius.",
      common::Processing::Sync);

  addCommand(
      {"orthomosaic", "orth"},
      [this]() -> int { return createOrthomosaic(); },
      "Create orthomosaic of the filled elevation layer.",
      common::Processing::Sync);
}

int GridMapPlugin::createElevationMapping() {
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }

  // get map
  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(selected_map_key);

  // first get landmark ids to then get landmarks
  vi_map::LandmarkIdSet all_landmark_ids;
  map->getAllLandmarkIds(&all_landmark_ids);
  Eigen::Matrix<double, 3, Eigen::Dynamic> all_landmarks;
  all_landmarks.resize(Eigen::NoChange, all_landmark_ids.size());

  size_t iter = 0;
  for (const vi_map::LandmarkId& landmark_id : all_landmark_ids) {
    const vi_map::Landmark& landmark = map->getLandmark(landmark_id);
    if (landmark.getQuality() != vi_map::Landmark::Quality::kGood) {
      continue;
    }
    // add landmark vector to all_landmarks matrix
    all_landmarks.col(iter) = map->getLandmark_G_p_fi(landmark_id);
    iter++;
  }

  all_landmarks.conservativeResize(Eigen::NoChange, iter);
  
  //set the resolution to 20 meters by default
  map_resolution_ = FLAGS_grid_map_res;
  if (map_resolution_ <= 0.0) {
    LOG(ERROR) << "Please specify a positive value for the grid map resolution.";
    return common::kStupidUserError;
  }

  // initialize the grid map
  grid_map_.reset(new grid_map::GridMap({"elevation", "elevation_filled", "uncertainty", "temperature", "obs_angle_temp", "orthomosaic", "obs_angle_ortho"}));
  grid_map_->get("elevation").setConstant(NAN);
  grid_map_->get("elevation_filled").setConstant(NAN);
  grid_map_->get("temperature").setConstant(NAN);
  grid_map_->get("uncertainty").setConstant(NAN);
  grid_map::Position position(0.0, 0.0);
  grid_map::Length length(map_resolution_, map_resolution_);
  grid_map_->setGeometry(length, map_resolution_, position);
  
  // resize the map to incorporate the new measurements
  const double east_max = all_landmarks.row(0).maxCoeff() + 0.5 * map_resolution_;
  const double east_min = all_landmarks.row(0).minCoeff() - 0.5 * map_resolution_;
  const double north_max = all_landmarks.row(1).maxCoeff() + 0.5 * map_resolution_;
  const double north_min = all_landmarks.row(1).minCoeff() - 0.5 * map_resolution_;

  grid_map::GridMap map_tmp({"tmp"});
  position = grid_map::Position((east_max - east_min) * 0.5 + east_min,
                                (north_max - north_min) * 0.5 + north_min);
  length = grid_map::Length(std::ceil((east_max - east_min) / map_resolution_) * map_resolution_,
                            std::ceil((north_max - north_min) / map_resolution_) * map_resolution_);

  map_tmp.setGeometry(length, map_resolution_, position);

  grid_map_->extendToInclude(map_tmp);

  //set uncertainty to 1 everywhere
  landmarks_uncertainty.resize(Eigen::NoChange, all_landmarks.cols());
  landmarks_uncertainty.setConstant(1);

  // pass the empty grid map instead of the maplab map
  grid_map_amo::update_elevation_layer(grid_map_, all_landmarks, landmarks_uncertainty);

  // publish the map
  grid_map_msgs::GridMap message;
  grid_map::GridMapRosConverter::toMessage(*grid_map_, message);
  message.info.header.stamp = ros::Time::now();
  message.info.header.frame_id = map_frame_id_;

  visualization::RVizVisualizationSink::publish(map_topic_, message);
  
  return common::kSuccess;
}

int GridMapPlugin::createElevationFill() {

  inpaint_radius_ = FLAGS_inp_rad;
  if (inpaint_radius_ <= 0.0) {
    LOG(ERROR) << "Please specify a positive value for the inpaint radius.";
    return common::kStupidUserError;
  }

  grid_map_amo::inpaint_layer(grid_map_, "elevation", "elevation_filled", inpaint_radius_);

  // publish the map
  grid_map_msgs::GridMap message;
  grid_map::GridMapRosConverter::toMessage(*grid_map_, message);
  message.info.header.stamp = ros::Time::now();
  message.info.header.frame_id = map_frame_id_;

  visualization::RVizVisualizationSink::publish(map_topic_, message);

  return common::kSuccess;

}

int GridMapPlugin::createOrthomosaic() {

  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }

  // get map
  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(selected_map_key);

  vi_map::VIMap* vi_map = map.get();

  int optical_camera_index_ = 0; //for now, as in amo worker

  grid_map_amo::update_ortho_layer(grid_map_, "orthomosaic", "obs_angle_ortho", "elevation_filled", *vi_map, optical_camera_index_);

  // publish the map
  grid_map_msgs::GridMap message;
  grid_map::GridMapRosConverter::toMessage(*grid_map_, message);
  message.info.header.stamp = ros::Time::now();
  message.info.header.frame_id = map_frame_id_;

  visualization::RVizVisualizationSink::publish(map_topic_, message);

  return common::kSuccess;

}

}  // namespace grid_map_plugin

MAPLAB_CREATE_CONSOLE_PLUGIN(grid_map_plugin::GridMapPlugin);
