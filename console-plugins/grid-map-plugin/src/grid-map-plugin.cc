#include "grid-map-plugin/grid-map-plugin.h"

#include <iostream>  //NOLINT

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

#include <grid_map_core/grid_map_core.hpp>
#include <Eigen/Core>

#include <glog/logging.h>

#include <grid_map_ros/grid_map_ros.hpp>

DEFINE_double(grid_map_res, 20.0, "The resolution of the grid map in meters.");

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

  LOG(INFO) << all_landmark_ids.size();
  LOG(INFO) << all_landmarks.cols() << " " << all_landmarks.rows();
  size_t iter = 0;
  for (const vi_map::LandmarkId& landmark_id : all_landmark_ids) {
    const vi_map::Landmark& landmark = map->getLandmark(landmark_id);
    if (landmark.getQuality() != vi_map::Landmark::Quality::kGood) {
      continue;
    }

    // add landmark vector to all_landmarks matrix
    //LOG(INFO) << iter;
    
    all_landmarks.col(iter) = map->getLandmark_G_p_fi(landmark_id);//landmark.get_p_B(); // SEGMENTATION ERROR
    iter++;
    //test successful, problem is, landmark doesnt get added to all_landmarks: std::cout << landmark.get_p_B() << std::endl;
  }

  all_landmarks.resize(Eigen::NoChange, iter);
  
  //set the resolution to 20 meters by default
  /*map_resolution_ = FLAGS_grid_map_res;
  if (map_resolution_ <= 0.0) {
    LOG(ERROR) << "Please specify a positive value for the grid map resolution.";
    return common::kStupidUserError;
  }*/

  // initialize the grid map
  grid_map_.reset(new grid_map::GridMap({"elevation", "elevation_filled", "uncertainty", "temperature", "obs_angle_temp", "orthomosaic", "obs_angle_ortho"}));
  grid_map_->get("elevation").setConstant(NAN);
  grid_map_->get("elevation_filled").setConstant(NAN);
  grid_map_->get("temperature").setConstant(NAN);
  grid_map_->get("uncertainty").setConstant(NAN);
  grid_map::Position position(0.0, 0.0);
  grid_map::Length length(map_resolution_, map_resolution_);
  grid_map_->setGeometry(length, map_resolution_, position);
  //offset_map_ = nullptr;
  //landmarks_uncertainty;

  //set map offset to zero (or rather, make it point to zero)
  /*grid_map::Position3 off_val {
    {0},{0},{0}
  };
  offset_map_ = &off_val;*/

  /*
  if (!offset_map_) {
    // initialize the map offset to the first gps measurement
    GpsLlhMeasurement gps_oldest;
    gps_buffer_->getOldestValue(&gps_oldest);

    double northing, easting;
    char utm_zone[10];
    UTM::LLtoUTM(gps_oldest.gps_position_lat_lon_alt_deg_m(0), gps_oldest.gps_position_lat_lon_alt_deg_m(1),
        northing, easting, utm_zone);

    offset_map_.reset(new grid_map::Position3(easting, northing, gps_oldest.gps_position_lat_lon_alt_deg_m(2)));
  }
  */
  
  // resize the map to incorporate the new measurements
  const double east_max = all_landmarks.row(0).maxCoeff() + 0.5 * grid_map_->getResolution();
  std::cout << "east_max: " << east_max << std::endl;
  std::cout << "all_landmarks.row(0).maxCoeff(): " << all_landmarks.row(0).maxCoeff() << std::endl;
  LOG(INFO) << all_landmarks;
  const double east_min = all_landmarks.row(0).minCoeff() - 0.5 * grid_map_->getResolution();
  std::cout << "east_min: " << east_min << std::endl;
  const double north_max = all_landmarks.row(1).maxCoeff() + 0.5 * grid_map_->getResolution();
  std::cout << "north_max: " << north_max << std::endl;
  const double north_min = all_landmarks.row(1).minCoeff() - 0.5 * grid_map_->getResolution();
  std::cout << "north_min: " << north_min << std::endl;

  grid_map::GridMap map_tmp({"tmp"});
  position = grid_map::Position((east_max - east_min) * 0.5 + east_min /*- (*offset_map_)(0)*/,
                                (north_max - north_min) * 0.5 + north_min /*- (*offset_map_)(1)*/);
  length = grid_map::Length(std::ceil((east_max - east_min) / grid_map_->getResolution()) * grid_map_->getResolution(),
                            std::ceil((north_max - north_min) / grid_map_->getResolution()) * grid_map_->getResolution());
  map_tmp.setGeometry(length, grid_map_->getResolution(), position);

  grid_map_->extendToInclude(map_tmp);
  std::cout << "getRes: " << grid_map_->getResolution() << std::endl;
  std::cout << "Res: " << map_resolution_ << std::endl;

  // set landmarks_uncertainty to 1
  /*landmarks_uncertainty.resize(Eigen::NoChange, all_landmarks.cols());
  for(int i = 0; i < all_landmarks.cols(); i++) {
    landmarks_uncertainty.col(i) = 1;
    std::cout << i << std::endl;
  }*/

  std::cout << "pos: " << position << " len: " << length << std::endl;

  landmarks_uncertainty.resize(Eigen::NoChange, all_landmarks.cols());
  landmarks_uncertainty.setConstant(1);

  std::cout << "all_landmarks.cols(): " << all_landmarks.cols() << " all_landmarks.rows(): " << all_landmarks.rows() << std::endl; // just for testing something, never actually gets output. so i guess the error is between the all_landmarks initialization and this line

  // pass the empty grid map instead of the maplab map
  grid_map_amo::update_elevation_layer(grid_map_, all_landmarks, landmarks_uncertainty/*, offset_map_*/);

  std::cout << "Did we pass?" << std::endl;

  // publish the map
  grid_map_msgs::GridMap message;
  grid_map::GridMapRosConverter::toMessage(*grid_map_, message);
  message.info.header.stamp = ros::Time::now();
  message.info.header.frame_id = map_frame_id_;

  visualization::RVizVisualizationSink::publish(map_topic_, message);
  
  return common::kSuccess;
}

// Currently just a copy from another plug-in. Can be used as a template for the grid map.
/*
int GridMapPlugin::createGridMap() {
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }

  

  vi_map_helpers::evaluateLandmarkQuality(mission_ids_to_process, map.get());
  return common::kSuccess;
}
*/
}  // namespace grid_map_plugin

MAPLAB_CREATE_CONSOLE_PLUGIN(grid_map_plugin::GridMapPlugin);
