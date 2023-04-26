/*
 *    Filename: elevation-mapping.cc
 *  Created on: April 4, 2021
 *      Author: Florian Achermann (acfloria@ethz.ch)
 *              Timo Hinzmann (hitimo@ethz.ch)
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid-map-amo/elevation-mapping.h"

#include <glog/logging.h>

#include <iostream> // for debugging

namespace grid_map_amo {

void update_elevation_layer(std::unique_ptr<grid_map::GridMap>& map,
    const Eigen::Matrix<double, 3, Eigen::Dynamic>& landmarks,
    const Eigen::Matrix<double, 1, Eigen::Dynamic>& landmarks_uncertainty/*,
    const std::unique_ptr<grid_map::Position3>& map_offset*/) {

  std::cout << "We are in" << std::endl;

  CHECK_NOTNULL(map);
  CHECK_EQ(landmarks.cols(), landmarks_uncertainty.cols());

  std::cout << "All equal" << std::endl;

  for (size_t i = 0; i < landmarks.cols(); ++i) {
    const grid_map::Position position(landmarks.col(i)(0) /*- (*map_offset)(0)*/, landmarks.col(i)(1) /*- (*map_offset)(1)*/);
    grid_map::Index index;
    if (!map->getIndex(position, index)) {
      // the point lies outside the current map
      // TODO print warning
      //std::cout << "Point outside of map" << std::endl;
      continue;
    }

    if (!map->isValid(index)) {
      // directly insert the measurements in previously unseen cells
      map->at("elevation", index) = landmarks.col(i)(2) /*- (*map_offset)(2)*/;
      map->at("uncertainty", index) = landmarks_uncertainty.col(i)(0);

      //std::cout << "Second if" << std::endl;

    } else {

      std::cout << "We are in else" << std::endl;

      // ekf update
      auto height_prior = map->at("elevation", index);
      auto var_prior = map->at("uncertainty", index);
      const float var_meas = landmarks_uncertainty.col(i)(0);
      const float height_meas = landmarks.col(i)(2) /*- (*map_offset)(2)*/;
      map->at("elevation", index) =
          (height_prior * var_meas + height_meas * var_prior) /
          (var_meas + var_prior);
      map->at("uncertainty", index) =
          (var_meas * var_prior) / (var_meas + var_prior);

      std::cout << "End of else" << std::endl;
    }
  }
  std::cout << "Outta the loop" << std::endl;
}

}  // namespace grid_map_amo
