/*
 *    Filename: amo-combined.h
 *  Created on: April 4, 2021
 *      Author: Florian Achermann (acfloria@ethz.ch)
 *              Timo Hinzmann (hitimo@ethz.ch)
 *    Modified: Luka Dragomirovic (lukavuk01@sunrise.ch)
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#ifndef AMO_LIB_ELEVATION_MAPPER_ELEVATION_MAPPING_H_
#define AMO_LIB_ELEVATION_MAPPER_ELEVATION_MAPPING_H_

#include <grid_map_core/grid_map_core.hpp>
#include <Eigen/Core>

#include <memory>
#include <string>
#include <vi-map/vi-map.h>
#include <aslam/frames/visual-nframe.h>

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
    int optical_cam_idx);

}  // namespace grid_map_amo
#endif  // AMO_LIB_ELEVATION_MAPPER_ELEVATION_MAPPING_H_
