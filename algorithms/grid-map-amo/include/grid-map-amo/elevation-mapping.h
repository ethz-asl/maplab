/*
 *    Filename: elevation-mapping.h
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

namespace grid_map_amo {

void update_elevation_layer(std::unique_ptr<grid_map::GridMap>& map,
    const Eigen::Matrix<double, 3, Eigen::Dynamic>& landmarks,
    const Eigen::Matrix<double, 1, Eigen::Dynamic>& landmarks_uncertainty);

}  // namespace grid_map_amo
#endif  // AMO_LIB_ELEVATION_MAPPER_ELEVATION_MAPPING_H_
