/*
 *    Filename: grid-map-filtering.h
 *  Created on: April 12, 2021
 *      Author: Florian Achermann (acfloria@ethz.ch)
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#ifndef AMO_LIB_ELEVATION_MAPPER_GRID_MAP_FILTERING_H_
#define AMO_LIB_ELEVATION_MAPPER_GRID_MAP_FILTERING_H_

#include <memory>
#include <string>

#include <grid_map_core/grid_map_core.hpp>

namespace grid_map_amo {

// adaptation of the grid map InpaintFilter for this framework
void inpaint_layer(std::unique_ptr<grid_map::GridMap>& map,
    std::string input_layer,
    std::string output_layer,
    double radius);

}  // namespace grid_map_amo
#endif  // AMO_LIB_ELEVATION_MAPPER_GRID_MAP_FILTERING_H_
