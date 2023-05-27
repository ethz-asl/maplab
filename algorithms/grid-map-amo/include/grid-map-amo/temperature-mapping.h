/*
 *    Filename: temperature_mapping.h
 *  Created on: April 27, 2021
 *      Author: Florian Achermann (acfloria@ethz.ch)
 *    Modified: Luka Dragomirovic (lukavuk01@sunrise.ch)
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#ifndef AMO_LIB_GRID_MAP_TEMPERATURE_MAPPING_H_
#define AMO_LIB_GRID_MAP_TEMPERATURE_MAPPING_H_

#include <memory>
#include <string>
#include <vi-map/vi-map.h>
#include <aslam/frames/visual-nframe.h>
#include <grid_map_core/grid_map_core.hpp>



namespace grid_map_amo {

void update_temperature_layer(std::unique_ptr<grid_map::GridMap>& map,
    std::string temperature_layer,
    std::string observation_angle_layer,
    std::string elevation_layer,
    const vi_map::VIMap& vi_map,
    int thermal_cam_idx);

void update_temperature_layer_projection(std::unique_ptr<grid_map::GridMap>& map,
    std::string temperature_layer,
    std::string observation_angle_layer,
    std::string elevation_layer,
    const vi_map::VIMap& vi_map,
    int thermal_cam_idx);


}  // namespace grid_map_amo
#endif  // AMO_LIB_GRID_MAP_TEMPERATURE_MAPPING_H_
