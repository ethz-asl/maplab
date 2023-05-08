/*
 *    Filename: orhomosaic.h
 *  Created on: May 19, 2021
 *      Author: Florian Achermann (acfloria@ethz.ch)
 *    Modified: Luka Dragomirovic (lukavuk01@sunrise.ch)
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#ifndef AMO_LIB_GRID_MAP_ORTHOMOSAIC_H_
#define AMO_LIB_GRID_MAP_ORTHOMOSAIC_H_

#include <memory>
#include <string>
#include <vi-map/vi-map.h>
#include <aslam/frames/visual-nframe.h>
#include <grid_map_core/grid_map_core.hpp>

namespace amo {

void update_ortho_layer(std::unique_ptr<grid_map::GridMap>& map,
    std::string ortho_layer,
    std::string observation_angle_layer,
    std::string elevation_layer,
    //const std::unique_ptr<grid_map::Position3>& map_offset,
    const aslam::VisualNFrame::PtrVector& nframes_in,
    //const swe::NFrameIdViNodeStateMap& vi_map,
    const vi_map::VIMap& vi_map,
    int thermal_cam_idx);

}  // namespace grid_map_amo
#endif  // AMO_LIB_GRID_MAP_ORTHOMOSAIC_H_

