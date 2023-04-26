#ifndef GRID_MAP_PLUGIN_GRID_MAP_PLUGIN_H_
#define GRID_MAP_PLUGIN_GRID_MAP_PLUGIN_H_

#include <string>

#include <console-common/console-plugin-base.h>
#include <console-common/console.h>

#include <vi-map/vi-map.h>

#include <grid_map_core/grid_map_core.hpp>
#include <Eigen/Core>

//#include <grid-map-amo/elevation-mapping.h>

namespace grid_map_plugin {

class GridMapPlugin : public common::ConsolePluginBase {
 public:
  explicit GridMapPlugin(common::Console* console);

  virtual std::string getPluginId() const override {
    return "grid_map_amo";
  }

 private:
  int createElevationMapping();

  double map_resolution_=20; // for now
  std::unique_ptr<grid_map::GridMap> grid_map_;
  //std::unique_ptr<grid_map::Position3> offset_map_;
  Eigen::Matrix<double, 1, Eigen::Dynamic> landmarks_uncertainty;

  std::string map_topic_ = "grid_map";
  std::string map_frame_id_ = "map";
};

}  // namespace grid_map_plugin

#endif  // GRID_MAP_PLUGIN_GRID_MAP_PLUGIN_H_
