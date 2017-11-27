#ifndef MAP_OPTIMIZATION_VI_MAP_RELAXATION_H_
#define MAP_OPTIMIZATION_VI_MAP_RELAXATION_H_

#include <string>

#include <ceres/ceres.h>
#include <vi-map/unique-id.h>

namespace visualization {
class ViwlsGraphRvizPlotter;
}  // namespace visualization

namespace vi_map {
class VIMap;
}  // namespace vi_map

namespace map_optimization {
struct OptimizationOptions;

class VIMapRelaxation {
 public:
  VIMapRelaxation(
      visualization::ViwlsGraphRvizPlotter* plotter,
      bool signal_handler_enabled);

  bool relax(const vi_map::MissionIdList& mission_id_list, vi_map::VIMap* map);
  bool relax(
      const ceres::Solver::Options& solver_options,
      const vi_map::MissionIdList& mission_id_list, vi_map::VIMap* map);

 private:
  void detectLoopclosures(vi_map::MissionIdSet mission_ids, vi_map::VIMap* map);

  void visualizePosegraph(const vi_map::VIMap& map) const;
  int numLoopclosureEdges(const vi_map::VIMap& map) const;

  visualization::ViwlsGraphRvizPlotter* plotter_;
  bool signal_handler_enabled_;
};

}  // namespace map_optimization

#endif  // MAP_OPTIMIZATION_VI_MAP_RELAXATION_H_
