#ifndef LOOP_CLOSURE_PLUGIN_VI_MAP_MERGER_H_
#define LOOP_CLOSURE_PLUGIN_VI_MAP_MERGER_H_

#include <string>

#include <console-common/command-registerer.h>
#include <vi-map/unique-id.h>

namespace vi_map {
class VIMap;
}  // namespace vi_map

namespace visualization {
class ViwlsGraphRvizPlotter;
}  // namespace visualization

namespace loop_closure_plugin {
class VIMapMerger {
 public:
  VIMapMerger() = delete;
  VIMapMerger(
      vi_map::VIMap* map, visualization::ViwlsGraphRvizPlotter* plotter);
  enum ConsistencyStatus {
    kInconsistent = common::kCustomStatusOffset,
    kNoData,
  };

  int findLoopClosuresBetweenAllMissions();
  int findLoopClosuresBetweenMissions(const vi_map::MissionIdList& mission_ids);

 private:
  vi_map::VIMap* map_;
  visualization::ViwlsGraphRvizPlotter* plotter_;
};
}  // namespace loop_closure_plugin

#endif  // LOOP_CLOSURE_PLUGIN_VI_MAP_MERGER_H_
