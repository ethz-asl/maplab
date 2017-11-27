#ifndef LOOP_CLOSURE_PLUGIN_VI_LOCALIZATION_EVALUATOR_H_
#define LOOP_CLOSURE_PLUGIN_VI_LOCALIZATION_EVALUATOR_H_

#include <console-common/command-registerer.h>
#include <vi-map/unique-id.h>

namespace vi_map {
class VIMap;
}  // namespace vi_map

namespace visualization {
class ViwlsGraphRvizPlotter;
}  // namespace visualization

namespace loop_closure_plugin {
class VILocalizationEvaluator {
 public:
  VILocalizationEvaluator() = delete;
  VILocalizationEvaluator(
      vi_map::VIMap* map, visualization::ViwlsGraphRvizPlotter* plotter);
  enum ConsistencyStatus {
    kInconsistent = common::kCustomStatusOffset,
    kNoData,
  };

  void alignMissionsForEvaluation(const vi_map::MissionId& query_mission_id);
  void evaluateLocalizationPerformance(
      const vi_map::MissionId& query_mission_id);

 private:
  vi_map::VIMap* map_;
  visualization::ViwlsGraphRvizPlotter* plotter_;
};

}  // namespace loop_closure_plugin

#endif  // LOOP_CLOSURE_PLUGIN_VI_LOCALIZATION_EVALUATOR_H_
