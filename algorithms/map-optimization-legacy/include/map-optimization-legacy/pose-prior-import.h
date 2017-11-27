#ifndef MAP_OPTIMIZATION_LEGACY_POSE_PRIOR_IMPORT_H_
#define MAP_OPTIMIZATION_LEGACY_POSE_PRIOR_IMPORT_H_
#include <string>

#include <vi-map/vi-map.h>

namespace map_optimization_legacy {

// Import pose priors from a CSV file by matching closest timestamps to poses.
// This will set the positions of the vertices to these values in the MISSION
// frame.
// CSV input: time [s], x [m], y [m], z [m], q_w, q_x, q_y, q_z [active]
// No header line.
class PosePriorImport {
 public:
  void importPosesFromCSV(
      const std::string& import_filename, const vi_map::MissionId& mission_id,
      vi_map::VIMap* map);

  void runBAWithPosePriors(vi_map::VIMap* map) const;

 private:
  pose_graph::VertexIdSet vertices_with_priors_;
};

}  // namespace map_optimization_legacy

#endif  // MAP_OPTIMIZATION_LEGACY_POSE_PRIOR_IMPORT_H_
