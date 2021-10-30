#ifndef VI_MAP_STRUCTURE_LOOPCLOSURE_EDGE_H_
#define VI_MAP_STRUCTURE_LOOPCLOSURE_EDGE_H_

#include <string>

#include <maplab-common/pose_types.h>
#include <maplab-common/traits.h>
#include <vector>

#include "vi-map/edge.h"
#include "vi-map/landmark.h"
#include "vi-map/unique-id.h"
#include "vi-map/vi-mission.h"
#include "vi-map/vi_map.pb.h"

namespace vi_map {

struct VertexLandmarkObservation {
  pose_graph::VertexId corresponding_vertex_id;
  unsigned int frame_idx;
  unsigned int keypoint_idx;
  vi_map::LandmarkId landmark_id;

  bool operator==(const VertexLandmarkObservation& other) const {
    bool is_same = true;
    is_same &= corresponding_vertex_id == other.corresponding_vertex_id;
    is_same &= frame_idx == other.frame_idx;
    is_same &= keypoint_idx == other.keypoint_idx;
    is_same &= landmark_id == other.landmark_id;
    return is_same;
  }

  VertexLandmarkObservation(
      const pose_graph::VertexId& _corresponding_vertex_id,
      const unsigned int& _frame_idx,     // NOLINT
      const unsigned int& _keypoint_idx,  // NOLINT
      const vi_map::LandmarkId& _landmark_id)
      : corresponding_vertex_id(_corresponding_vertex_id),
        frame_idx(_frame_idx),
        keypoint_idx(_keypoint_idx),
        landmark_id(_landmark_id) {}

  VertexLandmarkObservation() : frame_idx(-1), keypoint_idx(-1) {}
};

struct LandmarkObservationGroup {
  std::vector<VertexLandmarkObservation> keypoint_vertex_observation_list;
  bool is_outlier;
  bool operator==(const LandmarkObservationGroup& other) const {
    bool is_same = true;
    is_same &= keypoint_vertex_observation_list ==
               other.keypoint_vertex_observation_list;
    is_same &= is_outlier == other.is_outlier;
    return is_same;
  }
  LandmarkObservationGroup() : is_outlier(false) {}
};

class StructureLoopclosureEdge : public vi_map::Edge {
 public:
  MAPLAB_POINTER_TYPEDEFS(StructureLoopclosureEdge);
  StructureLoopclosureEdge();
  StructureLoopclosureEdge(const StructureLoopclosureEdge&) = default;

  StructureLoopclosureEdge(
      const pose_graph::EdgeId& id, const pose_graph::VertexId& from,
      const pose_graph::VertexId& to, const double switch_variable);
  virtual ~StructureLoopclosureEdge() {}

  inline void setSwitchVariable(double switch_variable) {
    switch_variable_ = switch_variable;
  }

  virtual bool operator==(const StructureLoopclosureEdge& other) const {
    bool is_same = true;
    is_same &= static_cast<const vi_map::Edge&>(*this) == other;
    is_same &= id_ == other.id_;
    is_same &= from_ == other.from_;
    is_same &= to_ == other.to_;
    is_same &= switch_variable_ == other.switch_variable_;
    is_same &= landmark_observations_ == other.landmark_observations_;
    return is_same;
  }

  inline double getSwitchVariable() const {
    return switch_variable_;
  }
  inline double* getSwitchVariableMutable() {
    return &switch_variable_;
  }
  inline vi_map::LandmarkObservationGroup* getLandmarkObservationsMutable() {
    return &landmark_observations_;
  }
  inline const vi_map::LandmarkObservationGroup& getLandmarkObservations()
      const {
    return landmark_observations_;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  StructureLoopclosureEdge& operator=(const StructureLoopclosureEdge&) = delete;

  double switch_variable_;
  vi_map::LandmarkObservationGroup landmark_observations_;
};

}  // namespace vi_map

#endif  // VI_MAP_STRUCTURE_LOOPCLOSURE_EDGE_H_
