#ifndef VI_MAP_MISSION_H_
#define VI_MAP_MISSION_H_

#include <memory>
#include <unordered_set>

#include <Eigen/Core>
#include <aslam/common/memory.h>
#include <maplab-common/macros.h>
#include <maplab-common/pose_types.h>
#include <posegraph/unique-id.h>

#include "vi-map/mission-baseframe.h"
#include "vi-map/unique-id.h"

namespace vi_map {
class Mission {
 public:
  enum class BackBone { kViwls = 0, kOdometry = 1 };

  Mission() : backbone_type_(BackBone::kViwls) {}

  Mission(
      const MissionId& mission_id,
      const MissionBaseFrameId& mission_base_frame_id, BackBone back_bone_type)
      : mission_id_(mission_id),
        base_frame_id_(mission_base_frame_id),
        backbone_type_(back_bone_type) {
    CHECK(mission_id.isValid());
    CHECK(mission_base_frame_id.isValid());
  }

  bool operator==(const Mission& lhs) const {
    bool is_same = true;
    is_same &= mission_id_ == lhs.mission_id_;
    is_same &= root_vertex_id_ == lhs.root_vertex_id_;
    is_same &= base_frame_id_ == lhs.base_frame_id_;
    is_same &= backbone_type_ == lhs.backbone_type_;
    return is_same;
  }

  virtual ~Mission() {}
  void setId(const MissionId& id);
  const MissionId& id() const;

  void setBaseFrameId(const MissionBaseFrameId& base_frame_id);
  const MissionBaseFrameId& getBaseFrameId() const;

  void setRootVertexId(const pose_graph::VertexId& vertex_id);
  const pose_graph::VertexId& getRootVertexId() const;

  BackBone backboneType() const;
  void setBackboneType(BackBone backbone_type);

  MAPLAB_POINTER_TYPEDEFS(Mission);
  MAPLAB_GET_AS_CASTER

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 protected:
  MissionId mission_id_;
  pose_graph::VertexId root_vertex_id_;
  MissionBaseFrameId base_frame_id_;
  BackBone backbone_type_;
};

typedef std::unordered_map<MissionId, Mission::UniquePtr> MissionMap;

}  // namespace vi_map

#endif  // VI_MAP_MISSION_H_
