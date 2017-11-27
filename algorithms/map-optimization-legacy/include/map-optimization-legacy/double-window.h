#ifndef MAP_OPTIMIZATION_LEGACY_DOUBLE_WINDOW_H_
#define MAP_OPTIMIZATION_LEGACY_DOUBLE_WINDOW_H_

#include <memory>
#include <vector>

#include <Eigen/Core>
#include <vi-map/unique-id.h>

namespace map_optimization_legacy {

class DoubleWindow {
 public:
  typedef std::unique_ptr<DoubleWindow> Ptr;
  typedef std::vector<Ptr> PtrVector;

  DoubleWindow() : is_mission_to_optimize_set_(false) {}
  inline virtual ~DoubleWindow() {}

  inline const pose_graph::EdgeIdList& getInnerWindowEdges() const {
    return edges_inner_window_;
  }
  inline const pose_graph::EdgeIdList& getOuterWindowEdges() const {
    return edges_outer_window_;
  }
  inline const pose_graph::VertexIdSet& getInnerWindowVertices() const {
    return vertices_inner_window_;
  }
  inline const pose_graph::VertexIdSet& getOuterWindowVertices() const {
    return vertices_outer_window_;
  }
  /// Return vertices from both inner and outer window.
  void getAllWindowVertices(
      pose_graph::VertexIdSet* all_window_vertex_ids) const;
  inline const pose_graph::VertexIdList& getFixedVertices() const {
    return fixed_vertices_;
  }
  inline bool isMissionToOptimizeSet() const {
    return is_mission_to_optimize_set_;
  }
  inline const vi_map::MissionId& getMissionToOptimize() const {
    return mission_to_optimize_;
  }
  inline bool isLandmarkSeenFromInnerWindow(
      const vi_map::LandmarkId& landmark_id) const {
    return landmarks_seen_from_inner_window.count(landmark_id) > 0u;
  }
  bool isVertexInDoubleWindow(const pose_graph::VertexId& query_verex_id) const;

  inline const vi_map::LandmarkIdSet& getLandmarksSeenFromInnerWindow() const {
    return landmarks_seen_from_inner_window;
  }

  virtual double getSquaredDistanceToInnerWindow(
      const Eigen::Vector3d& p_G) const = 0;

 protected:
  pose_graph::VertexIdSet vertices_inner_window_;
  pose_graph::VertexIdList fixed_vertices_;
  pose_graph::VertexIdSet vertices_outer_window_;
  pose_graph::EdgeIdList edges_inner_window_;
  pose_graph::EdgeIdList edges_outer_window_;
  vi_map::LandmarkIdSet landmarks_seen_from_inner_window;
  vi_map::MissionId mission_to_optimize_;
  bool is_mission_to_optimize_set_;
};

}  // namespace map_optimization_legacy

#endif  // MAP_OPTIMIZATION_LEGACY_DOUBLE_WINDOW_H_
