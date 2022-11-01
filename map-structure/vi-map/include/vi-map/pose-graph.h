#ifndef VI_MAP_POSE_GRAPH_H_
#define VI_MAP_POSE_GRAPH_H_

#include <memory>
#include <vector>

#include <Eigen/Core>
#include <aslam/cameras/ncamera.h>
#include <aslam/common/hash-id.h>
#include <aslam/common/memory.h>
#include <aslam/frames/visual-frame.h>
#include <maplab-common/macros.h>
#include <posegraph/pose-graph.h>

#include "vi-map/landmark.h"
#include "vi-map/mission.h"
#include "vi-map/transformation-edge.h"
#include "vi-map/unique-id.h"
#include "vi-map/vi_map.pb.h"
#include "vi-map/viwls-edge.h"

namespace vi_map {

class PoseGraph : public pose_graph::PoseGraph {
 public:
  MAPLAB_POINTER_TYPEDEFS(PoseGraph);

  virtual ~PoseGraph() {}

  void addVIVertex(
      const pose_graph::VertexId& id,
      const Eigen::Matrix<double, 6, 1>& imu_ba_bw,
      const Eigen::Matrix<double, 2, Eigen::Dynamic>& img_points,
      const Eigen::VectorXd& uncertainties,
      const aslam::VisualFrame::DescriptorsT& descriptors,
      const std::vector<LandmarkId>& observed_landmarks,
      const vi_map::MissionId& mission_id, const aslam::FrameId& frame_id,
      int64_t frame_timestamp, const aslam::NCamera::Ptr cameras);

  void addVIEdge(
      const pose_graph::EdgeId& id, const pose_graph::VertexId& from,
      const pose_graph::VertexId& to,
      const Eigen::Matrix<int64_t, 1, Eigen::Dynamic>& imu_timestamps,
      const Eigen::Matrix<double, 6, Eigen::Dynamic>& imu_data);

  // merge_into_vertex_id
  //   |                            edge_after_next_vertex
  //   |    edge_between_vertices        |
  //   |         |                       |
  //   v         v                       v
  // VID_k <-- edge_i -- > VID_kp1 <-- edge_ip1 --> VID_kp2
  //
  //    ... is transformed into:
  //
  // VID_k <------------- new edge  --------------> VID_kp2
  template <typename EdgeType>
  void mergeNeighboringEdges(
      const pose_graph::VertexId& merge_into_vertex_id,
      const EdgeType& edge_between_vertices,
      const EdgeType& edge_after_next_vertex);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace vi_map

#endif  // VI_MAP_POSE_GRAPH_H_
