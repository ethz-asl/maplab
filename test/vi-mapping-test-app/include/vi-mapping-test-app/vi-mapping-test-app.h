#ifndef VI_MAPPING_TEST_APP_VI_MAPPING_TEST_APP_H_
#define VI_MAPPING_TEST_APP_VI_MAPPING_TEST_APP_H_

#include <memory>
#include <string>

#include <Eigen/Core>
#include <aslam/common/memory.h>
#include <maplab-common/pose_types.h>
#include <vi-map/edge.h>
#include <vi-map/vertex.h>
#include <vi-map/vi-map.h>

namespace visual_inertial_mapping {

class VIMappingTestApp {
 public:
  typedef AlignedUnorderedMap<pose_graph::VertexId, pose::Transformation>
      VertexIdPosePairs;
  typedef AlignedUnorderedMap<vi_map::LandmarkId, Eigen::Vector3d>
      LandmarkIdPositionPairs;

  VIMappingTestApp();
  virtual ~VIMappingTestApp();

  void loadDataset(const std::string& folder_name);
  vi_map::VIMap* getMapMutable();
  bool isMapConsistent();

  void sparsifyMission();
  size_t numVerticesOnMap() const;

  void corruptLandmarkPositions(double std_dev_m, int every_nth);
  void corruptKeyframePoses(
      double position_std_dev_m, double orientation_std_dev_quat,
      int every_nth);
  void corruptAbs6DoFSensorExtrinsics(
      double position_std_dev_m, double orientation_std_dev_quat);
  void corruptCameraExtrinsics(
      double position_std_dev_m, double orientation_std_dev_quat);

  // This function adds loop closure edges that correspond exactly to the
  // current state of the map between pairs of vertices. The pairs are selected
  // by sorting all vertices based on their id and then building pairs based on
  // stepping through the list with the step size provided to the function.
  void addLoopClosureEdges(const size_t add_lc_edge_between_every_nth_vertex);

  void addCorruptDuplicateLandmarkObservations(int every_nth);

  pose_graph::EdgeId addWrongLoopClosureEdge();

  // This function adds absolute 6DoF constraints that correspond exactly to the
  // current state of the map.
  void addAbsolute6DoFConstraints(
      const size_t add_constraint_at_every_nth_vertex);

  const Eigen::Vector3d& getLandmarkReferencePosition(
      const vi_map::LandmarkId& landmark_id) const;
  const pose::Transformation& getVertexReferencePose(
      const pose_graph::VertexId& vertex_id) const;

  void testIfKeyframesMatchReference(double precision_m) const;
  void testIfLandmarksMatchReference(
      double max_distance, double min_required_fraction) const;
  void testIfSwitchVariablesLargerThan(
      double min_value, double min_required_fraction) const;
  double getSpecificSwitchVariable(const pose_graph::EdgeId& edge_id) const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  const int random_seed_;
  std::string map_key_;

  VertexIdPosePairs vertex_reference_poses_;
  LandmarkIdPositionPairs landmark_reference_positions_;
};

}  // namespace visual_inertial_mapping

#endif  // VI_MAPPING_TEST_APP_VI_MAPPING_TEST_APP_H_
