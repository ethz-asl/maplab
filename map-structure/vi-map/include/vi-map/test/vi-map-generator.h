#ifndef VI_MAP_TEST_VI_MAP_GENERATOR_H_
#define VI_MAP_TEST_VI_MAP_GENERATOR_H_

#include <initializer_list>
#include <random>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Eigen/Core>

#include <aslam/common/memory.h>
#include <maplab-common/pose_types.h>
#include <posegraph/unique-id.h>

#include "vi-map/unique-id.h"
#include "vi-map/vi-map.h"

namespace aslam {
class NCamera;
}  // namespace aslam

namespace vi_map {

/**
 * This class provides a wrapper for the following VIMap functions:
 * - addNewMissionWithBaseFrame
 * - addNewLandmark
 * - associateKeypointWithExistingLandmark
 * - addVertex
 * - addEdge
 *
 * The goal is to avoid the householding that is typically required with the
 * use of these functions.
 */
static constexpr size_t kDescriptorSize = 48;
static constexpr uint32_t kCameraWidth = 640;
static constexpr uint32_t kCameraHeight = 480;
static constexpr double kMockF = 1;
static constexpr double kMockC = 1;

class VIMapGenerator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MAPLAB_POINTER_TYPEDEFS(VIMapGenerator);

  VIMapGenerator(VIMap& map, int seed);  // NOLINT

  // * T_G_M covariance will be zero
  // * camera will be distortion-free
  // * T_C_I will be identity
  // * all biases will be zero
  // feel free to add an overload to specify these parameters
  MissionId createMission();
  MissionId createMission(const pose::Transformation& T_G_M);

  // Automatically adds a kViwls edge from the previously added vertex.
  pose_graph::VertexId createVertex(
      const MissionId& mission, const pose::Transformation& T_G_I);
  pose_graph::VertexId createVertex(
      const MissionId& mission, const pose::Transformation& T_G_I,
      const int64_t timestamp_nanoseconds);

  // The keypoint descriptor will be randomly generated and used by all
  // vertices.
  vi_map::LandmarkId createLandmark(
      const Eigen::Vector3d& p_G_fi, const pose_graph::VertexId& storing_vertex,
      const std::initializer_list<pose_graph::VertexId>& non_storing_observers);
  vi_map::LandmarkId createLandmark(
      const Eigen::Vector3d& p_G_fi, const pose_graph::VertexId& storing_vertex,
      const pose_graph::VertexIdList& non_storing_observers);
  vi_map::LandmarkId createLandmarkWithMissingReferences(
      const Eigen::Vector3d& p_G_fi, const pose_graph::VertexId& storing_vertex,
      const std::initializer_list<pose_graph::VertexId>& non_storing_observers,
      const std::initializer_list<pose_graph::VertexId>&
          non_referring_observers);
  vi_map::LandmarkId createLandmarkWithMissingReferences(
      const Eigen::Vector3d& p_G_fi, const pose_graph::VertexId& storing_vertex,
      const pose_graph::VertexIdList& non_storing_observers,
      const pose_graph::VertexIdList& non_referring_observers);
  vi_map::LandmarkId createLandmarkWithoutReferences(
      const Eigen::Vector3d& p_G_fi,
      const pose_graph::VertexIdList& non_referring_observers);

  void setCameraRig(const std::shared_ptr<aslam::NCamera>& n_camera);

  void setDefaultTransformationCovariancePQ(
      const Eigen::Matrix<double, 6, 6>& covariance_p_q);

  // Currently supported: Transformation, Viwls.
  template <typename EdgeType = TransformationEdge>
  void generateMap() const;

  Eigen::Matrix2Xd centerMeasurementsOnPrincipalPoint(
      const Eigen::Matrix2Xd& image_points, const size_t frame_index) const;

 private:
  struct LandmarkInfo;
  struct ObservationIndices {
    unsigned int frame_id;
    unsigned int keypoint_id;
  };
  typedef std::unordered_map<pose_graph::VertexId,
                             std::unordered_map<LandmarkId, ObservationIndices>>
      ObservationIndexMap;
  void generateLandmarkObservations(
      const pose_graph::VertexId& vertex_id, Eigen::Matrix2Xd* image_points,
      aslam::VisualFrame::DescriptorsT* descriptors,
      ObservationIndexMap* observation_index) const;
  void projectLandmark(
      const LandmarkInfo& landmark_info, const pose::Transformation& T_C_G,
      Eigen::Matrix2Xd* keypoints, size_t index) const;

  VIMap& map_;
  std::mt19937 rng_;
  std::shared_ptr<aslam::NCamera> n_camera_;
  typedef AlignedUnorderedMap<MissionId, pose::Transformation> MissionInfoMap;
  MissionInfoMap missions_;
  struct VertexInfo {
    MissionId mission;
    pose::Transformation T_G_I;
    std::unordered_set<vi_map::LandmarkId> landmarks;
    int64_t timestamp_nanoseconds;
    VertexInfo(
        const MissionId& _mission, const pose::Transformation& _T_G_I,
        const int64_t _timestamp_nanoseconds);
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
  typedef AlignedUnorderedMap<pose_graph::VertexId, VertexInfo> VertexInfoMap;
  VertexInfoMap vertices_;
  std::unordered_map<MissionId, pose_graph::VertexId> last_vertex_id_;

  struct EdgeInfo {
    MissionId mission;
    pose_graph::VertexId from;
    pose_graph::VertexId to;
    inline EdgeInfo(
        MissionId _mission, pose_graph::VertexId _from,
        pose_graph::VertexId _to)
        : mission(_mission), from(_from), to(_to) {}
  };
  template <typename EdgeType>
  EdgeType* generateEdge(const EdgeInfo& edge_info) const;
  template <typename EdgeType>
  static Mission::BackBone backBoneType();

  std::vector<EdgeInfo> edges_;
  struct LandmarkInfo {
    Eigen::Vector3d p_G_fi;
    VIMap::DescriptorType descriptor;
    pose_graph::VertexId storing_vertex_id;
    pose_graph::VertexIdList non_storing_vertex_ids;
    pose_graph::VertexIdList non_referring_vertex_ids;
    LandmarkInfo(
        const Eigen::Vector3d& _p_G_fi,
        const VIMap::DescriptorType& _descriptor,
        const pose_graph::VertexId& _storing_vertex_id,
        const pose_graph::VertexIdList& _non_storing_vertex_ids,
        const pose_graph::VertexIdList& _non_referring_vertex_ids);
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
  typedef AlignedUnorderedMap<vi_map::LandmarkId, LandmarkInfo> LandmarkInfoMap;
  LandmarkInfoMap landmarks_;

  Eigen::Matrix<double, 6, 6> default_edge_T_covariance_p_q_;
};

}  // namespace vi_map

#endif  // VI_MAP_TEST_VI_MAP_GENERATOR_H_
