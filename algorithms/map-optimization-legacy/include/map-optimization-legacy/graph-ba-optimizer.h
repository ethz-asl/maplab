#ifndef MAP_OPTIMIZATION_LEGACY_GRAPH_BA_OPTIMIZER_H_
#define MAP_OPTIMIZATION_LEGACY_GRAPH_BA_OPTIMIZER_H_

#include <list>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <ceres/ceres.h>
#include <glog/logging.h>
#include <gtest/gtest_prod.h>

#include <aslam/cameras/camera-pinhole.h>
#include <aslam/frames/visual-frame.h>
#include <ceres-error-terms/ceres-signal-handler.h>
#include <ceres-error-terms/common.h>
#include <ceres-error-terms/problem-information.h>
#include <ceres-error-terms/visual-error-term-base.h>
#include <ceres-error-terms/visual-error-term.h>
#include <vi-map/landmark.h>
#include <vi-map/loop-constraint.h>
#include <vi-map/mission-baseframe.h>
#include <vi-map/mission.h>
#include <vi-map/pose-graph.h>
#include <vi-map/sensor-manager.h>
#include <vi-map/unique-id.h>
#include <vi-map/vi-map.h>
#include <vi-map/vi-mission.h>

#include "map-optimization-legacy/ba-optimization-options.h"

DECLARE_bool(verbose_ba);

namespace ceres {
class CostFunction;
class LocalParameterization;
class LossFunction;
class LossFunctionWrapper;
}

namespace map_optimization_legacy {

class DoubleWindow;

class GraphBaOptimizer {
 public:
  template <typename CallbackType>
  friend class GraphIterationCallback;
  friend class OutlierRejectionCallback;
  friend class PosegraphRelaxation;

  typedef std::unordered_map<pose_graph::VertexId, int> VertexIdPoseIdxMap;
  typedef std::unordered_map<vi_map::MissionBaseFrameId, int>
      MissionBaseFrameIdBaseFrameIdxMap;
  typedef vi_map::VIMissionMap MissionMap;
  typedef vi_map::MissionBaseFrameMap MissionBaseFrameMap;

  typedef std::shared_ptr<ceres_error_terms::SignalHandlerCallback>
      CeresSignalHandlerPtr;

  explicit GraphBaOptimizer(vi_map::VIMap* map);
  virtual ~GraphBaOptimizer();

  void visualBaOptimization(
      const pose_graph::VertexIdSet& fixed_vertices,
      const BaOptimizationOptions& options);
  void visualBaOptimizationWithCallback(
      const pose_graph::VertexIdSet& fixed_vertices,
      const BaOptimizationOptions& options,
      std::function<void(const vi_map::VIMap&)> callback,
      ceres::Solver::Summary* summary);

  void visualInertialBaOptimizationWithCallback(
      const vi_map::MissionBaseFrameIdSet& fixed_baseframes,
      const pose_graph::VertexIdSet& fixed_vertices,
      const pose_graph::VertexIdSet& velocity_prior_for_vertices,
      const BaOptimizationOptions& options,
      std::function<void(const vi_map::VIMap&)> callback,
      ceres::Solver::Summary* summary);

  void doubleWindowBaOptimization(
      const DoubleWindow& double_window, int num_iterations);

  void alignMissions(
      const std::function<void(const vi_map::VIMap&)>& callback,
      const vi_map::MissionIdSet& missions,
      const vi_map::MissionBaseFrameIdSet& baseframes_to_fix);

  void addPosePriorOnCameraExtrinsics(
      const double prior_position_std_dev_meters,
      const double prior_orientation_std_dev_radians);

  const ceres_error_terms::ProblemInformation& getProblemInformation() const {
    return problem_information_;
  }
  ceres_error_terms::ProblemInformation& getProblemInformation() {
    return problem_information_;
  }

  friend class ViwlsGraph;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  // Adds a Ceres signal handler callback to this optimizer.
  // Make sure only one optimizer is listening for callbacks
  // at a single point in time!
  inline void enableCeresSignalHandlerCallback(bool enable) {
    if (enable) {
      signal_handler_callback_ptr_.reset(
          new ceres_error_terms::SignalHandlerCallback());
    } else {
      signal_handler_callback_ptr_.reset();
    }
  }

 protected:
  ceres::Solver::Options getDefaultSolverOptions() const;

  void fixVertices(const pose_graph::VertexIdSet& vertex_ids);
  void fixVerticesAndObservedLandmarks(
      const pose_graph::VertexIdSet& vertex_ids);
  void fixMissionBaseframe(const vi_map::MissionBaseFrameId& baseframe_to_fix);

  void fixMissionBaseframes(
      const vi_map::MissionBaseFrameIdSet& baseframes_to_fix);

  void addPosePriorResidualBlocks(
      const pose_graph::VertexIdSet& vertex_ids,
      const double prior_position_std_dev_meters,
      const double prior_orientation_std_dev_radian);

  void addPosePriorOnOptionalSensorExtrinsics(
      const double prior_position_std_dev_meters,
      const double prior_orientation_std_dev_radians);

  void addDecayingPosePriorResidualBlocks(
      const DoubleWindow& double_window,
      const double prior_position_std_dev_meters,
      const double prior_orientation_std_dev_radians,
      pose_graph::VertexIdSet* vertices_to_fix);

  void addVelocityPriorResidualBlocks(
      const pose_graph::VertexIdSet& vertex_ids,
      const double prior_velocity_std_dev_meter_seconds);

  bool addVisualResidualBlockOfKeypoint(
      const Eigen::Matrix<double, 2, 1>& image_point_distorted,
      double image_point_uncertainty, const vi_map::LandmarkId& landmark_id,
      bool fix_landmark_positions,
      unsigned int min_observer_vertices_per_landmark,
      unsigned int min_landmarks_per_frame,
      unsigned int min_num_observer_missions,
      std::shared_ptr<ceres::LossFunction> loss_function, aslam::Camera* camera,
      unsigned int* external_landmarks_added, vi_map::Vertex* ba_vertex);

  void addVisualResidualBlocks(
      bool fix_intrinsics, bool fix_extrinsics_rotation,
      bool fix_extrinsics_translation, bool fix_landmark_positions,
      bool include_only_merged_landmarks,
      unsigned int min_observer_vertices_per_landmark,
      unsigned int min_landmarks_per_frame,
      unsigned int min_num_observer_missions);

  void addVisualResidualBlocks(
      bool fix_intrinsics, bool fix_extrinsics_rotation,
      bool fix_extrinsics_translation, bool fix_landmark_positions,
      bool include_only_merged_landmarks,
      unsigned int min_observer_vertices_per_landmark,
      unsigned int min_landmarks_per_frame,
      unsigned int min_num_observer_missions,
      pose_graph::VertexIdSet* vertices_with_visual_residuals);

  void addDoubleWindowVisualResidualBlocks(const DoubleWindow& double_window);

  void addInertialResidualBlocks(
      bool fix_gyro_bias, bool fix_accel_bias, bool fix_velocity,
      bool use_given_edges, const pose_graph::EdgeIdList& edges,
      bool store_imu_edge_covariances, const double gravity_magnitude,
      AlignedUnorderedMap<pose_graph::EdgeId, Eigen::Matrix<double, 6, 6> >*
          imu_edge_covariances);

  void addRelativePoseResidualBlocks(
      vi_map::Edge::EdgeType edge_type, bool use_given_edges,
      const pose_graph::EdgeIdList& provided_edges, bool fix_extrinsics);

  void addLoopClosureEdges(
      bool use_given_edges, const pose_graph::EdgeIdList& provided_edges,
      const bool use_switchable_constraints, const double cauchy_loss);

  void addPositionOnlyGPSResidualBlock(
      pose_graph::Edge::EdgeType edge_type, bool use_given_edges,
      const pose_graph::EdgeIdList& provided_edges);

  void addMissionAlignResidualBlocks(const vi_map::MissionIdSet& missions);

  void buildProblem(ceres::Problem* problem);
  void solve(
      bool copy_data_from_solver_back_to_map,
      const ceres::Solver::Options& solver_options, ceres::Problem* problem,
      ceres::Solver::Summary* summary);

  void addIterationCallback(std::function<void(const vi_map::VIMap&)> callback);

  void setCameraParameterizationIfPartOfTheProblem(
      const aslam::Camera::Ptr& camera_ptr, bool fix_intrinsics,
      bool fix_extrinsics_rotation, bool fix_extrinsics_translation);

  void copyDataFromMap();
  void copyDataToMap();
  void copyDataToMap(
      bool copy_vertices, bool copy_baseframes, bool copy_cameras,
      bool copy_sensor_extrinsics);
  void copyDataOfSelectedVerticesToMap(
      bool copy_vertices, bool copy_baseframes, bool copy_cameras,
      bool copy_optional_sensor_extrinsics,
      const pose_graph::VertexIdList& optimized_vertices);

  const std::shared_ptr<ceres::Problem>& getCeresProblem();

  void removeLandmarksBehindCamera();
  void visualErrorTermsOutlierRejection(ceres::Problem* problem);

  void markLandmarkAsBad(const vi_map::LandmarkId& landmark_id);

  unsigned int getNumOfWellConstrainedLandmarksInFrame(
      const vi_map::Vertex& ba_vertex, unsigned int frame_idx);

  double* get_p_C_I_Mutable(const aslam::CameraId& camera_id);
  double* get_q_C_I_JPL_Mutable(const aslam::CameraId& camera_id);

  void fixBaseframes();

  vi_map::VIMap& map_;
  const vi_map::VIMap& const_map_;

  // We need to store vertex poses separately for two reasons:
  // 1) 7-element pose blocks are required by the error terms we use
  // 2) We keep q_M_I on posegraph, but require q_I_M in optimization
  VertexIdPoseIdxMap vertex_id_to_pose_idx_;
  Eigen::Matrix<double, 7, Eigen::Dynamic> vertex_poses_;

  // We also require 7-element pose block for mission baseframes.
  MissionBaseFrameIdBaseFrameIdxMap baseframe_id_to_baseframe_idx_;
  Eigen::Matrix<double, 7, Eigen::Dynamic> baseframe_poses_;

  // Poses of the optional sensor extrinsics.
  typedef std::unordered_map<vi_map::SensorId, size_t> SensorExtrinsicsIdxMap;
  SensorExtrinsicsIdxMap sensor_id_to_extrinsics_col_idx_;
  Eigen::Matrix<double, 7, Eigen::Dynamic> sensor_extrinsics_;

  // Structure to store the T_I_C_JPL for each camera.
  typedef std::unordered_map<aslam::CameraId, int> CameraIdCameraIdxMap;
  typedef std::unordered_map<aslam::CameraId, aslam::NCameraId>
      CameraIdNcameraIdMap;
  CameraIdCameraIdxMap camera_id_to_T_C_I_idx_;
  CameraIdNcameraIdMap camera_id_to_ncamera_ids_;
  Eigen::Matrix<double, 7, Eigen::Dynamic> T_C_I_JPL_;

  ceres_error_terms::ProblemInformation problem_information_;
  typedef std::unordered_map<ceres::CostFunction*, vi_map::LandmarkId>
      CostFunctionToLandmarkMap;
  CostFunctionToLandmarkMap residual_to_landmark;

  std::shared_ptr<ceres::LocalParameterization> quaternion_parameterization_;
  std::shared_ptr<ceres::LocalParameterization> pose_parameterization_;
  std::shared_ptr<ceres::LocalParameterization> yaw_only_pose_parameterization_;
  std::vector<std::shared_ptr<ceres::IterationCallback> > solver_callbacks_;
  CeresSignalHandlerPtr signal_handler_callback_ptr_;
  std::shared_ptr<ceres::Problem> ceres_problem_;

  typedef ceres::CauchyLoss DefaultLossFunction;
  static constexpr double kDefaultLossParameter = 3 * 0.8;
  static constexpr double kBearingLossParameter = 3 * 0.174;

  // Ordering is [orientation position] -> [xyzw xyz].
  Eigen::Matrix<double, 7, 1> dummy_7d_0_;
  Eigen::Matrix<double, 7, 1> dummy_7d_1_;
  Eigen::Matrix<double, 7, 1> dummy_7d_2_;
  Eigen::Matrix<double, 7, 1> dummy_7d_3_;

  static constexpr unsigned int kMinObserversPerLandmarkThreshold = 5;
};

}  // namespace map_optimization_legacy

#endif  // MAP_OPTIMIZATION_LEGACY_GRAPH_BA_OPTIMIZER_H_
