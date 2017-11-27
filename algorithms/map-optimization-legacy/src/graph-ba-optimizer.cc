#include "map-optimization-legacy/graph-ba-optimizer.h"

#include <cmath>

#include <ceres/loss_function.h>
#include <gflags/gflags.h>

#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/camera-unified-projection.h>
#include <aslam/cameras/distortion-equidistant.h>
#include <aslam/cameras/distortion-fisheye.h>
#include <aslam/cameras/distortion-null.h>
#include <aslam/cameras/distortion-radtan.h>
#include <aslam/common/timer.h>
#include <ceres-error-terms/block-pose-prior-error-term.h>
#include <ceres-error-terms/common.h>
#include <ceres-error-terms/generic-prior-error-term.h>
#include <ceres-error-terms/inertial-error-term.h>
#include <ceres-error-terms/loop-closure-block-pose-error-term.h>
#include <ceres-error-terms/parameterization/pose-param-jpl.h>
#include <ceres-error-terms/parameterization/quaternion-param-jpl.h>
#include <ceres-error-terms/pose-prior-error-term.h>
#include <ceres-error-terms/position-error-term.h>
#include <ceres-error-terms/six-dof-block-pose-error-term-autodiff.h>
#include <ceres-error-terms/six-dof-block-pose-error-term-with-extrinsics-autodiff.h>
#include <ceres-error-terms/switch-prior-error-term.h>
#include <ceres-error-terms/visual-error-term-factory.h>
#include <ceres-error-terms/visual-error-term.h>
#include <maplab-common/gravity-provider.h>
#include <maplab-common/pose_types.h>
#include <maplab-common/progress-bar.h>
#include <maplab-common/quaternion-math.h>
#include <maplab-common/threading-helpers.h>
#include <vi-map-helpers/vi-map-landmark-quality-evaluation.h>
#include <vi-map/landmark-quality-metrics.h>
#include <vi-map/vertex.h>
#include <vi-map/vi-map.h>
#include <vi-map/viwls-edge.h>

#include "map-optimization-legacy/double-window.h"
#include "map-optimization-legacy/graph-iteration-callback.h"

DEFINE_bool(
    optimizer_yaw_only_baseframe_parametrization, true,
    "Whether to use a yaw-only baseframe pose parameterization (hold "
    "pitch and roll fixed). Assumes all missions are gravity-aligned.");
DEFINE_double(
    optimizer_localization_robust_loss, 10.0,
    "How robust to make the cauchy loss function for localization.");
DEFINE_double(
    dwo_decay_factor, 0.5,
    "Decaying rate of pose prior error terms covariance for DW opt.");
DEFINE_bool(
    dwo_fix_given_baseframes, true,
    "Fix baseframes passed to DW optimization.");
DEFINE_bool(
    dwo_only_optimize_selected_mission, false,
    "Fixes all vertices that are not in the mission selected for DW "
    "optimization.");

namespace map_optimization_legacy {

GraphBaOptimizer::GraphBaOptimizer(vi_map::VIMap* map)
    : map_(*CHECK_NOTNULL(map)),
      const_map_(*CHECK_NOTNULL(map)),
      quaternion_parameterization_(nullptr),
      pose_parameterization_(nullptr),
      yaw_only_pose_parameterization_(nullptr),
      signal_handler_callback_ptr_(nullptr) {
  // Set unit rotation and zero translation to dummy pose objects.
  dummy_7d_0_ << 0, 0, 0, 1, 0, 0, 0;
  dummy_7d_1_ << 0, 0, 0, 1, 0, 0, 0;
  dummy_7d_2_ << 0, 0, 0, 1, 0, 0, 0;
  dummy_7d_3_ << 0, 0, 0, 1, 0, 0, 0;

  pose_parameterization_.reset(new ceres_error_terms::JplPoseParameterization);
  quaternion_parameterization_.reset(
      new ceres_error_terms::JplQuaternionParameterization);

  copyDataFromMap();
}

GraphBaOptimizer::~GraphBaOptimizer() {}

ceres::Solver::Options GraphBaOptimizer::getDefaultSolverOptions() const {
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = VLOG_IS_ON(1);
  options.max_num_iterations = 10;
  options.function_tolerance = 1e-12;
  options.gradient_tolerance = 1e-10;
  options.parameter_tolerance = 1e-8;
  options.num_threads = common::getNumHardwareThreads();
  options.num_linear_solver_threads = common::getNumHardwareThreads();
  options.jacobi_scaling = false;
  options.initial_trust_region_radius = 1e5;
  options.max_trust_region_radius = 1e20;
  options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
  for (std::shared_ptr<ceres::IterationCallback> callback : solver_callbacks_) {
    options.callbacks.push_back(callback.get());
  }
  if (!solver_callbacks_.empty()) {
    options.update_state_every_iteration = true;
  }

  // Add the ceres signal handler to be able to terminate the optimization
  // with CTRL+C.
  if (signal_handler_callback_ptr_) {
    options.callbacks.emplace_back(
        const_cast<ceres_error_terms::SignalHandlerCallback*>(
            signal_handler_callback_ptr_.get()));
  }
  return options;
}

void GraphBaOptimizer::copyDataFromMap() {
  pose_graph::VertexIdList all_vertices;
  const_map_.getAllVertexIds(&all_vertices);

  vertex_poses_.resize(Eigen::NoChange, all_vertices.size());

  int vertex_idx = 0;
  for (const pose_graph::VertexId& vertex_id : all_vertices) {
    const vi_map::Vertex& ba_vertex = const_map_.getVertex(vertex_id);

    Eigen::Quaterniond q_M_I = ba_vertex.get_q_M_I();
    if (q_M_I.w() < 0.) {
      q_M_I.coeffs() = -q_M_I.coeffs();
    }
    CHECK_GE(q_M_I.w(), 0.);

    // ba_vertex.getOrientation() returns active q_M_I rotation, while the
    // error terms require passive q_I_M. In consequence, we don't need
    // any inverse here.
    vertex_poses_.col(vertex_idx) << q_M_I.coeffs(), ba_vertex.get_p_M_I();

    vertex_id_to_pose_idx_.emplace(ba_vertex.id(), vertex_idx);

    ++vertex_idx;
  }

  CHECK_EQ(
      static_cast<unsigned int>(vertex_poses_.cols()),
      vertex_id_to_pose_idx_.size());
  CHECK_EQ(all_vertices.size(), vertex_id_to_pose_idx_.size());

  vi_map::MissionBaseFrameIdList mission_base_frame_ids;
  const_map_.getAllMissionBaseFrameIds(&mission_base_frame_ids);

  int base_frame_idx = 0;
  baseframe_poses_.resize(Eigen::NoChange, mission_base_frame_ids.size());

  for (const vi_map::MissionBaseFrameId& baseframe_id :
       mission_base_frame_ids) {
    const vi_map::MissionBaseFrame& baseframe =
        const_map_.getMissionBaseFrame(baseframe_id);
    baseframe_poses_.col(base_frame_idx)
        << baseframe.get_q_G_M().inverse().coeffs(),
        baseframe.get_p_G_M();

    baseframe_id_to_baseframe_idx_.emplace(baseframe_id, base_frame_idx);
    ++base_frame_idx;
  }

  CHECK_EQ(
      static_cast<unsigned int>(baseframe_poses_.cols()),
      baseframe_id_to_baseframe_idx_.size());
  CHECK_EQ(
      mission_base_frame_ids.size(), baseframe_id_to_baseframe_idx_.size());

  const vi_map::SensorManager& sensor_manager = map_.getSensorManager();
  vi_map::SensorIdSet sensor_ids;
  sensor_manager.getAllSensorIds(&sensor_ids);

  const size_t total_num_sensors = sensor_ids.size();

  sensor_extrinsics_.resize(Eigen::NoChange, total_num_sensors);

  size_t sensor_extrinsics_col_idx = 0u;
  for (const vi_map::SensorId& sensor_id : sensor_ids) {
    CHECK(sensor_id.isValid());
    aslam::Transformation T_R_S;
    if (sensor_manager.getSensor_T_R_S(sensor_id, &T_R_S)) {
      const aslam::Transformation T_S_I = T_R_S.inverse();
      sensor_extrinsics_.col(sensor_extrinsics_col_idx)
          << T_S_I.getRotation().toImplementation().inverse().coeffs(),
          T_S_I.getPosition();

      CHECK(
          sensor_id_to_extrinsics_col_idx_
              .emplace(sensor_id, sensor_extrinsics_col_idx)
              .second);
      ++sensor_extrinsics_col_idx;
    } else {
      LOG(WARNING) << "Unable to retrieve the sensor extrinsics of sensor "
                   << sensor_id.hexString();
    }
  }

  aslam::NCameraIdSet ncamera_sensor_ids;
  sensor_manager.getAllNCameraIds(&ncamera_sensor_ids);

  size_t total_num_cameras = 0u;
  for (const aslam::NCameraId& ncamera_id : ncamera_sensor_ids) {
    CHECK(ncamera_id.isValid());
    total_num_cameras += sensor_manager.getNCamera(ncamera_id).numCameras();
  }

  // Now loop over unique CameraIds in the system and store the
  // T_C_B transformations in the format accepted by ceres error terms.
  T_C_I_JPL_.resize(Eigen::NoChange, total_num_cameras);
  size_t T_C_I_column_index = 0u;
  for (const aslam::NCameraId& ncamera_id : ncamera_sensor_ids) {
    const aslam::NCamera& ncamera = sensor_manager.getNCamera(ncamera_id);
    for (size_t camera_idx = 0u; camera_idx < ncamera.numCameras();
         ++camera_idx) {
      const aslam::Transformation& T_C_I = ncamera.get_T_C_B(camera_idx);

      T_C_I_JPL_.col(T_C_I_column_index)
          << T_C_I.getRotation().toImplementation().inverse().coeffs(),
          T_C_I.getPosition();

      const aslam::CameraId& camera_id = ncamera.getCamera(camera_idx).getId();
      CHECK(camera_id.isValid());
      CHECK(camera_id_to_ncamera_ids_.emplace(camera_id, ncamera_id).second);
      CHECK(
          camera_id_to_T_C_I_idx_.emplace(camera_id, T_C_I_column_index)
              .second);
      ++T_C_I_column_index;
    }
  }
  CHECK_EQ(T_C_I_column_index, total_num_cameras);
}

double* GraphBaOptimizer::get_p_C_I_Mutable(const aslam::CameraId& camera_id) {
  CameraIdCameraIdxMap::iterator it = camera_id_to_T_C_I_idx_.find(camera_id);
  CHECK(it != camera_id_to_T_C_I_idx_.end());
  CHECK_LT(it->second, T_C_I_JPL_.cols());

  static constexpr unsigned int kOrientationBlockSize = 4;
  return &T_C_I_JPL_(kOrientationBlockSize, it->second);
}

double* GraphBaOptimizer::get_q_C_I_JPL_Mutable(
    const aslam::CameraId& camera_id) {
  CameraIdCameraIdxMap::iterator it;
  it = camera_id_to_T_C_I_idx_.find(camera_id);
  CHECK(it != camera_id_to_T_C_I_idx_.end());
  CHECK_LT(it->second, T_C_I_JPL_.cols());
  return T_C_I_JPL_.col(it->second).data();
}

void GraphBaOptimizer::buildProblem(ceres::Problem* problem) {
  CHECK_NOTNULL(problem);
  ceres_error_terms::buildCeresProblemFromProblemInformation(
      &problem_information_, problem);
}

void GraphBaOptimizer::copyDataToMap() {
  const bool copy_vertices = true;
  const bool copy_baseframes = true;
  const bool copy_cameras = true;
  const bool copy_sensor_extrinsics = true;
  copyDataToMap(
      copy_vertices, copy_baseframes, copy_cameras, copy_sensor_extrinsics);
}

void GraphBaOptimizer::copyDataToMap(
    bool copy_vertices, bool copy_baseframes, bool copy_cameras,
    bool copy_sensor_extrinsics) {
  pose_graph::VertexIdList all_vertices;
  const_map_.getAllVertexIds(&all_vertices);

  copyDataOfSelectedVerticesToMap(
      copy_vertices, copy_baseframes, copy_cameras, copy_sensor_extrinsics,
      all_vertices);
}

void GraphBaOptimizer::copyDataOfSelectedVerticesToMap(
    bool copy_vertices, bool copy_baseframes, bool copy_cameras,
    bool copy_sensor_extrinsics,
    const pose_graph::VertexIdList& optimized_vertices) {
  if (copy_vertices) {
    CHECK_LE(optimized_vertices.size(), vertex_id_to_pose_idx_.size());
    CHECK_EQ(
        static_cast<unsigned int>(vertex_poses_.cols()),
        vertex_id_to_pose_idx_.size());

    for (const pose_graph::VertexId& vertex_id : optimized_vertices) {
      vi_map::Vertex& ba_vertex = map_.getVertex(vertex_id);

      VertexIdPoseIdxMap::const_iterator it;
      it = vertex_id_to_pose_idx_.find(ba_vertex.id());
      CHECK(it != vertex_id_to_pose_idx_.end());
      CHECK_GE(it->second, 0);
      CHECK_LT(it->second, vertex_poses_.cols());

      Eigen::Map<Eigen::Quaterniond> vertex_rotation(
          ba_vertex.get_q_M_I_Mutable());
      Eigen::Map<Eigen::Vector3d> vertex_position(
          ba_vertex.get_p_M_I_Mutable());

      // I_q_G_JPL is in fact equal to active G_q_I - no inverse is needed.
      Eigen::Quaterniond q_I_G_JPL;
      q_I_G_JPL.coeffs() = vertex_poses_.col(it->second).head(4);
      q_I_G_JPL.coeffs().normalize();
      vertex_rotation = q_I_G_JPL;
      vertex_position = vertex_poses_.col(it->second).tail(3);
    }
  }

  if (copy_baseframes) {
    vi_map::MissionBaseFrameIdList mission_base_frame_ids;
    const_map_.getAllMissionBaseFrameIds(&mission_base_frame_ids);

    for (const vi_map::MissionBaseFrameId& baseframe_id :
         mission_base_frame_ids) {
      MissionBaseFrameIdBaseFrameIdxMap::iterator baseframe_it;
      baseframe_it = baseframe_id_to_baseframe_idx_.find(baseframe_id);
      CHECK(baseframe_it != baseframe_id_to_baseframe_idx_.end());

      baseframe_poses_.col(baseframe_it->second).head(4).normalize();
      Eigen::Matrix<double, 3, 1> p_G_M(
          baseframe_poses_.col(baseframe_it->second).tail(3));
      Eigen::Quaterniond q_G_M_JPL(
          baseframe_poses_.col(baseframe_it->second).head(4).data());

      vi_map::MissionBaseFrame& baseframe =
          map_.getMissionBaseFrame(baseframe_id);
      baseframe.set_p_G_M(p_G_M);
      // Change from JPL passive quaternion used by error terms to active
      // quaternion in the system.
      Eigen::Quaterniond q_G_M = q_G_M_JPL.inverse();
      q_G_M.normalize();
      baseframe.set_q_G_M(q_G_M);
    }
  }

  if (copy_cameras) {
    CHECK_EQ(
        static_cast<size_t>(T_C_I_JPL_.cols()), camera_id_to_T_C_I_idx_.size());

    vi_map::SensorManager& sensor_manager = map_.getSensorManager();
    for (const std::pair<aslam::CameraId, aslam::NCameraId>&
             camera_id_ncamera_id_pair : camera_id_to_ncamera_ids_) {
      const aslam::CameraId& camera_id = camera_id_ncamera_id_pair.first;
      CHECK(camera_id.isValid());
      const aslam::NCameraId& ncamera_id = camera_id_ncamera_id_pair.second;
      CHECK(ncamera_id.isValid());

      aslam::NCamera::Ptr ncamera = sensor_manager.getNCameraShared(ncamera_id);
      CHECK(ncamera);

      Eigen::Map<const Eigen::Vector3d> p_C_I(get_p_C_I_Mutable(camera_id));
      Eigen::Map<const Eigen::Quaterniond> q_C_I_JPL(
          get_q_C_I_JPL_Mutable(camera_id));

      // Inverse on quaternion to convert from JPL passive rotation to active
      // rotation.
      Eigen::Quaterniond q_C_I_active = q_C_I_JPL.inverse();
      q_C_I_active.normalize();
      aslam::Transformation T_C_I(p_C_I, q_C_I_active);

      int camera_idx_in_ncamera = -1;
      for (size_t camera_idx = 0u; camera_idx < ncamera->numCameras();
           ++camera_idx) {
        if (ncamera->getCameraId(camera_idx) == camera_id) {
          camera_idx_in_ncamera = static_cast<int>(camera_idx);
          break;
        }
      }
      CHECK_GE(camera_idx_in_ncamera, 0);
      ncamera->set_T_C_B(static_cast<size_t>(camera_idx_in_ncamera), T_C_I);
    }
  }

  if (copy_sensor_extrinsics) {
    vi_map::SensorManager& sensor_manager = map_.getSensorManager();

    for (const SensorExtrinsicsIdxMap::value_type& sensor_id_with_col_idx :
         sensor_id_to_extrinsics_col_idx_) {
      const vi_map::SensorId& sensor_id = sensor_id_with_col_idx.first;
      CHECK(sensor_id.isValid());
      const size_t col_idx = sensor_id_with_col_idx.second;
      CHECK_LT(col_idx, static_cast<size_t>(sensor_extrinsics_.cols()));

      sensor_extrinsics_.col(col_idx).head(4).normalize();
      const aslam::Position3D p_S_I(sensor_extrinsics_.col(col_idx).tail(3));
      const Eigen::Quaterniond q_S_I_JPL(
          sensor_extrinsics_.col(col_idx).head(4).data());
      // Change from JPL passive quaternion used by error terms to active
      // quaternion in the system.
      Eigen::Quaterniond q_S_I = q_S_I_JPL.inverse();

      aslam::Transformation T_S_I_updated;
      T_S_I_updated.getPosition() = p_S_I;
      T_S_I_updated.getRotation().toImplementation() = q_S_I;

      sensor_manager.setSensor_T_R_S(sensor_id, T_S_I_updated.inverse());
    }
  }
}

const std::shared_ptr<ceres::Problem>& GraphBaOptimizer::getCeresProblem() {
  CHECK(ceres_problem_ != nullptr);
  return ceres_problem_;
}

void GraphBaOptimizer::visualBaOptimization(
    const pose_graph::VertexIdSet& fixed_vertices,
    const BaOptimizationOptions& options) {
  std::function<void(const vi_map::VIMap&)> null_callback;
  ceres::Solver::Summary summary;
  visualBaOptimizationWithCallback(
      fixed_vertices, options, null_callback, &summary);
}

void GraphBaOptimizer::doubleWindowBaOptimization(
    const DoubleWindow& double_window, int num_iterations) {
  problem_information_.clearProblemInformation();

  const BaOptimizationOptions options;

  addDoubleWindowVisualResidualBlocks(double_window);

  pose_graph::EdgeIdList all_window_edges = double_window.getInnerWindowEdges();
  all_window_edges.insert(
      all_window_edges.end(), double_window.getOuterWindowEdges().begin(),
      double_window.getOuterWindowEdges().end());
  constexpr bool kStoreImuEdgeCovariances = false;
  constexpr bool kUseGivenEdges = true;
  addInertialResidualBlocks(
      options.fix_gyro_bias, options.fix_accel_bias, options.fix_velocity,
      kUseGivenEdges, all_window_edges, kStoreImuEdgeCovariances,
      options.gravity_magnitude, nullptr);
  constexpr bool kFixWheelOdometryExtrinsics = true;
  addRelativePoseResidualBlocks(
      vi_map::Edge::EdgeType::kOdometry, kUseGivenEdges, all_window_edges,
      kFixWheelOdometryExtrinsics);
  VLOG(2) << "Including loop-closure edges.";
  addLoopClosureEdges(
      kUseGivenEdges, all_window_edges,
      options.use_switchable_constraints_for_loop_closure_edges,
      options.loop_closure_error_term_cauchy_loss);

  pose_graph::VertexIdSet fixed_vertices_set(
      double_window.getFixedVertices().begin(),
      double_window.getFixedVertices().end());

  pose_graph::VertexIdSet fixed_decaying_pose_prior_vertices_set;
  addDecayingPosePriorResidualBlocks(
      double_window, options.prior_position_std_dev_meters,
      options.prior_orientation_std_dev_radians,
      &fixed_decaying_pose_prior_vertices_set);
  fixed_vertices_set.insert(
      fixed_decaying_pose_prior_vertices_set.begin(),
      fixed_decaying_pose_prior_vertices_set.end());

  pose_graph::VertexIdSet fixed_mission_vertices_set;
  if (FLAGS_dwo_only_optimize_selected_mission) {
    if (!double_window.isMissionToOptimizeSet()) {
      LOG(ERROR) << "Mission ID to optimize is not set. "
                 << "Aborting optimization...";
      return;
    } else {
      CHECK(map_.hasMission(double_window.getMissionToOptimize()));
      pose_graph::VertexIdSet all_dw_vertices;
      double_window.getAllWindowVertices(&all_dw_vertices);
      const vi_map::MissionId& mission_to_optimize =
          double_window.getMissionToOptimize();
      for (const pose_graph::VertexId& vertex_id : all_dw_vertices) {
        if (mission_to_optimize != map_.getMissionId(vertex_id)) {
          fixed_mission_vertices_set.insert(vertex_id);
        }
      }
    }
  }

  vi_map::MissionBaseFrameIdSet fixed_baseframe_ids;
  vi_map::MissionIdList all_mission_ids;
  map_.getAllMissionIds(&all_mission_ids);

  if (fixed_vertices_set.empty() && fixed_mission_vertices_set.empty() &&
      double_window.getOuterWindowVertices().empty()) {
    const vi_map::MissionId first_mission_id = map_.getIdOfFirstMission();
    pose_graph::VertexIdSet root_vertex;
    root_vertex.insert(map_.getMission(first_mission_id).getRootVertexId());
    addPosePriorResidualBlocks(
        root_vertex, options.prior_position_std_dev_meters,
        options.prior_orientation_std_dev_radians);
  } else {
    vi_map::MissionIdSet mission_ids_of_fixed_vertices;
    vi_map::MissionIdSet mission_ids_of_outer_vertices;
    vi_map::MissionIdSet mission_ids_of_fixed_mission_vertices;
    vi_map::MissionIdSet mission_ids_to_fix;
    map_.getMissionIds(
        double_window.getFixedVertices(), &mission_ids_of_fixed_vertices);
    map_.getMissionIds(
        double_window.getOuterWindowVertices(), &mission_ids_of_outer_vertices);
    map_.getMissionIds(
        fixed_mission_vertices_set, &mission_ids_of_fixed_mission_vertices);

    mission_ids_to_fix.insert(
        mission_ids_of_fixed_vertices.begin(),
        mission_ids_of_fixed_vertices.end());
    mission_ids_to_fix.insert(
        mission_ids_of_outer_vertices.begin(),
        mission_ids_of_outer_vertices.end());
    mission_ids_to_fix.insert(
        mission_ids_of_fixed_mission_vertices.begin(),
        mission_ids_of_fixed_mission_vertices.end());

    for (const vi_map::MissionId& mission_id : mission_ids_to_fix) {
      fixed_baseframe_ids.insert(map_.getMission(mission_id).getBaseFrameId());
    }

    // Fix baseframes, vertices and landmarks.
    fixMissionBaseframes(fixed_baseframe_ids);
    fixVertices(fixed_vertices_set);
    fixVerticesAndObservedLandmarks(fixed_mission_vertices_set);
  }

  ceres::Solver::Options solver_options = getDefaultSolverOptions();
  solver_options.max_num_iterations = num_iterations;
  solver_options.gradient_tolerance = 1e2;
  solver_options.function_tolerance = 1e-4;
  ceres_problem_.reset(
      new ceres::Problem(ceres_error_terms::getDefaultProblemOptions()));
  buildProblem(ceres_problem_.get());

  // Don't copy the data using the default method, we will do it selectively
  // below.
  static constexpr bool kCopyDataFromSolverBackToMap = false;
  ceres::Solver::Summary summary;
  solve(
      kCopyDataFromSolverBackToMap, solver_options, ceres_problem_.get(),
      &summary);

  pose_graph::VertexIdSet all_dw_vertices_set;
  double_window.getAllWindowVertices(&all_dw_vertices_set);
  pose_graph::VertexIdList all_dw_vertices(
      all_dw_vertices_set.begin(), all_dw_vertices_set.end());
  const bool kCopyVertices = true;
  const bool kCopyBaseframes = true;
  const bool kCopyCameras = true;
  const bool kCopyOptionalSensorExtrinsics = true;
  copyDataOfSelectedVerticesToMap(
      kCopyVertices, kCopyBaseframes, kCopyCameras,
      kCopyOptionalSensorExtrinsics, all_dw_vertices);

  // This function will flag all landmarks behind the camera as bad.
  removeLandmarksBehindCamera();
}

void GraphBaOptimizer::visualBaOptimizationWithCallback(
    const pose_graph::VertexIdSet& fixed_vertices,
    const BaOptimizationOptions& options,
    std::function<void(const vi_map::VIMap&)> callback,
    ceres::Solver::Summary* summary) {
  CHECK_NOTNULL(summary);
  bool kFixIntrinsics = true;
  bool kFixExtrinsicsRotation = true;
  bool kFixExtrinsicsTranslation = true;
  bool kFixLandmarkPosition = false;

  if (options.remove_behind_camera_landmarks) {
    removeLandmarksBehindCamera();
  }

  addVisualResidualBlocks(
      kFixIntrinsics, kFixExtrinsicsRotation, kFixExtrinsicsTranslation,
      kFixLandmarkPosition, options.include_only_merged_landmarks,
      kMinObserversPerLandmarkThreshold,
      options.min_number_of_visible_landmarks_at_vertex,
      options.min_number_of_landmark_observer_missions);

  if (!options.add_pose_prior_for_fixed_vertices) {
    fixVertices(fixed_vertices);
  } else {
    addPosePriorResidualBlocks(
        fixed_vertices, options.prior_position_std_dev_meters,
        options.prior_orientation_std_dev_radians);
  }
  if (options.run_custom_post_iteration_callback_on_vi_map && callback) {
    addIterationCallback(callback);
  }
  ceres::Solver::Options solver_options = getDefaultSolverOptions();
  solver_options.gradient_tolerance = 10.0;
  solver_options.function_tolerance = 1e-5;
  solver_options.max_num_iterations = options.num_iterations;

  static constexpr bool kCopyDataFromSolverBackToMap = true;
  ceres_problem_.reset(
      new ceres::Problem(ceres_error_terms::getDefaultProblemOptions()));
  buildProblem(ceres_problem_.get());
  solve(
      kCopyDataFromSolverBackToMap, solver_options, ceres_problem_.get(),
      summary);
}

void GraphBaOptimizer::visualInertialBaOptimizationWithCallback(
    const vi_map::MissionBaseFrameIdSet& fixed_baseframes,
    const pose_graph::VertexIdSet& fixed_vertices,
    const pose_graph::VertexIdSet& velocity_prior_for_vertices,
    const BaOptimizationOptions& options,
    std::function<void(const vi_map::VIMap&)> callback,
    ceres::Solver::Summary* summary) {
  CHECK_NOTNULL(summary);
  if (options.remove_behind_camera_landmarks) {
    removeLandmarksBehindCamera();
  }

  if (options.include_visual) {
    constexpr unsigned int kMinLandmarksPerFrame = 0;
    addVisualResidualBlocks(
        options.fix_ncamera_intrinsics, options.fix_ncamera_extrinsics_rotation,
        options.fix_ncamera_extrinsics_translation,
        options.fix_landmark_positions, options.include_only_merged_landmarks,
        kMinObserversPerLandmarkThreshold, kMinLandmarksPerFrame,
        options.min_number_of_landmark_observer_missions);

    if (options.fix_landmark_positions_of_fixed_vertices) {
      for (const pose_graph::VertexId& fixed_vertex_id : fixed_vertices) {
        vi_map::Vertex& fixed_vertex = map_.getVertex(fixed_vertex_id);
        vi_map::LandmarkStore& landmark_store = fixed_vertex.getLandmarks();
        for (vi_map::Landmark& landmark : landmark_store) {
          if (vi_map::isLandmarkWellConstrained(const_map_, landmark)) {
            problem_information_.setParameterBlockConstant(
                landmark.get_p_B_Mutable());
          }
        }
      }
    }
  }

  for (const vi_map::MissionBaseFrameId& baseframe_id : fixed_baseframes) {
    MissionBaseFrameIdBaseFrameIdxMap::const_iterator baseframe_it;
    baseframe_it = baseframe_id_to_baseframe_idx_.find(baseframe_id);
    CHECK(baseframe_it != baseframe_id_to_baseframe_idx_.end());
    problem_information_.setParameterBlockConstant(
        baseframe_poses_.col(baseframe_it->second).data());
  }

  const bool kStoreCachedImuCovariances = false;
  constexpr bool kUseGivenEdges = false;
  const pose_graph::EdgeIdList kProvidedEdges;

  if (options.include_inertial) {
    addInertialResidualBlocks(
        options.fix_gyro_bias, options.fix_accel_bias, options.fix_velocity,
        kUseGivenEdges, kProvidedEdges, kStoreCachedImuCovariances,
        options.gravity_magnitude, nullptr);
  }

  if (options.include_wheel_odometry) {
    VLOG(2) << "Including wheel-odometry.";
    addRelativePoseResidualBlocks(
        vi_map::Edge::EdgeType::kOdometry, kUseGivenEdges, kProvidedEdges,
        options.fix_wheel_odometry_extrinsics);
  }

  if (options.include_loop_closure_edges) {
    VLOG(2) << "Including loop-closure edges.";
    addLoopClosureEdges(
        kUseGivenEdges, kProvidedEdges,
        options.use_switchable_constraints_for_loop_closure_edges,
        options.loop_closure_error_term_cauchy_loss);
  }

  if (options.include_gps) {
    if (options.position_only_gps) {
      VLOG(2) << "Including position-only gps.";
      addPositionOnlyGPSResidualBlock(
          vi_map::Edge::EdgeType::k6DoFGps, kUseGivenEdges, kProvidedEdges);
    } else {
      VLOG(2) << "Including 6DoF gps.";
      constexpr bool kFixGPSExtrinsics = true;
      addRelativePoseResidualBlocks(
          vi_map::Edge::EdgeType::k6DoFGps, kUseGivenEdges, kProvidedEdges,
          kFixGPSExtrinsics);
    }
  }

  if (!velocity_prior_for_vertices.empty()) {
    addVelocityPriorResidualBlocks(
        velocity_prior_for_vertices,
        options.prior_velocity_std_dev_meter_seconds);
  }

  if (options.add_pose_prior_for_fixed_vertices) {
    addPosePriorResidualBlocks(
        fixed_vertices, options.prior_position_std_dev_meters,
        options.prior_orientation_std_dev_radians);
  } else {
    fixVertices(fixed_vertices);
  }

  if (options.add_pose_prior_on_camera_extrinsics) {
    addPosePriorOnCameraExtrinsics(
        options.camera_extrinsics_position_prior_std_dev_meters,
        options.camera_extrinsics_orientation_prior_std_dev_radians);
  }

  if (options.add_pose_prior_on_wheel_odometry_extrinsics) {
    addPosePriorOnOptionalSensorExtrinsics(
        options.wheel_odometry_extrinsics_position_prior_std_dev_meters,
        options.wheel_odometry_extrinsics_orientation_prior_std_dev_radians);
  }

  if (options.run_custom_post_iteration_callback_on_vi_map && callback) {
    addIterationCallback(callback);
  }

  if (options.fix_wheel_odometry_extrinsics_position) {
    for (const SensorExtrinsicsIdxMap::value_type& sensor_id_idx_pair :
         sensor_id_to_extrinsics_col_idx_) {
      CHECK(sensor_id_idx_pair.first.isValid());
      CHECK_LT(
          static_cast<int>(sensor_id_idx_pair.second),
          sensor_extrinsics_.cols());

      problem_information_.setParameterBlockConstant(
          sensor_extrinsics_.col(sensor_id_idx_pair.second).data() +
          ceres_error_terms::poseblocks::kOrientationBlockSize);
    }
    VLOG(1) << "Fixed wheel-odometry extrinsics positions.";
  }

  if (options.fix_baseframes) {
    fixBaseframes();
  }

  ceres::Solver::Options solver_options = getDefaultSolverOptions();
  solver_options.max_num_iterations = options.num_iterations;
  solver_options.gradient_tolerance = 10.0;
  solver_options.function_tolerance = 1e-4;
  static constexpr bool kCopyDataFromSolverBackToMap = true;

  if (options.visual_outlier_rejection) {
    CHECK_GT(options.num_visual_outlier_rejection_loops, 0u);
    for (size_t visual_outlier_rejection_loop_idx = 0u;
         visual_outlier_rejection_loop_idx <
         options.num_visual_outlier_rejection_loops + 1;
         ++visual_outlier_rejection_loop_idx) {
      if (visual_outlier_rejection_loop_idx == 0u) {
        // For the first round, we let the solver minimize the landmarks first.
        solver_options.max_num_iterations = 20;
      } else if (
          visual_outlier_rejection_loop_idx <
          options.num_visual_outlier_rejection_loops) {
        // Then we do n outlier rejection loops.
        solver_options.max_num_iterations = 20;
      } else {
        // Finally we run to convergence or until we hit num_iterations.
        solver_options.max_num_iterations =
            options.num_iterations - options.num_visual_outlier_rejection_loops;
      }
      ceres_problem_.reset(
          new ceres::Problem(ceres_error_terms::getDefaultProblemOptions()));
      buildProblem(ceres_problem_.get());
      solve(
          kCopyDataFromSolverBackToMap, solver_options, ceres_problem_.get(),
          summary);
      if (options.visual_outlier_rejection) {
        visualErrorTermsOutlierRejection(ceres_problem_.get());
      }
    }
  } else {
    ceres_problem_.reset(
        new ceres::Problem(ceres_error_terms::getDefaultProblemOptions()));
    buildProblem(ceres_problem_.get());
    solve(
        kCopyDataFromSolverBackToMap, solver_options, ceres_problem_.get(),
        summary);
  }

  // This function will flag all landmarks behind the camera as bad.
  removeLandmarksBehindCamera();
}

void GraphBaOptimizer::alignMissions(
    const std::function<void(const vi_map::VIMap&)>& callback,
    const vi_map::MissionIdSet& missions,
    const vi_map::MissionBaseFrameIdSet& baseframes_to_fix) {
  CHECK(!missions.empty());
  CHECK(!baseframes_to_fix.empty())
      << "At least one baseframe has to be fixed.";

  vi_map_helpers::evaluateLandmarkQuality(&map_);
  LOG(INFO) << "Running align on the following missions:";
  for (const vi_map::MissionId& mission_id : missions) {
    LOG(INFO) << "\t" << mission_id;
  }

  LOG(INFO) << "The following baseframes are fixed:";
  for (const vi_map::MissionBaseFrameId& baseframe_id : baseframes_to_fix) {
    LOG(INFO) << "\t" << baseframe_id;
  }

  addMissionAlignResidualBlocks(missions);

  // Get the baseframe ids of the missions to fix.
  for (const vi_map::MissionBaseFrameId& baseframe_to_fix : baseframes_to_fix) {
    fixMissionBaseframe(baseframe_to_fix);
  }

  BaOptimizationOptions ba_options;
  if (ba_options.run_custom_post_iteration_callback_on_vi_map && callback) {
    addIterationCallback(callback);
  }
  bool copy_data_from_solver_back_to_map = true;

  ceres::Solver::Options options = getDefaultSolverOptions();
  options.max_num_iterations = 50;
  options.function_tolerance = 1e-12;
  ceres_problem_.reset(
      new ceres::Problem(ceres_error_terms::getDefaultProblemOptions()));
  buildProblem(ceres_problem_.get());
  ceres::Solver::Summary summary;
  solve(
      copy_data_from_solver_back_to_map, options, ceres_problem_.get(),
      &summary);
}

void GraphBaOptimizer::fixMissionBaseframe(
    const vi_map::MissionBaseFrameId& baseframe_to_fix) {
  CHECK_GT(baseframe_id_to_baseframe_idx_.count(baseframe_to_fix), 0u);
  int index_of_baseframe = baseframe_id_to_baseframe_idx_[baseframe_to_fix];
  VLOG(3) << "Fixing mission baseframe " << baseframe_to_fix;
  CHECK_GT(baseframe_poses_.cols(), index_of_baseframe);

  problem_information_.setParameterBlockConstant(
      baseframe_poses_.col(index_of_baseframe).data());
}

void GraphBaOptimizer::fixMissionBaseframes(
    const vi_map::MissionBaseFrameIdSet& baseframes_to_fix) {
  for (const vi_map::MissionBaseFrameId& baseframe_to_fix : baseframes_to_fix) {
    fixMissionBaseframe(baseframe_to_fix);
  }
}

void GraphBaOptimizer::fixVertices(const pose_graph::VertexIdSet& vertex_ids) {
  VLOG(3) << "Fixing " << vertex_ids.size() << " vertices.";

  for (const pose_graph::VertexId& vertex_id : vertex_ids) {
    VertexIdPoseIdxMap::const_iterator it;
    it = vertex_id_to_pose_idx_.find(vertex_id);
    CHECK(it != vertex_id_to_pose_idx_.end());
    CHECK_GE(it->second, 0);
    CHECK_LT(it->second, vertex_poses_.cols());

    VLOG(4) << "  Position: "
            << vertex_poses_.col(it->second).tail(3).transpose()
            << " -- rotation:  "
            << vertex_poses_.col(it->second).head(4).transpose();

    double* vertex_pose_data = vertex_poses_.col(it->second).data();
    problem_information_.setParameterBlockConstant(vertex_pose_data);
  }
}
void GraphBaOptimizer::fixVerticesAndObservedLandmarks(
    const pose_graph::VertexIdSet& vertex_ids) {
  fixVertices(vertex_ids);
  for (const pose_graph::VertexId& fixed_vertex_id : vertex_ids) {
    vi_map::Vertex& fixed_vertex = map_.getVertex(fixed_vertex_id);
    vi_map::LandmarkStore& landmark_store = fixed_vertex.getLandmarks();
    for (vi_map::Landmark& landmark : landmark_store) {
      problem_information_.setParameterBlockConstant(
          landmark.get_p_B_Mutable());
    }
  }
}

void GraphBaOptimizer::visualErrorTermsOutlierRejection(
    ceres::Problem* problem) {
  CHECK_NOTNULL(problem);
  LOG(INFO) << "Running outlier rejection.";
  constexpr double kPixelSigma = 0.8;
  // Use (3*sigma)^2 as threshold for outlier rejection.
  constexpr double kThresholdSquared = 9 * kPixelSigma * kPixelSigma;

  typedef std::unordered_map<ceres::ResidualBlockId, vi_map::LandmarkId>
      ResidualBlockLandmarkIdMap;
  ResidualBlockLandmarkIdMap residual_block_id_to_landmark;

  ceres::Problem::EvaluateOptions evaluate_options;
  evaluate_options.apply_loss_function = false;
  evaluate_options.num_threads = common::getNumHardwareThreads();

  for (const ceres_error_terms::ProblemInformation::ResidualInformationMap::
           value_type& item : problem_information_.residual_blocks) {
    const ceres_error_terms::ResidualInformation& residual_information =
        item.second;
    ceres::ResidualBlockId residual_block_id =
        residual_information.latest_residual_block_id;
    CHECK_NOTNULL(residual_block_id);
    if (residual_information.residual_type !=
            ceres_error_terms::ResidualType::kVisualReprojectionError ||
        !residual_information.active_) {
      continue;
    }
    CostFunctionToLandmarkMap::const_iterator it =
        residual_to_landmark.find(residual_information.cost_function.get());
    CHECK(it != residual_to_landmark.end());
    residual_block_id_to_landmark[residual_block_id] = it->second;
    // Mark this residual as a residual to evaluate.
    evaluate_options.residual_blocks.push_back(residual_block_id);
  }

  std::vector<double> residuals;
  timing::Timer timer_eval("Outlierrejection evaluate");
  problem->Evaluate(evaluate_options, nullptr, &residuals, nullptr, nullptr);
  timer_eval.Stop();

  const std::vector<ceres::ResidualBlockId>& residual_blocks =
      evaluate_options.residual_blocks;
  CHECK_EQ(residual_blocks.size() * 2, residuals.size());

  std::unordered_map<vi_map::LandmarkId, std::vector<ceres::ResidualBlockId>>
      landmark_residual_blocks;

  // Check every residual if it violates the threshold.
  for (size_t i = 0; i < residual_blocks.size(); ++i) {
    const ceres::ResidualBlockId& residual_block_id = residual_blocks[i];
    const double residual_squared = residuals[i * 2] * residuals[i * 2] +
                                    residuals[i * 2 + 1] * residuals[i * 2 + 1];
    if (residual_squared <= kThresholdSquared) {
      ResidualBlockLandmarkIdMap::const_iterator it =
          residual_block_id_to_landmark.find(residual_block_id);
      CHECK(it != residual_block_id_to_landmark.end());
      const vi_map::LandmarkId& landmark_id = it->second;

      // If the residual is small, store this as valid constraint for this
      // landmark.
      landmark_residual_blocks[landmark_id].push_back(residual_block_id);
    }
  }

  vi_map::LandmarkIdList bad_landmark_ids;
  std::unordered_set<ceres::ResidualBlockId> visual_residual_blocks_to_keep;
  // Check which landmarks have not enough observations anymore.
  for (const std::pair<const vi_map::LandmarkId,
                       std::vector<ceres::ResidualBlockId>>&
           landmark_and_residual_blocks : landmark_residual_blocks) {
    if (landmark_and_residual_blocks.second.size() < 2u) {
      // Remember this landmark for later removal.
      bad_landmark_ids.push_back(landmark_and_residual_blocks.first);
    } else {
      // Keep all the residual blocks for this landmark that didn't violate
      // the threshold.
      visual_residual_blocks_to_keep.insert(
          landmark_and_residual_blocks.second.begin(),
          landmark_and_residual_blocks.second.end());
    }
  }

  // Copy the residual blocks that are valid to the new set of residual blocks.
  int rejected_residual_blocks = 0;
  int num_visual_residual_blocks = 0;
  for (ceres_error_terms::ProblemInformation::ResidualInformationMap::
           value_type& item : problem_information_.residual_blocks) {
    ceres_error_terms::ResidualInformation& residual_information = item.second;
    ceres::ResidualBlockId residual_block_id =
        residual_information.latest_residual_block_id;
    CHECK_NOTNULL(residual_block_id);
    if (residual_information.residual_type ==
            ceres_error_terms::ResidualType::kVisualReprojectionError &&
        residual_information.active_) {
      ++num_visual_residual_blocks;
      // If it is a visual residual block, we deactivate it if it is not in the
      // set of valid residual blocks.
      if (visual_residual_blocks_to_keep.count(residual_block_id) == 0u) {
        residual_information.active_ = false;
        ++rejected_residual_blocks;
      }
    }
  }

  // Mark the bad landmarks as invalid.
  for (const vi_map::LandmarkId& bad_landmark_id : bad_landmark_ids) {
    markLandmarkAsBad(bad_landmark_id);
  }

  LOG(INFO) << "Done outlier rejection. Removed " << rejected_residual_blocks
            << " residuals out of " << num_visual_residual_blocks;
}

void GraphBaOptimizer::addIterationCallback(
    std::function<void(const vi_map::VIMap&)> callback) {
  typedef std::function<void(const vi_map::VIMap&)> CallbackType;
  std::shared_ptr<ceres::IterationCallback> posegraph_callback(
      new GraphIterationCallback<CallbackType>(callback, const_map_, this));
  CHECK(callback);
  solver_callbacks_.push_back(posegraph_callback);
}

void GraphBaOptimizer::addPosePriorResidualBlocks(
    const pose_graph::VertexIdSet& vertex_ids,
    const double prior_position_std_dev_meters,
    const double prior_orientation_std_dev_radians) {
  VLOG(3) << "Adding pose prior term residual blocks for " << vertex_ids.size()
          << " vertices.";
  CHECK_GT(const_map_.numVertices(), 0u) << "No vertices on the posegraph";

  Eigen::Matrix<double, 6, 6> prior_covariance =
      Eigen::Matrix<double, 6, 6>::Identity();

  prior_covariance.block<3, 3>(0, 0) *=
      prior_orientation_std_dev_radians * prior_orientation_std_dev_radians;
  prior_covariance.block<3, 3>(3, 3) *=
      prior_position_std_dev_meters * prior_position_std_dev_meters;

  for (const pose_graph::VertexId& vertex_id : vertex_ids) {
    VertexIdPoseIdxMap::const_iterator it;
    it = vertex_id_to_pose_idx_.find(vertex_id);
    CHECK(it != vertex_id_to_pose_idx_.end());
    CHECK_GE(it->second, 0);
    CHECK_LT(it->second, vertex_poses_.cols());

    std::shared_ptr<ceres::CostFunction> cost_function(
        new ceres_error_terms::BlockPosePriorErrorTerm(
            vertex_poses_.col(it->second).head<4>(),
            vertex_poses_.col(it->second).tail<3>(), prior_covariance));
    problem_information_.addResidualBlock(
        ceres_error_terms::ResidualType::kPosePrior, cost_function, nullptr,
        {vertex_poses_.col(it->second).data()});
    problem_information_.setParameterizationNonChecked(
        vertex_poses_.col(it->second).data(), pose_parameterization_);
  }
}

void GraphBaOptimizer::addPosePriorOnCameraExtrinsics(
    const double prior_position_std_dev_meters,
    const double prior_orientation_std_dev_radians) {
  Eigen::Matrix<double, 6, 6> prior_covariance =
      Eigen::Matrix<double, 6, 6>::Identity();

  prior_covariance.block<3, 3>(0, 0) *=
      prior_orientation_std_dev_radians * prior_orientation_std_dev_radians;
  prior_covariance.block<3, 3>(3, 3) *=
      prior_position_std_dev_meters * prior_position_std_dev_meters;

  for (const CameraIdCameraIdxMap::value_type& camera_id_idx_pair :
       camera_id_to_T_C_I_idx_) {
    const aslam::CameraId camera_id = camera_id_idx_pair.first;
    CHECK(camera_id.isValid());
    const int index = camera_id_idx_pair.second;
    CHECK_GE(index, 0);
    CHECK_LT(index, T_C_I_JPL_.cols());

    std::shared_ptr<ceres::CostFunction> cost_function(
        new ceres_error_terms::PosePriorErrorTerm(
            T_C_I_JPL_.col(index).head<4>(), T_C_I_JPL_.col(index).tail<3>(),
            prior_covariance));
    problem_information_.addResidualBlock(
        ceres_error_terms::ResidualType::kPosePrior, cost_function, nullptr,
        {T_C_I_JPL_.col(index).data(), &T_C_I_JPL_(4, index)});
    CHECK(quaternion_parameterization_);
    problem_information_.setParameterizationNonChecked(
        T_C_I_JPL_.col(index).data(), quaternion_parameterization_);
  }
  VLOG(1) << "Added " << camera_id_to_T_C_I_idx_.size()
          << " pose prior residuals on camera extrinsics.";
}

void GraphBaOptimizer::addPosePriorOnOptionalSensorExtrinsics(
    const double prior_position_std_dev_meters,
    const double prior_orientation_std_dev_radians) {
  Eigen::Matrix<double, 6, 6> prior_covariance =
      Eigen::Matrix<double, 6, 6>::Identity();

  prior_covariance.block<3, 3>(0, 0) *=
      prior_orientation_std_dev_radians * prior_orientation_std_dev_radians;
  prior_covariance.block<3, 3>(3, 3) *=
      prior_position_std_dev_meters * prior_position_std_dev_meters;

  for (const SensorExtrinsicsIdxMap::value_type& sensor_extrinsics_id_idx_pair :
       sensor_id_to_extrinsics_col_idx_) {
    const size_t index = sensor_extrinsics_id_idx_pair.second;
    CHECK_LT(index, static_cast<size_t>(sensor_extrinsics_.cols()));

    std::shared_ptr<ceres::CostFunction> cost_function(
        new ceres_error_terms::PosePriorErrorTerm(
            sensor_extrinsics_.col(index).head<4>(),
            sensor_extrinsics_.col(index).tail<3>(), prior_covariance));

    problem_information_.addResidualBlock(
        ceres_error_terms::ResidualType::kPosePrior, cost_function, nullptr,
        {sensor_extrinsics_.col(index).data(),
         &sensor_extrinsics_(
             ceres_error_terms::poseblocks::kOrientationBlockSize, index)});
    CHECK(quaternion_parameterization_);
    problem_information_.setParameterization(
        sensor_extrinsics_.col(index).data(), quaternion_parameterization_);
  }
  VLOG(1) << "Added " << sensor_id_to_extrinsics_col_idx_.size()
          << " pose prior residuals on sensor extrinsics.";
}

void GraphBaOptimizer::addVelocityPriorResidualBlocks(
    const pose_graph::VertexIdSet& vertex_ids,
    const double prior_velocity_std_dev_meter_seconds) {
  VLOG(3) << "Adding velocity prior term residual blocks for "
          << vertex_ids.size() << " vertices.";
  CHECK_GT(const_map_.numVertices(), 0u) << "No vertices on the posegraph";

  Eigen::Matrix3d velocity_covariance = Eigen::Matrix3d::Identity();
  velocity_covariance *= prior_velocity_std_dev_meter_seconds *
                         prior_velocity_std_dev_meter_seconds;

  static constexpr int kParameterBlockSize = 3;
  static constexpr int kVelocityBlockIndex = 0;
  static constexpr int kVelocityBlockSize = 3;

  for (const pose_graph::VertexId& vertex_id : vertex_ids) {
    vi_map::Vertex& vertex = map_.getVertex(vertex_id);

    std::shared_ptr<ceres::CostFunction> cost_function(
        new ceres_error_terms::GenericPriorErrorTerm<
            kParameterBlockSize, kVelocityBlockIndex, kVelocityBlockSize>(
            vertex.get_v_M(), velocity_covariance));

    problem_information_.addResidualBlock(
        ceres_error_terms::ResidualType::kVelocityPrior, cost_function, nullptr,
        {vertex.get_v_M_Mutable()});
  }
}

void GraphBaOptimizer::addDecayingPosePriorResidualBlocks(
    const DoubleWindow& double_window,
    const double prior_position_std_dev_meters,
    const double prior_orientation_std_dev_radians,
    pose_graph::VertexIdSet* vertices_to_fix) {
  CHECK(vertices_to_fix);
  vertices_to_fix->clear();
  VLOG(3) << "Adding decaying pose prior term residual blocks for "
          << double_window.getOuterWindowVertices().size() << " vertices.";

  CHECK_GT(const_map_.numVertices(), 0u) << "No vertices on the posegraph";

  Eigen::Matrix<double, 6, 6> prior_covariance =
      Eigen::Matrix<double, 6, 6>::Identity();

  prior_covariance.block<3, 3>(0, 0) *=
      prior_orientation_std_dev_radians * prior_orientation_std_dev_radians;
  prior_covariance.block<3, 3>(3, 3) *=
      prior_position_std_dev_meters * prior_position_std_dev_meters;

  for (const pose_graph::VertexId& vertex_id :
       double_window.getOuterWindowVertices()) {
    VertexIdPoseIdxMap::const_iterator it;
    it = vertex_id_to_pose_idx_.find(vertex_id);
    CHECK(it != vertex_id_to_pose_idx_.end());
    CHECK_GE(it->second, 0);
    CHECK_LT(it->second, vertex_poses_.cols());
    double sq_distance_meters = double_window.getSquaredDistanceToInnerWindow(
        const_map_.getVertex_G_p_I(vertex_id));
    CHECK_GE(sq_distance_meters, 0.);
    CHECK_GT(FLAGS_dwo_decay_factor, 0.)
        << "Decay factor must be larger than 0!";

    double tmp_factor = 1. / (FLAGS_dwo_decay_factor * FLAGS_dwo_decay_factor);
    double cov_factor = exp(-sq_distance_meters * 0.25 * tmp_factor) * 3;
    if (cov_factor < 1.) {
      vertices_to_fix->insert(vertex_id);
      continue;
    }
    prior_covariance.block<3, 3>(0, 0) *= cov_factor;
    prior_covariance.block<3, 3>(3, 3) *= cov_factor;

    std::shared_ptr<ceres::CostFunction> cost_function(
        new ceres_error_terms::BlockPosePriorErrorTerm(
            vertex_poses_.col(it->second).head<4>(),
            vertex_poses_.col(it->second).tail<3>(), prior_covariance));

    problem_information_.addResidualBlock(
        ceres_error_terms::ResidualType::kPosePrior, cost_function, nullptr,
        {vertex_poses_.col(it->second).data()});
    problem_information_.setParameterizationNonChecked(
        vertex_poses_.col(it->second).data(), pose_parameterization_);
  }
}

void GraphBaOptimizer::addInertialResidualBlocks(
    bool fix_gyro_bias, bool fix_accel_bias, bool fix_velocity,
    bool use_given_edges, const pose_graph::EdgeIdList& provided_edges,
    bool store_imu_edge_covariances, const double gravity_magnitude,
    AlignedUnorderedMap<pose_graph::EdgeId, Eigen::Matrix<double, 6, 6>>*
        imu_edge_covariances) {
  if (store_imu_edge_covariances) {
    CHECK_NOTNULL(imu_edge_covariances);
  }
  VLOG(1) << "Adding inertial term residual blocks...";

  size_t num_inertial_residuals_added = 0;

  CHECK_GT(const_map_.numVertices(), 0u) << "No vertices on the posegraph";

  pose_graph::EdgeIdList edges;

  if (use_given_edges) {
    edges = provided_edges;
  } else {
    const_map_.getAllEdgeIds(&edges);
  }

  const vi_map::SensorManager& sensor_manger = const_map_.getSensorManager();

  common::ProgressBar progress_bar(edges.size());
  size_t edge_idx = 0;
  for (const pose_graph::EdgeId& edge_id : edges) {
    if (VLOG_IS_ON(2)) {
      progress_bar.update(++edge_idx);
    }
    if (const_map_.getEdgeType(edge_id) != pose_graph::Edge::EdgeType::kViwls) {
      continue;
    }
    const vi_map::ViwlsEdge& ba_edge =
        const_map_.getEdgeAs<vi_map::ViwlsEdge>(edge_id);

    vi_map::Vertex& vertex_from = map_.getVertex(ba_edge.from());
    vi_map::Vertex& vertex_to = map_.getVertex(ba_edge.to());

    const vi_map::MissionId& mission_id = vertex_from.getMissionId();
    CHECK(mission_id.isValid());

    VertexIdPoseIdxMap::const_iterator it_from, it_to;
    it_from = vertex_id_to_pose_idx_.find(vertex_from.id());
    it_to = vertex_id_to_pose_idx_.find(vertex_to.id());
    CHECK(it_from != vertex_id_to_pose_idx_.end());
    CHECK(it_to != vertex_id_to_pose_idx_.end());
    CHECK_GE(it_from->second, 0);
    CHECK_LT(it_from->second, vertex_poses_.cols());
    CHECK_GE(it_to->second, 0);
    CHECK_LT(it_to->second, vertex_poses_.cols());

    const vi_map::Imu& imu_sensor =
        sensor_manger.getSensorForMission<vi_map::Imu>(mission_id);
    const vi_map::ImuSigmas& imu_sigmas = imu_sensor.getImuSigmas();

    std::shared_ptr<ceres_error_terms::InertialErrorTerm> inertial_term_cost(
        new ceres_error_terms::InertialErrorTerm(
            ba_edge.getImuData(), ba_edge.getImuTimestamps(),
            imu_sigmas.gyro_noise_density,
            imu_sigmas.gyro_bias_random_walk_noise_density,
            imu_sigmas.acc_noise_density,
            imu_sigmas.acc_bias_random_walk_noise_density, gravity_magnitude));

    if (store_imu_edge_covariances) {
      std::pair<AlignedUnorderedMap<pose_graph::EdgeId,
                                    Eigen::Matrix<double, 6, 6>>::iterator,
                bool>
          it_success = imu_edge_covariances->insert(
              std::make_pair(edge_id, Eigen::Matrix<double, 6, 6>()));
      CHECK(it_success.second);
      Eigen::Matrix<double, 6, 6>& A_T_B_imu_covariance =
          it_success.first->second;
      inertial_term_cost->setCachedImuCovariancePointer(&A_T_B_imu_covariance);
    }

    problem_information_.addResidualBlock(
        ceres_error_terms::ResidualType::kInertial, inertial_term_cost, nullptr,
        {vertex_poses_.col(it_from->second).data(),
         vertex_from.getGyroBiasMutable(), vertex_from.get_v_M_Mutable(),
         vertex_from.getAccelBiasMutable(),
         vertex_poses_.col(it_to->second).data(),
         vertex_to.getGyroBiasMutable(), vertex_to.get_v_M_Mutable(),
         vertex_to.getAccelBiasMutable()});

    problem_information_.setParameterizationNonChecked(
        vertex_poses_.col(it_from->second).data(), pose_parameterization_);
    problem_information_.setParameterizationNonChecked(
        vertex_poses_.col(it_to->second).data(), pose_parameterization_);

    if (fix_gyro_bias) {
      problem_information_.setParameterBlockConstant(
          vertex_to.getGyroBiasMutable());
      problem_information_.setParameterBlockConstant(
          vertex_from.getGyroBiasMutable());
    }

    if (fix_accel_bias) {
      problem_information_.setParameterBlockConstant(
          vertex_to.getAccelBiasMutable());
      problem_information_.setParameterBlockConstant(
          vertex_from.getAccelBiasMutable());
    }

    if (fix_velocity) {
      problem_information_.setParameterBlockConstant(
          vertex_to.get_v_M_Mutable());
    }

    ++num_inertial_residuals_added;
  }
  VLOG(1) << "Added " << num_inertial_residuals_added << " inertial residuals.";
}

void GraphBaOptimizer::addPositionOnlyGPSResidualBlock(
    pose_graph::Edge::EdgeType edge_type, bool use_given_edges,
    const pose_graph::EdgeIdList& provided_edges) {
  CHECK(edge_type == pose_graph::Edge::EdgeType::k6DoFGps)
      << "The given edge type is not gps!";

  VLOG(1) << "Adding position only relative residual blocks.";

  CHECK_GT(const_map_.numVertices(), 0u) << "No vertices on the posegraph.";

  pose_graph::EdgeIdList edges;

  if (!use_given_edges) {
    const_map_.getAllEdgeIds(&edges);
  } else {
    edges = provided_edges;
  }

  size_t num_residual_blocks_added = 0u;

  for (const pose_graph::EdgeId& edge_id : edges) {
    const pose_graph::Edge* edge =
        const_map_.getEdgePtrAs<vi_map::Edge>(edge_id);
    CHECK(edge);
    if (edge->getType() != edge_type) {
      continue;
    }

    const vi_map::Vertex& vertex_from = map_.getVertex(edge->from());
    const vi_map::Vertex& vertex_to = map_.getVertex(edge->to());

    CHECK_EQ(vertex_from.getMissionId(), vertex_to.getMissionId())
        << "The two vertices this edge connects don't belong to the same "
        << "mission.";

    const vi_map::TransformationEdge& transformation_edge =
        edge->getAs<vi_map::TransformationEdge>();

    VertexIdPoseIdxMap::const_iterator it_from, it_to;
    it_from = vertex_id_to_pose_idx_.find(vertex_from.id());
    it_to = vertex_id_to_pose_idx_.find(vertex_to.id());
    CHECK(it_from != vertex_id_to_pose_idx_.end());
    CHECK(it_to != vertex_id_to_pose_idx_.end());
    CHECK_GE(it_from->second, 0);
    CHECK_LT(it_from->second, vertex_poses_.cols());
    CHECK_GE(it_to->second, 0);
    CHECK_LT(it_to->second, vertex_poses_.cols());

    std::shared_ptr<ceres::CostFunction> cost_function(
        new ceres_error_terms::PositionErrorTerm(
            transformation_edge.getT_A_B().getPosition(),
            transformation_edge.get_T_A_B_Covariance_p_q()
                .topLeftCorner<3, 3>()));

    problem_information_.addResidualBlock(
        ceres_error_terms::ResidualType::k3DoFGPS, cost_function, nullptr,
        {vertex_poses_.col(it_to->second).data()});

    problem_information_.setParameterizationNonChecked(
        vertex_poses_.col(it_from->second).data(), pose_parameterization_);
    problem_information_.setParameterizationNonChecked(
        vertex_poses_.col(it_to->second).data(), pose_parameterization_);

    ++num_residual_blocks_added;
  }
  VLOG(3) << "Added " << num_residual_blocks_added << " position only error "
          << "terms";
}

void GraphBaOptimizer::addLoopClosureEdges(
    bool use_given_edges, const pose_graph::EdgeIdList& provided_edges,
    const bool use_switchable_constraints, const double cauchy_loss) {
  pose_graph::EdgeIdList edges;

  if (!use_given_edges) {
    const_map_.getAllEdgeIds(&edges);
  } else {
    edges = provided_edges;
  }

  size_t num_residual_blocks_added = 0u;

  for (const pose_graph::EdgeId& edge_id : edges) {
    pose_graph::Edge* edge =
        CHECK_NOTNULL(map_.getEdgePtrAs<vi_map::Edge>(edge_id));
    if (edge->getType() != pose_graph::Edge::EdgeType::kLoopClosure) {
      continue;
    }

    vi_map::Vertex& vertex_from = map_.getVertex(edge->from());
    vi_map::Vertex& vertex_to = map_.getVertex(edge->to());
    CHECK_NE(vertex_from.id(), vertex_to.id());

    vi_map::LoopClosureEdge& loop_closure_edge =
        edge->getAs<vi_map::LoopClosureEdge>();
    const aslam::Transformation& T_A_B = loop_closure_edge.getT_A_B();
    const aslam::TransformationCovariance& T_A_B_covariance =
        loop_closure_edge.getT_A_BCovariance();

    VertexIdPoseIdxMap::const_iterator vertex_from_iterator, vertex_to_iterator;
    vertex_from_iterator = vertex_id_to_pose_idx_.find(vertex_from.id());
    vertex_to_iterator = vertex_id_to_pose_idx_.find(vertex_to.id());
    CHECK(vertex_from_iterator != vertex_id_to_pose_idx_.end());
    CHECK(vertex_to_iterator != vertex_id_to_pose_idx_.end());
    CHECK_GE(vertex_from_iterator->second, 0);
    CHECK_LT(vertex_from_iterator->second, vertex_poses_.cols());
    CHECK_GE(vertex_to_iterator->second, 0);
    CHECK_LT(vertex_to_iterator->second, vertex_poses_.cols());

    MissionBaseFrameIdBaseFrameIdxMap::const_iterator baseframe_from_iterator,
        baseframe_to_iterator;

    const vi_map::MissionBaseFrameId& baseframe_id_from =
        map_.getMissionBaseFrameForVertex(vertex_from.id()).id();
    CHECK(baseframe_id_from.isValid());
    baseframe_from_iterator =
        baseframe_id_to_baseframe_idx_.find(baseframe_id_from);
    CHECK(baseframe_from_iterator != baseframe_id_to_baseframe_idx_.end());
    CHECK_LT(baseframe_from_iterator->second, baseframe_poses_.cols());

    const vi_map::MissionBaseFrameId& baseframe_id_to =
        map_.getMissionBaseFrameForVertex(vertex_to.id()).id();
    CHECK(baseframe_id_to.isValid());
    baseframe_to_iterator =
        baseframe_id_to_baseframe_idx_.find(baseframe_id_to);
    CHECK(baseframe_to_iterator != baseframe_id_to_baseframe_idx_.end());
    CHECK_LT(baseframe_to_iterator->second, baseframe_poses_.cols());

    const bool vertices_are_in_same_mission =
        vertex_from.getMissionId() == vertex_to.getMissionId();

    // We only add one baseframe transformation for both vertices if
    // they are part of the same mission.
    using ceres_error_terms::LoopClosureBlockPoseErrorTerm;
    if (vertices_are_in_same_mission) {
      std::shared_ptr<ceres::CostFunction> loop_closure_cost(
          new ceres::AutoDiffCostFunction<
              LoopClosureBlockPoseErrorTerm,
              LoopClosureBlockPoseErrorTerm::residualBlockSize,
              ceres_error_terms::poseblocks::kPoseSize,
              ceres_error_terms::poseblocks::kPoseSize,
              LoopClosureBlockPoseErrorTerm::switchVariableBlockSize>(
              new LoopClosureBlockPoseErrorTerm(T_A_B, T_A_B_covariance)));
      if (use_switchable_constraints) {
        problem_information_.addResidualBlock(
            ceres_error_terms::ResidualType::kLoopClosure, loop_closure_cost,
            NULL, {vertex_poses_.col(vertex_from_iterator->second).data(),
                   vertex_poses_.col(vertex_to_iterator->second).data(),
                   loop_closure_edge.getSwitchVariableMutable()});
      } else {
        std::shared_ptr<ceres::LossFunction> loss_function;
        if (cauchy_loss > 0.0) {
          loss_function.reset(new ceres::CauchyLoss(cauchy_loss));
        }
        problem_information_.addResidualBlock(
            ceres_error_terms::ResidualType::kLoopClosure, loop_closure_cost,
            loss_function,
            {vertex_poses_.col(vertex_from_iterator->second).data(),
             vertex_poses_.col(vertex_to_iterator->second).data(),
             loop_closure_edge.getSwitchVariableMutable()});
        problem_information_.setParameterBlockConstant(
            loop_closure_edge.getSwitchVariableMutable());
      }
    } else {
      std::shared_ptr<ceres::CostFunction> loop_closure_cost(
          new ceres::AutoDiffCostFunction<
              LoopClosureBlockPoseErrorTerm,
              LoopClosureBlockPoseErrorTerm::residualBlockSize,
              ceres_error_terms::poseblocks::kPoseSize,
              ceres_error_terms::poseblocks::kPoseSize,
              ceres_error_terms::poseblocks::kPoseSize,
              ceres_error_terms::poseblocks::kPoseSize,
              LoopClosureBlockPoseErrorTerm::switchVariableBlockSize>(
              new LoopClosureBlockPoseErrorTerm(
                  loop_closure_edge.getT_A_B(),
                  loop_closure_edge.getT_A_BCovariance())));
      if (use_switchable_constraints) {
        problem_information_.addResidualBlock(
            ceres_error_terms::ResidualType::kLoopClosure, loop_closure_cost,
            nullptr,
            {baseframe_poses_.col(baseframe_from_iterator->second).data(),
             vertex_poses_.col(vertex_from_iterator->second).data(),
             baseframe_poses_.col(baseframe_to_iterator->second).data(),
             vertex_poses_.col(vertex_to_iterator->second).data(),
             loop_closure_edge.getSwitchVariableMutable()});
      } else {
        std::shared_ptr<ceres::LossFunction> loss_function;
        if (cauchy_loss > 0) {
          loss_function.reset(new ceres::CauchyLoss(cauchy_loss));
        }
        problem_information_.addResidualBlock(
            ceres_error_terms::ResidualType::kLoopClosure, loop_closure_cost,
            loss_function,
            {baseframe_poses_.col(baseframe_from_iterator->second).data(),
             vertex_poses_.col(vertex_from_iterator->second).data(),
             baseframe_poses_.col(baseframe_to_iterator->second).data(),
             vertex_poses_.col(vertex_to_iterator->second).data(),
             loop_closure_edge.getSwitchVariableMutable()});
        problem_information_.setParameterBlockConstant(
            loop_closure_edge.getSwitchVariableMutable());
      }

      problem_information_.setParameterizationNonChecked(
          baseframe_poses_.col(baseframe_from_iterator->second).data(),
          pose_parameterization_);
      problem_information_.setParameterizationNonChecked(
          baseframe_poses_.col(baseframe_to_iterator->second).data(),
          pose_parameterization_);
    }
    problem_information_.setParameterizationNonChecked(
        vertex_poses_.col(vertex_from_iterator->second).data(),
        pose_parameterization_);
    problem_information_.setParameterizationNonChecked(
        vertex_poses_.col(vertex_to_iterator->second).data(),
        pose_parameterization_);

    if (use_switchable_constraints) {
      const double kSwitchPrior = 1.0;
      const double kSwitchVariableVarianceSq =
          loop_closure_edge.getSwitchVariableVariance() *
          loop_closure_edge.getSwitchVariableVariance();

      using ceres_error_terms::SwitchPriorErrorTermLegacy;
      std::shared_ptr<ceres::CostFunction> switch_variable_cost(
          new ceres::AutoDiffCostFunction<
              SwitchPriorErrorTermLegacy,
              SwitchPriorErrorTermLegacy::residualBlockSize,
              SwitchPriorErrorTermLegacy::switchVariableBlockSize>(
              new SwitchPriorErrorTermLegacy(
                  kSwitchPrior, kSwitchVariableVarianceSq)));
      problem_information_.addResidualBlock(
          ceres_error_terms::ResidualType::kSwitchVariable,
          switch_variable_cost, NULL,
          {loop_closure_edge.getSwitchVariableMutable()});
    }

    ++num_residual_blocks_added;
  }
  VLOG(1) << "Added " << num_residual_blocks_added
          << " loop-closure error terms.";
}

void GraphBaOptimizer::addRelativePoseResidualBlocks(
    pose_graph::Edge::EdgeType edge_type, bool use_given_edges,
    const pose_graph::EdgeIdList& provided_edges, bool fix_extrinsics) {
  std::string edge_type_str;
  ceres_error_terms::ResidualType residual_type;

  if (edge_type == pose_graph::Edge::EdgeType::kOdometry) {
    edge_type_str = "odometry";
    residual_type = ceres_error_terms::ResidualType::kOdometry;
  } else if (edge_type == pose_graph::Edge::EdgeType::k6DoFGps) {
    edge_type_str = "gps";
    residual_type = ceres_error_terms::ResidualType::k6DoFGPS;
  } else {
    LOG(FATAL) << "The given edge_type is neither of type odometry nor gps.";
  }
  VLOG(1) << "Adding " << edge_type_str << " term residual blocks...";

  CHECK_GT(const_map_.numVertices(), 0u) << "No vertices on the posegraph.";

  pose_graph::EdgeIdList edges;

  if (!use_given_edges) {
    const_map_.getAllEdgeIds(&edges);
  } else {
    edges = provided_edges;
  }

  size_t num_residual_blocks_added_with_extrinsics = 0u;
  size_t num_residual_blocks_added_without_extrinsics = 0u;

  for (const pose_graph::EdgeId& edge_id : edges) {
    const pose_graph::Edge* edge =
        const_map_.getEdgePtrAs<vi_map::Edge>(edge_id);
    CHECK(edge);
    if (edge->getType() != edge_type) {
      continue;
    }

    vi_map::Vertex& vertex_from = map_.getVertex(edge->from());
    vi_map::Vertex& vertex_to = map_.getVertex(edge->to());

    CHECK(vertex_from.getMissionId() == vertex_to.getMissionId())
        << "The two vertices this edge connects don't belong to the same "
           "mission.";

    const vi_map::TransformationEdge& transformation_edge =
        edge->getAs<vi_map::TransformationEdge>();
    const aslam::Transformation T_A_B = transformation_edge.getT_A_B();
    const aslam::TransformationCovariance T_A_B_covariance =
        transformation_edge.get_T_A_B_Covariance_p_q();

    VertexIdPoseIdxMap::const_iterator it_from, it_to;
    it_from = vertex_id_to_pose_idx_.find(vertex_from.id());
    it_to = vertex_id_to_pose_idx_.find(vertex_to.id());
    CHECK(it_from != vertex_id_to_pose_idx_.end());
    CHECK(it_to != vertex_id_to_pose_idx_.end());
    CHECK_GE(it_from->second, 0);
    CHECK_LT(it_from->second, vertex_poses_.cols());
    CHECK_GE(it_to->second, 0);
    CHECK_LT(it_to->second, vertex_poses_.cols());

    // Account for optional sensor extrinsics.
    const vi_map::SensorId& sensor_id = transformation_edge.getSensorId();
    if (sensor_id.isValid()) {
      // Figure out which optional sensor extrinsics belongs to the current
      // transformation edge.
      std::shared_ptr<ceres::CostFunction> relative_pose_cost(
          new ceres::AutoDiffCostFunction<
              ceres_error_terms::SixDoFBlockPoseErrorTermWithExtrinsics,
              ceres_error_terms::SixDoFBlockPoseErrorTermWithExtrinsics::
                  kResidualBlockSize,
              ceres_error_terms::poseblocks::kPoseSize,
              ceres_error_terms::poseblocks::kPoseSize,
              ceres_error_terms::poseblocks::kOrientationBlockSize,
              ceres_error_terms::poseblocks::kPositionBlockSize>(
              new ceres_error_terms::SixDoFBlockPoseErrorTermWithExtrinsics(
                  T_A_B, T_A_B_covariance)));

      SensorExtrinsicsIdxMap::const_iterator
          sensor_extrinsics_col_index_iterator =
              sensor_id_to_extrinsics_col_idx_.find(sensor_id);
      CHECK(
          sensor_extrinsics_col_index_iterator !=
          sensor_id_to_extrinsics_col_idx_.end())
          << "Unable to find sensor with id " << sensor_id.hexString();

      const size_t sensor_extrinsics_col_index =
          sensor_extrinsics_col_index_iterator->second;
      CHECK_LT(
          static_cast<int>(sensor_extrinsics_col_index),
          sensor_extrinsics_.cols());

      problem_information_.addResidualBlock(
          residual_type, relative_pose_cost, nullptr,
          {vertex_poses_.col(it_from->second).data(),
           vertex_poses_.col(it_to->second).data(),
           sensor_extrinsics_.col(sensor_extrinsics_col_index).data(),
           &sensor_extrinsics_(
               ceres_error_terms::poseblocks::kOrientationBlockSize,
               sensor_extrinsics_col_index)});
      problem_information_.setParameterizationNonChecked(
          sensor_extrinsics_.col(sensor_extrinsics_col_index).data(),
          quaternion_parameterization_);

      if (fix_extrinsics) {
        problem_information_.setParameterBlockConstant(
            sensor_extrinsics_.col(sensor_extrinsics_col_index).data());
        problem_information_.setParameterBlockConstant(
            &sensor_extrinsics_(
                ceres_error_terms::poseblocks::kOrientationBlockSize,
                sensor_extrinsics_col_index));
      }
      ++num_residual_blocks_added_with_extrinsics;
    } else {
      std::shared_ptr<ceres::CostFunction> relative_pose_cost(
          new ceres::AutoDiffCostFunction<
              ceres_error_terms::SixDoFBlockPoseErrorTerm,
              ceres_error_terms::SixDoFBlockPoseErrorTerm::residualBlockSize,
              ceres_error_terms::poseblocks::kPoseSize,
              ceres_error_terms::poseblocks::kPoseSize>(
              new ceres_error_terms::SixDoFBlockPoseErrorTerm(
                  T_A_B, T_A_B_covariance)));
      problem_information_.addResidualBlock(
          residual_type, relative_pose_cost, nullptr,
          {vertex_poses_.col(it_from->second).data(),
           vertex_poses_.col(it_to->second).data()});
      ++num_residual_blocks_added_without_extrinsics;
    }
    problem_information_.setParameterizationNonChecked(
        vertex_poses_.col(it_from->second).data(), pose_parameterization_);
    problem_information_.setParameterizationNonChecked(
        vertex_poses_.col(it_to->second).data(), pose_parameterization_);
  }

  VLOG(1) << "Added " << num_residual_blocks_added_with_extrinsics
          << " relative pose error terms with and "
          << num_residual_blocks_added_without_extrinsics
          << " without extrinsics.";
}

void GraphBaOptimizer::removeLandmarksBehindCamera() {
  LOG(INFO) << "Removing landmarks that are still located behind the camera...";
  CHECK_GT(const_map_.numVertices(), 0u) << "No vertices on the posegraph";

  pose_graph::VertexIdList all_vertices;
  const_map_.getAllVertexIds(&all_vertices);

  LOG(INFO) << "Landmark count before: " << const_map_.numLandmarks();

  for (const pose_graph::VertexId& vertex_id : all_vertices) {
    vi_map::Vertex& ba_vertex = map_.getVertex(vertex_id);

    const unsigned int num_frames = ba_vertex.numFrames();
    for (unsigned int frame_idx = 0; frame_idx < num_frames; ++frame_idx) {
      if (ba_vertex.isVisualFrameSet(frame_idx) &&
          ba_vertex.isVisualFrameValid(frame_idx)) {
        const Eigen::Matrix2Xd& image_points_distorted =
            ba_vertex.getVisualFrame(frame_idx).getKeypointMeasurements();

        VertexIdPoseIdxMap::const_iterator vertex_it;
        vertex_it = vertex_id_to_pose_idx_.find(ba_vertex.id());
        CHECK(vertex_it != vertex_id_to_pose_idx_.end());
        CHECK_GE(vertex_it->second, 0);
        CHECK_LT(vertex_it->second, vertex_poses_.cols());

        for (int j = 0; j < image_points_distorted.cols(); ++j) {
          vi_map::LandmarkId landmark_id =
              ba_vertex.getObservedLandmarkId(frame_idx, j);

          if (landmark_id.isValid()) {
            CHECK(const_map_.hasLandmark(landmark_id))
                << "Valid landmark ID " << landmark_id.hexString()
                << " could not be retrieved.";

            const Eigen::Vector3d C_p_fi = const_map_.getLandmark_p_C_fi(
                landmark_id, ba_vertex, frame_idx);

            if (C_p_fi[2] <= 0) {
              markLandmarkAsBad(landmark_id);
            }
          }
        }
      }
    }  // Loop over frames in a vertex.
  }    // Loop over vertices.
  LOG(INFO) << "Landmarks count after: " << const_map_.numLandmarks();
}

void GraphBaOptimizer::markLandmarkAsBad(
    const vi_map::LandmarkId& landmark_id) {
  map_.getLandmark(landmark_id).setQuality(vi_map::Landmark::Quality::kBad);
}

bool GraphBaOptimizer::addVisualResidualBlockOfKeypoint(
    const Eigen::Matrix<double, 2, 1>& image_point_distorted,
    double image_point_uncertainty, const vi_map::LandmarkId& landmark_id,
    bool fix_landmark_positions,
    unsigned int /*min_observer_vertices_per_landmark*/,
    unsigned int /*min_landmarks_per_frame*/,
    unsigned int min_num_observer_missions,
    std::shared_ptr<ceres::LossFunction> loss_function,
    aslam::Camera* camera_ptr, unsigned int* external_landmarks_added,
    vi_map::Vertex* ba_vertex) {
  CHECK_NOTNULL(ba_vertex);
  CHECK_NOTNULL(camera_ptr);
  CHECK_NOTNULL(external_landmarks_added);

  // Invalid landmark_id means that the keypoint is not actually associated
  // to an existing landmark object.
  if (!landmark_id.isValid()) {
    return false;
  }

  // loss_function may explicitly be nullptr
  vi_map::VIMission& mission = map_.getMission(ba_vertex->getMissionId());

  const aslam::CameraId& camera_id = camera_ptr->getId();
  CHECK(camera_id.isValid());

  VertexIdPoseIdxMap::const_iterator vertex_it;
  vertex_it = vertex_id_to_pose_idx_.find(ba_vertex->id());
  CHECK(vertex_it != vertex_id_to_pose_idx_.end());
  CHECK_GE(vertex_it->second, 0);
  CHECK_LT(vertex_it->second, vertex_poses_.cols());

  BaOptimizationOptions ba_options;
  std::shared_ptr<ceres::LocalParameterization>
      baseframe_pose_parameterization = pose_parameterization_;
  if (FLAGS_optimizer_yaw_only_baseframe_parametrization) {
    if (yaw_only_pose_parameterization_ == nullptr) {
      yaw_only_pose_parameterization_.reset(
          new ceres_error_terms::JplYawOnlyPoseParameterization);
    }
    baseframe_pose_parameterization = yaw_only_pose_parameterization_;
  }

  if (min_num_observer_missions > 0u) {
    vi_map::MissionIdSet observer_missions;
    map_.getLandmarkObserverMissions(landmark_id, &observer_missions);
    if (observer_missions.size() < min_num_observer_missions) {
      return false;
    }
  }

  vi_map::Vertex& landmark_base_vertex =
      map_.getLandmarkStoreVertex(landmark_id);
  vi_map::Landmark& landmark = map_.getLandmark(landmark_id);

  // Skip if the current landmark has not enough observers.
  if (!vi_map::isLandmarkWellConstrained(const_map_, landmark)) {
    return false;
  }

  VertexIdPoseIdxMap::const_iterator landmark_base_vertex_it;
  landmark_base_vertex_it =
      vertex_id_to_pose_idx_.find(landmark_base_vertex.id());
  CHECK(landmark_base_vertex_it != vertex_id_to_pose_idx_.end());
  CHECK_GE(landmark_base_vertex_it->second, 0);
  CHECK_LT(landmark_base_vertex_it->second, vertex_poses_.cols());

  // Verify if the landmark base vertex is not actually equivalent to
  // the current keyframe pose.
  if (vertex_it->first != landmark_base_vertex_it->first) {
    // Verify if the landmark and keyframe belong to the same mission.
    if (ba_vertex->getMissionId() == landmark_base_vertex.getMissionId()) {
      std::shared_ptr<ceres::CostFunction> visual_term_cost(
          ceres_error_terms::createVisualCostFunction<
              ceres_error_terms::VisualReprojectionError>(
              image_point_distorted, image_point_uncertainty,
              ceres_error_terms::visual::VisualErrorType::kLocalMission,
              camera_ptr));
      if (camera_ptr->getDistortion().getType() !=
          aslam::Distortion::Type::kNoDistortion) {
        problem_information_.addResidualBlock(
            ceres_error_terms::ResidualType::kVisualReprojectionError,
            visual_term_cost, loss_function,
            {landmark.get_p_B_Mutable(),
             vertex_poses_.col(landmark_base_vertex_it->second).data(),
             dummy_7d_2_.data(), dummy_7d_3_.data(),
             vertex_poses_.col(vertex_it->second).data(),
             get_q_C_I_JPL_Mutable(camera_id), get_p_C_I_Mutable(camera_id),
             camera_ptr->getParametersMutable(),
             camera_ptr->getDistortionMutable()->getParametersMutable()});
        residual_to_landmark[visual_term_cost.get()] = landmark_id;
      } else {
        problem_information_.addResidualBlock(
            ceres_error_terms::ResidualType::kVisualReprojectionError,
            visual_term_cost, loss_function,
            {landmark.get_p_B_Mutable(),
             vertex_poses_.col(landmark_base_vertex_it->second).data(),
             dummy_7d_2_.data(), dummy_7d_3_.data(),
             vertex_poses_.col(vertex_it->second).data(),
             get_q_C_I_JPL_Mutable(camera_id), get_p_C_I_Mutable(camera_id),
             camera_ptr->getParametersMutable()});
        residual_to_landmark[visual_term_cost.get()] = landmark_id;
      }
      problem_information_.setParameterBlockConstant(dummy_7d_2_.data());
      problem_information_.setParameterBlockConstant(dummy_7d_3_.data());
    } else {
      std::shared_ptr<ceres::CostFunction> visual_term_cost(
          ceres_error_terms::createVisualCostFunction<
              ceres_error_terms::VisualReprojectionError>(
              image_point_distorted, image_point_uncertainty,
              ceres_error_terms::visual::VisualErrorType::kGlobal, camera_ptr));

      // Retrieve mission for the landmark vertex.
      vi_map::VIMission& landmark_mission =
          map_.getMission(landmark_base_vertex.getMissionId());

      // Find baseframes for both missions.
      MissionBaseFrameIdBaseFrameIdxMap::const_iterator landmark_baseframe_it;
      MissionBaseFrameIdBaseFrameIdxMap::const_iterator vertex_baseframe_it;

      vertex_baseframe_it =
          baseframe_id_to_baseframe_idx_.find(mission.getBaseFrameId());
      CHECK(vertex_baseframe_it != baseframe_id_to_baseframe_idx_.end());
      landmark_baseframe_it = baseframe_id_to_baseframe_idx_.find(
          landmark_mission.getBaseFrameId());
      CHECK(landmark_baseframe_it != baseframe_id_to_baseframe_idx_.end());

      if (camera_ptr->getDistortion().getType() !=
          aslam::Distortion::Type::kNoDistortion) {
        problem_information_.addResidualBlock(
            ceres_error_terms::ResidualType::kVisualReprojectionError,
            visual_term_cost, loss_function,
            {landmark.get_p_B_Mutable(),
             vertex_poses_.col(landmark_base_vertex_it->second).data(),
             baseframe_poses_.col(landmark_baseframe_it->second).data(),
             baseframe_poses_.col(vertex_baseframe_it->second).data(),
             vertex_poses_.col(vertex_it->second).data(),
             get_q_C_I_JPL_Mutable(camera_id), get_p_C_I_Mutable(camera_id),
             camera_ptr->getParametersMutable(),
             camera_ptr->getDistortionMutable()->getParametersMutable()});
        residual_to_landmark[visual_term_cost.get()] = landmark_id;
      } else {
        problem_information_.addResidualBlock(
            ceres_error_terms::ResidualType::kVisualReprojectionError,
            visual_term_cost, loss_function,
            {landmark.get_p_B_Mutable(),
             vertex_poses_.col(landmark_base_vertex_it->second).data(),
             baseframe_poses_.col(landmark_baseframe_it->second).data(),
             baseframe_poses_.col(vertex_baseframe_it->second).data(),
             vertex_poses_.col(vertex_it->second).data(),
             get_q_C_I_JPL_Mutable(camera_id), get_p_C_I_Mutable(camera_id),
             camera_ptr->getParametersMutable()});
        residual_to_landmark[visual_term_cost.get()] = landmark_id;
      }

      problem_information_.setParameterization(
          baseframe_poses_.col(landmark_baseframe_it->second).data(),
          baseframe_pose_parameterization);
      problem_information_.setParameterization(
          baseframe_poses_.col(vertex_baseframe_it->second).data(),
          baseframe_pose_parameterization);
    }
    problem_information_.setParameterization(
        vertex_poses_.col(landmark_base_vertex_it->second).data(),
        pose_parameterization_);

    ++(*external_landmarks_added);
  } else {
    std::shared_ptr<ceres::CostFunction> visual_term_cost(
        ceres_error_terms::createVisualCostFunction<
            ceres_error_terms::VisualReprojectionError>(
            image_point_distorted, image_point_uncertainty,
            ceres_error_terms::visual::VisualErrorType::kLocalKeyframe,
            camera_ptr));

    // For this error term we add dummy pointers for ceres for
    // the parameter blocks we don't use in order to have a single
    // error term for both parameterizations.
    if (camera_ptr->getDistortion().getType() !=
        aslam::Distortion::Type::kNoDistortion) {
      problem_information_.addResidualBlock(
          ceres_error_terms::ResidualType::kVisualReprojectionError,
          visual_term_cost, loss_function,
          {landmark.get_p_B_Mutable(), dummy_7d_0_.data(), dummy_7d_2_.data(),
           dummy_7d_3_.data(), dummy_7d_1_.data(),
           get_q_C_I_JPL_Mutable(camera_id), get_p_C_I_Mutable(camera_id),
           camera_ptr->getParametersMutable(),
           camera_ptr->getDistortionMutable()->getParametersMutable()});
      residual_to_landmark[visual_term_cost.get()] = landmark_id;
    } else {
      problem_information_.addResidualBlock(
          ceres_error_terms::ResidualType::kVisualReprojectionError,
          visual_term_cost, loss_function,
          {landmark.get_p_B_Mutable(), dummy_7d_0_.data(), dummy_7d_2_.data(),
           dummy_7d_3_.data(), dummy_7d_1_.data(),
           get_q_C_I_JPL_Mutable(camera_id), get_p_C_I_Mutable(camera_id),
           camera_ptr->getParametersMutable()});
      residual_to_landmark[visual_term_cost.get()] = landmark_id;
    }

    // We fix the dummy parameter blocks because they have no meaning.
    problem_information_.setParameterBlockConstant(dummy_7d_0_.data());
    problem_information_.setParameterBlockConstant(dummy_7d_1_.data());
    problem_information_.setParameterBlockConstant(dummy_7d_2_.data());
    problem_information_.setParameterBlockConstant(dummy_7d_3_.data());
  }

  if (fix_landmark_positions) {
    problem_information_.setParameterBlockConstant(
        landmark_base_vertex.getLandmarks().get_p_B_Mutable(landmark_id));
  }
  return true;
}

void GraphBaOptimizer::addVisualResidualBlocks(
    bool fix_intrinsics, bool fix_extrinsics_rotation,
    bool fix_extrinsics_translation, bool fix_landmark_positions,
    bool include_only_merged_landmarks,
    unsigned int min_observer_vertices_per_landmark,
    unsigned int min_landmarks_per_frame,
    unsigned int min_num_observer_missions) {
  addVisualResidualBlocks(
      fix_intrinsics, fix_extrinsics_rotation, fix_extrinsics_translation,
      fix_landmark_positions, include_only_merged_landmarks,
      min_observer_vertices_per_landmark, min_landmarks_per_frame,
      min_num_observer_missions, nullptr);
}

void GraphBaOptimizer::addVisualResidualBlocks(
    bool fix_intrinsics, bool fix_extrinsics_rotation,
    bool fix_extrinsics_translation, bool fix_landmark_positions,
    bool include_only_merged_landmarks,
    unsigned int min_observer_vertices_per_landmark,
    unsigned int min_landmarks_per_frame,
    unsigned int min_num_observer_missions,
    pose_graph::VertexIdSet* vertices_with_visual_residuals) {
  LOG(INFO) << "Adding visual term residual blocks...";
  CHECK_NE(0u, const_map_.numVertices()) << "No vertices on the posegraph";

  pose_graph::VertexIdList vertices;
  const_map_.getAllVertexIds(&vertices);

  VLOG(3) << "Before keyframes loop " << vertices.size() << " vertices.";

  size_t num_visual_residuals_added = 0u;
  size_t vertex_idx = 0;
  const size_t num_vertices = vertices.size();
  common::ProgressBar progress_bar(num_vertices);

  for (const pose_graph::VertexId& vertex_id : vertices) {
    size_t num_visual_residuals_added_for_this_vertex = 0u;

    vi_map::Vertex& ba_vertex = map_.getVertex(vertex_id);
    const unsigned int num_frames = ba_vertex.numFrames();
    for (unsigned int frame_idx = 0; frame_idx < num_frames; ++frame_idx) {
      if (!ba_vertex.isVisualFrameSet(frame_idx) ||
          !ba_vertex.isVisualFrameValid(frame_idx)) {
        continue;
      }
      const Eigen::Matrix2Xd& image_points_distorted =
          ba_vertex.getVisualFrame(frame_idx).getKeypointMeasurements();
      const Eigen::VectorXd& image_points_uncertainties =
          ba_vertex.getVisualFrame(frame_idx)
              .getKeypointMeasurementUncertainties();

      const aslam::Camera::Ptr camera_ptr = ba_vertex.getCamera(frame_idx);
      CHECK(camera_ptr != nullptr);

      unsigned int well_constrained_landmarks =
          getNumOfWellConstrainedLandmarksInFrame(ba_vertex, frame_idx);
      if (well_constrained_landmarks < min_landmarks_per_frame) {
        VLOG(3) << " Skipping this visual keyframe. Only "
                << well_constrained_landmarks
                << " well constrained landmarks, but "
                << min_landmarks_per_frame << " required";
        continue;
      }

      VertexIdPoseIdxMap::const_iterator vertex_it;
      vertex_it = vertex_id_to_pose_idx_.find(ba_vertex.id());
      CHECK(vertex_it != vertex_id_to_pose_idx_.end());
      CHECK_GE(vertex_it->second, 0);
      CHECK_LT(vertex_it->second, vertex_poses_.cols());

      unsigned int external_landmarks_added = 0;
      for (int i = 0; i < image_points_distorted.cols(); ++i) {
        std::shared_ptr<ceres::LossFunction> loss_function(
            new ceres::LossFunctionWrapper(
                new ceres::CauchyLoss(3.0 * image_points_uncertainties(i)),
                ceres::TAKE_OWNERSHIP));

        vi_map::LandmarkId landmark_id =
            ba_vertex.getObservedLandmarkId(frame_idx, i);

        bool include_this_landmark = true;
        if (include_only_merged_landmarks && landmark_id.isValid()) {
          const int num_observer_missions =
              map_.numLandmarkObserverMissions(landmark_id);
          CHECK_GT(num_observer_missions, 0);
          if (num_observer_missions < 2) {
            include_this_landmark = false;
          }
        }
        if (include_this_landmark &&
            addVisualResidualBlockOfKeypoint(
                image_points_distorted.col(i), image_points_uncertainties(i),
                landmark_id, fix_landmark_positions,
                min_observer_vertices_per_landmark, min_landmarks_per_frame,
                min_num_observer_missions, loss_function, camera_ptr.get(),
                &external_landmarks_added, &ba_vertex)) {
          ++num_visual_residuals_added;
          ++num_visual_residuals_added_for_this_vertex;
        }
      }

      if (image_points_distorted.cols() > 0) {
        setCameraParameterizationIfPartOfTheProblem(
            camera_ptr, fix_intrinsics, fix_extrinsics_rotation,
            fix_extrinsics_translation);
      }

      if (external_landmarks_added > 0) {
        problem_information_.setParameterization(
            vertex_poses_.col(vertex_it->second).data(),
            pose_parameterization_);
      }
      if (num_visual_residuals_added_for_this_vertex > 0u) {
        if (vertices_with_visual_residuals != nullptr) {
          vertices_with_visual_residuals->emplace(vertex_id);
        }
      }
    }  // Loop over all frames in a vertex.
    if (VLOG_IS_ON(2)) {
      progress_bar.update(++vertex_idx);
    }
  }  // Loop over all vertices.
  VLOG(1) << "Added " << num_visual_residuals_added << " visual residuals.";
}

void GraphBaOptimizer::addDoubleWindowVisualResidualBlocks(
    const DoubleWindow& double_window) {
  // Dummies for passing to function where not used.
  double kMinObserverVerticesPerLandmark = 0;
  double kMinLandmarksPerFrame = 0;
  unsigned int kMinNumberOfObsererMissionsPerLandmark = 0u;

  pose_graph::VertexIdSet both_windows_vertex_ids;
  double_window.getAllWindowVertices(&both_windows_vertex_ids);

  for (const pose_graph::VertexId& vertex_id : both_windows_vertex_ids) {
    vi_map::Vertex& ba_vertex = map_.getVertex(vertex_id);

    const unsigned int num_frames = ba_vertex.numFrames();
    for (unsigned int frame_idx = 0; frame_idx < num_frames; ++frame_idx) {
      if (!ba_vertex.isVisualFrameSet(frame_idx) ||
          !ba_vertex.isVisualFrameValid(frame_idx)) {
        continue;
      }
      const Eigen::Matrix2Xd& image_points_distorted =
          ba_vertex.getVisualFrame(frame_idx).getKeypointMeasurements();
      const Eigen::VectorXd& image_points_uncertainties =
          ba_vertex.getVisualFrame(frame_idx)
              .getKeypointMeasurementUncertainties();

      const aslam::Camera::Ptr camera_ptr = ba_vertex.getCamera(frame_idx);
      CHECK(camera_ptr != nullptr);

      VertexIdPoseIdxMap::const_iterator vertex_it;
      vertex_it = vertex_id_to_pose_idx_.find(ba_vertex.id());
      CHECK(vertex_it != vertex_id_to_pose_idx_.end());
      CHECK_GE(vertex_it->second, 0);
      CHECK_LT(vertex_it->second, vertex_poses_.cols());

      unsigned int external_landmarks_added = 0;
      for (int i = 0; i < image_points_distorted.cols(); ++i) {
        const Eigen::Matrix<double, 2, 1>& image_point_distorted =
            image_points_distorted.col(i);
        const double image_point_uncertainty = image_points_uncertainties(i);

        std::shared_ptr<ceres::LossFunction> loss_function(
            new ceres::LossFunctionWrapper(
                new ceres::HuberLoss(3.0 * image_point_uncertainty),
                ceres::TAKE_OWNERSHIP));

        const vi_map::LandmarkId landmark_id =
            ba_vertex.getObservedLandmarkId(frame_idx, i);
        static constexpr bool kFixLandmark = false;

        if (double_window.isLandmarkSeenFromInnerWindow(landmark_id)) {
          addVisualResidualBlockOfKeypoint(
              image_point_distorted, image_point_uncertainty, landmark_id,
              kFixLandmark, kMinObserverVerticesPerLandmark,
              kMinLandmarksPerFrame, kMinNumberOfObsererMissionsPerLandmark,
              loss_function, camera_ptr.get(), &external_landmarks_added,
              &ba_vertex);
        }
        if (image_points_distorted.cols() > 0) {
          setCameraParameterizationIfPartOfTheProblem(
              camera_ptr, true, true, true);
        }
        if (external_landmarks_added > 0) {
          problem_information_.setParameterization(
              vertex_poses_.col(vertex_it->second).data(),
              pose_parameterization_);
        }
      }
    }  // Loop over all frames in a vertex.
  }    // Loop over all vertices.
}

unsigned int GraphBaOptimizer::getNumOfWellConstrainedLandmarksInFrame(
    const vi_map::Vertex& ba_vertex, unsigned int frame_idx) {
  CHECK(ba_vertex.isVisualFrameSet(frame_idx));
  const Eigen::Matrix2Xd& image_points_distorted =
      ba_vertex.getVisualFrame(frame_idx).getKeypointMeasurements();

  // Count well-constrained observations for the current vertex.
  unsigned int well_constrained_landmarks = 0;
  for (int i = 0; i < image_points_distorted.cols(); ++i) {
    vi_map::LandmarkId landmark_id =
        ba_vertex.getObservedLandmarkId(frame_idx, i);
    if (landmark_id.isValid()) {
      CHECK(const_map_.hasLandmark(landmark_id));
      if (vi_map::isLandmarkWellConstrained(
              const_map_, const_map_.getLandmark(landmark_id))) {
        ++well_constrained_landmarks;
      }
    }
  }
  return well_constrained_landmarks;
}

void GraphBaOptimizer::setCameraParameterizationIfPartOfTheProblem(
    const aslam::Camera::Ptr& camera_ptr, bool fix_intrinsics,
    bool fix_extrinsics_rotation, bool fix_extrinsics_translation) {
  CHECK(camera_ptr) << "Null camera pointer!";

  const aslam::CameraId& camera_id = camera_ptr->getId();
  CHECK(camera_id.isValid());
  if (fix_extrinsics_rotation) {
    problem_information_.setParameterBlockConstant(
        get_q_C_I_JPL_Mutable(camera_id));
  }
  if (fix_extrinsics_translation) {
    problem_information_.setParameterBlockConstant(
        get_p_C_I_Mutable(camera_id));
  }
  problem_information_.setParameterization(
      get_q_C_I_JPL_Mutable(camera_id), quaternion_parameterization_);

  if (fix_intrinsics) {
    problem_information_.setParameterBlockConstant(
        camera_ptr->getParametersMutable());
  }
  if (camera_ptr->getDistortion().getType() !=
      aslam::Distortion::Type::kNoDistortion) {
    problem_information_.setParameterBlockConstant(
        camera_ptr->getDistortionMutable()->getParametersMutable());
  }
}

void GraphBaOptimizer::addMissionAlignResidualBlocks(
    const vi_map::MissionIdSet& missions) {
  CHECK_NE(0u, const_map_.numVertices()) << "No vertices on the posegraph";

  pose_graph::VertexIdList all_vertices;
  const_map_.getAllVertexIds(&all_vertices);

  const BaOptimizationOptions ba_options;
  LOG(INFO) << "Adding mission alignment residual blocks...";
  for (const pose_graph::VertexId& vertex_id : all_vertices) {
    vi_map::Vertex& ba_vertex = map_.getVertex(vertex_id);
    // Skip vertices that are not from the provided missions.
    if (missions.count(ba_vertex.getMissionId()) == 0) {
      continue;
    }

    vi_map::VIMission& mission = map_.getMission(ba_vertex.getMissionId());

    const unsigned int num_frames = ba_vertex.numFrames();
    for (unsigned int frame_idx = 0; frame_idx < num_frames; ++frame_idx) {
      if (ba_vertex.isVisualFrameSet(frame_idx) &&
          ba_vertex.isVisualFrameValid(frame_idx)) {
        const Eigen::Matrix2Xd& image_points_distorted =
            ba_vertex.getVisualFrame(frame_idx).getKeypointMeasurements();
        const Eigen::VectorXd& image_points_uncertainties =
            ba_vertex.getVisualFrame(frame_idx)
                .getKeypointMeasurementUncertainties();

        const aslam::Camera::Ptr camera_ptr = ba_vertex.getCamera(frame_idx);
        CHECK(camera_ptr != nullptr);

        const aslam::CameraId& camera_id = camera_ptr->getId();
        CHECK(camera_id.isValid());

        // Count well-constrained observations for the current vertex.
        int well_constrained_landmarks = 0;
        for (int i = 0; i < image_points_distorted.cols(); ++i) {
          vi_map::LandmarkId landmark_id =
              ba_vertex.getObservedLandmarkId(frame_idx, i);
          if (landmark_id.isValid()) {
            CHECK(const_map_.hasLandmark(landmark_id));
            if (vi_map::isLandmarkWellConstrained(
                    const_map_, const_map_.getLandmark(landmark_id))) {
              ++well_constrained_landmarks;
            }
            ++well_constrained_landmarks;
          }
        }
        if (static_cast<size_t>(well_constrained_landmarks) <
            ba_options.min_number_of_visible_landmarks_at_vertex) {
          VLOG(3) << " Skipping this visual keyframe. Only "
                  << well_constrained_landmarks
                  << " well constrained landmarks.";
          continue;
        }

        VertexIdPoseIdxMap::const_iterator vertex_it;
        vertex_it = vertex_id_to_pose_idx_.find(ba_vertex.id());
        CHECK(vertex_it != vertex_id_to_pose_idx_.end());
        CHECK_GE(vertex_it->second, 0);
        CHECK_LT(vertex_it->second, vertex_poses_.cols());

        unsigned int external_landmarks_added = 0;
        for (int i = 0; i < image_points_distorted.cols(); ++i) {
          vi_map::LandmarkId landmark_id =
              ba_vertex.getObservedLandmarkId(frame_idx, i);

          // Valid landmark_id means that the keypoint is actually associated
          // to an existing landmark object.
          if (landmark_id.isValid()) {
            CHECK(const_map_.hasLandmark(landmark_id));
            const vi_map::Landmark& landmark =
                const_map_.getLandmark(landmark_id);
            if (!vi_map::isLandmarkWellConstrained(const_map_, landmark)) {
              continue;
            }
            vi_map::Vertex& landmark_vertex =
                map_.getLandmarkStoreVertex(landmark_id);

            VertexIdPoseIdxMap::const_iterator landmark_vertex_it;
            landmark_vertex_it =
                vertex_id_to_pose_idx_.find(landmark_vertex.id());
            CHECK(landmark_vertex_it != vertex_id_to_pose_idx_.end());
            CHECK_GE(landmark_vertex_it->second, 0);
            CHECK_LT(landmark_vertex_it->second, vertex_poses_.cols());

            // Verify if the landmark base frame is not actually equivalent to
            // the current keyframe pose.
            if (vertex_it->first != landmark_vertex_it->first) {
              // Verify if the landmark and keyframe belong to the same mission.
              if (ba_vertex.getMissionId() != landmark_vertex.getMissionId()) {
                std::shared_ptr<ceres::CostFunction> visual_term_cost;
                ceres_error_terms::ResidualType residual_type =
                    ceres_error_terms::ResidualType::kVisualReprojectionError;
                visual_term_cost.reset(
                    ceres_error_terms::createVisualCostFunction<
                        ceres_error_terms::VisualReprojectionError>(
                        image_points_distorted.col(i),
                        image_points_uncertainties(i),
                        ceres_error_terms::visual::VisualErrorType::kGlobal,
                        camera_ptr.get()));
                residual_type =
                    ceres_error_terms::ResidualType::kVisualReprojectionError;

                // Retrieve mission for the landmark vertex.
                const vi_map::VIMission& landmark_mission =
                    const_map_.getMission(landmark_vertex.getMissionId());

                // Find baseframes for both missions.
                MissionBaseFrameIdBaseFrameIdxMap::const_iterator
                    landmark_baseframe_it;
                MissionBaseFrameIdBaseFrameIdxMap::const_iterator
                    vertex_baseframe_it;

                vertex_baseframe_it = baseframe_id_to_baseframe_idx_.find(
                    mission.getBaseFrameId());
                CHECK(
                    vertex_baseframe_it !=
                    baseframe_id_to_baseframe_idx_.end());
                landmark_baseframe_it = baseframe_id_to_baseframe_idx_.find(
                    landmark_mission.getBaseFrameId());
                CHECK(
                    landmark_baseframe_it !=
                    baseframe_id_to_baseframe_idx_.end());

                std::shared_ptr<ceres::LossFunction> loss_function(
                    new DefaultLossFunction(kBearingLossParameter));

                if (camera_ptr->getDistortion().getType() !=
                    aslam::Distortion::Type::kNoDistortion) {
                  problem_information_.addResidualBlock(
                      residual_type, visual_term_cost, loss_function,
                      {landmark_vertex.getLandmarks().get_p_B_Mutable(
                           landmark_id),
                       vertex_poses_.col(landmark_vertex_it->second).data(),
                       baseframe_poses_.col(landmark_baseframe_it->second)
                           .data(),
                       baseframe_poses_.col(vertex_baseframe_it->second).data(),
                       vertex_poses_.col(vertex_it->second).data(),
                       get_q_C_I_JPL_Mutable(camera_id),
                       get_p_C_I_Mutable(camera_id),
                       camera_ptr->getParametersMutable(),
                       camera_ptr->getDistortionMutable()
                           ->getParametersMutable()});
                } else {
                  problem_information_.addResidualBlock(
                      residual_type, visual_term_cost, loss_function,
                      {landmark_vertex.getLandmarks().get_p_B_Mutable(
                           landmark_id),
                       vertex_poses_.col(landmark_vertex_it->second).data(),
                       baseframe_poses_.col(landmark_baseframe_it->second)
                           .data(),
                       baseframe_poses_.col(vertex_baseframe_it->second).data(),
                       vertex_poses_.col(vertex_it->second).data(),
                       get_q_C_I_JPL_Mutable(camera_id),
                       get_p_C_I_Mutable(camera_id),
                       camera_ptr->getParametersMutable()});
                }

                problem_information_.setParameterization(
                    vertex_poses_.col(landmark_vertex_it->second).data(),
                    pose_parameterization_);
                problem_information_.setParameterBlockConstant(
                    vertex_poses_.col(landmark_vertex_it->second).data());

                problem_information_.setParameterBlockConstant(
                    landmark_vertex.getLandmarks().get_p_B_Mutable(
                        landmark_id));

                problem_information_.setParameterization(
                    baseframe_poses_.col(vertex_baseframe_it->second).data(),
                    pose_parameterization_);
                problem_information_.setParameterization(
                    baseframe_poses_.col(landmark_baseframe_it->second).data(),
                    pose_parameterization_);

                ++external_landmarks_added;
              }
            }
          }
        }

        if (external_landmarks_added > 0) {
          problem_information_.setParameterBlockConstant(
              get_q_C_I_JPL_Mutable(camera_id));
          problem_information_.setParameterBlockConstant(
              get_p_C_I_Mutable(camera_id));
          problem_information_.setParameterization(
              get_q_C_I_JPL_Mutable(camera_id), quaternion_parameterization_);

          problem_information_.setParameterBlockConstant(
              camera_ptr->getParametersMutable());
          if (camera_ptr->getDistortion().getType() !=
              aslam::Distortion::Type::kNoDistortion) {
            problem_information_.setParameterBlockConstant(
                camera_ptr->getDistortionMutable()->getParametersMutable());
          }

          problem_information_.setParameterBlockConstant(
              vertex_poses_.col(vertex_it->second).data());
          problem_information_.setParameterization(
              vertex_poses_.col(vertex_it->second).data(),
              pose_parameterization_);
        }
      }
    }  // Loop over all frames in a vertex.
  }    // Loop over all vertices.
}

void GraphBaOptimizer::fixBaseframes() {
  for (int col_idx = 0; col_idx < baseframe_poses_.cols(); ++col_idx) {
    problem_information_.setParameterBlockConstant(
        baseframe_poses_.col(col_idx).data());
  }
}

void GraphBaOptimizer::solve(
    bool copy_data_from_solver_back_to_map,
    const ceres::Solver::Options& options, ceres::Problem* problem,
    ceres::Solver::Summary* summary) {
  CHECK_NOTNULL(problem);
  CHECK_NOTNULL(summary);
  ceres::Solve(options, problem, summary);

  if (copy_data_from_solver_back_to_map) {
    copyDataToMap();
  }

  VLOG(2) << summary->message;
  VLOG(3) << summary->FullReport();
}
}  // namespace map_optimization_legacy
