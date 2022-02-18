#include "map-optimization/optimization-terms-addition.h"

#include <memory>

#include <ceres-error-terms/block-pose-prior-error-term-v2.h>
#include <ceres-error-terms/inertial-error-term.h>
#include <ceres-error-terms/pose-prior-error-term.h>
#include <ceres-error-terms/six-dof-block-pose-error-term-autodiff.h>
#include <ceres-error-terms/six-dof-block-pose-error-term-with-extrinsics-autodiff.h>
#include <ceres-error-terms/visual-error-term-factory.h>
#include <ceres-error-terms/visual-error-term.h>
#include <ceres/ceres.h>
#include <maplab-common/progress-bar.h>
#include <vi-map-helpers/vi-map-queries.h>
#include <vi-map/landmark-quality-metrics.h>

namespace map_optimization {

void addVisualTermForKeypoint(
    const int keypoint_idx, const int frame_idx,
    const bool fix_landmark_positions, const bool fix_intrinsics,
    const bool fix_extrinsics_rotation, const bool fix_extrinsics_translation,
    const std::shared_ptr<ceres::LocalParameterization>& pose_parameterization,
    const std::shared_ptr<ceres::LocalParameterization>&
        baseframe_parameterization,
    const std::shared_ptr<ceres::LocalParameterization>&
        camera_parameterization,
    vi_map::Vertex* vertex_ptr, OptimizationProblem* problem) {
  CHECK_NOTNULL(vertex_ptr);
  CHECK_NOTNULL(problem);

  CHECK(pose_parameterization != nullptr);
  CHECK(baseframe_parameterization != nullptr);
  CHECK(camera_parameterization != nullptr);

  OptimizationStateBuffer* buffer =
      CHECK_NOTNULL(problem->getOptimizationStateBufferMutable());
  vi_map::VIMap* map = CHECK_NOTNULL(problem->getMapMutable());
  ceres_error_terms::ProblemInformation* problem_information =
      CHECK_NOTNULL(problem->getProblemInformationMutable());

  const aslam::VisualFrame& visual_frame =
      vertex_ptr->getVisualFrame(frame_idx);
  CHECK_GE(keypoint_idx, 0);
  CHECK_LT(
      keypoint_idx,
      static_cast<int>(visual_frame.getNumKeypointMeasurements()));

  const vi_map::LandmarkId landmark_id =
      vertex_ptr->getObservedLandmarkId(frame_idx, keypoint_idx);

  // The keypoint must have a valid association with a landmark.
  CHECK(landmark_id.isValid());

  const vi_map::Vertex& landmark_store_vertex =
      map->getLandmarkStoreVertex(landmark_id);
  vi_map::Landmark& landmark = map->getLandmark(landmark_id);

  const aslam::Camera::Ptr camera_ptr = vertex_ptr->getCamera(frame_idx);
  CHECK(camera_ptr != nullptr);

  const Eigen::Vector2d& image_point_distorted =
      vertex_ptr->getVisualFrame(frame_idx).getKeypointMeasurement(
          keypoint_idx);
  const double image_point_uncertainty =
      vertex_ptr->getVisualFrame(frame_idx).getKeypointMeasurementUncertainty(
          keypoint_idx);

  // As defined here: http://en.wikipedia.org/wiki/Huber_Loss_Function
  double huber_loss_delta = 3.0;

  ceres_error_terms::visual::VisualErrorType error_term_type;
  if (vertex_ptr->id() != landmark_store_vertex.id()) {
    // Verify if the landmark and keyframe belong to the same mission.
    if (vertex_ptr->getMissionId() == landmark_store_vertex.getMissionId()) {
      error_term_type =
          ceres_error_terms::visual::VisualErrorType::kLocalMission;
    } else {
      error_term_type = ceres_error_terms::visual::VisualErrorType::kGlobal;
      huber_loss_delta = 10.0;
    }
  } else {
    error_term_type =
        ceres_error_terms::visual::VisualErrorType::kLocalKeyframe;
  }

  double* distortion_params = nullptr;
  if (camera_ptr->getDistortion().getType() !=
      aslam::Distortion::Type::kNoDistortion) {
    distortion_params =
        camera_ptr->getDistortionMutable()->getParametersMutable();
    CHECK_NOTNULL(distortion_params);
  }

  vi_map::MissionBaseFrameId observer_baseframe_id =
      map->getMissionForVertex(vertex_ptr->id()).getBaseFrameId();
  vi_map::MissionBaseFrameId store_baseframe_id =
      map->getMissionForVertex(landmark_store_vertex.id()).getBaseFrameId();
  double* observer_baseframe_q_GM__G_p_GM =
      buffer->get_baseframe_q_GM__G_p_GM_JPL(observer_baseframe_id);
  double* landmark_store_baseframe_q_GM__G_p_GM =
      buffer->get_baseframe_q_GM__G_p_GM_JPL(store_baseframe_id);

  double* vertex_q_IM__M_p_MI =
      buffer->get_vertex_q_IM__M_p_MI_JPL(vertex_ptr->id());
  double* landmark_store_vertex_q_IM__M_p_MI =
      buffer->get_vertex_q_IM__M_p_MI_JPL(landmark_store_vertex.id());

  const aslam::CameraId& camera_id = camera_ptr->getId();
  CHECK(camera_id.isValid());
  double* camera_q_CI =
      buffer->get_camera_extrinsics_q_CI__C_p_CI_JPL(camera_id);
  // The visual error term requires the camera rotation and translation
  // to be feeded separately. Shifting by 4 = the quaternione size.
  double* camera_C_p_CI = camera_q_CI + 4;

  std::shared_ptr<ceres::CostFunction> visual_term_cost(
      ceres_error_terms::createVisualCostFunction<
          ceres_error_terms::VisualReprojectionError>(
          image_point_distorted, image_point_uncertainty, error_term_type,
          camera_ptr.get()));

  std::vector<double*> cost_term_args = {
      landmark.get_p_B_Mutable(),
      landmark_store_vertex_q_IM__M_p_MI,
      landmark_store_baseframe_q_GM__G_p_GM,
      observer_baseframe_q_GM__G_p_GM,
      vertex_q_IM__M_p_MI,
      camera_q_CI,
      camera_C_p_CI,
      camera_ptr->getParametersMutable(),
      camera_ptr->getDistortionMutable()->getParametersMutable()};

  // Certain types of visual cost terms (as indicated by error_term_type) do not
  // use all of the pointer arguments. Ceres, however, requires us to provide
  // valid pointers so we replace unnecessary arguments with dummy variables
  // filled with NaNs. The function also returns the pointers of the dummies
  // used so that we can set them constant below.
  std::vector<double*> dummies_to_set_constant;
  ceres_error_terms::replaceUnusedArgumentsOfVisualCostFunctionWithDummies(
      error_term_type, &cost_term_args, &dummies_to_set_constant);

  for (double* dummy : dummies_to_set_constant) {
    problem_information->setParameterBlockConstant(dummy);
  }

  std::shared_ptr<ceres::LossFunction> loss_function(
      new ceres::LossFunctionWrapper(
          new ceres::HuberLoss(huber_loss_delta * image_point_uncertainty),
          ceres::TAKE_OWNERSHIP));

  problem_information->addResidualBlock(
      ceres_error_terms::ResidualType::kVisualReprojectionError,
      visual_term_cost, loss_function, cost_term_args);

  if (error_term_type !=
      ceres_error_terms::visual::VisualErrorType::kLocalKeyframe) {
    problem_information->setParameterization(
        landmark_store_vertex_q_IM__M_p_MI, pose_parameterization);
    problem_information->setParameterization(
        vertex_q_IM__M_p_MI, pose_parameterization);

    if (error_term_type ==
        ceres_error_terms::visual::VisualErrorType::kGlobal) {
      problem_information->setParameterization(
          landmark_store_baseframe_q_GM__G_p_GM, baseframe_parameterization);
      problem_information->setParameterization(
          observer_baseframe_q_GM__G_p_GM, baseframe_parameterization);
    }
  }

  problem_information->setParameterization(
      camera_q_CI, camera_parameterization);

  if (fix_landmark_positions) {
    problem_information->setParameterBlockConstant(landmark.get_p_B_Mutable());
  }
  if (fix_intrinsics) {
    problem_information->setParameterBlockConstant(
        camera_ptr->getParametersMutable());
    if (camera_ptr->getDistortion().getType() !=
        aslam::Distortion::Type::kNoDistortion) {
      problem_information->setParameterBlockConstant(
          camera_ptr->getDistortionMutable()->getParametersMutable());
    }
  }
  if (fix_extrinsics_rotation) {
    problem_information->setParameterBlockConstant(camera_q_CI);
  }
  if (fix_extrinsics_translation) {
    problem_information->setParameterBlockConstant(camera_C_p_CI);
  }

  // NOTE: Whether or not the baseframes are fixed is decided when setting up
  // the problem on a higher level.

  problem->getProblemBookkeepingMutable()->landmarks_in_problem.emplace(
      landmark_id, visual_term_cost.get());
}

int addVisualTermsForVertices(
    const bool fix_landmark_positions, const bool fix_intrinsics,
    const bool fix_extrinsics_rotation, const bool fix_extrinsics_translation,
    const size_t min_landmarks_per_frame,
    const vi_map::FeatureType feature_type,
    const std::shared_ptr<ceres::LocalParameterization>& pose_parameterization,
    const std::shared_ptr<ceres::LocalParameterization>&
        baseframe_parameterization,
    const std::shared_ptr<ceres::LocalParameterization>&
        camera_parameterization,
    const pose_graph::VertexIdList& vertices, OptimizationProblem* problem) {
  CHECK_NOTNULL(problem);

  vi_map::VIMap* map = CHECK_NOTNULL(problem->getMapMutable());
  const vi_map::MissionIdSet& missions_to_optimize = problem->getMissionIds();

  size_t num_visual_constraints = 0u;
  for (const pose_graph::VertexId& vertex_id : vertices) {
    vi_map::Vertex& vertex = map->getVertex(vertex_id);
    const size_t num_frames = vertex.numFrames();
    for (size_t frame_idx = 0u; frame_idx < num_frames; ++frame_idx) {
      if (!vertex.isVisualFrameSet(frame_idx) ||
          !vertex.isVisualFrameValid(frame_idx)) {
        continue;
      }

      if (min_landmarks_per_frame > 0) {
        vi_map_helpers::VIMapQueries queries(*map);
        const size_t num_frame_good_landmarks =
            queries.getNumWellConstrainedLandmarks(vertex, frame_idx);
        if (num_frame_good_landmarks < min_landmarks_per_frame) {
          VLOG(3) << " Skipping this visual keyframe. Only "
                  << num_frame_good_landmarks
                  << " well constrained landmarks, but "
                  << min_landmarks_per_frame << " required";
          continue;
        }
      }
      problem->getProblemBookkeepingMutable()->keyframes_in_problem.emplace(
          vertex_id);

      const aslam::VisualFrame& visual_frame = vertex.getVisualFrame(frame_idx);
      const size_t num_keypoints = visual_frame.getNumKeypointMeasurements();

      for (size_t keypoint_idx = 0u; keypoint_idx < num_keypoints;
           ++keypoint_idx) {
        const vi_map::LandmarkId landmark_id =
            vertex.getObservedLandmarkId(frame_idx, keypoint_idx);
        // Invalid landmark_id means that the keypoint is not actually
        // associated to an existing landmark object.
        if (!landmark_id.isValid()) {
          continue;
        }

        const vi_map::Vertex& landmark_store_vertex =
            map->getLandmarkStoreVertex(landmark_id);

        // Skip if the landmark is stored in a mission that should not be
        // optimized.
        if (missions_to_optimize.count(landmark_store_vertex.getMissionId()) ==
            0u) {
          continue;
        }

        vi_map::Landmark& landmark = map->getLandmark(landmark_id);

        // Skip if the current landmark is not well constrained.
        if (!vi_map::isLandmarkWellConstrained(*map, landmark)) {
          continue;
        }

        // Filter by feature type
        if (feature_type != vi_map::FeatureType::kInvalid &&
            landmark.getFeatureType() != feature_type) {
          continue;
        }

        addVisualTermForKeypoint(
            keypoint_idx, frame_idx, fix_landmark_positions, fix_intrinsics,
            fix_extrinsics_rotation, fix_extrinsics_translation,
            pose_parameterization, baseframe_parameterization,
            camera_parameterization, &vertex, problem);
        num_visual_constraints++;
      }
    }
  }
  return num_visual_constraints;
}

int addVisualTerms(
    const bool fix_landmark_positions, const bool fix_intrinsics,
    const bool fix_extrinsics_rotation, const bool fix_extrinsics_translation,
    const size_t min_landmarks_per_frame,
    const vi_map::FeatureType feature_type, OptimizationProblem* problem) {
  CHECK_NOTNULL(problem);

  vi_map::VIMap* map = CHECK_NOTNULL(problem->getMapMutable());

  const OptimizationProblem::LocalParameterizations& parameterizations =
      problem->getLocalParameterizations();

  size_t num_visual_constraints = 0u;
  const vi_map::MissionIdSet& missions_to_optimize = problem->getMissionIds();
  for (const vi_map::MissionId& mission_id : missions_to_optimize) {
    pose_graph::VertexIdList vertices;
    map->getAllVertexIdsInMissionAlongGraph(mission_id, &vertices);
    num_visual_constraints += addVisualTermsForVertices(
        fix_landmark_positions, fix_intrinsics, fix_extrinsics_rotation,
        fix_extrinsics_translation, min_landmarks_per_frame, feature_type,
        parameterizations.pose_parameterization,
        parameterizations.baseframe_parameterization,
        parameterizations.quaternion_parameterization, vertices, problem);
  }
  VLOG(1) << "Added " << num_visual_constraints << " visual residuals.";
  return num_visual_constraints;
}

int addInertialTerms(
    const bool fix_gyro_bias, const bool fix_accel_bias,
    const bool fix_velocity, const double gravity_magnitude,
    OptimizationProblem* problem) {
  CHECK_NOTNULL(problem);

  vi_map::VIMap* map = CHECK_NOTNULL(problem->getMapMutable());

  const OptimizationProblem::LocalParameterizations& parameterizations =
      problem->getLocalParameterizations();

  size_t num_residuals_added = 0u;
  const vi_map::MissionIdSet& missions_to_optimize = problem->getMissionIds();
  for (const vi_map::MissionId& mission_id : missions_to_optimize) {
    pose_graph::EdgeIdList edges;
    map->getAllEdgeIdsInMissionAlongGraph(
        mission_id, pose_graph::Edge::EdgeType::kViwls, &edges);

    const vi_map::Imu& imu_sensor = map->getMissionImu(mission_id);
    const vi_map::ImuSigmas& imu_sigmas = imu_sensor.getImuSigmas();

    num_residuals_added += addInertialTermsForEdges(
        fix_gyro_bias, fix_accel_bias, fix_velocity, gravity_magnitude,
        imu_sigmas, parameterizations.pose_parameterization, edges, problem);
  }

  VLOG(1) << "Added " << num_residuals_added << " inertial residuals.";
  return num_residuals_added;
}

int addAbsolutePoseConstraintsTerms(
    const bool fix_extrinsics, OptimizationProblem* problem) {
  CHECK_NOTNULL(problem);

  vi_map::VIMap* map = CHECK_NOTNULL(problem->getMapMutable());

  const OptimizationProblem::LocalParameterizations& parameterizations =
      problem->getLocalParameterizations();

  size_t num_absolute_constraints_added = 0u;
  const vi_map::MissionIdSet& missions_to_optimize = problem->getMissionIds();
  for (const vi_map::MissionId& mission_id : missions_to_optimize) {
    pose_graph::VertexIdList vertices;
    map->getAllVertexIdsInMissionAlongGraph(mission_id, &vertices);

    const vi_map::VIMission& mission = map->getMission(mission_id);
    if (mission.hasAbsolute6DoFSensor()) {
      num_absolute_constraints_added +=
          addAbsolutePoseConstraintTermsForVertices(
              fix_extrinsics, parameterizations.pose_parameterization,
              parameterizations.baseframe_parameterization, mission_id,
              vertices, problem);
    }
  }

  VLOG(1) << "Added " << num_absolute_constraints_added
          << " absolute 6DoF residuals.";
  return num_absolute_constraints_added;
}

int addInertialTermsForEdges(
    const bool fix_gyro_bias, const bool fix_accel_bias,
    const bool fix_velocity, const double gravity_magnitude,
    const vi_map::ImuSigmas& imu_sigmas,
    const std::shared_ptr<ceres::LocalParameterization>& pose_parameterization,
    const pose_graph::EdgeIdList& edges, OptimizationProblem* problem) {
  CHECK(pose_parameterization != nullptr);
  CHECK_NOTNULL(problem);

  vi_map::VIMap* map = CHECK_NOTNULL(problem->getMapMutable());
  OptimizationStateBuffer* buffer =
      CHECK_NOTNULL(problem->getOptimizationStateBufferMutable());

  int num_residuals_added = 0;
  for (const pose_graph::EdgeId edge_id : edges) {
    const vi_map::ViwlsEdge& inertial_edge =
        map->getEdgeAs<vi_map::ViwlsEdge>(edge_id);

    std::shared_ptr<ceres_error_terms::InertialErrorTerm> inertial_term_cost(
        new ceres_error_terms::InertialErrorTerm(
            inertial_edge.getImuData(), inertial_edge.getImuTimestamps(),
            imu_sigmas.gyro_noise_density,
            imu_sigmas.gyro_bias_random_walk_noise_density,
            imu_sigmas.acc_noise_density,
            imu_sigmas.acc_bias_random_walk_noise_density, gravity_magnitude));

    vi_map::Vertex& vertex_from = map->getVertex(inertial_edge.from());
    vi_map::Vertex& vertex_to = map->getVertex(inertial_edge.to());

    problem->getProblemBookkeepingMutable()->keyframes_in_problem.emplace(
        vertex_from.id());
    problem->getProblemBookkeepingMutable()->keyframes_in_problem.emplace(
        vertex_to.id());

    double* vertex_from_q_IM__M_p_MI =
        buffer->get_vertex_q_IM__M_p_MI_JPL(inertial_edge.from());
    double* vertex_to_q_IM__M_p_MI =
        buffer->get_vertex_q_IM__M_p_MI_JPL(inertial_edge.to());

    problem->getProblemInformationMutable()->addResidualBlock(
        ceres_error_terms::ResidualType::kInertial, inertial_term_cost, nullptr,
        {vertex_from_q_IM__M_p_MI, vertex_from.getGyroBiasMutable(),
         vertex_from.get_v_M_Mutable(), vertex_from.getAccelBiasMutable(),
         vertex_to_q_IM__M_p_MI, vertex_to.getGyroBiasMutable(),
         vertex_to.get_v_M_Mutable(), vertex_to.getAccelBiasMutable()});

    problem->getProblemInformationMutable()->setParameterization(
        vertex_from_q_IM__M_p_MI, pose_parameterization);
    problem->getProblemInformationMutable()->setParameterization(
        vertex_to_q_IM__M_p_MI, pose_parameterization);

    if (fix_gyro_bias) {
      problem->getProblemInformationMutable()->setParameterBlockConstant(
          vertex_to.getGyroBiasMutable());
      problem->getProblemInformationMutable()->setParameterBlockConstant(
          vertex_from.getGyroBiasMutable());
    }
    if (fix_accel_bias) {
      problem->getProblemInformationMutable()->setParameterBlockConstant(
          vertex_to.getAccelBiasMutable());
      problem->getProblemInformationMutable()->setParameterBlockConstant(
          vertex_from.getAccelBiasMutable());
    }
    if (fix_velocity) {
      problem->getProblemInformationMutable()->setParameterBlockConstant(
          vertex_to.get_v_M_Mutable());
      problem->getProblemInformationMutable()->setParameterBlockConstant(
          vertex_from.get_v_M_Mutable());
    }

    ++num_residuals_added;
  }

  return num_residuals_added;
}

int addWheelOdometryTerms(
    const bool fix_extrinsics, OptimizationProblem* problem) {
  CHECK_NOTNULL(problem);

  vi_map::VIMap* map = CHECK_NOTNULL(problem->getMapMutable());
  const OptimizationProblem::LocalParameterizations& parameterizations =
      problem->getLocalParameterizations();

  size_t num_residuals_added = 0u;
  const vi_map::MissionIdSet& missions_to_optimize = problem->getMissionIds();
  for (const vi_map::MissionId& mission_id : missions_to_optimize) {
    pose_graph::EdgeIdList edges;
    map->getAllEdgeIdsInMissionAlongGraph(
        mission_id, pose_graph::Edge::EdgeType::kWheelOdometry, &edges);
    num_residuals_added += addRelativePoseTermsForEdges(
        pose_graph::Edge::EdgeType::kWheelOdometry, edges, fix_extrinsics,
        parameterizations.pose_parameterization,
        parameterizations.quaternion_parameterization, problem);
  }

  return num_residuals_added;
}

int add6DoFOdometryTerms(
    const bool fix_extrinsics, OptimizationProblem* problem) {
  CHECK_NOTNULL(problem);

  vi_map::VIMap* map = CHECK_NOTNULL(problem->getMapMutable());
  const OptimizationProblem::LocalParameterizations& parameterizations =
      problem->getLocalParameterizations();

  size_t num_residuals_added = 0u;
  const vi_map::MissionIdSet& missions_to_optimize = problem->getMissionIds();
  for (const vi_map::MissionId& mission_id : missions_to_optimize) {
    pose_graph::EdgeIdList edges;
    map->getAllEdgeIdsInMissionAlongGraph(
        mission_id, pose_graph::Edge::EdgeType::kOdometry, &edges);
    num_residuals_added += addRelativePoseTermsForEdges(
        pose_graph::Edge::EdgeType::kOdometry, edges, fix_extrinsics,
        parameterizations.pose_parameterization,
        parameterizations.quaternion_parameterization, problem);
  }

  return num_residuals_added;
}

int addRelativePoseTermsForEdges(
    const vi_map::Edge::EdgeType edge_type,
    const pose_graph::EdgeIdList& provided_edges, const bool fix_extrinsics,
    const std::shared_ptr<ceres::LocalParameterization>& pose_parameterization,
    const std::shared_ptr<ceres::LocalParameterization>&
        quaternion_parameterization,
    OptimizationProblem* problem) {
  CHECK_NOTNULL(problem);
  CHECK_NOTNULL(pose_parameterization);
  CHECK_NOTNULL(quaternion_parameterization);

  vi_map::VIMap* map = CHECK_NOTNULL(problem->getMapMutable());
  ceres_error_terms::ProblemInformation* problem_information =
      CHECK_NOTNULL(problem->getProblemInformationMutable());
  OptimizationStateBuffer* buffer =
      CHECK_NOTNULL(problem->getOptimizationStateBufferMutable());

  ceres_error_terms::ResidualType residual_type;

  if (edge_type == pose_graph::Edge::EdgeType::kWheelOdometry ||
      edge_type == pose_graph::Edge::EdgeType::kOdometry) {
    residual_type = ceres_error_terms::ResidualType::kOdometry;
  } else {
    LOG(FATAL)
        << "The given edge_type is not of a supported TransformationEdge type.";
  }
  VLOG(1) << "Adding " << pose_graph::Edge::edgeTypeToString(edge_type)
          << " term residual blocks...";

  size_t num_residual_blocks_added_with_extrinsics = 0u;

  for (const pose_graph::EdgeId& edge_id : provided_edges) {
    const pose_graph::Edge* edge = map->getEdgePtrAs<vi_map::Edge>(edge_id);
    CHECK_NOTNULL(edge);
    CHECK(edge->getType() == edge_type);
    vi_map::Vertex& vertex_from = map->getVertex(edge->from());
    vi_map::Vertex& vertex_to = map->getVertex(edge->to());

    CHECK(vertex_from.getMissionId() == vertex_to.getMissionId())
        << "The two vertices this edge connects don't belong to the same "
        << "mission.";

    const vi_map::TransformationEdge& transformation_edge =
        edge->getAs<vi_map::TransformationEdge>();
    const aslam::Transformation& T_A_B = transformation_edge.get_T_A_B();
    const aslam::TransformationCovariance& T_A_B_covariance =
        transformation_edge.get_T_A_B_Covariance_p_q();

    // optimization uses other quaternion convention, thus need to fetch
    // p_MI and not p_IM from buffer
    double* vertex_from_q_IM__M_p_MI =
        buffer->get_vertex_q_IM__M_p_MI_JPL(edge->from());
    double* vertex_to_q_IM__M_p_MI =
        buffer->get_vertex_q_IM__M_p_MI_JPL(edge->to());

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

    problem->getProblemBookkeepingMutable()->keyframes_in_problem.emplace(
        vertex_from.id());
    problem->getProblemBookkeepingMutable()->keyframes_in_problem.emplace(
        vertex_to.id());

    // Account for sensor extrinsics.
    const aslam::SensorId& sensor_id = transformation_edge.getSensorId();
    CHECK(sensor_id.isValid());

    double* sensor_extrinsics_q_SB__S_p_SB_JPL =
        buffer->get_sensor_extrinsics_q_SB__S_p_SB_JPL(sensor_id);
    double* sensor_extrinsics_q_SB = sensor_extrinsics_q_SB__S_p_SB_JPL;
    double* sensor_extrinsics_p_SB = sensor_extrinsics_q_SB__S_p_SB_JPL + 4;

    problem_information->addResidualBlock(
        residual_type, relative_pose_cost, nullptr /* loss_function */,
        {vertex_from_q_IM__M_p_MI, vertex_to_q_IM__M_p_MI,
         sensor_extrinsics_q_SB, sensor_extrinsics_p_SB});
    problem_information->setParameterization(
        sensor_extrinsics_q_SB, quaternion_parameterization);

    // TODO(ben): add flag to only fix extrinsics on z for wheel odometry
    // because then z remains unobserable.
    if (fix_extrinsics) {
      problem_information->setParameterBlockConstant(sensor_extrinsics_q_SB);
      problem_information->setParameterBlockConstant(sensor_extrinsics_p_SB);
    }

    ++num_residual_blocks_added_with_extrinsics;
    problem_information->setParameterization(
        vertex_from_q_IM__M_p_MI, pose_parameterization);
    problem_information->setParameterization(
        vertex_to_q_IM__M_p_MI, pose_parameterization);
  }

  VLOG(1) << "Added " << num_residual_blocks_added_with_extrinsics
          << " relative pose error terms with extrinsics.";
  return num_residual_blocks_added_with_extrinsics;
}

int addAbsolutePoseConstraintTermsForVertices(
    const bool fix_extrinsics,
    const std::shared_ptr<ceres::LocalParameterization>& pose_parameterization,
    const std::shared_ptr<ceres::LocalParameterization>&
        baseframe_parameterization,
    const vi_map::MissionId& mission_id,
    const pose_graph::VertexIdList& vertices, OptimizationProblem* problem) {
  CHECK_NOTNULL(problem);

  CHECK(pose_parameterization);
  CHECK(baseframe_parameterization);

  vi_map::VIMap* map = CHECK_NOTNULL(problem->getMapMutable());

  OptimizationStateBuffer* buffer =
      CHECK_NOTNULL(problem->getOptimizationStateBufferMutable());
  ceres_error_terms::ProblemInformation* problem_information =
      CHECK_NOTNULL(problem->getProblemInformationMutable());

  const vi_map::VIMission& mission = map->getMission(mission_id);

  CHECK(mission.hasAbsolute6DoFSensor());

  const aslam::SensorId& sensor_id = mission.getAbsolute6DoFSensor();
  const vi_map::MissionBaseFrameId& baseframe_id = mission.getBaseFrameId();

  size_t num_absolute_6dof_constraints = 0u;
  for (const pose_graph::VertexId& vertex_id : vertices) {
    const vi_map::Vertex& vertex = map->getVertex(vertex_id);
    const std::vector<vi_map::Absolute6DoFMeasurement>& abs_6dof_measurements =
        vertex.getAbsolute6DoFMeasurements();

    for (const vi_map::Absolute6DoFMeasurement& abs_6dof_measurement :
         abs_6dof_measurements) {
      const aslam::TransformationCovariance& covariance =
          abs_6dof_measurement.get_T_G_S_covariance();
      const aslam::Transformation& T_G_S = abs_6dof_measurement.get_T_G_S();
      const aslam::SensorId& meas_sensor_id =
          abs_6dof_measurement.getSensorId();
      const int64_t meas_timestamp_ns =
          abs_6dof_measurement.getTimestampNanoseconds();

      CHECK(meas_sensor_id == sensor_id)
          << "Vertex " << vertex.id()
          << " contains an absolute 6DoF measurement from an unkown "
          << "Absolute6DoF sensor (= not assigned to this mission).";
      CHECK(meas_timestamp_ns == vertex.getMinTimestampNanoseconds())
          << "Vertex " << vertex.id()
          << " contains an absolute 6DoF measurement with a different "
             "timestamp than the vertex itself! measurement: "
          << meas_timestamp_ns
          << "ns vertex: " << vertex.getMinTimestampNanoseconds() << "ns";

      CHECK(covariance.allFinite());
      CHECK(!covariance.hasNaN());

      std::shared_ptr<ceres::CostFunction> error_term(
          new ceres::AutoDiffCostFunction<
              ceres_error_terms::BlockPosePriorErrorTermV2,
              ceres_error_terms::BlockPosePriorErrorTermV2::kResidualBlockSize,
              ceres_error_terms::poseblocks::kPoseSize,
              ceres_error_terms::poseblocks::kPoseSize,
              ceres_error_terms::poseblocks::kPoseSize>(
              new ceres_error_terms::BlockPosePriorErrorTermV2(
                  T_G_S, covariance)));

      double* baseframe_q_GM__G_p_GM_JPL =
          buffer->get_baseframe_q_GM__G_p_GM_JPL(baseframe_id);
      double* vertex_q_IM__M_p_MI_JPL =
          buffer->get_vertex_q_IM__M_p_MI_JPL(vertex_id);
      double* absolute_6dof_sensor_extrinsics_q_SB__S_p_SB_JPL =
          buffer->get_sensor_extrinsics_q_SB__S_p_SB_JPL(sensor_id);

      std::vector<double*> cost_term_args = {
          baseframe_q_GM__G_p_GM_JPL, vertex_q_IM__M_p_MI_JPL,
          absolute_6dof_sensor_extrinsics_q_SB__S_p_SB_JPL};

      const std::shared_ptr<ceres::LossFunction> kLossFunction(nullptr);
      problem_information->addResidualBlock(
          ceres_error_terms::ResidualType::kPosePrior, error_term,
          kLossFunction, cost_term_args);

      problem_information->setParameterization(
          vertex_q_IM__M_p_MI_JPL, pose_parameterization);
      problem_information->setParameterization(
          baseframe_q_GM__G_p_GM_JPL, baseframe_parameterization);
      problem_information->setParameterization(
          absolute_6dof_sensor_extrinsics_q_SB__S_p_SB_JPL,
          pose_parameterization);

      // NOTE: Whether or not the baseframes are fixed is decided when setting
      // up the problem on a higher level.

      if (fix_extrinsics) {
        problem_information->setParameterBlockConstant(
            absolute_6dof_sensor_extrinsics_q_SB__S_p_SB_JPL);
      }

      ++num_absolute_6dof_constraints;
    }
  }
  return num_absolute_6dof_constraints;
}

}  // namespace map_optimization
