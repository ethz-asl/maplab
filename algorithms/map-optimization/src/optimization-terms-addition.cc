#include "map-optimization/optimization-terms-addition.h"

#include <memory>

#include <ceres-error-terms/inertial-error-term.h>
#include <ceres-error-terms/visual-error-term-factory.h>
#include <ceres-error-terms/visual-error-term.h>
#include <ceres/ceres.h>
#include <maplab-common/progress-bar.h>
#include <vi-map-helpers/vi-map-queries.h>
#include <vi-map/landmark-quality-metrics.h>

namespace map_optimization {

bool addVisualTermForKeypoint(
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

  vi_map::Vertex& landmark_store_vertex =
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

  problem->getProblemBookkeepingMutable()->landmarks_in_problem.emplace(
      landmark_id, visual_term_cost.get());
  return true;
}

void addVisualTermsForVertices(
    const bool fix_landmark_positions, const bool fix_intrinsics,
    const bool fix_extrinsics_rotation, const bool fix_extrinsics_translation,
    const size_t min_landmarks_per_frame,
    const std::shared_ptr<ceres::LocalParameterization>& pose_parameterization,
    const std::shared_ptr<ceres::LocalParameterization>&
        baseframe_parameterization,
    const std::shared_ptr<ceres::LocalParameterization>&
        camera_parameterization,
    const pose_graph::VertexIdList& vertices, OptimizationProblem* problem) {
  CHECK_NOTNULL(problem);

  vi_map::VIMap* map = CHECK_NOTNULL(problem->getMapMutable());
  const vi_map::MissionIdSet& missions_to_optimize = problem->getMissionIds();

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

        addVisualTermForKeypoint(
            keypoint_idx, frame_idx, fix_landmark_positions, fix_intrinsics,
            fix_extrinsics_rotation, fix_extrinsics_translation,
            pose_parameterization, baseframe_parameterization,
            camera_parameterization, &vertex, problem);
      }
    }
  }
}

void addVisualTerms(
    const bool fix_landmark_positions, const bool fix_intrinsics,
    const bool fix_extrinsics_rotation, const bool fix_extrinsics_translation,
    const size_t min_landmarks_per_frame, OptimizationProblem* problem) {
  CHECK_NOTNULL(problem);

  vi_map::VIMap* map = CHECK_NOTNULL(problem->getMapMutable());

  const OptimizationProblem::LocalParameterizations& parameterizations =
      problem->getLocalParameterizations();

  const vi_map::MissionIdSet& missions_to_optimize = problem->getMissionIds();
  for (const vi_map::MissionId& mission_id : missions_to_optimize) {
    pose_graph::VertexIdList vertices;
    map->getAllVertexIdsInMissionAlongGraph(mission_id, &vertices);
    addVisualTermsForVertices(
        fix_landmark_positions, fix_intrinsics, fix_extrinsics_rotation,
        fix_extrinsics_translation, min_landmarks_per_frame,
        parameterizations.pose_parameterization,
        parameterizations.baseframe_parameterization,
        parameterizations.quaternion_parameterization, vertices, problem);
  }
}

void addInertialTerms(
    const bool fix_gyro_bias, const bool fix_accel_bias,
    const bool fix_velocity, const double gravity_magnitude,
    OptimizationProblem* problem) {
  CHECK_NOTNULL(problem);

  vi_map::VIMap* map = CHECK_NOTNULL(problem->getMapMutable());
  const vi_map::SensorManager& sensor_manger = map->getSensorManager();

  const OptimizationProblem::LocalParameterizations& parameterizations =
      problem->getLocalParameterizations();

  size_t num_residuals_added = 0;
  const vi_map::MissionIdSet& missions_to_optimize = problem->getMissionIds();
  for (const vi_map::MissionId& mission_id : missions_to_optimize) {
    pose_graph::EdgeIdList edges;
    map->getAllEdgeIdsInMissionAlongGraph(
        mission_id, pose_graph::Edge::EdgeType::kViwls, &edges);

    const vi_map::Imu& imu_sensor =
        sensor_manger.getSensorForMission<vi_map::Imu>(mission_id);
    const vi_map::ImuSigmas& imu_sigmas = imu_sensor.getImuSigmas();

    num_residuals_added += addInertialTermsForEdges(
        fix_gyro_bias, fix_accel_bias, fix_velocity, gravity_magnitude,
        imu_sigmas, parameterizations.pose_parameterization, edges, problem);
  }

  VLOG(1) << "Added " << num_residuals_added << " inertial residuals.";
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

}  // namespace map_optimization
