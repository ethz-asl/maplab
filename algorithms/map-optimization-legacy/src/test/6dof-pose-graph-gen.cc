#include "map-optimization-legacy/test/6dof-pose-graph-gen.h"

#include <cmath>
#include <memory>
#include <random>
#include <unordered_map>
#include <vector>

#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/distortion-fisheye.h>
#include <aslam/common/memory.h>
#include <aslam/frames/visual-frame.h>
#include <ceres-error-terms/inertial-error-term.h>
#include <ceres-error-terms/parameterization/pose-param-jpl.h>
#include <ceres-error-terms/parameterization/quaternion-param-jpl.h>
#include <ceres-error-terms/visual-error-term.h>
#include <glog/logging.h>
#include <maplab-common/file-logger.h>
#include <maplab-common/gravity-provider.h>
#include <maplab-common/pose_types.h>
#include <maplab-common/quaternion-math.h>
#include <posegraph/unique-id.h>
#include <vi-map/unique-id.h>
#include <vi-map/viwls-edge.h>

#include "map-optimization-legacy/test/6dof-test-trajectory-gen.h"

namespace map_optimization_legacy {

typedef aslam::FisheyeDistortion DistortionType;
typedef aslam::PinholeCamera CameraType;

SixDofPoseGraphGenerator::SixDofPoseGraphGenerator() {
  distortion_param_ = 0.0;
  fu_ = 100;
  fv_ = 100;
  res_u_ = 640;
  res_v_ = 480;
  cu_ = res_u_ / 2.0;
  cv_ = res_v_ / 2.0;

  common::GravityProvider gravity_provider(
      common::locations::kAltitudeZurichMeters,
      common::locations::kLatitudeZurichDegrees);

  // No IMU noise considered.
  settings_.imu_sigmas.acc_bias_random_walk_noise_density = 0.0;
  settings_.imu_sigmas.acc_noise_density = 0.0;
  settings_.gravity_meter_by_second2 = gravity_provider.getGravityMagnitude();
  settings_.imu_sigmas.gyro_bias_random_walk_noise_density = 0.0;
  settings_.imu_sigmas.gyro_noise_density = 0.0;
  // Seed does not matter here, we don't want noise anyways.
  settings_.imu_noise_bias_seed = 1;
  settings_.landmark_variance_meter = 3.0;
  settings_.mode = test_trajectory_gen::Path::kCircular;
  settings_.num_of_landmarks = 500;
  // 8 points for path constraints.
  settings_.num_of_path_constraints = 8;
  settings_.landmark_seed = 10;
  settings_.sampling_time_second = 0.01;
  id_counter_ = 0;

  G_q_B_.coeffs() << 0, 0, 0, 1;
  G_p_B_ << 0, 0, 0;

  // IMU to camera transformation.
  C_p_I_ << 0, 0, 0;
  C_q_I_ = Eigen::Quaterniond(sqrt(2.0) / 2.0, 0, 0, sqrt(2.0) / 2.0);
  C_q_I_.normalize();

  pose_parameterization_ = new ceres_error_terms::JplPoseParameterization;
}

void SixDofPoseGraphGenerator::copyDataFromPosegraph() {
  pose_graph::VertexIdList all_vertex_ids;
  posegraph_.getAllVertexIds(&all_vertex_ids);

  keyframe_poses_.resize(Eigen::NoChange, all_vertex_ids.size());

  unsigned int col_idx = 0;
  for (pose_graph::VertexId vertex_id : all_vertex_ids) {
    const vi_map::Vertex* ba_vertex = dynamic_cast<const vi_map::Vertex*>(
        posegraph_.getVertexPtr(vertex_id));  // NOLINT
    CHECK_NOTNULL(ba_vertex);

    vertex_id_to_pose_idx_.emplace(vertex_id, col_idx);

    pose::Transformation M_T_I(ba_vertex->get_q_M_I(), ba_vertex->get_p_M_I());

    // ba_vertex.getOrientation() returns active q_M_I rotation, while the
    // error terms require passive q_I_M. In consequence, we don't need
    // any inverse here.
    keyframe_poses_.col(col_idx)
        << M_T_I.getRotation().toImplementation().coeffs(),
        M_T_I.getPosition();

    ++col_idx;
  }

  pose::Transformation G_T_M(G_q_B_, G_p_B_);
  pose::Transformation G_T_M_JPL(
      G_T_M.getRotation().toImplementation().inverse(), G_T_M.getPosition());

  base_frame_ << G_T_M_JPL.getRotation().toImplementation().coeffs(),
      G_T_M_JPL.getPosition();
  C_T_I_JPL_ = pose::Transformation(C_q_I_.inverse(), C_p_I_);
}

void SixDofPoseGraphGenerator::copyDataToPosegraph() {
  for (const VertexIdRotationMap::value_type& vertex_id_to_idx :
       vertex_id_to_pose_idx_) {
    vi_map::Vertex* ba_vertex = dynamic_cast<vi_map::Vertex*>( // NOLINT
        posegraph_.getVertexPtrMutable(vertex_id_to_idx.first));
    CHECK_NOTNULL(ba_vertex);

    Eigen::Quaterniond I_q_M_JPL;
    I_q_M_JPL.coeffs() = keyframe_poses_.col(vertex_id_to_idx.second).head(4);
    Eigen::Vector3d M_p_I_JPL(
        keyframe_poses_.col(vertex_id_to_idx.second).tail(3));

    // I_q_G_JPL is in fact equal to active G_q_I - no inverse is needed.
    ba_vertex->set_p_M_I(M_p_I_JPL);
    ba_vertex->set_q_M_I(I_q_M_JPL);
  }

  Eigen::Quaterniond G_q_M_JPL;
  G_q_M_JPL.coeffs() = base_frame_.head(4);

  G_q_B_ = G_q_M_JPL.inverse();
  G_p_B_ = base_frame_.tail(3);

  C_q_I_ = C_T_I_JPL_.getRotation().toImplementation().inverse();
  C_p_I_ = C_T_I_JPL_.getPosition();
}

void SixDofPoseGraphGenerator::constructCamera() {
  Eigen::VectorXd distortion_parameters(1);
  distortion_parameters << distortion_param_;
  aslam::Distortion::UniquePtr distortion(
      new DistortionType(distortion_parameters));

  Eigen::VectorXd intrinsics(4);
  intrinsics << fu_, fv_, cu_, cv_;

  aslam::Camera::Ptr camera = std::shared_ptr<CameraType>(
      new CameraType(intrinsics, res_u_, res_v_, distortion));

  aslam::CameraId camera_id;
  common::generateId(&camera_id);
  camera->setId(camera_id);

  std::vector<aslam::Camera::Ptr> camera_vector;
  camera_vector.push_back(camera);
  aslam::TransformationVector T_C_I_vector;
  // We use identity transformation to T_C_B from default constructor.
  aslam::Transformation T_C_I(C_p_I_, C_q_I_);
  T_C_I_vector.push_back(T_C_I);
  aslam::NCameraId n_camera_id;
  common::generateId(&n_camera_id);
  cameras_.reset(
      new aslam::NCamera(
          n_camera_id, T_C_I_vector, camera_vector, "Test camera rig"));
}

void SixDofPoseGraphGenerator::generatePathAndLandmarks() {
  SixDofTestTrajectoryGenerator generator(settings_);

  LOG(INFO) << "Generating trajectory, may take a while...";
  generator.generate6DofPath();
  positions_ = generator.getTruePositions();
  rotations_ = generator.getTrueRotations();
  velocities_ = generator.getTrueVelocities();
  imu_data_ = generator.getImuData();
  imu_timestamps_seconds_ = generator.getTimestampsInSeconds();
  generator.saveTrajectory("6dof_traj.csv");
  LOG(INFO) << "Number of path points: " << positions_.cols();

  generator.generateLandmarks();
  Eigen::Matrix3Xd G_landmarks = generator.getLandmarks();

  // A slight simplification below - all the landmarks are
  // kept on a single map, as agreed they should be kept as continuous
  // memory in one of the vertices.
  for (int i = 0; i < G_landmarks.cols(); ++i) {
    vi_map::Landmark::Ptr landmark_ptr(new vi_map::Landmark);
    vi_map::LandmarkId landmark_id;
    generateIdFromInt(id_counter_, &landmark_id);
    landmark_ptr->setId(landmark_id);
    ++id_counter_;

    landmark_ptr->set_p_B(G_landmarks.col(i));
    landmark_ptr->set_p_B_Covariance(Eigen::Matrix<double, 3, 3>::Identity());
    landmarks_.insert(std::make_pair(landmark_id, landmark_ptr));
    landmark_observation_count_.insert(std::make_pair(landmark_id, 0));

    true_landmark_positions_.insert(
        std::make_pair(landmark_ptr->id(), landmark_ptr->get_p_B()));

    constexpr int kNumBytesPerDescriptor = 48;
    aslam::VisualFrame::DescriptorsT descriptor;
    descriptor.setRandom(kNumBytesPerDescriptor, 1);
    landmark_descriptors_.insert(std::make_pair(landmark_id, descriptor));
  }
}

void SixDofPoseGraphGenerator::fillPosegraph(
    int num_of_vertices, double vertex_position_sigma,
    double vertex_rotation_sigma, const Eigen::Vector3d& gyro_bias,
    const Eigen::Vector3d& accel_bias, int num_of_no_noise_vertices) {
  CHECK(cameras_) << "Camera must be constructed first";

  std::mt19937 gen(settings_.landmark_seed);
  std::normal_distribution<> dis(0, vertex_position_sigma);
  std::normal_distribution<> dis_quat(0, vertex_rotation_sigma);

  // Zero bias is the best guess.
  Eigen::Matrix<double, 6, 1> imu_bias;
  imu_bias.setZero();

  common::FileLogger vertices_log("vertices6dof.csv");
  common::FileLogger landmarks_log("landmarks6dof.csv");

  vertex_ids_.resize(num_of_vertices);
  int num_of_measurements_per_edge =
      ceil(static_cast<double>(imu_data_.cols()) / num_of_vertices);
  LOG(INFO) << num_of_measurements_per_edge << " measurements per edge";

  // Create a single mission for all the keyframes/landmarks.
  vi_map::MissionId mission_id;
  generateIdFromInt(id_counter_, &mission_id);
  ++id_counter_;
  std::shared_ptr<vi_map::VIMission> mission_ptr(new vi_map::VIMission);
  mission_ptr->setId(mission_id);
  missions_.insert(std::make_pair(mission_id, mission_ptr));

  pose_graph::VertexId prev_vertex_id;
  int vertex_idx = 0;
  for (int i = 0; i < imu_data_.cols(); i += num_of_measurements_per_edge) {
    generateIdFromInt(id_counter_, &vertex_ids_[vertex_idx]);
    ++id_counter_;
    Eigen::Vector3d G_p_I = positions_.col(i);
    Eigen::Quaterniond G_q_I;
    G_q_I.coeffs() = rotations_.block<4, 1>(0, i);
    G_q_I.normalize();

    // Make sure the scalar part is positive.
    if (G_q_I.w() < 0) {
      G_q_I.coeffs() *= -1;
    }
    CHECK_GE(G_q_I.w(), 0.);

    Eigen::Quaterniond G_q_C = G_q_I * C_q_I_.inverse();
    pose::Transformation G_T_C(G_q_C, G_p_I);

    Eigen::Vector3d zaxis(0, 0, 1);
    vertices_log << G_p_I.transpose() << " " << (G_T_C * zaxis).transpose()
                 << std::endl;

    Aligned<std::vector, Eigen::Vector2d> visible_points;
    std::vector<vi_map::LandmarkId> landmark_ids;
    // For each landmark, verify if it's visible from the current pose.
    typedef std::unordered_map<vi_map::LandmarkId,
                               vi_map::Landmark::Ptr>::iterator it_type;
    unsigned int counter = 0;
    for (it_type iterator = landmarks_.begin(); iterator != landmarks_.end();
         ++iterator) {
      vi_map::Landmark::Ptr landmark_ptr = iterator->second;

      Eigen::Vector3d G_p_fi = landmark_ptr->get_p_B();
      Eigen::Vector3d C_p_fi = G_T_C.inverse() * G_p_fi;

      landmarks_log << G_p_fi.transpose() << std::endl;

      Eigen::Vector2d keypoint;
      aslam::ProjectionResult projection_result =
          cameras_->getCamera(kVisualFrameIndex).project3(C_p_fi, &keypoint);
      bool is_visible = C_p_fi[2] > 0 && projection_result;
      if (is_visible) {
        landmark_ids.push_back(iterator->first);
        visible_points.push_back(keypoint);
        ++(landmark_observation_count_.find(iterator->first)->second);

        landmark_ptr->addObservation(
            vertex_ids_[vertex_idx], kVisualFrameIndex, counter);
      }
      if (projection_result.isKeypointVisible()) {
        ++counter;
      }
    }
    LOG(INFO) << i << "th pose, visible points: " << visible_points.size();

    Eigen::Matrix2Xd points;
    points.resize(2, visible_points.size());
    for (unsigned int j = 0; j < visible_points.size(); ++j) {
      points.col(j) << visible_points[j];
    }

    Eigen::VectorXd uncertainties;
    uncertainties.resize(visible_points.size());
    uncertainties.setConstant(0.8);

    aslam::VisualFrame::DescriptorsT descriptors;
    descriptors.resize(48, visible_points.size());
    for (size_t landmark_index = 0; landmark_index < landmark_ids.size();
         ++landmark_index) {
      const vi_map::LandmarkId& landmark_id = landmark_ids[landmark_index];
      typename AlignedUnorderedMap<
          vi_map::LandmarkId, aslam::VisualFrame::DescriptorsT>::const_iterator
          it = landmark_descriptors_.find(landmark_id);
      CHECK(it != landmark_descriptors_.end());
      aslam::VisualFrame::DescriptorsT descriptor = it->second;
      constexpr int kNumBitsToPerturb = 30;
      for (int i = 0; i < kNumBitsToPerturb; ++i) {
        constexpr int kNumBitsPerByte = 8;
        descriptor.coeffRef(i % descriptors.rows(), 0) |=
            1 << (i % kNumBitsPerByte);
      }
      descriptors.col(landmark_index) = descriptor;
    }

    CHECK_EQ(landmark_ids.size(), static_cast<unsigned int>(points.cols()));

    aslam::FrameId frame_id;
    common::generateId(&frame_id);
    int64_t frame_timestamp = 0;

    posegraph_.addVIVertex(
        vertex_ids_[vertex_idx], imu_bias, points, uncertainties, descriptors,
        landmark_ids, mission_ptr->id(), frame_id, frame_timestamp, cameras_);

    true_vertex_positions_.insert(
        std::make_pair(vertex_ids_[vertex_idx], G_p_I));
    true_vertex_rotations_.insert(
        std::make_pair(vertex_ids_[vertex_idx], G_q_I));

    Eigen::Vector3d noise_position, noise_velocity;
    // We shouldn't add any noise to the initial pose, because it is fixed
    // when doing inertial BA.
    if (vertex_idx <= (num_of_no_noise_vertices - 1)) {
      noise_position.setZero();
      noise_velocity.setZero();
    } else {
      noise_position << dis(gen), dis(gen), dis(gen);
      noise_velocity << dis(gen), dis(gen), dis(gen);

      // Create noise delta quaternion using small angle approximation.
      Eigen::Quaterniond delta_q(
          1, dis_quat(gen), dis_quat(gen), dis_quat(gen));
      delta_q.normalize();
      G_q_I *= delta_q;
    }
    G_q_I.normalize();

    vi_map::Vertex* ba_vertex = dynamic_cast<vi_map::Vertex*>( // NOLINT
        posegraph_.getVertexPtrMutable(vertex_ids_[vertex_idx]));
    CHECK_NOTNULL(ba_vertex);
    ba_vertex->set_q_M_I(G_q_I);
    ba_vertex->set_p_M_I(G_p_I + noise_position);
    ba_vertex->set_v_M(velocities_.col(i) + noise_velocity);

    // We don't add edge after the first vertex, only after first two are added.
    if (i != 0) {
      pose_graph::EdgeId edge_id;
      generateIdFromInt(id_counter_, &edge_id);
      ++id_counter_;

      Eigen::Matrix<int64_t, 1, Eigen::Dynamic> imu_timestamps;
      Eigen::Matrix<double, 6, Eigen::Dynamic> imu_data;
      imu_timestamps.resize(1, num_of_measurements_per_edge + 1);
      imu_data.resize(6, num_of_measurements_per_edge + 1);

      int measurement_idx = 0;
      // We should add measurements INCLUDING the one of the next vertex
      // (because of interpolated IMU data).
      for (int j = (i - num_of_measurements_per_edge); j <= i; ++j) {
        imu_timestamps(0, measurement_idx) =
            j * 1e9 * settings_.sampling_time_second;
        imu_data.col(measurement_idx) = imu_data_.col(j);

        // Add gyro and accelerometer biases are passed in the method
        // arguments.
        imu_data.col(measurement_idx)
            .block<3, 1>(imu_integrator::kGyroReadingOffset, 0) += gyro_bias;
        imu_data.col(measurement_idx)
            .block<3, 1>(imu_integrator::kAccelReadingOffset, 0) += accel_bias;

        ++measurement_idx;
      }

      posegraph_.addVIEdge(
          edge_id, prev_vertex_id, vertex_ids_[vertex_idx], imu_timestamps,
          imu_data);
    }
    prev_vertex_id = vertex_ids_[vertex_idx];
    ++vertex_idx;
  }

  copyDataFromPosegraph();
  vertices_log.flushBuffer();
  landmarks_log.flushBuffer();
}

void SixDofPoseGraphGenerator::corruptLandmarkPositions(double sigma) {
  std::mt19937 gen(settings_.landmark_seed);
  std::normal_distribution<> dis(0, sigma);
  Eigen::Vector3d noise_vector;

  typedef std::unordered_map<vi_map::LandmarkId,
                             vi_map::Landmark::Ptr>::iterator it_type;
  for (it_type iterator = landmarks_.begin(); iterator != landmarks_.end();
       ++iterator) {
    vi_map::Landmark::Ptr landmark_ptr = iterator->second;

    noise_vector << dis(gen), dis(gen), dis(gen);
    Eigen::Vector3d G_p_fi = landmark_ptr->get_p_B();
    G_p_fi += noise_vector;
    landmark_ptr->set_p_B(pose::Position3D(G_p_fi));
  }
}

void SixDofPoseGraphGenerator::addInertialResidualBlocks(
    bool fix_gyro_bias, bool fix_accel_bias) {
  LOG(INFO) << "Adding inertial term residual blocks...";

  pose_graph::EdgeIdList all_edge_ids;
  posegraph_.getAllEdgeIds(&all_edge_ids);
  CHECK(!all_edge_ids.empty()) << "No edges on the posegraph.";

  for (pose_graph::EdgeId edge_id : all_edge_ids) {
    const vi_map::ViwlsEdge* ba_edge = dynamic_cast<const vi_map::ViwlsEdge*>(
        posegraph_.getEdgePtr(edge_id));  // NOLINT
    CHECK_NOTNULL(ba_edge);

    vi_map::Vertex* vertex_from = dynamic_cast<vi_map::Vertex*>( // NOLINT
        posegraph_.getVertexPtrMutable(ba_edge->from()));
    CHECK_NOTNULL(vertex_from);

    vi_map::Vertex* vertex_to = dynamic_cast<vi_map::Vertex*>( // NOLINT
        posegraph_.getVertexPtrMutable(ba_edge->to()));
    CHECK_NOTNULL(vertex_to);

    // Retrieve keyframe pose idx.
    VertexIdRotationMap::const_iterator it_from;
    it_from = vertex_id_to_pose_idx_.find(vertex_from->id());
    CHECK(it_from != vertex_id_to_pose_idx_.end());
    int pose_idx_from = it_from->second;
    VertexIdRotationMap::const_iterator it_to;
    it_to = vertex_id_to_pose_idx_.find(vertex_to->id());
    CHECK(it_to != vertex_id_to_pose_idx_.end());
    int pose_idx_to = it_to->second;

    // Noise and bias sigmas can't be zero as it will yield infinite
    // values in information matrix that is used by visual error term.
    vi_map::ImuSigmas& imu_sigmas = settings_.imu_sigmas;
    if (imu_sigmas.gyro_noise_density == 0.0) {
      imu_sigmas.gyro_noise_density = 1e-4;
    }
    if (imu_sigmas.gyro_bias_random_walk_noise_density == 0.0) {
      imu_sigmas.gyro_bias_random_walk_noise_density = 1e-4;
    }
    if (imu_sigmas.acc_noise_density == 0.0) {
      imu_sigmas.acc_noise_density = 1e-4;
    }
    if (imu_sigmas.acc_bias_random_walk_noise_density == 0.0) {
      imu_sigmas.acc_bias_random_walk_noise_density = 1e-4;
    }

    ceres::CostFunction* inertial_term_cost =
        new ceres_error_terms::InertialErrorTerm(
            ba_edge->getImuData(), ba_edge->getImuTimestamps(),
            imu_sigmas.gyro_noise_density,
            imu_sigmas.gyro_bias_random_walk_noise_density,
            imu_sigmas.acc_noise_density,
            imu_sigmas.acc_bias_random_walk_noise_density,
            settings_.gravity_meter_by_second2);
    problem_.AddResidualBlock(
        inertial_term_cost, NULL, keyframe_poses_.col(pose_idx_from).data(),
        vertex_from->getGyroBiasMutable(), vertex_from->get_v_M_Mutable(),
        vertex_from->getAccelBiasMutable(),
        keyframe_poses_.col(pose_idx_to).data(),
        vertex_to->getGyroBiasMutable(), vertex_to->get_v_M_Mutable(),
        vertex_to->getAccelBiasMutable());

    problem_.SetParameterization(
        keyframe_poses_.col(pose_idx_from).data(), pose_parameterization_);
    problem_.SetParameterization(
        keyframe_poses_.col(pose_idx_to).data(), pose_parameterization_);

    if (fix_gyro_bias) {
      problem_.SetParameterBlockConstant(vertex_to->getGyroBiasMutable());
      problem_.SetParameterBlockConstant(vertex_from->getGyroBiasMutable());
    }

    if (fix_accel_bias) {
      problem_.SetParameterBlockConstant(vertex_to->getAccelBiasMutable());
      problem_.SetParameterBlockConstant(vertex_from->getAccelBiasMutable());
    }
  }

  // Fix the first vertex. We need to do it because all IMU constraints
  // are relative.
  fixFirstVertices(1, true);
}

void SixDofPoseGraphGenerator::fixFirstVertices(
    unsigned int num_of_fixed_vertices, bool fix_velocity) {
  CHECK_GT(num_of_fixed_vertices, 0u) << "You can't fix zero vertices.";
  LOG(INFO) << "Fixing " << num_of_fixed_vertices << " first vertices";
  for (unsigned int i = 0; i <= (num_of_fixed_vertices - 1); ++i) {
    vi_map::Vertex* vertex = dynamic_cast<vi_map::Vertex*>( // NOLINT
        posegraph_.getVertexPtrMutable(vertex_ids_[i]));
    CHECK(vertex) << "Couldn't cast to BA vertex type.";

    // Retrieve keyframe pose idx.
    VertexIdRotationMap::const_iterator it;
    it = vertex_id_to_pose_idx_.find(vertex->id());
    CHECK(it != vertex_id_to_pose_idx_.end());
    int pose_idx = it->second;

    problem_.SetParameterBlockConstant(keyframe_poses_.col(pose_idx).data());

    if (fix_velocity) {
      problem_.SetParameterBlockConstant(vertex->get_v_M_Mutable());
    }
  }
}

void SixDofPoseGraphGenerator::addVisualResidualBlocks(
    bool fix_intrinsics, bool fix_landmark_positions) {
  LOG(INFO) << "Adding visual term residual blocks...";
  pose_graph::VertexIdList all_vertex_ids;
  posegraph_.getAllVertexIds(&all_vertex_ids);
  CHECK(!all_vertex_ids.empty()) << "No vertices on the posegraph.";

  ceres::LocalParameterization* quaternion_parameterization =
      new ceres_error_terms::JplQuaternionParameterization;

  for (pose_graph::VertexId vertex_id : all_vertex_ids) {
    vi_map::Vertex* ba_vertex = dynamic_cast<vi_map::Vertex*>( // NOLINT
        posegraph_.getVertexPtrMutable(vertex_id));
    CHECK_NOTNULL(ba_vertex);

    const Eigen::Matrix2Xd& image_points_distorted =
        ba_vertex->getVisualFrame(kVisualFrameIndex).getKeypointMeasurements();
    const Eigen::VectorXd& image_points_uncertainties =
        ba_vertex->getVisualFrame(kVisualFrameIndex)
            .getKeypointMeasurementUncertainties();

    const std::shared_ptr<CameraType> camera_ptr =
        std::dynamic_pointer_cast<CameraType>( // NOLINT
            ba_vertex->getCamera(kVisualFrameIndex));
    CHECK(camera_ptr != nullptr);

    // Retrieve keyframe pose idx.
    VertexIdRotationMap::const_iterator it;
    it = vertex_id_to_pose_idx_.find(ba_vertex->id());
    CHECK(it != vertex_id_to_pose_idx_.end());
    int pose_idx = it->second;

    for (int i = 0; i < image_points_distorted.cols(); ++i) {
      if (landmark_observation_count_
              .find(ba_vertex->getObservedLandmarkId(kVisualFrameIndex, i))
              ->second >= 2) {
        ceres::CostFunction* visual_term_cost =
            new ceres_error_terms::VisualReprojectionError<CameraType,
                                                           DistortionType>(
                image_points_distorted.col(i), image_points_uncertainties(i),
                ceres_error_terms::visual::VisualErrorType::kLocalMission,
                camera_ptr.get());
        problem_.AddResidualBlock(
            visual_term_cost, NULL,
            landmarks_
                .find(ba_vertex->getObservedLandmarkId(kVisualFrameIndex, i))
                ->second->get_p_B_Mutable(),
            base_frame_.data(), dummy_7d_0_.data(), dummy_7d_1_.data(),
            keyframe_poses_.col(pose_idx).data(),
            C_T_I_JPL_.getRotation().toImplementation().coeffs().data(),
            C_T_I_JPL_.getPosition().data(), camera_ptr->getParametersMutable(),
            camera_ptr->getDistortionMutable()->getParametersMutable());

        if (fix_landmark_positions) {
          problem_.SetParameterBlockConstant(
              landmarks_
                  .find(ba_vertex->getObservedLandmarkId(kVisualFrameIndex, i))
                  ->second->get_p_B_Mutable());
        }
      }
    }

    if (image_points_distorted.cols() > 0) {
      // We fix the dummy parameter blocks because
      // they have no meaning.
      problem_.SetParameterBlockConstant(dummy_7d_0_.data());
      problem_.SetParameterBlockConstant(dummy_7d_1_.data());

      problem_.SetParameterBlockConstant(C_T_I_JPL_.getPosition().data());
      problem_.SetParameterBlockConstant(
          C_T_I_JPL_.getRotation().toImplementation().coeffs().data());

      if (fix_intrinsics) {
        problem_.SetParameterBlockConstant(camera_ptr->getParametersMutable());
      }

      problem_.SetParameterBlockConstant(
          camera_ptr->getDistortionMutable()->getParametersMutable());

      problem_.SetParameterization(
          keyframe_poses_.col(pose_idx).data(), pose_parameterization_);
      problem_.SetParameterization(base_frame_.data(), pose_parameterization_);
      problem_.SetParameterization(
          C_T_I_JPL_.getRotation().toImplementation().coeffs().data(),
          quaternion_parameterization);
    }
  }

  problem_.SetParameterBlockConstant(base_frame_.data());
  problem_.SetParameterization(base_frame_.data(), pose_parameterization_);
}

void SixDofPoseGraphGenerator::solve(int max_num_of_iters) {
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_SCHUR;
  options.minimizer_progress_to_stdout = true;
  options.gradient_tolerance = 1e-20;
  options.function_tolerance = 1e-20;
  options.parameter_tolerance = 1e-20;
  options.num_threads = 8;
  options.num_linear_solver_threads = 8;
  options.max_num_iterations = max_num_of_iters;
  ceres::Solve(options, &problem_, &summary_);

  LOG(INFO) << summary_.message;
  LOG(INFO) << summary_.BriefReport();

  copyDataToPosegraph();
}

};  // namespace map_optimization_legacy
