#include <memory>
#include <random>
#include <vector>

#include <Eigen/Core>
#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/distortion-fisheye.h>
#include <aslam/common/memory.h>
#include <aslam/frames/visual-frame.h>
#include <ceres-error-terms/parameterization/pose-param-jpl.h>
#include <ceres-error-terms/parameterization/quaternion-param-jpl.h>
#include <ceres-error-terms/visual-error-term.h>
#include <ceres/ceres.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <maplab-common/file-logger.h>
#include <maplab-common/gravity-provider.h>
#include <maplab-common/pose_types.h>
#include <maplab-common/quaternion-math.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>
#include <simulation/generic-path-generator.h>
#include <vi-map/landmark.h>
#include <vi-map/mission.h>
#include <vi-map/pose-graph.h>
#include <vi-map/vertex.h>
#include <vi-map/viwls-edge.h>

namespace map_optimization_legacy {

// TODO(dymczykm) merge with other vi_map fixtures?
class ViwlsGraph : public ::testing::Test {
 protected:
  typedef aslam::FisheyeDistortion DistortionType;
  typedef aslam::PinholeCamera CameraType;

  virtual void SetUp() {
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
    settings_.landmark_seed = 10;
    settings_.landmark_variance_meter = 3.0;
    settings_.mode = test_trajectory_gen::Path::kCircular;
    settings_.num_of_landmarks = 500;
    // 8 points for path constraints.
    settings_.num_of_path_constraints = 8;
    settings_.sampling_time_second = 1.0;
    id_counter_ = 0;
  }

  void constructCamera() {
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
    aslam::TransformationVector T_C_B_vector;
    // We use identity transformation to T_C_B from default constructor.
    T_C_B_vector.resize(kNumCameras);
    aslam::NCameraId n_camera_id;
    common::generateId(&n_camera_id);
    cameras_.reset(
        new aslam::NCamera(
            n_camera_id, T_C_B_vector, camera_vector, "Test camera rig"));
  }

  void constructTrajectoryGenerator() {
    generator_ = std::shared_ptr<test_trajectory_gen::GenericPathGenerator>(
        new test_trajectory_gen::GenericPathGenerator(settings_));
  }

  void generatePathAndLandmarks() {
    LOG(INFO) << "Generating trajectory, may take a while...";

    generator_->generatePath();
    generator_->generateLandmarks();
    positions_ = generator_->getTruePositions();
    rotations_ = generator_->getTrueRotations();
    Eigen::Matrix3Xd keypoints_ = generator_->getLandmarks();

    // A slight simplification below - all the landmarks are
    // kept on a single map, as agreed they should be kept as continuous
    // memory in one of the vertices.
    for (int i = 0; i < keypoints_.cols(); ++i) {
      vi_map::Landmark::Ptr landmark_ptr(new vi_map::Landmark);
      vi_map::LandmarkId landmark_id;
      generateIdFromInt(id_counter_, &landmark_id);
      landmark_ptr->setId(landmark_id);
      ++id_counter_;

      landmark_ptr->set_p_B(keypoints_.block<3, 1>(0, i));
      landmark_ptr->set_p_B_Covariance(Eigen::Matrix<double, 3, 3>::Identity());
      landmark_store_.insert(std::make_pair(landmark_ptr->id(), landmark_ptr));
      landmark_observation_count_.insert(std::make_pair(landmark_ptr->id(), 0));
    }

    LOG(INFO) << "Number of points: " << positions_.cols();
  }

  void addPosegraphVertices();
  void solve();
  void copyDataFromPosegraph();
  void copyDataToPosegraph();

  vi_map::PoseGraph posegraph_;
  std::vector<pose_graph::VertexId> vertex_ids_;
  std::unordered_map<vi_map::LandmarkId, vi_map::Landmark::Ptr> landmark_store_;
  std::unordered_map<vi_map::LandmarkId, unsigned int>
      landmark_observation_count_;
  std::unordered_map<vi_map::MissionId, std::shared_ptr<vi_map::VIMission> >
      missions_;
  AlignedUnorderedMap<pose_graph::VertexId, Eigen::Vector3d>
      true_vertex_positions_;
  AlignedUnorderedMap<pose_graph::VertexId, Eigen::Quaterniond>
      true_vertex_rotations_;

  aslam::NCamera::Ptr cameras_;

  double distortion_param_;
  double fu_, fv_;
  double cu_, cv_;
  double res_u_, res_v_;

  std::shared_ptr<test_trajectory_gen::GenericPathGenerator> generator_;

  // Path and landmark settings.
  test_trajectory_gen::PathAndLandmarkSettings settings_;

  unsigned int id_counter_;

  Eigen::Matrix3Xd positions_;
  Eigen::Matrix4Xd rotations_;

  // Landmark base pose, common for all.
  Eigen::Vector4d q_G_B_;
  Eigen::Vector3d p_G_B_;

  // Containers for 7-element pose objects.
  Eigen::Matrix<double, 7, 1> base_frame_;
  typedef std::unordered_map<pose_graph::VertexId, int> VertexIdRotationMap;
  VertexIdRotationMap vertex_id_to_pose_idx_;
  Eigen::Matrix<double, 7, Eigen::Dynamic> keyframe_poses_;

  // Ordering is [orientation position] -> [xyzw xyz].
  Eigen::Matrix<double, 7, 1> dummy_7d_0_;
  Eigen::Matrix<double, 7, 1> dummy_7d_1_;

  static constexpr int kNumCameras = 1;
  static constexpr int kVisualFrameIndex = 0;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

void ViwlsGraph::addPosegraphVertices() {
  std::mt19937 gen(settings_.landmark_seed);
  std::normal_distribution<> dis(0, 0.3);
  std::normal_distribution<> dis_quat(0, 0.02);

  // We won't use IMU bias in this test scenario.
  Eigen::Matrix<double, 6, 1> imu_bias;
  imu_bias << 1, 2, 3, 4, 5, 6;

  // Base position and rotation.
  q_G_B_ << 0, 0, 0, 1;
  p_G_B_ << 0, 0, 0;
  dummy_7d_0_.setZero();
  dummy_7d_1_.setZero();

  int num_of_poses = positions_.cols();
  vertex_ids_.resize(num_of_poses);

  // Create a mission for all the keyframes and landmarks.
  vi_map::MissionId mission_id;
  generateIdFromInt(id_counter_, &mission_id);
  ++id_counter_;
  vi_map::VIMission::Ptr mission_ptr(new vi_map::VIMission);
  mission_ptr->setId(mission_id);
  missions_.insert(std::make_pair(mission_id, mission_ptr));

  // Rotate the camera 90 deg y so it is looking outwards to the points.
  Eigen::Quaterniond q_I_C(sqrt(2) / 2, 0, sqrt(2) / 2, 0);

  common::FileLogger vertices_log("vertices.csv");
  common::FileLogger landmarks_log("landmarks.csv");

  for (int i = 0; i < num_of_poses; ++i) {
    generateIdFromInt(id_counter_, &vertex_ids_[i]);
    ++id_counter_;

    Eigen::Vector3d p_G_C(positions_.block<3, 1>(0, i));
    Eigen::Quaterniond q_I_G;
    q_I_G.coeffs() = rotations_.block<4, 1>(0, i);
    Eigen::Quaterniond q_G_I = q_I_G.inverse();

    // Make sure the scalar part is positive.
    Eigen::Quaterniond q_G_C = q_G_I * q_I_C;
    if (q_G_C.w() < 0) {
      q_G_C.coeffs() *= -1;
    }

    true_vertex_positions_.insert(std::make_pair(vertex_ids_[i], p_G_C));
    true_vertex_rotations_.insert(std::make_pair(vertex_ids_[i], q_G_C));

    pose::Transformation T_G_C(q_G_C, p_G_C);
    Eigen::Vector3d zaxis(0, 0, 1);
    vertices_log << p_G_C.transpose() << " " << (T_G_C * zaxis).transpose()
                 << std::endl;

    Aligned<std::vector, Eigen::Vector2d> visible_points;
    std::vector<vi_map::LandmarkId> landmark_ids;
    // For each landmark, verify if it's visible from the current pose.
    typedef std::unordered_map<vi_map::LandmarkId,
                               vi_map::Landmark::Ptr>::iterator LandmarkStoreIt;
    for (LandmarkStoreIt iterator = landmark_store_.begin();
         iterator != landmark_store_.end(); ++iterator) {
      vi_map::Landmark::Ptr landmark_ptr = iterator->second;

      Eigen::Vector3d p_G_fi = landmark_ptr->get_p_B();
      Eigen::Vector3d p_C_fi = T_G_C.inverse() * p_G_fi;

      landmarks_log << p_G_fi.transpose() << std::endl;

      Eigen::Vector2d keypoint;
      bool is_visible = static_cast<bool>(
          cameras_->getCamera(kVisualFrameIndex).project3(p_C_fi, &keypoint));

      if (is_visible) {
        landmark_ids.push_back(iterator->first);
        visible_points.push_back(keypoint);
        ++(landmark_observation_count_.find(iterator->first)->second);
      }
    }
    LOG(INFO) << i << "th pose, visible points: " << visible_points.size();

    Eigen::Matrix2Xd points;
    points.resize(Eigen::NoChange, visible_points.size());
    for (unsigned int j = 0; j < visible_points.size(); ++j) {
      points.col(j) << visible_points[j];
    }

    Eigen::VectorXd uncertainties;
    uncertainties.resize(visible_points.size());
    uncertainties.setConstant(0.8);

    aslam::VisualFrame::DescriptorsT descriptors;
    descriptors.resize(48, visible_points.size());
    descriptors.setRandom();

    aslam::FrameId frame_id;
    common::generateId(&frame_id);
    int64_t frame_timestamp = 0;

    CHECK_EQ(landmark_ids.size(), static_cast<unsigned int>(points.cols()));
    posegraph_.addVIVertex(
        vertex_ids_[i], imu_bias, points, uncertainties, descriptors,
        landmark_ids, mission_ptr->id(), frame_id, frame_timestamp, cameras_);

    Eigen::Vector3d vertex_position =  // p_G_C;
        Eigen::Vector3d(
            p_G_C(0) + dis(gen), p_G_C(1) + dis(gen), p_G_C(2) + dis(gen));

    Eigen::Quaterniond delta_q(1, dis_quat(gen), dis_quat(gen), dis_quat(gen));
    delta_q.normalize();
    T_G_C.getRotation().toImplementation() *= delta_q;
    T_G_C.getRotation().toImplementation().normalize();
    if (T_G_C.getRotation().toImplementation().w() < 0) {
      T_G_C.getRotation().toImplementation().coeffs() *= -1;
    }

    vi_map::Vertex& ba_vertex =
        posegraph_.getVertexPtrMutable(vertex_ids_[i])->getAs<vi_map::Vertex>();
    ba_vertex.set_p_M_I(vertex_position);
    ba_vertex.set_q_M_I(T_G_C.getRotation().toImplementation());
  }

  vertices_log.flushBuffer();
  landmarks_log.flushBuffer();
}

void ViwlsGraph::copyDataFromPosegraph() {
  pose_graph::VertexIdList all_vertices;
  posegraph_.getAllVertexIds(&all_vertices);

  keyframe_poses_.resize(Eigen::NoChange, all_vertices.size());

  unsigned int col_idx = 0;
  for (const pose_graph::VertexId& vertex_id : all_vertices) {
    const vi_map::Vertex* ba_vertex =
        dynamic_cast<const vi_map::Vertex*>(posegraph_.getVertexPtr(vertex_id));
    CHECK_NOTNULL(ba_vertex);

    vertex_id_to_pose_idx_.emplace(vertex_id, col_idx);

    pose::Transformation T_G_I = ba_vertex->get_T_M_I();

    keyframe_poses_.col(col_idx)
        << T_G_I.getRotation().toImplementation().coeffs(),
        T_G_I.getPosition();

    ++col_idx;
  }

  base_frame_ << q_G_B_, p_G_B_;
}

void ViwlsGraph::copyDataToPosegraph() {
  for (const VertexIdRotationMap::value_type& vertex_id_to_idx :
       vertex_id_to_pose_idx_) {
    vi_map::Vertex& ba_vertex =
        posegraph_.getVertexPtrMutable(vertex_id_to_idx.first)
            ->getAs<vi_map::Vertex>();

    pose::Transformation T_G_I;
    T_G_I.getPosition() = keyframe_poses_.col(vertex_id_to_idx.second).tail(3);
    T_G_I.getRotation().toImplementation().coeffs() =
        keyframe_poses_.col(vertex_id_to_idx.second).head(4);

    Eigen::Map<Eigen::Vector4d> rotation(ba_vertex.get_q_M_I_Mutable());
    Eigen::Map<Eigen::Vector3d> position(ba_vertex.get_p_M_I_Mutable());

    rotation = T_G_I.getRotation().toImplementation().coeffs();
    position = T_G_I.getPosition();
  }

  q_G_B_ = base_frame_.head(4);
  p_G_B_ = base_frame_.tail(3);
}

void ViwlsGraph::solve() {
  ceres::Problem problem;
  ceres::LocalParameterization* quaternion_parameterization =
      new ceres_error_terms::JplQuaternionParameterization;
  ceres::LocalParameterization* pose_parameterization =
      new ceres_error_terms::JplPoseParameterization;

  pose_graph::VertexIdList all_vertices;
  posegraph_.getAllVertexIds(&all_vertices);
  CHECK(!all_vertices.empty()) << "No vertices on the posegraph";

  copyDataFromPosegraph();

  for (const pose_graph::VertexId& vertex_id : all_vertices) {
    vi_map::Vertex* ba_vertex = dynamic_cast<vi_map::Vertex*>(
        posegraph_.getVertexPtrMutable(vertex_id));
    CHECK_NOTNULL(ba_vertex);

    const Eigen::Matrix2Xd& image_points_distorted =
        ba_vertex->getVisualFrame(kVisualFrameIndex).getKeypointMeasurements();
    const Eigen::VectorXd& image_points_uncertainties =
        ba_vertex->getVisualFrame(kVisualFrameIndex)
            .getKeypointMeasurementUncertainties();

    const std::shared_ptr<CameraType> camera_ptr =
        std::dynamic_pointer_cast<CameraType>(
            ba_vertex->getCamera(kVisualFrameIndex));
    CHECK(camera_ptr != nullptr);

    // Retrieve keyframe pose idx.
    VertexIdRotationMap::const_iterator it;
    it = vertex_id_to_pose_idx_.find(vertex_id);
    CHECK(it != vertex_id_to_pose_idx_.end());
    int pose_idx = it->second;

    for (int i = 0; i < image_points_distorted.cols(); ++i) {
      vi_map::LandmarkId landmark_id =
          ba_vertex->getObservedLandmarkId(kVisualFrameIndex, i);

      if (landmark_observation_count_.find(landmark_id)->second >= 2) {
        ceres::CostFunction* visual_term_cost =
            new ceres_error_terms::VisualReprojectionError<CameraType,
                                                           DistortionType>(
                image_points_distorted.col(i), image_points_uncertainties(i),
                ceres_error_terms::visual::VisualErrorType::kLocalMission,
                camera_ptr.get());

        problem.AddResidualBlock(
            visual_term_cost, NULL,
            landmark_store_.find(landmark_id)->second->get_p_B_Mutable(),
            base_frame_.data(), dummy_7d_0_.data(), dummy_7d_1_.data(),
            keyframe_poses_.col(pose_idx).data(),
            cameras_->get_T_C_B_Mutable(kVisualFrameIndex)
                .getRotation()
                .toImplementation()
                .coeffs()
                .data(),
            cameras_->get_T_C_B_Mutable(kVisualFrameIndex).getPosition().data(),
            camera_ptr->getParametersMutable(),
            camera_ptr->getDistortionMutable()->getParametersMutable());

        problem.SetParameterBlockConstant(
            landmark_store_.find(landmark_id)->second->get_p_B_Mutable());
      }
    }

    // We fix the dummy parameter blocks because they have no meaning.
    problem.SetParameterBlockConstant(dummy_7d_0_.data());
    problem.SetParameterBlockConstant(dummy_7d_1_.data());

    problem.SetParameterBlockConstant(
        cameras_->get_T_C_B_Mutable(kVisualFrameIndex)
            .getRotation()
            .toImplementation()
            .coeffs()
            .data());
    problem.SetParameterBlockConstant(
        cameras_->get_T_C_B_Mutable(kVisualFrameIndex).getPosition().data());
    problem.SetParameterBlockConstant(camera_ptr->getParametersMutable());
    problem.SetParameterBlockConstant(
        camera_ptr->getDistortionMutable()->getParametersMutable());

    problem.SetParameterization(
        keyframe_poses_.col(pose_idx).data(), pose_parameterization);
    problem.SetParameterization(
        cameras_->get_T_C_B_Mutable(kVisualFrameIndex)
            .getRotation()
            .toImplementation()
            .coeffs()
            .data(),
        quaternion_parameterization);
  }

  problem.SetParameterBlockConstant(base_frame_.data());
  problem.SetParameterization(base_frame_.data(), pose_parameterization);

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_SCHUR;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 8;
  options.gradient_tolerance = 1e-30;
  options.function_tolerance = 1e-30;
  options.parameter_tolerance = 1e-30;
  options.num_threads = 8;
  options.num_linear_solver_threads = 8;
  options.check_gradients = false;
  ceres::Solver::Summary summary;

  ceres::Solve(options, &problem, &summary);

  LOG(INFO) << summary.message;
  LOG(INFO) << summary.BriefReport();

  copyDataToPosegraph();
}

TEST_F(ViwlsGraph, 4DofTrajectoryVisualBundleAdj) {
  constructCamera();
  constructTrajectoryGenerator();
  generatePathAndLandmarks();

  addPosegraphVertices();
  pose_graph::VertexIdList all_vertices;
  posegraph_.getAllVertexIds(&all_vertices);
  EXPECT_EQ(positions_.cols(), static_cast<int>(all_vertices.size()));

  solve();

  for (const pose_graph::VertexId& vertex_id : all_vertices) {
    const vi_map::Vertex* ba_vertex =
        dynamic_cast<const vi_map::Vertex*>(posegraph_.getVertexPtr(vertex_id));
    CHECK_NOTNULL(ba_vertex);

    AlignedUnorderedMap<pose_graph::VertexId, Eigen::Vector3d>::iterator it_pos;
    AlignedUnorderedMap<pose_graph::VertexId, Eigen::Quaterniond>::iterator
        it_rot;
    it_pos = true_vertex_positions_.find(vertex_id);
    it_rot = true_vertex_rotations_.find(vertex_id);

    EXPECT_NEAR_EIGEN(it_pos->second, ba_vertex->get_p_M_I(), 1e-10);
    EXPECT_NEAR_EIGEN_QUATERNION(it_rot->second, ba_vertex->get_q_M_I(), 1e-10);
  }
}

}  // namespace map_optimization_legacy

MAPLAB_UNITTEST_ENTRYPOINT
