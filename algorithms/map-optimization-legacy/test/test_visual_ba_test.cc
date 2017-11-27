#include <memory>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/distortion-fisheye.h>
#include <aslam/common/hash-id.h>
#include <aslam/frames/visual-frame.h>
#include <ceres-error-terms/parameterization/pose-param-jpl.h>
#include <ceres-error-terms/parameterization/quaternion-param-jpl.h>
#include <ceres-error-terms/visual-error-term.h>
#include <maplab-common/pose_types.h>
#include <maplab-common/quaternion-math.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>
#include <vi-map/landmark.h>
#include <vi-map/mission.h>
#include <vi-map/pose-graph.h>
#include <vi-map/vertex.h>
#include <vi-map/viwls-edge.h>

namespace map_optimization_legacy {

class ViwlsGraph : public testing::Test {
 protected:
  typedef aslam::FisheyeDistortion DistortionType;
  typedef aslam::PinholeCamera CameraType;

  virtual void SetUp() {
    distortion_param_ = 1.0;
    fu_ = 10;
    fv_ = 10;
    res_u_ = 640;
    res_v_ = 480;
    cu_ = res_u_ / 2.0;
    cv_ = res_v_ / 2.0;
    dummy_7d_0_.setZero();
    dummy_7d_1_.setZero();
    base_frame_.setZero();
    G_q_B_ << 1, 0, 0, 0;
    G_p_B_.setZero();
  }

  void solve();
  void copyDataFromPosegraph();
  void copyDataToPosegraph();

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

  vi_map::PoseGraph posegraph_;
  std::vector<pose_graph::VertexId> vertex_ids_;
  std::unordered_map<vi_map::LandmarkId, vi_map::Landmark::Ptr> landmarks_;
  std::vector<vi_map::LandmarkId> landmark_ids_;
  std::unordered_map<vi_map::MissionId, std::shared_ptr<vi_map::VIMission> >
      missions_;

  aslam::NCamera::Ptr cameras_;

  double distortion_param_;

  double fu_, fv_;
  double cu_, cv_;
  double res_u_, res_v_;

  Eigen::Vector4d G_q_B_;
  Eigen::Vector3d G_p_B_;

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

void ViwlsGraph::copyDataFromPosegraph() {
  pose_graph::VertexIdList all_vertex_ids;
  posegraph_.getAllVertexIds(&all_vertex_ids);

  keyframe_poses_.resize(Eigen::NoChange, all_vertex_ids.size());

  unsigned int col_idx = 0;
  for (pose_graph::VertexId vertex_id : all_vertex_ids) {
    vi_map::Vertex* ba_vertex = dynamic_cast<vi_map::Vertex*>(
        posegraph_.getVertexPtrMutable(vertex_id));
    CHECK(ba_vertex) << "Couldn't cast to BA edge type.";

    vertex_id_to_pose_idx_.emplace(vertex_id, col_idx);
    keyframe_poses_.col(col_idx) << ba_vertex->get_q_M_I().coeffs(),
        ba_vertex->get_p_M_I();

    ++col_idx;
  }

  base_frame_ << G_q_B_, G_p_B_;
}

void ViwlsGraph::copyDataToPosegraph() {
  for (const VertexIdRotationMap::value_type& vertex_id_to_idx :
       vertex_id_to_pose_idx_) {
    vi_map::Vertex* ba_vertex = dynamic_cast<vi_map::Vertex*>(
        posegraph_.getVertexPtrMutable(vertex_id_to_idx.first));
    CHECK(ba_vertex) << "Couldn't cast to BA edge type.";

    Eigen::Map<Eigen::Vector3d> position(ba_vertex->get_p_M_I_Mutable());
    Eigen::Map<Eigen::Vector4d> rotation(ba_vertex->get_q_M_I_Mutable());

    position = keyframe_poses_.col(vertex_id_to_idx.second).tail(3);
    rotation = keyframe_poses_.col(vertex_id_to_idx.second).head(4);
  }

  G_q_B_ = base_frame_.head(4);
  G_p_B_ = base_frame_.tail(3);
}

void ViwlsGraph::solve() {
  ceres::Problem problem;
  ceres::LocalParameterization* quaternion_parameterization =
      new ceres_error_terms::JplQuaternionParameterization;
  ceres::LocalParameterization* pose_parameterization =
      new ceres_error_terms::JplPoseParameterization;

  pose_graph::VertexIdList all_vertex_ids;
  posegraph_.getAllVertexIds(&all_vertex_ids);
  CHECK(!all_vertex_ids.empty()) << "No vertices on the posegraph";

  copyDataFromPosegraph();

  for (const pose_graph::VertexId& vertex_id : all_vertex_ids) {
    vi_map::Vertex* ba_vertex = dynamic_cast<vi_map::Vertex*>(
        posegraph_.getVertexPtrMutable(vertex_id));
    CHECK(ba_vertex) << "Couldn't cast to BA edge type.";

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
      ceres::CostFunction* visual_term_cost =
          new ceres_error_terms::VisualReprojectionError<CameraType,
                                                         DistortionType>(
              image_points_distorted.col(i), image_points_uncertainties(i),
              ceres_error_terms::visual::VisualErrorType::kLocalMission,
              camera_ptr.get());

      problem.AddResidualBlock(
          visual_term_cost, NULL,
          landmarks_
              .find(ba_vertex->getObservedLandmarkId(kVisualFrameIndex, i))
              ->second->get_p_B_Mutable(),
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
          landmarks_
              .find(ba_vertex->getObservedLandmarkId(kVisualFrameIndex, i))
              ->second->get_p_B_Mutable());
    }

    // We fix the dummy parameter blocks because
    // they have no meaning.
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
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = false;
  options.max_num_iterations = 200;
  options.gradient_tolerance = 1e-20;
  options.function_tolerance = 1e-20;
  options.parameter_tolerance = 1e-20;
  ceres::Solver::Summary summary;

  ceres::Solve(options, &problem, &summary);

  LOG(INFO) << summary.BriefReport();

  copyDataToPosegraph();
}

TEST_F(ViwlsGraph, VisualBundleAdjTestCamPositionOptimization) {
  constructCamera();

  vi_map::MissionId mission_id;
  mission_id.fromHexString("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa0");
  std::shared_ptr<vi_map::VIMission> mission_ptr(new vi_map::VIMission);
  mission_ptr->setId(mission_id);
  missions_.insert(std::make_pair(mission_id, mission_ptr));

  vi_map::Landmark::Ptr landmark_ptr(new vi_map::Landmark);
  vi_map::LandmarkId landmark_id;
  landmark_id.fromHexString("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa1");
  landmark_ptr->setId(landmark_id);
  landmark_ptr->set_p_B(pose::Position3D(0, 0, 1));
  landmark_ptr->set_p_B_Covariance(Eigen::Matrix<double, 3, 3>::Identity());
  landmarks_.insert(std::make_pair(landmark_ptr->id(), landmark_ptr));
  landmark_ids_.push_back(landmark_ptr->id());

  Eigen::Matrix<double, 6, 1> imu_bias;
  imu_bias << 1, 2, 3, 4, 5, 6;

  Eigen::Matrix<double, 2, 1> points;
  points << cu_, cv_;

  Eigen::VectorXd uncertainties;
  uncertainties.resize(1);
  uncertainties << 0.5;

  aslam::VisualFrame::DescriptorsT descriptors;
  descriptors.resize(48, 1);
  descriptors.setRandom();

  pose_graph::VertexId vertex0;
  vertex0.fromHexString("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa2");
  vertex_ids_.push_back(vertex0);

  aslam::FrameId frame_id;
  frame_id.fromHexString("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa3");
  int64_t frame_timestamp = 0;

  // Base position and rotation.
  G_q_B_ << 0, 0, 0, 1;
  G_p_B_ << 0, 0, 0;

  posegraph_.addVIVertex(
      vertex0, imu_bias, points, uncertainties, descriptors, landmark_ids_,
      mission_ptr->id(), frame_id, frame_timestamp, cameras_);
  EXPECT_TRUE(posegraph_.vertexExists(vertex0));
  Eigen::Vector3d vertex_position(0.2, -0.3, 0.2);
  Eigen::Quaterniond vertex_orientation(1, 0, 0, 0);

  vi_map::Vertex* ba_vertex =
      dynamic_cast<vi_map::Vertex*>(posegraph_.getVertexPtrMutable(vertex0));
  CHECK_NOTNULL(ba_vertex);
  ba_vertex->set_q_M_I(vertex_orientation);
  ba_vertex->set_p_M_I(vertex_position);
  solve();

  const Eigen::Quaterniond& G_q_I = ba_vertex->get_q_M_I();
  const Eigen::Vector3d& G_p_I = ba_vertex->get_p_M_I();
  Eigen::Matrix3d I_R_G = G_q_I.toRotationMatrix().transpose();
  const Eigen::Vector3d C_p_fi = I_R_G * (landmark_ptr->get_p_B() - G_p_I);

  // We don't really care about exact keyframe pose, but we do care
  // about landmark location in the camera coordinate frame: it needs to be
  // located on the optical axis.
  EXPECT_NEAR_EIGEN(C_p_fi.normalized(), Eigen::Vector3d(0, 0, 1), 1e-9);
}

}  // namespace map_optimization_legacy

MAPLAB_UNITTEST_ENTRYPOINT
