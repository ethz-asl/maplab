#include <Eigen/Core>
#include <glog/logging.h>

#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/distortion-fisheye.h>
#include <aslam/cameras/ncamera.h>
#include <aslam/frames/visual-frame.h>
#include <maplab-common/accessors.h>
#include <maplab-common/pose_types.h>
#include <maplab-common/quaternion-math.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>
#include <vi-map/landmark-index.h>
#include <vi-map/mission-baseframe.h>
#include <vi-map/unique-id.h>
#include <vi-map/vi-mission.h>

#include "map-optimization-legacy/landmark-geometry-verification.h"

namespace map_optimization_legacy {

class ViwlsGraph : public ::testing::Test {
 protected:
  typedef aslam::FisheyeDistortion DistortionType;
  typedef aslam::PinholeCamera CameraType;

  virtual void SetUp() {
    constructCamera();
    createMission();
    populatePosegraphAndLandmarks();
    landmark_geometry_ = std::shared_ptr<LandmarkGeometryVerification>(
        new LandmarkGeometryVerification(
            posegraph_, landmarks_, missions_, mission_base_frames_,
            sensor_manager_));
  }

  void constructCamera();
  const vi_map::MissionId createMission();
  void prepareVertex(
      const Eigen::Quaterniond& G_q_I, const Eigen::Matrix<double, 3, 1>& G_p_I,
      const vi_map::MissionId& mission_id,
      vi_map::Vertex::UniquePtr* vertex_ptr);
  const vi_map::LandmarkId addLandmarkToVertex(
      const Eigen::Vector3d& G_p_fi, vi_map::Vertex* vertex);
  void addKeypointToVertex(
      const Eigen::Vector3d& G_p_fi, const vi_map::LandmarkId& landmark_id,
      vi_map::Vertex* vertex_ptr);
  void populatePosegraphAndLandmarks();
  Eigen::Vector2d calculateErrors();

  std::unordered_map<vi_map::LandmarkId, pose_graph::VertexId> landmarks_;
  vi_map::MissionMap missions_;
  vi_map::MissionBaseFrameMap mission_base_frames_;
  vi_map::PoseGraph posegraph_;
  vi_map::SensorManager sensor_manager_;

  std::shared_ptr<LandmarkGeometryVerification> landmark_geometry_;

  vi_map::LandmarkId map_landmark_id_;
  vi_map::LandmarkId vertex_landmark_id_;

  aslam::NCamera::Ptr cameras_;

  vi_map::LandmarkId landmark_ids_[2];

  Eigen::Vector3d G_p_fi0_;
  Eigen::Vector3d G_p_fi1_;

  // C_T_G transformations for observer vertices.
  aslam::Transformation C0_T_G_;
  aslam::Transformation C1_T_G_;

  static constexpr int kDescriptorBytes = 48;
  static constexpr int kNumCameras = 1;
  static constexpr int kVisualFrameIndex = 0;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

void ViwlsGraph::constructCamera() {
  Eigen::VectorXd distortion_param(1);
  distortion_param << 0.94;
  double fu = 120;
  double fv = 90;
  int res_u = 640;
  int res_v = 480;
  double cu = res_u / 2.0;
  double cv = res_v / 2.0;

  aslam::Distortion::UniquePtr distortion(new DistortionType(distortion_param));

  Eigen::VectorXd intrinsics(4);
  intrinsics << fu, fv, cu, cv;

  aslam::CameraId camera_id;
  common::generateId(&camera_id);
  aslam::Camera::Ptr camera = std::shared_ptr<CameraType>(
      new CameraType(intrinsics, res_u, res_v, distortion));
  camera->setId(camera_id);

  std::vector<aslam::Camera::Ptr> camera_vector;
  camera_vector.push_back(camera);
  aslam::TransformationVector T_C_B_vector;
  aslam::Transformation T_C_B(
      Eigen::Quaterniond(1, 0, 0.2, 0).normalized(), Eigen::Vector3d(0, 1, 2));
  T_C_B_vector.push_back(T_C_B);
  aslam::NCameraId n_camera_id;
  common::generateId(&n_camera_id);
  cameras_.reset(
      new aslam::NCamera(
          n_camera_id, T_C_B_vector, camera_vector, "Test camera rig"));
}

const vi_map::MissionId ViwlsGraph::createMission() {
  vi_map::MissionBaseFrame baseframe;
  vi_map::MissionBaseFrameId baseframe_id;
  common::generateId(&baseframe_id);
  baseframe.setId(baseframe_id);

  baseframe.set_p_G_M(Eigen::Matrix<double, 3, 1>::Zero());
  baseframe.set_q_G_M(Eigen::Quaterniond::Identity());

  vi_map::VIMission::UniquePtr mission_ptr(new vi_map::VIMission);
  vi_map::MissionId mission_id;
  generateId(&mission_id);
  mission_ptr->setId(mission_id);
  mission_ptr->setBaseFrameId(baseframe_id);

  missions_.emplace(mission_ptr->id(), std::move(mission_ptr));
  mission_base_frames_.emplace(baseframe.id(), baseframe);

  sensor_manager_.addNCamera(cameras_, mission_id);

  return mission_id;
}

void ViwlsGraph::prepareVertex(
    const Eigen::Quaterniond& G_q_I, const Eigen::Matrix<double, 3, 1>& G_p_I,
    const vi_map::MissionId& mission_id,
    vi_map::Vertex::UniquePtr* vertex_ptr) {
  CHECK_NOTNULL(vertex_ptr);
  vertex_ptr->reset(new vi_map::Vertex(cameras_));

  pose_graph::VertexId vertex_id;
  generateId(&vertex_id);
  (*vertex_ptr)->setId(vertex_id);

  (*vertex_ptr)->set_p_M_I(G_p_I);
  Eigen::Quaterniond G_q_I_positive = G_q_I;
  if (G_q_I_positive.w() < 0.0) {
    G_q_I_positive.coeffs() = -G_q_I_positive.coeffs();
  }
  (*vertex_ptr)->set_q_M_I(G_q_I_positive);

  CHECK_GT(missions_.count(mission_id), 0u);
  (*vertex_ptr)->setMissionId(mission_id);

  aslam::VisualFrame::Ptr frame(new aslam::VisualFrame);
  aslam::FrameId frame_id;
  common::generateId(&frame_id);
  frame->setId(frame_id);
  frame->setCameraGeometry(cameras_->getCameraShared(kVisualFrameIndex));
  (*vertex_ptr)->getVisualNFrame().setFrame(kVisualFrameIndex, frame);
}

const vi_map::LandmarkId ViwlsGraph::addLandmarkToVertex(
    const Eigen::Vector3d& G_p_fi, vi_map::Vertex* vertex_ptr) {
  CHECK_NOTNULL(vertex_ptr);

  std::shared_ptr<vi_map::Landmark> landmark_ptr(new vi_map::Landmark);
  vi_map::LandmarkId landmark_id;
  generateId(&landmark_id);
  landmark_ptr->setId(landmark_id);

  pose::Transformation G_T_I = vertex_ptr->get_T_M_I();

  Eigen::Vector3d I_p_fi = G_T_I.inverse() * G_p_fi;
  landmark_ptr->set_p_B(I_p_fi);
  vertex_ptr->getLandmarks().addLandmark(*landmark_ptr);

  CHECK(vertex_ptr->id().isValid());
  landmarks_.emplace(landmark_id, vertex_ptr->id());
  return landmark_id;
}

void ViwlsGraph::addKeypointToVertex(
    const Eigen::Vector3d& p_G_fi, const vi_map::LandmarkId& landmark_id,
    vi_map::Vertex* vertex_ptr) {
  CHECK_NOTNULL(vertex_ptr);

  const vi_map::MissionId& mission_id = vertex_ptr->getMissionId();
  CHECK_GT(missions_.count(mission_id), 0u);
  const pose::Transformation T_C_I = cameras_->get_T_C_B(kVisualFrameIndex);

  pose::Transformation T_M_I = vertex_ptr->get_T_M_I();
  const Eigen::Vector3d p_C_fi = T_C_I * T_M_I.inverse() * p_G_fi;

  Eigen::Vector2d reprojected_keypoint;
  aslam::ProjectionResult result =
      vertex_ptr->getCamera(kVisualFrameIndex)
          ->project3(p_C_fi, &reprojected_keypoint);

  ASSERT_EQ(result, aslam::ProjectionResult::KEYPOINT_VISIBLE);

  Eigen::Matrix2Xd measurements;
  measurements.resize(Eigen::NoChange, 1);
  measurements = reprojected_keypoint;
  vertex_ptr->getVisualFrame(kVisualFrameIndex)
      .setKeypointMeasurements(measurements);
  CHECK(
      vertex_ptr->getVisualFrame(kVisualFrameIndex).hasKeypointMeasurements());

  vertex_ptr->addObservedLandmarkId(kVisualFrameIndex, landmark_id);

  CHECK_GT(landmarks_.count(landmark_id), 0u);
  const pose_graph::VertexId& landmark_vertex_id =
      common::getChecked(landmarks_, landmark_id);
  CHECK(landmark_vertex_id.isValid());
  vi_map::Vertex& landmark_vertex =
      posegraph_.getVertexPtrMutable(landmark_vertex_id)
          ->getAs<vi_map::Vertex>();

  unsigned int keypoint_idx =
      vertex_ptr->observedLandmarkIdsSize(kVisualFrameIndex) - 1;
  landmark_vertex.getLandmarks()
      .getLandmark(landmark_id)
      .addObservation(vertex_ptr->id(), kVisualFrameIndex, keypoint_idx);
}

void ViwlsGraph::populatePosegraphAndLandmarks() {
  CHECK(cameras_);

  vi_map::MissionId mission_id = createMission();

  vi_map::Vertex::UniquePtr vertex0;
  vi_map::Vertex::UniquePtr vertex1;
  vi_map::Vertex::UniquePtr vertex2;
  vi_map::Vertex::UniquePtr vertex3;

  // vertex0: 1st observer
  // vertex1: 2nd observer
  // vertex2: base for initial landmark position
  // vertex3: base for final landmark position
  Eigen::Vector3d G_p_I0(0, 0, 0);
  Eigen::Quaterniond G_q_I0(1, 0, 0, 0);
  Eigen::Vector3d G_p_I1(1, 0, 0);
  Eigen::Quaterniond G_q_I1(1, 0, 0, 0);
  Eigen::Vector3d G_p_I2(0.43, 0.2, 5);
  Eigen::Quaterniond G_q_I2(0, 0, 1, 0);
  Eigen::Vector3d G_p_I3(-0.46, -0.14, 8);
  Eigen::Quaterniond G_q_I3(sqrt(2) / 2, 0, sqrt(2) / 2, 0);

  prepareVertex(G_q_I0, G_p_I0, mission_id, &vertex0);
  prepareVertex(G_q_I1, G_p_I1, mission_id, &vertex1);
  prepareVertex(G_q_I2, G_p_I2, mission_id, &vertex2);
  prepareVertex(G_q_I3, G_p_I3, mission_id, &vertex3);

  const unsigned int kCameraIndex = 0;
  const aslam::Transformation& C_T_I = cameras_->get_T_C_B(kCameraIndex);
  C0_T_G_ = C_T_I * aslam::Transformation(G_q_I0, G_p_I0).inverse();
  C1_T_G_ = C_T_I * aslam::Transformation(G_q_I1, G_p_I1).inverse();

  // Create landmarks.
  G_p_fi0_ << 0, 0.1, 1;
  G_p_fi1_ << 0, 0.9, 10;
  landmark_ids_[0] = addLandmarkToVertex(G_p_fi0_, vertex2.get());
  landmark_ids_[1] = addLandmarkToVertex(G_p_fi1_, vertex3.get());

  posegraph_.addVertex(std::move(vertex2));
  posegraph_.addVertex(std::move(vertex3));

  // Reproject landmark[0].
  addKeypointToVertex(G_p_fi0_, landmark_ids_[0], vertex0.get());
  addKeypointToVertex(G_p_fi0_, landmark_ids_[0], vertex1.get());

  posegraph_.addVertex(std::move(vertex0));
  posegraph_.addVertex(std::move(vertex1));
}

Eigen::Vector2d ViwlsGraph::calculateErrors() {
  Eigen::Vector2d error;

  Eigen::Vector3d C0_p_fi0 = C0_T_G_ * G_p_fi0_;
  Eigen::Vector3d C0_p_fi1 = C0_T_G_ * G_p_fi1_;
  Eigen::Vector3d C1_p_fi0 = C1_T_G_ * G_p_fi0_;
  Eigen::Vector3d C1_p_fi1 = C1_T_G_ * G_p_fi1_;

  Eigen::Vector2d reprojectionC0_fi0, reprojectionC0_fi1, reprojectionC1_fi0,
      reprojectionC1_fi1;

  cameras_->getCamera(kVisualFrameIndex)
      .project3(C0_p_fi0, &reprojectionC0_fi0);
  cameras_->getCamera(kVisualFrameIndex)
      .project3(C1_p_fi0, &reprojectionC1_fi0);
  cameras_->getCamera(kVisualFrameIndex)
      .project3(C0_p_fi1, &reprojectionC0_fi1);
  cameras_->getCamera(kVisualFrameIndex)
      .project3(C1_p_fi1, &reprojectionC1_fi1);

  error(0) = (reprojectionC0_fi0 - reprojectionC0_fi1).norm();
  error(1) = (reprojectionC1_fi0 - reprojectionC1_fi1).norm();

  return error;
}

TEST_F(ViwlsGraph, LandmarkGeometryVerification) {
  std::vector<double> errors_before;
  std::vector<double> errors_after;

  landmark_geometry_->getMergeReprojectionErrors(
      landmark_ids_[0], landmark_ids_[1], &errors_before, &errors_after);

  EXPECT_EQ(errors_before.size(), 2u);
  EXPECT_EQ(errors_after.size(), 2u);

  EXPECT_DOUBLE_EQ(errors_before[0], 0.0);
  EXPECT_DOUBLE_EQ(errors_before[1], 0.0);

  calculateErrors();

  EXPECT_NEAR(errors_after[0], calculateErrors()(0), 1e-10);
  EXPECT_NEAR(errors_after[1], calculateErrors()(1), 1e-10);
}

TEST_F(ViwlsGraph, LandmarkGeometryVerificationSameLandmark) {
  std::vector<double> errors_before;
  std::vector<double> errors_after;

  landmark_geometry_->getMergeReprojectionErrors(
      landmark_ids_[0], landmark_ids_[0], &errors_before, &errors_after);

  EXPECT_EQ(errors_before.size(), 2u);
  EXPECT_EQ(errors_after.size(), 2u);

  EXPECT_DOUBLE_EQ(errors_before[0], 0.0);
  EXPECT_DOUBLE_EQ(errors_before[1], 0.0);
  EXPECT_DOUBLE_EQ(errors_after[0], 0.0);
  EXPECT_DOUBLE_EQ(errors_after[1], 0.0);
}

}  // namespace map_optimization_legacy

MAPLAB_UNITTEST_ENTRYPOINT
