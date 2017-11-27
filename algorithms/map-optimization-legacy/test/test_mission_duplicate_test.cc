#include <memory>
#include <random>
#include <unordered_map>
#include <unordered_set>

#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/distortion-fisheye.h>
#include <aslam/cameras/ncamera.h>
#include <aslam/frames/visual-frame.h>
#include <glog/logging.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>
#include <maplab-common/unique-id.h>
#include <vi-map/check-map-consistency.h>
#include <vi-map/pose-graph.h>
#include <vi-map/vi-map.h>

namespace map_optimization_legacy {

class ViwlsGraph : public ::testing::Test {
  typedef aslam::FisheyeDistortion DistortionType;
  typedef aslam::PinholeCamera CameraType;

 protected:
  ViwlsGraph() {}
  virtual void SetUp() {
    constructCamera();
    addVertices();
    addLandmarksAndKeypoints();

    CHECK(!landmark_ids_.empty());
  }

  void constructCamera();
  void addVertices();
  void addVisualFrameToVertex(vi_map::Vertex* vertex_ptr);
  void addLandmarksAndKeypoints();

  vi_map::VIMap vi_map_;
  pose_graph::VertexIdList vertex_ids_;
  vi_map::LandmarkIdList landmark_ids_;

  vi_map::MissionId mission_id_;
  aslam::NCamera::Ptr cameras_;

  static constexpr unsigned int kNumOfVertices = 20;
  static constexpr unsigned int kNumOfLandmarksPerStoreLandmark = 5;
  static constexpr unsigned int kNumOfStoreLandmarks = 50;
  static constexpr unsigned int kNumOfKeypointsPerVertex =
      kNumOfStoreLandmarks * kNumOfLandmarksPerStoreLandmark;
  static constexpr int kDescriptorBytes = 48;
  static constexpr int kNumCameras = 1;
  static constexpr int kVisualFrameIndex = 0;
  static const unsigned int kRandomSeed;
};

const unsigned int ViwlsGraph::kRandomSeed = 5u;

void ViwlsGraph::constructCamera() {
  cameras_ = aslam::NCamera::createTestNCamera(kNumCameras);
}

void ViwlsGraph::addVisualFrameToVertex(vi_map::Vertex* vertex_ptr) {
  CHECK_NOTNULL(vertex_ptr);

  aslam::VisualFrame::Ptr frame(new aslam::VisualFrame);
  Eigen::Matrix2Xd img_points_distorted;
  Eigen::VectorXd uncertainties;
  aslam::VisualFrame::DescriptorsT descriptors;
  img_points_distorted.resize(Eigen::NoChange, kNumOfKeypointsPerVertex);
  uncertainties.resize(kNumOfKeypointsPerVertex);
  descriptors.resize(kDescriptorBytes, kNumOfKeypointsPerVertex);
  aslam::FrameId frame_id;
  common::generateId(&frame_id);
  frame->setId(frame_id);
  frame->setKeypointMeasurements(img_points_distorted);
  frame->setKeypointMeasurementUncertainties(uncertainties);
  frame->setDescriptors(descriptors);
  frame->setCameraGeometry(cameras_->getCameraShared(kVisualFrameIndex));
  vertex_ptr->getVisualNFrame().setFrame(kVisualFrameIndex, frame);
}

void ViwlsGraph::addVertices() {
  CHECK(cameras_ != nullptr);
  vertex_ids_.clear();

  generateId(&mission_id_);

  // Dummy IMU data that will be stored on edges.
  Eigen::Matrix<int64_t, 1, Eigen::Dynamic> imu_timestamps;
  Eigen::Matrix<double, 6, Eigen::Dynamic> imu_data;
  imu_timestamps.resize(Eigen::NoChange, 1);
  imu_data.resize(Eigen::NoChange, 1);
  imu_timestamps.setRandom();
  imu_data.setRandom();

  vi_map::Vertex::UniquePtr vertex(new vi_map::Vertex(cameras_));
  pose_graph::VertexId vertex_id;
  generateId(&vertex_id);

  vertex->setId(vertex_id);
  vertex->setMissionId(mission_id_);
  addVisualFrameToVertex(vertex.get());
  VLOG(3) << "Add vertex " << vertex_id.hexString();
  vertex_ids_.push_back(vertex_id);

  vi_map::MissionBaseFrameId baseframe_id;
  generateId(&baseframe_id);
  vi_map::MissionBaseFrame mission_baseframe;
  mission_baseframe.setId(baseframe_id);
  vi_map_.mission_base_frames.emplace(baseframe_id, mission_baseframe);

  vi_map::VIMission::UniquePtr mission(new vi_map::VIMission);
  mission->setId(mission_id_);
  mission->setRootVertexId(vertex_ids_.front());
  mission->setBaseFrameId(baseframe_id);
  vi_map_.missions.emplace(mission_id_, std::move(mission));

  vi_map_.sensor_manager_.addNCamera(cameras_, mission_id_);

  vi_map_.addVertex(std::move(vertex));

  for (unsigned int i = 1; i < kNumOfVertices; ++i) {
    pose_graph::VertexId vertex_id;
    vi_map::Vertex::UniquePtr vertex(new vi_map::Vertex(cameras_));
    generateId(&vertex_id);

    vertex->setId(vertex_id);
    vertex->setMissionId(mission_id_);
    addVisualFrameToVertex(vertex.get());
    vi_map_.addVertex(std::move(vertex));
    vertex_ids_.push_back(vertex_id);

    pose_graph::EdgeId edge_id;
    generateId(&edge_id);
    vi_map::ViwlsEdge::UniquePtr edge(
        new vi_map::ViwlsEdge(
            edge_id, vertex_ids_[i - 1], vertex_ids_[i], imu_timestamps,
            imu_data));
    vi_map_.addEdge(std::move(edge));
  }
}

void ViwlsGraph::addLandmarksAndKeypoints() {
  landmark_ids_.clear();

  CHECK(!vertex_ids_.empty());
  std::default_random_engine generator(kRandomSeed);
  std::uniform_int_distribution<int> distribution(0, vertex_ids_.size() - 1);

  for (unsigned int i = 0; i < kNumOfStoreLandmarks; ++i) {
    pose_graph::VertexId vertex_id_storing_landmark =
        vertex_ids_[distribution(generator)];

    vi_map::LandmarkId landmark_id;
    common::generateId(&landmark_id);
    landmark_ids_.push_back(landmark_id);

    vi_map::Landmark store_landmark;
    store_landmark.setId(landmark_id);
    CHECK(!vertex_ids_.empty());
    vi_map_.getVertex(vertex_id_storing_landmark)
        .getLandmarks()
        .addLandmark(store_landmark);
    vi_map_.landmark_index.addLandmarkAndVertexReference(
        landmark_id, vertex_id_storing_landmark);

    for (unsigned int j = 0; j < kNumOfLandmarksPerStoreLandmark; ++j) {
      for (unsigned int k = 0; k < kNumOfVertices; ++k) {
        vi_map_.getVertex(vertex_ids_[k])
            .addObservedLandmarkId(kVisualFrameIndex, landmark_id);
        vi_map_.getVertex(vertex_id_storing_landmark)
            .getLandmarks()
            .getLandmark(landmark_id)
            .addObservation(
                vertex_ids_[k], kVisualFrameIndex,
                vi_map_.getVertex(vertex_ids_[k])
                        .observedLandmarkIdsSize(kVisualFrameIndex) -
                    1);
      }
    }
  }

  // Also add dummy keypoint-uncertainty-descriptor data to vertices.
  for (unsigned int i = 0; i < kNumOfVertices; ++i) {
    vi_map::Vertex& vertex = vi_map_.getVertex(vertex_ids_[i]);
    const int num_of_keypoints =
        vertex.observedLandmarkIdsSize(kVisualFrameIndex);

    Eigen::Matrix2Xd img_points_distorted;
    Eigen::VectorXd uncertainties;
    aslam::VisualFrame::DescriptorsT descriptors;
    const unsigned int descriptor_size = 48;
    img_points_distorted.resize(Eigen::NoChange, num_of_keypoints);
    uncertainties.resize(num_of_keypoints);
    descriptors.resize(descriptor_size, num_of_keypoints);
    img_points_distorted.setZero();
    uncertainties.setZero();
    descriptors.setZero();
    vertex.getVisualFrame(kVisualFrameIndex)
        .setKeypointMeasurements(img_points_distorted);
    vertex.getVisualFrame(kVisualFrameIndex)
        .setKeypointMeasurementUncertainties(uncertainties);
    vertex.getVisualFrame(kVisualFrameIndex).setDescriptors(descriptors);
  }
}

TEST_F(ViwlsGraph, MissionDuplicateTest) {
  EXPECT_TRUE(checkMapConsistency(vi_map_));
  vi_map_.duplicateMission(mission_id_);
  EXPECT_TRUE(checkMapConsistency(vi_map_));
}

TEST_F(ViwlsGraph, NCameraTest) {
  vi_map_.duplicateMission(mission_id_);

  vi_map::MissionIdList all_missions;
  vi_map_.getAllMissionIds(&all_missions);
  vi_map::MissionId duplicated_mission_id;
  ASSERT_EQ(all_missions.size(), 2u);

  const vi_map::SensorManager& sensor_manager = vi_map_.getSensorManager();

  // Check that all camera IDs in the map are unique.
  aslam::CameraIdSet all_camera_ids;
  aslam::NCameraIdSet all_ncamea_ids;
  for (const vi_map::MissionId& mission_id : all_missions) {
    if (mission_id != mission_id_) {
      duplicated_mission_id = mission_id;
    }

    const aslam::NCamera& ncamera =
        sensor_manager.getNCameraForMission(mission_id);
    EXPECT_TRUE(ncamera.getId().isValid());
    EXPECT_TRUE(all_ncamea_ids.emplace(ncamera.getId()).second);
    for (size_t i = 0; i < ncamera.numCameras(); ++i) {
      const aslam::Camera& camera = ncamera.getCamera(i);
      EXPECT_TRUE(camera.getId().isValid());
      EXPECT_TRUE(all_camera_ids.emplace(camera.getId()).second);
    }
  }
  ASSERT_TRUE(duplicated_mission_id.isValid());

  // Check other properties of the ncamera systems.
  const aslam::NCamera& source_ncam =
      sensor_manager.getNCameraForMission(mission_id_);
  const aslam::NCamera& duplicated_ncam =
      sensor_manager.getNCameraForMission(duplicated_mission_id);

  ASSERT_EQ(source_ncam.getNumCameras(), duplicated_ncam.getNumCameras());
  // Check individual camera parameters.
  for (size_t i = 0; i < source_ncam.getNumCameras(); ++i) {
    EXPECT_EQ(duplicated_ncam.get_T_C_B(i), source_ncam.get_T_C_B(i));
    const aslam::Camera& source_cam = source_ncam.getCamera(i);
    const aslam::Camera& duplicated_cam = duplicated_ncam.getCamera(i);
    EXPECT_EQ(duplicated_cam.getType(), source_cam.getType());
    EXPECT_EQ(duplicated_cam.getParameters(), source_cam.getParameters());
    EXPECT_EQ(
        duplicated_cam.getDistortion().getType(),
        source_cam.getDistortion().getType());
    EXPECT_EQ(
        duplicated_cam.getDistortion().getParameters(),
        source_cam.getDistortion().getParameters());
  }
}

}  // namespace map_optimization_legacy

MAPLAB_UNITTEST_ENTRYPOINT
