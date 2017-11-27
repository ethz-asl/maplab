#include <memory>
#include <string>

#include <Eigen/Core>
#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/distortion-fisheye.h>
#include <aslam/common/hash-id.h>
#include <aslam/common/memory.h>
#include <aslam/frames/visual-frame.h>
#include <maplab-common/pose_types.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>
#include <maplab-common/unique-id.h>
#include <posegraph/unique-id.h>

#include "vi-map/unique-id.h"
#include "vi-map/vertex.h"
#include "vi-map/vi-mission.h"
#include "vi-map/vi_map.pb.h"

class ViwlsGraph : public ::testing::Test {
 protected:
  void constructVertex();
  void constructVertexWithVisualNFrame();
  void serializeAndDeserialize();

  vi_map::Vertex::UniquePtr initial_vertex_;
  vi_map::Vertex::UniquePtr vertex_from_msg_;

  // Vertex data.
  pose_graph::VertexId vertex_id_;
  Eigen::Matrix<double, 6, 1> biases_;
  Eigen::Vector3d position_;
  Eigen::Quaterniond orientation_;
  Eigen::Vector3d velocity_;
  vi_map::MissionId mission_id_;
  aslam::NFramesId n_frame_id_;
  std::vector<aslam::FrameId> frame_ids_;
  std::vector<int64_t> frame_timestamps_;

  Eigen::Matrix2Xd img_points_distorted_;
  Eigen::VectorXd uncertainties_;
  aslam::VisualFrame::DescriptorsT descriptors_;
  std::vector<vi_map::LandmarkIdList> img_landmarks_;

  aslam::NCamera::Ptr cameras_;

  static constexpr unsigned int kNumNFrames = 5;
};

void ViwlsGraph::serializeAndDeserialize() {
  vi_map::proto::ViwlsVertex vertex_proto, proto_from_msg;
  initial_vertex_->serialize(&vertex_proto);
  std::string serialized_vertex = vertex_proto.SerializeAsString();

  proto_from_msg.ParseFromString(serialized_vertex);

  vertex_from_msg_ = aligned_unique<vi_map::Vertex>(cameras_);
  vertex_from_msg_->deserialize(initial_vertex_->id(), proto_from_msg);
}

void ViwlsGraph::constructVertex() {
  static constexpr int kNumFrames = 1;
  static constexpr int kVisualFrameIndex = 0;

  common::generateId(&vertex_id_);

  biases_ << 1, 2, 3, 4, 5, 6;

  const int num_of_keypoints = 1000;

  img_points_distorted_.resize(2, num_of_keypoints);
  img_points_distorted_.setRandom();
  uncertainties_.resize(num_of_keypoints, 1);
  uncertainties_.setRandom();
  descriptors_.resize(48, num_of_keypoints);
  descriptors_.setRandom();

  img_landmarks_.resize(kNumFrames);
  for (int i = 0; i < num_of_keypoints; ++i) {
    vi_map::LandmarkId landmark_id;
    common::generateId(&landmark_id);
    img_landmarks_[kVisualFrameIndex].push_back(landmark_id);
  }

  mission_id_.fromHexString("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa0");
  aslam::FrameId frame_id;
  frame_id.fromHexString("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa1");
  frame_ids_.push_back(frame_id);
  frame_timestamps_.push_back(37u);

  cameras_ = aslam::NCamera::createTestNCamera(kNumFrames);

  initial_vertex_ = aligned_unique<vi_map::Vertex>(
      vertex_id_, biases_, img_points_distorted_, uncertainties_, descriptors_,
      img_landmarks_[kVisualFrameIndex], mission_id_, frame_id,
      frame_timestamps_.back(), cameras_);

  position_ << 1, 2, 3;
  orientation_ = Eigen::Quaterniond(sqrt(2) / 2, 0, sqrt(2) / 2, 0);

  initial_vertex_->set_p_M_I(position_);
  initial_vertex_->set_q_M_I(orientation_);

  velocity_ << 5, 6, 7;
  initial_vertex_->set_v_M(velocity_);
}

void ViwlsGraph::constructVertexWithVisualNFrame() {
  common::generateId(&vertex_id_);

  biases_ << 1, 2, 3, 4, 5, 6;

  const int num_of_keypoints = 1000;

  img_points_distorted_.resize(2, num_of_keypoints);
  img_points_distorted_.setRandom();
  uncertainties_.resize(num_of_keypoints, 1);
  uncertainties_.setRandom();
  descriptors_.resize(48, num_of_keypoints);
  descriptors_.setRandom();

  img_landmarks_.resize(kNumNFrames);

  mission_id_.fromHexString("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa0");
  cameras_ = aslam::NCamera::createTestNCamera(kNumNFrames);
  common::generateId(&n_frame_id_);
  aslam::VisualNFrame::Ptr visual_n_frame(
      new aslam::VisualNFrame(n_frame_id_, cameras_));

  for (unsigned int i = 0; i < kNumNFrames; ++i) {
    aslam::FrameId frame_id;
    common::generateId(&frame_id);
    frame_ids_.push_back(frame_id);
    const int64_t frame_timestamp = i;
    frame_timestamps_.push_back(frame_timestamp);

    for (int j = 0; j < num_of_keypoints; ++j) {
      vi_map::LandmarkId landmark_id;
      common::generateId(&landmark_id);
      img_landmarks_[i].push_back(landmark_id);
    }

    aslam::VisualFrame::Ptr frame(new aslam::VisualFrame);
    frame->setId(frame_id);
    frame->setTimestampNanoseconds(frame_timestamp);
    frame->setKeypointMeasurements(img_points_distorted_);
    frame->setKeypointMeasurementUncertainties(uncertainties_);
    frame->setDescriptors(descriptors_);
    frame->setCameraGeometry(cameras_->getCameraShared(i));

    visual_n_frame->setFrame(i, frame);
  }

  initial_vertex_ = aligned_unique<vi_map::Vertex>(
      vertex_id_, biases_, visual_n_frame, img_landmarks_, mission_id_);

  position_ << 1, 2, 3;
  orientation_ = Eigen::Quaterniond(sqrt(2) / 2, 0, sqrt(2) / 2, 0);

  initial_vertex_->set_p_M_I(position_);
  initial_vertex_->set_q_M_I(orientation_);

  velocity_ << 5, 6, 7;
  initial_vertex_->set_v_M(velocity_);
}

TEST_F(ViwlsGraph, ViwlsVertexWithSingleFrameSerializationTest) {
  static constexpr int kVisualFrameIndex = 0;

  constructVertex();
  serializeAndDeserialize();

  CHECK(vertex_from_msg_ != nullptr);
  EXPECT_EQ(vertex_id_, vertex_from_msg_->id());
  EXPECT_EQ(mission_id_, vertex_from_msg_->getMissionId());
  EXPECT_EQ(position_, vertex_from_msg_->get_p_M_I());
  EXPECT_EQ(orientation_.coeffs(), vertex_from_msg_->get_q_M_I().coeffs());
  EXPECT_EQ(velocity_, vertex_from_msg_->get_v_M());
  EXPECT_EQ(biases_.head(3), vertex_from_msg_->getAccelBias());
  EXPECT_EQ(biases_.tail(3), vertex_from_msg_->getGyroBias());

  for (unsigned int i = 0; i < img_landmarks_[kVisualFrameIndex].size(); ++i) {
    EXPECT_EQ(
        img_landmarks_[kVisualFrameIndex][i],
        vertex_from_msg_->getObservedLandmarkId(kVisualFrameIndex, i));
  }

  aslam::VisualFrame& visual_frame =
      vertex_from_msg_->getVisualFrame(kVisualFrameIndex);
  EXPECT_EQ(visual_frame.getKeypointMeasurements(), img_points_distorted_);
  EXPECT_EQ(visual_frame.getKeypointMeasurementUncertainties(), uncertainties_);
  EXPECT_EQ(visual_frame.getDescriptors(), descriptors_);
}

TEST_F(ViwlsGraph, ViwlsVertexWithMultiFrameSerializationTest) {
  constructVertexWithVisualNFrame();
  serializeAndDeserialize();

  const unsigned int expected_num_frames = kNumNFrames;

  CHECK(vertex_from_msg_ != nullptr);
  EXPECT_EQ(vertex_id_, vertex_from_msg_->id());
  EXPECT_EQ(mission_id_, vertex_from_msg_->getMissionId());
  EXPECT_EQ(position_, vertex_from_msg_->get_p_M_I());
  EXPECT_EQ(orientation_.coeffs(), vertex_from_msg_->get_q_M_I().coeffs());
  EXPECT_EQ(velocity_, vertex_from_msg_->get_v_M());
  EXPECT_EQ(biases_.head(3), vertex_from_msg_->getAccelBias());
  EXPECT_EQ(biases_.tail(3), vertex_from_msg_->getGyroBias());
  EXPECT_EQ(n_frame_id_, vertex_from_msg_->getVisualNFrame().getId());
  EXPECT_EQ(expected_num_frames, vertex_from_msg_->numFrames());

  for (unsigned int frame_idx = 0; frame_idx < kNumNFrames; ++frame_idx) {
    for (unsigned int i = 0; i < img_landmarks_[frame_idx].size(); ++i) {
      EXPECT_EQ(
          img_landmarks_[frame_idx][i],
          vertex_from_msg_->getObservedLandmarkId(frame_idx, i));
    }

    aslam::VisualFrame& visual_frame =
        vertex_from_msg_->getVisualFrame(frame_idx);
    EXPECT_EQ(
        visual_frame.getTimestampNanoseconds(), frame_timestamps_[frame_idx]);
    EXPECT_EQ(visual_frame.getId(), frame_ids_[frame_idx]);
    EXPECT_EQ(visual_frame.getKeypointMeasurements(), img_points_distorted_);
    EXPECT_EQ(
        visual_frame.getKeypointMeasurementUncertainties(), uncertainties_);
    EXPECT_EQ(visual_frame.getDescriptors(), descriptors_);
  }
}

MAPLAB_UNITTEST_ENTRYPOINT
