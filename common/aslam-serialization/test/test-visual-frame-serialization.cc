#include <maplab-common/test/testing-entrypoint.h>

#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/distortion-radtan.h>
#include <aslam/cameras/ncamera.h>
#include <aslam/cameras/random-camera-generator.h>
#include <aslam/frames/visual-frame.h>
#include <aslam/frames/visual-nframe.h>

#include "aslam-serialization/visual-frame-serialization.h"
#include "aslam-serialization/visual-frame.pb.h"

namespace aslam {

TEST(VisualFrameSerialization, SerializeDeserializeEmptyVisualFrame) {
  const Camera::ConstPtr camera =
      aslam::PinholeCamera::createTestCamera<RadTanDistortion>();
  constexpr int64_t kTimestampNs = 0;
  const VisualFrame::ConstPtr frame =
      VisualFrame::createEmptyTestVisualFrame(camera, kTimestampNs);

  proto::VisualFrame frame_proto;
  serialization::serializeVisualFrame(*frame, &frame_proto);

  VisualFrame::Ptr frame_deserialized;
  serialization::deserializeVisualFrame(frame_proto, &frame_deserialized);

  EXPECT_TRUE(frame->compareWithoutCameraGeometry(*frame_deserialized));

  frame_deserialized.reset();
  serialization::deserializeVisualFrame(
      frame_proto, camera, &frame_deserialized);
  EXPECT_EQ(*frame, *frame_deserialized);
}

TEST(VisualFrameSerialization, SerializeDeserializeVisualFrame) {
  const Camera::ConstPtr camera =
      PinholeCamera::createTestCamera<RadTanDistortion>();
  constexpr int64_t kTimestampNs = 12345678;
  VisualFrame::Ptr frame =
      VisualFrame::createEmptyTestVisualFrame(camera, kTimestampNs);

  // Set some random Data.
  constexpr size_t kNumRandomValues = 10;
  Eigen::Matrix2Xd keypoints = Eigen::Matrix2Xd::Random(2, kNumRandomValues);
  frame->setKeypointMeasurements(keypoints);
  Eigen::VectorXd uncertainties = Eigen::VectorXd::Random(kNumRandomValues);
  frame->setKeypointMeasurementUncertainties(uncertainties);
  Eigen::VectorXd scales = Eigen::VectorXd::Random(kNumRandomValues);
  frame->setKeypointScales(scales);
  aslam::VisualFrame::DescriptorsT descriptors =
      aslam::VisualFrame::DescriptorsT::Random(384, kNumRandomValues);
  frame->setDescriptors(descriptors);
  Eigen::VectorXi track_ids = Eigen::VectorXi::Random(kNumRandomValues);
  frame->setTrackIds(track_ids);

  Eigen::Matrix4Xd boxes = Eigen::Matrix4Xd::Random(4, kNumRandomValues);
  frame->setSemanticObjectMeasurements(boxes);
  Eigen::VectorXd sem_uncertainties = Eigen::VectorXd::Random(kNumRandomValues);
  frame->setSemanticObjectMeasurementUncertainties(sem_uncertainties);
  aslam::VisualFrame::SemanticObjectDescriptorsT sem_descriptors =
      aslam::VisualFrame::SemanticObjectDescriptorsT::Random(4096, kNumRandomValues);
  frame->setSemanticObjectDescriptors(sem_descriptors);
  Eigen::VectorXi sem_class_ids = Eigen::VectorXi::Random(kNumRandomValues);
  frame->setSemanticObjectClassIds(sem_class_ids);
  Eigen::VectorXi sem_track_ids = Eigen::VectorXi::Random(kNumRandomValues);
  frame->setSemanticObjectTrackIds(sem_track_ids);


  proto::VisualFrame frame_proto;
  serialization::serializeVisualFrame(*frame, &frame_proto);

  VisualFrame::Ptr frame_deserialized;
  serialization::deserializeVisualFrame(frame_proto, &frame_deserialized);

  EXPECT_TRUE(frame->compareWithoutCameraGeometry(*frame_deserialized))<< "compareWithoutCameraGeometry test fail";

  frame_deserialized.reset();
  serialization::deserializeVisualFrame(
      frame_proto, camera, &frame_deserialized);
  

  EXPECT_EQ(*frame, *frame_deserialized)<<"equal frame fail";
}

TEST(VisualFrameSerialization, SerializeDeserializeNFrame) {
  constexpr size_t kNumCameras = 2u;
  const NCamera::Ptr n_camera = createTestNCamera(kNumCameras);
  constexpr int64_t kTimestampNs = 0;
  const VisualNFrame::ConstPtr n_frame =
      VisualNFrame::createEmptyTestVisualNFrame(n_camera, kTimestampNs);

  proto::VisualNFrame n_frame_proto;
  serialization::serializeVisualNFrame(*n_frame, &n_frame_proto);

  VisualNFrame::Ptr n_frame_deserialized;
  serialization::deserializeVisualNFrame(n_frame_proto, &n_frame_deserialized);

  EXPECT_TRUE(n_frame->compareWithoutCameraSystem(*n_frame_deserialized));

  n_frame_deserialized.reset();
  serialization::deserializeVisualNFrame(
      n_frame_proto, n_camera, &n_frame_deserialized);
  EXPECT_EQ(*n_frame, *n_frame_deserialized);
}

}  // namespace aslam

MAPLAB_UNITTEST_ENTRYPOINT
