#include <maplab-common/test/testing-entrypoint.h>

#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/distortion-radtan.h>
#include <aslam/cameras/ncamera.h>
#include <aslam/frames/visual-frame.h>
#include <aslam/frames/visual-nframe.h>

#include "aslam-serialization/visual-frame-serialization.h"
#include "aslam-serialization/visual-frame.pb.h"

namespace aslam {

TEST(VisualFrameSerialization, SerializeDeserializeFrame) {
  const Camera::ConstPtr camera =
      PinholeCamera::createTestCamera<RadTanDistortion>();
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

TEST(VisualFrameSerialization, SerializeDeserializeNFrame) {
  constexpr size_t kNumCameras = 2u;
  const NCamera::Ptr n_camera = NCamera::createTestNCamera(kNumCameras);
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
