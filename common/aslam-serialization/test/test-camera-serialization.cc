#include <maplab-common/test/testing-entrypoint.h>

#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/distortion-radtan.h>
#include <aslam/cameras/ncamera.h>

#include "aslam-serialization/camera-serialization.h"
#include "aslam-serialization/camera.pb.h"

namespace aslam {

TEST(CameraSerialization, SerializeDeserializeCamera) {
  const Camera::ConstPtr camera =
      PinholeCamera::createTestCamera<RadTanDistortion>();
  proto::Camera camera_proto;
  serialization::serializeCamera(*camera, &camera_proto);

  Camera::Ptr camera_deserialized;
  serialization::deserializeCamera(camera_proto, &camera_deserialized);
  EXPECT_EQ(*camera, *camera_deserialized);
}

TEST(CameraSerialization, SerializeDeserializeNCamera) {
  constexpr size_t kNumCameras = 2u;
  const NCamera::Ptr n_camera = NCamera::createTestNCamera(kNumCameras);
  proto::NCamera n_camera_proto;
  serialization::serializeNCamera(*n_camera, &n_camera_proto);

  NCamera::Ptr n_camera_deserialized;
  serialization::deserializeNCamera(n_camera_proto, &n_camera_deserialized);
  EXPECT_EQ(*n_camera, *n_camera_deserialized);
}

}  // namespace aslam

MAPLAB_UNITTEST_ENTRYPOINT
