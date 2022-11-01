#include <aslam/cameras/ncamera.h>
#include <aslam/cameras/random-camera-generator.h>
#include <aslam/frames/visual-nframe.h>
#include <gtest/gtest.h>
#include <maplab-common/test/testing-entrypoint.h>
#include "maplab-node/map-update-builder.h"

namespace maplab {

class MapUpdateBuilderTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    vio_update_builder_.registerMapUpdatePublishFunction(
        [this](const vio::MapUpdate::ConstPtr& update) {
          received_vio_update_ = update;
        });

    // Send dummy ViNodeStates.
    for (int64_t timestamp_ms = 0; timestamp_ms <= kMaxViNodeTimestampMs;
         timestamp_ms += 1) {
      int64_t timestamp_ns = timestamp_ms * 1e6;
      addViNodeToVioUpdateBuilder(timestamp_ns);
      addIMUMeasurementToBuffer(timestamp_ns);
    }
    ASSERT_TRUE(received_vio_update_ == nullptr);

    constexpr size_t kNumCameras = 1u;
    n_camera_ = aslam::createTestNCamera(kNumCameras);
  }
  MapUpdateBuilderTest()
      : buffer_(aslam::time::seconds(30u), aslam::time::milliseconds(500)),
        vio_update_builder_(buffer_) {}

 protected:
  static constexpr int64_t kMaxViNodeTimestampMs = 500;
  static constexpr size_t kGyroBiasYOffset = 1000u;

  void addViNodeToVioUpdateBuilder(const int64_t timestamp_ns) {
    vio::ViNodeState vi_node;
    vi_node.set_T_M_I(aslam::Transformation(
        aslam::Position3D(timestamp_ns, 0, 0), aslam::Quaternion()));
    vi_node.set_v_M_I(Eigen::Vector3d(0, timestamp_ns, 0));
    vi_node.setAccBias(Eigen::Vector3d(0, 0, timestamp_ns));
    vi_node.setGyroBias(
        Eigen::Vector3d(timestamp_ns, kGyroBiasYOffset + timestamp_ns, 0));
    vi_node.setTimestamp(timestamp_ns);
    buffer_.bufferOdometryEstimate(vi_node);
  }

  void addIMUMeasurementToBuffer(const int64_t timestamp_ns) {
    vio::ImuData data;
    vio::ImuMeasurement imu_measurement(timestamp_ns, data);
    buffer_.bufferImuMeasurement(imu_measurement);
  }

  vio_common::PoseLookupBuffer buffer_;
  MapUpdateBuilder vio_update_builder_;
  vio::MapUpdate::ConstPtr received_vio_update_;
  aslam::NCamera::Ptr n_camera_;
};

TEST_F(MapUpdateBuilderTest, Builder_Works) {
  // initialize it for time 0
  const int64_t t0 = 0;
  received_vio_update_ = nullptr;
  VLOG(1) << "Checking with timestamp " << t0;
  vio::SynchronizedNFrame::Ptr synced_nframe =
      aligned_shared<vio::SynchronizedNFrame>();
  synced_nframe->nframe =
      aslam::VisualNFrame::createEmptyTestVisualNFrame(n_camera_, t0);
  vio_update_builder_.processTrackedNFrame(synced_nframe);

  for (int64_t timestamp_ms :
       {1,   2,   10,  100, 200, 250, 251, 252, 253, 255, 256, 258,
        259, 260, 261, 400, 401, 405, 409, 490, 498, 499, 500}) {
    const int64_t timestamp_ns = timestamp_ms * 1e6;

    // We can check match multiple times for 500 because it's the last entry and
    // it is kept for possible future interpolation.
    received_vio_update_ = nullptr;
    VLOG(1) << "Checking with timestamp " << timestamp_ns;
    vio::SynchronizedNFrame::Ptr synced_nframe_imu =
        aligned_shared<vio::SynchronizedNFrame>();
    synced_nframe_imu->nframe =
        aslam::VisualNFrame::createEmptyTestVisualNFrame(
            n_camera_, timestamp_ns);
    vio_update_builder_.processTrackedNFrame(synced_nframe_imu);

    ASSERT_TRUE(received_vio_update_ != nullptr);
    EXPECT_EQ(timestamp_ns, received_vio_update_->timestamp_ns);
    EXPECT_EQ(
        timestamp_ns,
        received_vio_update_->vinode.get_T_M_I().getPosition()[0]);
    EXPECT_EQ(timestamp_ns, received_vio_update_->vinode.get_v_M_I()[1]);
    EXPECT_EQ(timestamp_ns, received_vio_update_->vinode.getGyroBias()[0]);
    EXPECT_EQ(
        kGyroBiasYOffset + timestamp_ns,
        received_vio_update_->vinode.getGyroBias()[1]);
  }

  // Going back by timestamp should crash the program.
  for (int64_t timestamp_ms : {100, 499, 500, -1000}) {
    const int64_t timestamp_ns = timestamp_ms * 1e6;

    received_vio_update_ = nullptr;
    VLOG(1) << "Checking for failure with timestamp " << timestamp_ns;
    vio::SynchronizedNFrame::Ptr synced_nframe_imu =
        aligned_shared<vio::SynchronizedNFrame>();
    synced_nframe_imu->nframe =
        aslam::VisualNFrame::createEmptyTestVisualNFrame(
            n_camera_, timestamp_ns);
    EXPECT_DEATH(
        vio_update_builder_.processTrackedNFrame(synced_nframe_imu),
        "map-update-builder.*Check failed");
  }

  // These should fail because there are no matching ViNodes for interpolation.
  // The hard check inside the map update builder now fails because
  // the synchronizer releases data iff odometry is available
  for (int64_t timestamp_ms : {501, 505, 600}) {
    const int64_t timestamp_ns = timestamp_ms * 1e6;

    received_vio_update_ = nullptr;
    VLOG(1) << "Checking with timestamp_ns " << timestamp_ns;
    vio::SynchronizedNFrame::Ptr synced_nframe_imu =
        aligned_shared<vio::SynchronizedNFrame>();
    synced_nframe_imu->nframe =
        aslam::VisualNFrame::createEmptyTestVisualNFrame(
            n_camera_, timestamp_ns);
    EXPECT_DEATH(
        vio_update_builder_.processTrackedNFrame(synced_nframe_imu),
        "map-update-builder.*Check failed");
  }
}

}  // namespace maplab

MAPLAB_UNITTEST_ENTRYPOINT
