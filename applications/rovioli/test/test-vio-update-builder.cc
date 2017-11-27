#include <aslam/cameras/ncamera.h>
#include <aslam/frames/visual-nframe.h>
#include <gtest/gtest.h>
#include <maplab-common/test/testing-entrypoint.h>

#include "rovioli/vio-update-builder.h"

namespace rovioli {

class VioUpdateBuilderTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    vio_update_builder_.registerVioUpdatePublishFunction(
        [this](const vio::VioUpdate::ConstPtr& update) {
          received_vio_update_ = update;
        });

    // Send dummy ViNodeStates.
    for (int64_t timestamp_ms = 0; timestamp_ms <= kMaxViNodeTimestampMs;
         timestamp_ms += 10) {
      int64_t timestamp_ns = timestamp_ms * 1e6;
      addViNodeToVioUpdateBuilder(timestamp_ns);
    }

    ASSERT_TRUE(received_vio_update_ == nullptr);

    constexpr size_t kNumCameras = 1u;
    n_camera_ = aslam::NCamera::createTestNCamera(kNumCameras);
  }

 protected:
  static constexpr int64_t kMaxViNodeTimestampMs = 500;
  static constexpr size_t kGyroBiasYOffset = 1000u;

  void addViNodeToVioUpdateBuilder(const int64_t timestamp_ns) {
    vio::ViNodeState vi_node;
    vi_node.set_T_M_I(
        aslam::Transformation(
            aslam::Position3D(timestamp_ns, 0, 0), aslam::Quaternion()));
    vi_node.set_v_M_I(Eigen::Vector3d(0, timestamp_ns, 0));
    vi_node.setAccBias(Eigen::Vector3d(0, 0, timestamp_ns));
    vi_node.setGyroBias(
        Eigen::Vector3d(timestamp_ns, kGyroBiasYOffset + timestamp_ns, 0));
    RovioEstimate::Ptr rovio_estimate = aligned_shared<RovioEstimate>();
    rovio_estimate->vinode = vi_node;
    rovio_estimate->timestamp_s =
        aslam::time::nanoSecondsToSeconds(timestamp_ns);
    vio_update_builder_.processRovioEstimate(rovio_estimate);
  }

  VioUpdateBuilder vio_update_builder_;
  vio::VioUpdate::ConstPtr received_vio_update_;
  aslam::NCamera::Ptr n_camera_;
};

TEST_F(VioUpdateBuilderTest, VioUpdateBuilderTest) {
  for (int64_t timestamp_ms :
       {0,   1,   2,   10,  100, 200, 250, 251, 252, 253, 255, 256,
        258, 259, 260, 261, 400, 401, 405, 409, 490, 498, 499, 500}) {
    const int64_t timestamp_ns = timestamp_ms * 1e6;

    // We can check match multiple times for 500 because it's the last entry and
    // it is kept for possible future interpolation.
    received_vio_update_ = nullptr;
    VLOG(1) << "Checking with timestamp " << timestamp_ns;
    vio::SynchronizedNFrameImu::Ptr synced_nframe_imu =
        aligned_shared<vio::SynchronizedNFrameImu>();
    synced_nframe_imu->nframe =
        aslam::VisualNFrame::createEmptyTestVisualNFrame(
            n_camera_, timestamp_ns);
    vio_update_builder_.processSynchronizedNFrameImu(synced_nframe_imu);
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
  for (int64_t timestamp_ms : {0, 100, 499, 500, -1000}) {
    const int64_t timestamp_ns = timestamp_ms * 1e6;

    received_vio_update_ = nullptr;
    VLOG(1) << "Checking with timestamp " << timestamp_ns;
    vio::SynchronizedNFrameImu::Ptr synced_nframe_imu =
        aligned_shared<vio::SynchronizedNFrameImu>();
    synced_nframe_imu->nframe =
        aslam::VisualNFrame::createEmptyTestVisualNFrame(
            n_camera_, timestamp_ns);
    EXPECT_DEATH(
        vio_update_builder_.processSynchronizedNFrameImu(synced_nframe_imu),
        "vio-update-builder.*Check failed");
  }

  // These should fail because there are no matching ViNodes for interpolation.
  for (int64_t timestamp_ms : {501, 505, 600}) {
    const int64_t timestamp_ns = timestamp_ms * 1e6;

    received_vio_update_ = nullptr;
    VLOG(1) << "Checking with timestamp_ns " << timestamp_ns;
    vio::SynchronizedNFrameImu::Ptr synced_nframe_imu =
        aligned_shared<vio::SynchronizedNFrameImu>();
    synced_nframe_imu->nframe =
        aslam::VisualNFrame::createEmptyTestVisualNFrame(
            n_camera_, timestamp_ns);
    vio_update_builder_.processSynchronizedNFrameImu(synced_nframe_imu);
    EXPECT_TRUE(received_vio_update_ == nullptr);
    vio_update_builder_.clearSynchronizedNFrameImuQueue();
  }
}

}  // namespace rovioli

MAPLAB_UNITTEST_ENTRYPOINT
