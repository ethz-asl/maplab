#include <aslam/cameras/ncamera.h>
#include <eigen-checks/gtest.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>

#include "vio-common/test/vio-update-simulation.h"
#include "vio-common/vio-update-serialization.h"
#include "vio-common/vio-update.h"

#include "vio-common/vio_update.pb.h"

void compareVisualNFrame(
    const aslam::VisualNFrame& n_frame_a,
    const aslam::VisualNFrame& n_frame_b) {
  EXPECT_EQ(n_frame_a.getId(), n_frame_b.getId());
  EXPECT_EQ(n_frame_a.getNCamera(), n_frame_b.getNCamera());

  ASSERT_GT(n_frame_a.getNumFrames(), 0u);
  ASSERT_EQ(n_frame_a.getNumFrames(), n_frame_a.getNumFrames());

  constexpr double kPrecision = 1e-10;
  for (size_t frame_index = 0u; frame_index < n_frame_a.getNumFrames();
       ++frame_index) {
    const aslam::VisualFrame& frame_a = n_frame_a.getFrame(frame_index);
    const aslam::VisualFrame& frame_b = n_frame_b.getFrame(frame_index);

    EXPECT_EQ(
        frame_a.getTimestampNanoseconds(), frame_b.getTimestampNanoseconds());
    EXPECT_EQ(*frame_a.getCameraGeometry(), *frame_b.getCameraGeometry());

    // Check if the channels exist.
    for (const std::string& channel_name :
         {"DESCRIPTORS", "TRACK_IDS", "VISUAL_KEYPOINT_MEASUREMENTS",
          "VISUAL_KEYPOINT_MEASUREMENT_UNCERTAINTIES"}) {
      EXPECT_TRUE(frame_a.hasChannel(channel_name));
      EXPECT_TRUE(frame_b.hasChannel(channel_name));
    }

    ASSERT_TRUE(frame_a.hasDescriptors());
    ASSERT_TRUE(frame_b.hasDescriptors());
    EXPECT_NEAR_EIGEN(
        frame_a.getDescriptors(), frame_b.getDescriptors(), kPrecision);

    ASSERT_TRUE(frame_a.hasTrackIds());
    ASSERT_TRUE(frame_b.hasTrackIds());
    EXPECT_NEAR_EIGEN(frame_a.getTrackIds(), frame_b.getTrackIds(), kPrecision);

    ASSERT_TRUE(frame_a.hasKeypointMeasurements());
    ASSERT_TRUE(frame_b.hasKeypointMeasurements());
    EXPECT_NEAR_EIGEN(
        frame_a.getKeypointMeasurements(), frame_b.getKeypointMeasurements(),
        kPrecision);

    ASSERT_TRUE(frame_a.hasKeypointMeasurementUncertainties());
    ASSERT_TRUE(frame_b.hasKeypointMeasurementUncertainties());
    EXPECT_NEAR_EIGEN(
        frame_a.getKeypointMeasurementUncertainties(),
        frame_b.getKeypointMeasurementUncertainties(), kPrecision);
  }
}

void compareVioUpdates(
    const vio::VioUpdate& vio_update_a, const vio::VioUpdate& vio_update_b) {
  constexpr double kPrecision = 1e-10;
  EXPECT_EQ(vio_update_a.timestamp_ns, vio_update_b.timestamp_ns);
  EXPECT_EQ(vio_update_a.vio_state, vio_update_b.vio_state);
  EXPECT_EQ(vio_update_a.vio_update_type, vio_update_b.vio_update_type);

  // SynchronizedNFrameImu.
  ASSERT_NE(vio_update_a.keyframe_and_imudata, nullptr);
  ASSERT_NE(vio_update_b.keyframe_and_imudata, nullptr);
  EXPECT_NEAR_EIGEN(
      vio_update_a.keyframe_and_imudata->imu_timestamps,
      vio_update_b.keyframe_and_imudata->imu_timestamps, kPrecision);
  EXPECT_NEAR_EIGEN(
      vio_update_a.keyframe_and_imudata->imu_measurements,
      vio_update_b.keyframe_and_imudata->imu_measurements, kPrecision);
  compareVisualNFrame(
      *vio_update_a.keyframe_and_imudata->nframe,
      *vio_update_b.keyframe_and_imudata->nframe);
  EXPECT_EQ(
      vio_update_a.keyframe_and_imudata->motion_wrt_last_nframe,
      vio_update_b.keyframe_and_imudata->motion_wrt_last_nframe);

  // ViNodeState.
  EXPECT_NEAR_ASLAM_TRANSFORMATION(vio_update_a.vinode.get_T_M_I(),
                                   vio_update_b.vinode.get_T_M_I(), kPrecision);
  EXPECT_NEAR_EIGEN(vio_update_a.vinode.get_v_M_I(),
                    vio_update_b.vinode.get_v_M_I(), kPrecision);
  EXPECT_NEAR_EIGEN(
      vio_update_a.vinode.getImuBias(), vio_update_b.vinode.getImuBias(),
      kPrecision);

  // ViNodeCovariance.
  EXPECT_NEAR_EIGEN(vio_update_a.vinode_covariance.get_p_M_I_Covariance(),
                    vio_update_b.vinode_covariance.get_p_M_I_Covariance(),
                    kPrecision);
  EXPECT_NEAR_EIGEN(vio_update_a.vinode_covariance.get_q_M_I_Covariance(),
                    vio_update_b.vinode_covariance.get_q_M_I_Covariance(),
                    kPrecision);
  EXPECT_NEAR_EIGEN(vio_update_a.vinode_covariance.get_v_M_I_Covariance(),
                    vio_update_b.vinode_covariance.get_v_M_I_Covariance(),
                    kPrecision);
  EXPECT_NEAR_EIGEN(
      vio_update_a.vinode_covariance.getAccBiasCovariance(),
      vio_update_b.vinode_covariance.getAccBiasCovariance(), kPrecision);
  EXPECT_NEAR_EIGEN(
      vio_update_a.vinode_covariance.getGyroBiasCovariance(),
      vio_update_b.vinode_covariance.getGyroBiasCovariance(), kPrecision);

  EXPECT_EQ(vio_update_a.localization_state, vio_update_b.localization_state);
  EXPECT_NEAR_ASLAM_TRANSFORMATION(
      vio_update_a.T_G_M, vio_update_b.T_G_M, kPrecision);
}

TEST(VioUpdateSerializationTest, SerializeAndDeserialize) {
  vio::VioUpdateSimulation simulation;
  simulation.generateVioUpdates();

  // Use update 1 as the update 0 doesn't contain imu data (and therefore matrix
  // comparison will
  // fail).
  constexpr size_t kVioUpdateIndex = 1u;
  const vio::VioUpdate::ConstPtr vio_update =
      simulation.getVioUpdate(kVioUpdateIndex);
  const aslam::NCamera::Ptr& n_camera = simulation.getNCamera();

  vio::proto::VioUpdate vio_update_proto;
  vio::serialization::serializeVioUpdate(*vio_update, &vio_update_proto);

  vio::VioUpdate vio_update_deserialized;
  vio::serialization::deserializeVioUpdate(
      vio_update_proto, n_camera, &vio_update_deserialized);

  compareVioUpdates(*vio_update, vio_update_deserialized);
}

MAPLAB_UNITTEST_ENTRYPOINT
