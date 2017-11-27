#include "vio-common/vio-update-serialization.h"

#include <aslam-serialization/visual-frame-serialization.h>
#include <aslam/cameras/ncamera.h>
#include <maplab-common/aslam-id-proto.h>
#include <maplab-common/eigen-proto.h>

#include "vio-common/vio-types.h"
#include "vio-common/vio-update.h"
#include "vio-common/vio_update.pb.h"

namespace vio {
namespace serialization {

void serializeVioUpdate(
    const vio::VioUpdate& update, vio::proto::VioUpdate* proto) {
  CHECK_NOTNULL(proto);
  CHECK_NOTNULL(update.keyframe_and_imudata.get());
  CHECK_NOTNULL(update.keyframe_and_imudata->nframe.get());

  // Clear protocol buffer, then fill in the new values.
  proto->Clear();

  proto->set_timestamp_ns(update.timestamp_ns);

  proto->set_vio_state(static_cast<int>(update.vio_state));

  proto->set_vio_update_type(static_cast<int>(update.vio_update_type));

  // Serialize SynchronizedNFrameImu.
  vio::proto::SynchronizedNFrameImu* keyframe_and_imudata_proto =
      proto->mutable_keyframe_and_imudata();

  common::eigen_proto::serialize(
      update.keyframe_and_imudata->imu_timestamps,
      keyframe_and_imudata_proto->mutable_imu_timestamps());

  common::eigen_proto::serialize(
      update.keyframe_and_imudata->imu_measurements,
      keyframe_and_imudata_proto->mutable_imu_measurements());

  keyframe_and_imudata_proto->set_motion_wrt_last_nframe(
      static_cast<int>(update.keyframe_and_imudata->motion_wrt_last_nframe));

  // Add VisualNFrame to protocol buffer.
  aslam::serialization::serializeVisualNFrame(
      *update.keyframe_and_imudata->nframe,
      keyframe_and_imudata_proto->mutable_nframe());

  // Serialize ViNodeState, field 5.
  vio::proto::ViNodeState* vinode_proto = proto->mutable_vinode();
  common::eigen_proto::serialize(update.vinode.get_T_M_I(),
                                 vinode_proto->mutable_t_w_b());
  common::eigen_proto::serialize(update.vinode.get_v_M_I(),
                                 vinode_proto->mutable_w_v_b());
  common::eigen_proto::serialize(
      update.vinode.getAccBias(), vinode_proto->mutable_acc_bias());
  common::eigen_proto::serialize(
      update.vinode.getGyroBias(), vinode_proto->mutable_gyro_bias());

  proto->set_localization_state(static_cast<int>(update.localization_state));

  common::eigen_proto::serialize(update.T_G_M, proto->mutable_t_g_m());
}

void deserializeVioUpdate(
    const vio::proto::VioUpdate& proto, vio::VioUpdate* update) {
  deserializeVioUpdate(proto, nullptr, update);
}

void deserializeVioUpdate(
    const vio::proto::VioUpdate& proto, aslam::NCamera::Ptr n_camera,
    vio::VioUpdate* update) {
  CHECK_NOTNULL(update);

  CHECK(proto.has_timestamp_ns());
  update->timestamp_ns = proto.timestamp_ns();

  CHECK(proto.has_vio_state());
  update->vio_state = static_cast<EstimatorState>(proto.vio_state());

  CHECK(proto.has_vio_update_type());
  update->vio_update_type = static_cast<UpdateType>(proto.vio_update_type());

  // Deserialize SynchronizedNFrameImu.
  vio::SynchronizedNFrameImu::Ptr keyframe_and_imu =
      std::make_shared<vio::SynchronizedNFrameImu>();
  CHECK(proto.has_keyframe_and_imudata());
  vio::proto::SynchronizedNFrameImu keyframe_and_imu_proto =
      proto.keyframe_and_imudata();

  common::eigen_proto::deserialize(
      keyframe_and_imu_proto.imu_timestamps(),
      &keyframe_and_imu->imu_timestamps);
  common::eigen_proto::deserialize(
      keyframe_and_imu_proto.imu_measurements(),
      &keyframe_and_imu->imu_measurements);

  CHECK(keyframe_and_imu_proto.has_motion_wrt_last_nframe());
  keyframe_and_imu->motion_wrt_last_nframe = static_cast<vio::MotionType>(
      keyframe_and_imu_proto.motion_wrt_last_nframe());

  aslam::VisualNFrame::Ptr n_frame;
  aslam::serialization::deserializeVisualNFrame(
      keyframe_and_imu_proto.nframe(), n_camera, &n_frame);

  keyframe_and_imu->nframe = n_frame;
  update->keyframe_and_imudata = keyframe_and_imu;

  CHECK(proto.has_vinode());
  vio::proto::ViNodeState vinode_proto = proto.vinode();

  aslam::Transformation T_M_I = update->vinode.get_T_M_I();
  common::eigen_proto::deserialize(vinode_proto.t_w_b(), &T_M_I);
  update->vinode.set_T_M_I(T_M_I);

  Eigen::Vector3d v_M_I = update->vinode.get_v_M_I();
  common::eigen_proto::deserialize(vinode_proto.w_v_b(), &v_M_I);
  update->vinode.set_v_M_I(v_M_I);

  Eigen::Vector3d acc_bias = update->vinode.getAccBias();
  common::eigen_proto::deserialize(vinode_proto.acc_bias(), &acc_bias);
  update->vinode.setAccBias(acc_bias);

  Eigen::Vector3d gyro_bias = update->vinode.getGyroBias();
  common::eigen_proto::deserialize(vinode_proto.gyro_bias(), &gyro_bias);
  update->vinode.setGyroBias(gyro_bias);

  update->localization_state =
      static_cast<vio::LocalizationState>(proto.localization_state());

  common::eigen_proto::deserialize(proto.t_g_m(), &update->T_G_M);
}

}  // namespace serialization
}  // namespace vio
