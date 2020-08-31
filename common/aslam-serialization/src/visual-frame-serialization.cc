#include "aslam-serialization/visual-frame-serialization.h"

#include <aslam/cameras/camera.h>
#include <aslam/cameras/ncamera.h>
#include <aslam/frames/visual-frame.h>
#include <aslam/frames/visual-nframe.h>
#include <glog/logging.h>
#include <maplab-common/eigen-proto.h>

namespace aslam {
namespace serialization {

void serializeVisualFrame(
    const aslam::VisualFrame& frame, aslam::proto::VisualFrame* proto) {
  CHECK_NOTNULL(proto);

  frame.getId().serialize(proto->mutable_id());
  proto->set_timestamp(frame.getTimestampNanoseconds());

  if (frame.hasKeypointMeasurements()) {
    ::common::eigen_proto::serialize(
        frame.getKeypointMeasurements(),
        proto->mutable_keypoint_measurements());
    ::common::eigen_proto::serialize(
        frame.getKeypointMeasurementUncertainties(),
        proto->mutable_keypoint_measurement_sigmas());
    CHECK_EQ(
        proto->keypoint_measurements_size(),
        2 * proto->keypoint_measurement_sigmas_size());

    if (frame.hasKeypointScales()) {
      ::common::eigen_proto::serialize(
          frame.getKeypointScales(), proto->mutable_descriptor_scales());
      CHECK_EQ(
          proto->keypoint_measurement_sigmas_size(),
          proto->descriptor_scales_size());
    }

    const aslam::VisualFrame::DescriptorsT& descriptors =
        frame.getDescriptors();
    VLOG(200) << "Frame " << frame.getId() << " has " << descriptors.cols()
              << " descriptors!";
    internal::serializeDescriptors(descriptors, proto);

    proto->set_is_valid(frame.isValid());

    if (frame.hasTrackIds()) {
      ::common::eigen_proto::serialize(
          frame.getTrackIds(), proto->mutable_track_ids());
    }
  } else {
    VLOG(200) << "Frame " << frame.getId() << " has no descriptors!";
  }
  if (frame.hasLidarTrackIds()) {
    ::common::eigen_proto::serialize(
        frame.getLidarTrackIds(), proto->mutable_lidar_track_ids());
  }
  if (frame.hasLidarKeypoint3DMeasurements()) {
    ::common::eigen_proto::serialize(
        frame.getLidarKeypoint3DMeasurements(),
        proto->mutable_lidar_3d_measurements());
  }
  if (frame.hasLidarKeypoint2DMeasurements()) {
    ::common::eigen_proto::serialize(
        frame.getLidarKeypoint2DMeasurements(),
        proto->mutable_lidar_2d_measurements());
  }
  if (frame.hasLidarDescriptors()) {
    const aslam::VisualFrame::DescriptorsT& descriptors =
        frame.getLidarDescriptors();
    VLOG(200) << "Frame " << frame.getId() << " has " << descriptors.cols()
              << " LiDAR descriptors!";
    internal::serializeLidarDescriptors(descriptors, proto);
  }
}

void deserializeVisualFrame(
    const aslam::proto::VisualFrame& proto, aslam::VisualFrame::Ptr* frame) {
  CHECK_NOTNULL(frame);
  aslam::Camera::Ptr camera;
  deserializeVisualFrame(proto, camera, frame);
}

void deserializeVisualFrame(
    const aslam::proto::VisualFrame& proto,
    const aslam::Camera::ConstPtr& camera, aslam::VisualFrame::Ptr* frame) {
  CHECK_NOTNULL(frame)->reset();

  aslam::FrameId frame_id;
  frame_id.deserialize(proto.id());
  // If the frame_id is invalid this frame has been un-set.
  if (frame_id.isValid()) {
    *frame = aligned_shared<aslam::VisualFrame>();
    aslam::VisualFrame& frame_ref = **frame;

    bool success = true;
    success &=
        (2 * proto.keypoint_measurement_sigmas_size() ==
         proto.keypoint_measurements_size());
    if (proto.keypoint_descriptor_size() != 0) {
      success &=
          (proto.keypoint_descriptors().size() /
               proto.keypoint_descriptor_size() ==
           static_cast<unsigned int>(proto.keypoint_measurement_sigmas_size()));
    }

    CHECK(success) << "Inconsistent landmark Visual Frame field sizes.";

    frame_ref.setId(frame_id);
    frame_ref.setTimestampNanoseconds(proto.timestamp());

    if (proto.keypoint_descriptor_size() != 0) {
      Eigen::Map<const Eigen::Matrix2Xd> img_points_distorted(
          proto.keypoint_measurements().data(), 2,
          proto.keypoint_measurements_size() / 2);
      Eigen::Map<const Eigen::VectorXd> uncertainties(
          proto.keypoint_measurement_sigmas().data(),
          proto.keypoint_measurement_sigmas_size());
      Eigen::Map<const Eigen::VectorXd> scales(
          proto.descriptor_scales().data(), proto.descriptor_scales_size());
      Eigen::Map<const Eigen::VectorXi> track_ids(
          proto.track_ids().data(), proto.track_ids_size());

      if (camera != nullptr) {
        frame_ref.setCameraGeometry(camera);
      }

      frame_ref.setKeypointMeasurements(img_points_distorted);
      frame_ref.setKeypointMeasurementUncertainties(uncertainties);
      if (scales.rows() != 0) {
        CHECK_EQ(scales.rows(), uncertainties.rows());
        frame_ref.setKeypointScales(scales);
      }
      if (track_ids.rows() != 0) {
        CHECK_EQ(track_ids.rows(), uncertainties.rows());
        frame_ref.setTrackIds(track_ids);
      }

      // Need to set empty descriptors, otherwise getMutable call below fails.
      frame_ref.setDescriptors(aslam::VisualFrame::DescriptorsT());
      internal::deserializeDescriptors(
          proto, frame_ref.getDescriptorsMutable());

      CHECK(frame_ref.hasKeypointMeasurements());
      CHECK(frame_ref.hasKeypointMeasurementUncertainties());
      CHECK(frame_ref.hasDescriptors());
    }
    // LiDAR features
    Eigen::Map<const Eigen::Matrix3Xd> lidar_3d_measurements(
        proto.lidar_3d_measurements().data(), 3,
        proto.lidar_3d_measurements_size() / 3);
    Eigen::Map<const Eigen::Matrix2Xd> lidar_2d_measurements(
        proto.lidar_2d_measurements().data(), 2,
        proto.lidar_2d_measurements_size() / 2);
    Eigen::Map<const Eigen::VectorXi> lidar_track_ids(
        proto.lidar_track_ids().data(), proto.lidar_track_ids_size());

    if (proto.lidar_track_ids_size() != 0) {
      frame_ref.setLidarTrackIds(lidar_track_ids);
    }
    if (proto.lidar_3d_measurements_size() != 0) {
      frame_ref.setLidarKeypoint3DMeasurements(lidar_3d_measurements);
    }
    if (proto.lidar_2d_measurements_size() != 0) {
      frame_ref.setLidarKeypoint2DMeasurements(lidar_2d_measurements);
    }
    if (proto.lidar_3d_measurements_size() != 0) {
      frame_ref.setLidarDescriptors(aslam::VisualFrame::DescriptorsT());
      internal::deserializeLidarDescriptors(
          proto, frame_ref.getLidarDescriptorsMutable());
    }

    if (proto.has_is_valid() && !proto.is_valid()) {
      frame_ref.invalidate();
    }
  }
}

void serializeVisualNFrame(
    const aslam::VisualNFrame& n_frame, aslam::proto::VisualNFrame* proto) {
  CHECK_NOTNULL(proto);

  n_frame.getId().serialize(proto->mutable_id());
  const unsigned int num_frames = n_frame.getNumFrames();
  for (unsigned int i = 0u; i < num_frames; ++i) {
    aslam::proto::VisualFrame* visual_frame_proto =
        CHECK_NOTNULL(proto->add_frames());
    if (n_frame.isFrameSet(i)) {
      const aslam::VisualFrame& visual_frame = n_frame.getFrame(i);
      serializeVisualFrame(visual_frame, visual_frame_proto);
    } else {
      // Set invalid id to proto::VisualFrame.
      aslam::FrameId().serialize(visual_frame_proto->mutable_id());
    }
  }
}

void deserializeVisualNFrame(
    const aslam::proto::VisualNFrame& proto,
    aslam::VisualNFrame::Ptr* n_frame) {
  CHECK_NOTNULL(n_frame);
  aslam::NCamera::Ptr n_camera;
  deserializeVisualNFrame(proto, n_camera, n_frame);
}

void deserializeVisualNFrame(
    const aslam::proto::VisualNFrame& proto,
    const aslam::NCamera::Ptr& n_camera, aslam::VisualNFrame::Ptr* n_frame) {
  CHECK_NOTNULL(n_frame);

  aslam::NFramesId n_frame_id;
  n_frame_id.deserialize(proto.id());

  CHECK_GT(proto.frames_size(), 0);
  const int num_frames = proto.frames_size();
  if (*n_frame == nullptr) {
    // NFrame is not instantiated yet, let's construct the object with dummy
    // NCamera object (but with correct number of cameras).
    n_frame->reset(new aslam::VisualNFrame(n_frame_id, num_frames));
  } else {
    (*n_frame)->setId(n_frame_id);
  }

  aslam::VisualNFrame& n_frame_ref = **n_frame;

  CHECK(n_frame != nullptr);
  if (n_camera != nullptr) {
    n_frame_ref.setNCameras(n_camera);
  }

  for (int i = 0; i < num_frames; ++i) {
    const aslam::proto::VisualFrame& visual_frame_proto = proto.frames(i);
    aslam::VisualFrame::Ptr visual_frame;
    if (n_camera == nullptr && n_frame_ref.getNCameraShared() != nullptr) {
      const aslam::Camera::Ptr cam =
          n_frame_ref.getNCameraShared()->getCameraShared(i);
      deserializeVisualFrame(
          visual_frame_proto,
          n_frame_ref.getNCameraShared()->getCameraShared(i), &visual_frame);
    } else if (n_camera != nullptr) {
      deserializeVisualFrame(
          visual_frame_proto, n_camera->getCameraShared(i), &visual_frame);
    } else {
      deserializeVisualFrame(visual_frame_proto, &visual_frame);
    }

    if (visual_frame != nullptr) {
      n_frame_ref.setFrame(i, visual_frame);
    } else {
      n_frame_ref.unSetFrame(i);
    }
  }
}

namespace internal {

void serializeDescriptors(
    const aslam::VisualFrame::DescriptorsT& descriptors,
    aslam::proto::VisualFrame* proto) {
  CHECK_NOTNULL(proto);

  proto->set_keypoint_descriptor_size(
      descriptors.rows() * sizeof(aslam::VisualFrame::DescriptorsT::Scalar));

  std::string* descriptors_string = proto->mutable_keypoint_descriptors();
  descriptors_string->resize(
      descriptors.size() * sizeof(aslam::VisualFrame::DescriptorsT::Scalar));
  Eigen::Map<aslam::VisualFrame::DescriptorsT> descriptors_map(
      reinterpret_cast<unsigned char*>(&descriptors_string->front()),
      descriptors.rows(), descriptors.cols());
  descriptors_map = descriptors;
}

void serializeLidarDescriptors(
    const aslam::VisualFrame::DescriptorsT& descriptors,
    aslam::proto::VisualFrame* proto) {
  CHECK_NOTNULL(proto);

  proto->set_lidar_descriptor_size(
      descriptors.rows() * sizeof(aslam::VisualFrame::DescriptorsT::Scalar));

  std::string* descriptors_string = proto->mutable_lidar_descriptors();
  descriptors_string->resize(
      descriptors.size() * sizeof(aslam::VisualFrame::DescriptorsT::Scalar));
  Eigen::Map<aslam::VisualFrame::DescriptorsT> descriptors_map(
      reinterpret_cast<unsigned char*>(&descriptors_string->front()),
      descriptors.rows(), descriptors.cols());
  descriptors_map = descriptors;
}

void deserializeDescriptors(
    const aslam::proto::VisualFrame& proto,
    aslam::VisualFrame::DescriptorsT* descriptors) {
  CHECK_NOTNULL(descriptors);
  if (proto.keypoint_descriptor_size() != 0) {
    Eigen::Map<const aslam::VisualFrame::DescriptorsT> descriptor_map(
        reinterpret_cast<const unsigned char*>(
            &proto.keypoint_descriptors().front()),
        proto.keypoint_descriptor_size(),
        proto.keypoint_descriptors().size() / proto.keypoint_descriptor_size());
    *descriptors = descriptor_map;
  } else {
    descriptors->resize(0, 0);
  }
}

void deserializeLidarDescriptors(
    const aslam::proto::VisualFrame& proto,
    aslam::VisualFrame::DescriptorsT* descriptors) {
  CHECK_NOTNULL(descriptors);
  if (proto.lidar_descriptor_size() != 0) {
    Eigen::Map<const aslam::VisualFrame::DescriptorsT> descriptor_map(
        reinterpret_cast<const unsigned char*>(
            &proto.lidar_descriptors().front()),
        proto.lidar_descriptor_size(),
        proto.lidar_descriptors().size() / proto.lidar_descriptor_size());
    *descriptors = descriptor_map;
  } else {
    descriptors->resize(0, 0);
  }
}

}  // namespace internal

}  // namespace serialization
}  // namespace aslam
