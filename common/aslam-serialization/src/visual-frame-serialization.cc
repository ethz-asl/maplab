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

  // Serialize standard feature points (binary)
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
          frame.getKeypointScales(), proto->mutable_keypoint_scales());
      CHECK_EQ(
          proto->keypoint_measurements_size(),
          2 * proto->keypoint_scales_size());
    }

    const aslam::VisualFrame::DescriptorsT& descriptors =
        frame.getDescriptors();
    proto->set_keypoint_descriptor_size(
        descriptors.rows() * sizeof(aslam::VisualFrame::DescriptorsT::Scalar));
    internal::serializeDescriptors(
        descriptors, proto->mutable_keypoint_descriptors());

    CHECK_EQ(proto->keypoint_measurements_size(), 2 * descriptors.cols());
    VLOG(200) << "Frame " << frame.getId() << " has " << descriptors.cols()
              << " descriptors!";

    if (frame.hasTrackIds()) {
      ::common::eigen_proto::serialize(
          frame.getTrackIds(), proto->mutable_track_ids());
    }
  } else {
    VLOG(200) << "Frame " << frame.getId() << " has no descriptors!";
  }

  // Serialize external feature points
  if (frame.hasExternalKeypointMeasurements()) {
    ::common::eigen_proto::serialize(
        frame.getExternalKeypointMeasurements(),
        proto->mutable_external_measurements());
    ::common::eigen_proto::serialize(
        frame.getExternalKeypointMeasurementUncertainties(),
        proto->mutable_external_measurement_sigmas());
    CHECK_EQ(
        proto->external_measurements_size(),
        2 * proto->external_measurement_sigmas_size());

    if (frame.hasExternalKeypointScales()) {
      ::common::eigen_proto::serialize(
          frame.getExternalKeypointScales(), proto->mutable_external_scales());
      CHECK_EQ(
          proto->external_measurements_size(),
          2 * proto->external_scales_size());
    }

    const aslam::VisualFrame::DescriptorsT& descriptors =
        frame.getExternalDescriptors();
    proto->set_external_descriptor_size(
        descriptors.rows() * sizeof(aslam::VisualFrame::DescriptorsT::Scalar));
    internal::serializeDescriptors(
        descriptors, proto->mutable_external_descriptors());

    CHECK_EQ(proto->external_measurements_size(), 2 * descriptors.cols());
    VLOG(200) << "Frame " << frame.getId() << " has " << descriptors.cols()
              << " external descriptors!";

    if (frame.hasExternalTrackIds()) {
      ::common::eigen_proto::serialize(
          frame.getExternalTrackIds(), proto->mutable_external_track_ids());
    }
  }

  // check we serialized a valid frame
  proto->set_is_valid(frame.isValid());
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

    if (camera != nullptr) {
      frame_ref.setCameraGeometry(camera);
    }

    frame_ref.setId(frame_id);
    frame_ref.setTimestampNanoseconds(proto.timestamp());

    // Deserialize the standard binary feature points
    {
      Eigen::Map<const Eigen::Matrix2Xd> img_points_distorted(
          proto.keypoint_measurements().data(), 2,
          proto.keypoint_measurements_size() / 2);
      Eigen::Map<const Eigen::VectorXd> uncertainties(
          proto.keypoint_measurement_sigmas().data(),
          proto.keypoint_measurement_sigmas_size());
      Eigen::Map<const Eigen::VectorXd> scales(
          proto.keypoint_scales().data(), proto.keypoint_scales_size());
      Eigen::Map<const Eigen::VectorXi> track_ids(
          proto.track_ids().data(), proto.track_ids_size());

      CHECK_EQ(
          2 * proto.keypoint_measurement_sigmas_size(),
          proto.keypoint_measurements_size());

      frame_ref.setKeypointMeasurements(img_points_distorted);
      frame_ref.setKeypointMeasurementUncertainties(uncertainties);
      if (scales.rows() != 0) {
        CHECK_EQ(scales.rows(), img_points_distorted.cols());
        frame_ref.setKeypointScales(scales);
      }
      if (track_ids.rows() != 0) {
        CHECK_EQ(track_ids.rows(), img_points_distorted.cols());
        frame_ref.setTrackIds(track_ids);
      }

      CHECK_EQ(
          2 * proto.keypoint_descriptors().size() /
              proto.keypoint_descriptor_size(),
          static_cast<uint32_t>(proto.keypoint_measurements_size()));

      // Need to set empty descriptors, otherwise getMutable call below fails.
      frame_ref.setDescriptors(aslam::VisualFrame::DescriptorsT());
      internal::deserializeDescriptors(
          proto.keypoint_descriptors(), proto.keypoint_descriptor_size(),
          frame_ref.getDescriptorsMutable());

      CHECK(frame_ref.hasKeypointMeasurements());
      CHECK(frame_ref.hasKeypointMeasurementUncertainties());
      CHECK(frame_ref.hasDescriptors());
    }

    // Deserialize the external feature points
    if (proto.external_measurements_size() > 0) {
      Eigen::Map<const Eigen::Matrix2Xd> img_points_distorted(
          proto.external_measurements().data(), 2,
          proto.external_measurements_size() / 2);
      Eigen::Map<const Eigen::VectorXd> uncertainties(
          proto.external_measurement_sigmas().data(),
          proto.external_measurement_sigmas_size());
      Eigen::Map<const Eigen::VectorXd> scales(
          proto.external_scales().data(), proto.external_scales_size());
      Eigen::Map<const Eigen::VectorXi> track_ids(
          proto.external_track_ids().data(), proto.external_track_ids_size());

      CHECK_EQ(
          2 * proto.keypoint_measurement_sigmas_size(),
          proto.keypoint_measurements_size());

      frame_ref.setExternalKeypointMeasurements(img_points_distorted);
      frame_ref.setExternalKeypointMeasurementUncertainties(uncertainties);
      if (scales.rows() != 0) {
        CHECK_EQ(scales.rows(), img_points_distorted.cols());
        frame_ref.setExternalKeypointScales(scales);
      }
      if (track_ids.rows() != 0) {
        CHECK_EQ(track_ids.rows(), img_points_distorted.cols());
        frame_ref.setExternalTrackIds(track_ids);
      }

      LOG(INFO) << proto.external_descriptors().size();
      LOG(INFO) << proto.external_descriptor_size();
      LOG(INFO) << proto.external_measurements_size();

      CHECK_EQ(
          2 * proto.external_descriptors().size() /
              proto.external_descriptor_size(),
          static_cast<uint32_t>(proto.external_measurements_size()));

      // Need to set empty descriptors, otherwise getMutable call below fails.
      frame_ref.setExternalDescriptors(aslam::VisualFrame::DescriptorsT());
      internal::deserializeDescriptors(
          proto.external_descriptors(), proto.external_descriptor_size(),
          frame_ref.getExternalDescriptorsMutable());

      CHECK(frame_ref.hasExternalKeypointMeasurements());
      CHECK(frame_ref.hasExternalKeypointMeasurementUncertainties());
      //CHECK(frame_ref.hasExternalDescriptors());
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
    std::string* descriptors_string) {
  CHECK_NOTNULL(descriptors_string);
  descriptors_string->resize(
      descriptors.size() * sizeof(aslam::VisualFrame::DescriptorsT::Scalar));
  Eigen::Map<aslam::VisualFrame::DescriptorsT> descriptors_map(
      reinterpret_cast<unsigned char*>(&descriptors_string->front()),
      descriptors.rows(), descriptors.cols());
  descriptors_map = descriptors;
}

void deserializeDescriptors(
    const std::string& descriptors_raw, const uint32_t descriptor_size,
    aslam::VisualFrame::DescriptorsT* descriptors) {
  CHECK_NOTNULL(descriptors);

  if (descriptors_raw.size() != 0) {
    CHECK_GT(descriptor_size, 0u);
    Eigen::Map<const aslam::VisualFrame::DescriptorsT> descriptor_map(
        reinterpret_cast<const unsigned char*>(&descriptors_raw.front()),
        descriptor_size, descriptors_raw.size() / descriptor_size);
    *descriptors = descriptor_map;
  } else {
    descriptors->resize(0, 0);
  }
}

}  // namespace internal

}  // namespace serialization
}  // namespace aslam
