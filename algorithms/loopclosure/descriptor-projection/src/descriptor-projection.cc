#include <fstream>  // NOLINT

#include <Eigen/Core>
#include <Eigen/Dense>
#include <aslam/common/feature-descriptor-ref.h>
#include <descriptor-projection/descriptor-projection.h>
#include <descriptor-projection/flags.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <maplab-common/binary-serialization.h>
#include <maplab-common/eigen-proto.h>

namespace descriptor_projection {
void ProjectDescriptor(
    const aslam::common::FeatureDescriptorConstRef& raw_descriptor,
    const Eigen::MatrixXf& projection_matrix, int target_dimensions,
    std::vector<float>* projected_descriptor) {
  CHECK_NOTNULL(projected_descriptor);
  Eigen::Matrix<float, Eigen::Dynamic, 1> descriptor;
  const int descriptor_bits = raw_descriptor.size() * 8;
  descriptor.resize(descriptor_bits, Eigen::NoChange);
  DescriptorToEigenMatrix(raw_descriptor, descriptor);
  ProjectDescriptor(
      descriptor, projection_matrix, target_dimensions, projected_descriptor);
}

void ProjectDescriptorBlock(
    const std::vector<aslam::common::FeatureDescriptorConstRef>&
        raw_descriptors,
    const Eigen::MatrixXf& projection_matrix, int target_dimensions,
    Eigen::MatrixXf* projected_descriptors) {
  if (raw_descriptors.empty()) {
    return;
  }
  CHECK_NOTNULL(projected_descriptors);
  projected_descriptors->resize(target_dimensions, raw_descriptors.size());
  Eigen::MatrixXf converted_descriptors;
  const int num_descriptor_bytes = raw_descriptors[0].size();
  const int num_descriptor_bits = num_descriptor_bytes * 8;
  converted_descriptors.resize(num_descriptor_bits, raw_descriptors.size());
  for (size_t i = 0; i < raw_descriptors.size(); ++i) {
    DescriptorToEigenMatrix(
        raw_descriptors[i],
        converted_descriptors.block(0, i, num_descriptor_bits, 1));
  }
  *projected_descriptors =
      projection_matrix.block(
          0, 0, target_dimensions, projection_matrix.cols()) *
      converted_descriptors.block(
          0, 0, projection_matrix.cols(), projected_descriptors->cols());
}

void ProjectDescriptorBlock(
    const Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>&
        raw_descriptors,
    const Eigen::MatrixXf& projection_matrix, int target_dimensions,
    Eigen::MatrixXf* projected_descriptors) {
  if (raw_descriptors.cols() == 0) {
    return;
  }
  CHECK_NOTNULL(projected_descriptors);
  projected_descriptors->resize(target_dimensions, raw_descriptors.cols());
  Eigen::MatrixXf converted_descriptors;
  const int num_descriptor_bytes = raw_descriptors.rows();
  const int num_descriptor_bits = num_descriptor_bytes * 8;
  converted_descriptors.resize(num_descriptor_bits, raw_descriptors.cols());
  for (int i = 0; i < raw_descriptors.cols(); ++i) {
    DescriptorToEigenMatrix(
        raw_descriptors.block(0, i, raw_descriptors.rows(), 1),
        converted_descriptors.block(0, i, num_descriptor_bits, 1));
  }

  if (projection_matrix.cols() == 471) {
    CHECK_EQ(512, num_descriptor_bits)
        << "Projection matrix dimensions don't match the descriptor length. "
        << "Double check your setting for feature_descriptor_type.";
  } else {
    CHECK_EQ(projection_matrix.cols(), num_descriptor_bits)
        << "Projection matrix dimensions don't match the descriptor length. "
        << "Double check your setting for feature_descriptor_type.";
  }

  *projected_descriptors =
      projection_matrix.block(
          0, 0, target_dimensions, projection_matrix.cols()) *
      converted_descriptors.block(
          0, 0, projection_matrix.cols(), projected_descriptors->cols());
}

bool LoadprojectionMatrix(Eigen::MatrixXf* projection_matrix) {
  CHECK_NOTNULL(projection_matrix);

  std::ifstream deserializer(FLAGS_lc_projection_matrix_filename);
  CHECK(deserializer.is_open()) << "Cannot load projection matrix from file: "
                                << FLAGS_lc_projection_matrix_filename;
  common::Deserialize(projection_matrix, &deserializer);
  return true;
}
}  // namespace descriptor_projection

namespace loop_closure {

void ProjectedImage::serialize(
    proto::ProjectedImage* proto_projected_image) const {
  CHECK_NOTNULL(proto_projected_image);
  proto_projected_image->set_timestamp_nanoseconds(timestamp_nanoseconds);
  keyframe_id.vertex_id.serialize(proto_projected_image->mutable_vertex_id());
  proto_projected_image->set_frame_index(keyframe_id.frame_index);
  dataset_id.serialize(proto_projected_image->mutable_dataset_id());

  common::eigen_proto::serialize(
      projected_descriptors,
      proto_projected_image->mutable_projected_descriptors());

  common::eigen_proto::serialize(
      measurements, proto_projected_image->mutable_measurements());

  for (const vi_map::LandmarkId& landmark_id : landmarks) {
    CHECK(landmark_id.isValid());
    common::proto::Id* proto_id = proto_projected_image->add_landmarks();
    CHECK_NOTNULL(proto_id);
    landmark_id.serialize(proto_id);
  }
}

void ProjectedImage::deserialize(
    const proto::ProjectedImage& proto_projected_image) {
  timestamp_nanoseconds = proto_projected_image.timestamp_nanoseconds();
  CHECK_GE(timestamp_nanoseconds, 0);

  keyframe_id.vertex_id.deserialize(proto_projected_image.vertex_id());
  CHECK(keyframe_id.vertex_id.isValid());

  keyframe_id.frame_index = proto_projected_image.frame_index();

  dataset_id.deserialize(proto_projected_image.dataset_id());
  CHECK(dataset_id.isValid());

  common::eigen_proto::deserialize(
      proto_projected_image.projected_descriptors(), &projected_descriptors);

  common::eigen_proto::deserialize(
      proto_projected_image.measurements(), &measurements);

  const int num_landmarks = proto_projected_image.landmarks_size();
  CHECK_GE(num_landmarks, 0);

  landmarks.clear();
  landmarks.reserve(num_landmarks);
  for (int landmark_idx = 0; landmark_idx < num_landmarks; ++landmark_idx) {
    vi_map::LandmarkId landmark_id;
    landmark_id.deserialize(
        proto_projected_image.landmarks(landmark_idx));
    CHECK(landmark_id.isValid());
    landmarks.push_back(landmark_id);
  }
}

}  // namespace loop_closure
