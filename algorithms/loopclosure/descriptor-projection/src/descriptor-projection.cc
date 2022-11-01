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
