#ifndef DESCRIPTOR_PROJECTION_BUILD_PROJECTION_MATRIX_H_
#define DESCRIPTOR_PROJECTION_BUILD_PROJECTION_MATRIX_H_

#include <algorithm>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <descriptor-projection/descriptor-projection.h>
#include <gflags/gflags.h>
#include <glog/logging.h>

namespace descriptor_projection {
void ComputeCovariance(
    const Eigen::MatrixXf& data, Eigen::MatrixXf* covariance);

void BuildListOfMatchesAndNonMatches(
    const Eigen::MatrixXf& all_descriptors, const std::vector<Track>& tracks,
    std::vector<descriptor_projection::DescriptorMatch>* matches,
    std::vector<descriptor_projection::DescriptorMatch>* non_matches);

void BuildCovarianceMatricesOfMatchesAndNonMatches(
    unsigned int descriptor_size, const Eigen::MatrixXf& all_descriptors,
    const std::vector<Track>& tracks, unsigned int* sample_size_matches,
    unsigned int* sample_size_non_matches, Eigen::MatrixXf* cov_matches,
    Eigen::MatrixXf* cov_non_matches);

void ComputeProjectionMatrix(
    const Eigen::MatrixXf& cov_matches, const Eigen::MatrixXf& cov_non_matches,
    Eigen::MatrixXf* A);
}  // namespace descriptor_projection
#endif  // DESCRIPTOR_PROJECTION_BUILD_PROJECTION_MATRIX_H_
