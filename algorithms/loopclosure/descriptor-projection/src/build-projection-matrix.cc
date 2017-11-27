#include <algorithm>
#include <unordered_map>

#include <Eigen/QR>
#include <descriptor-projection/build-projection-matrix.h>
#include <descriptor-projection/descriptor-projection.h>
#include <descriptor-projection/flags.h>

#include <vi-map/vertex.h>

namespace descriptor_projection {

// Compute the covariance of the descriptors, rows are states, columns are
// samples.
void ComputeCovariance(
    const Eigen::MatrixXf& data, Eigen::MatrixXf* covariance) {
  CHECK_NOTNULL(covariance);
  CHECK_GT(data.cols(), 0) << "Data must not be empty!";
  VLOG(4) << "Got " << data.cols()
          << " samples to compute the covariance from.";
  covariance->setZero(data.rows(), data.rows());
  constexpr int kBlockSize = 10000;
  const int num_blocks = data.cols() / kBlockSize + 1;
  for (int i = 0; i < num_blocks; ++i) {
    const int block_start = i * kBlockSize;
    const int block_size =
        std::min<int>((i + 1) * kBlockSize, data.cols()) - block_start;
    const Eigen::Block<const Eigen::MatrixXf>& data_block =
        data.block(0, block_start, data.rows(), block_size);

    const Eigen::MatrixXf centered =
        data_block.colwise() - data_block.rowwise().mean();
    double normalizer = std::max(static_cast<int>(data_block.cols() - 1), 1);
    covariance->noalias() += (centered * centered.adjoint()) / normalizer;
  }
  (*covariance) /= num_blocks;
}

void BuildListOfMatchesAndNonMatches(
    const Eigen::MatrixXf& all_descriptors, const std::vector<Track>& tracks,
    std::vector<descriptor_projection::DescriptorMatch>* matches,
    std::vector<descriptor_projection::DescriptorMatch>* non_matches) {
  CHECK_NOTNULL(matches);
  CHECK_NOTNULL(non_matches);
  for (const Track& track : tracks) {
    // Add pairs of matching descriptors to the list of matches.
    // TODO(slynen): Consider taking random pairs to not under-estimate the
    // variance.
    for (size_t i = 1; i < track.size(); ++i) {
      matches->emplace_back(track[i - 1], track[i]);
    }
  }

  std::random_device device;
  std::mt19937 generator(device());
  std::uniform_int_distribution<> distribution(0, all_descriptors.cols() - 1);

  for (size_t i = 1; i < static_cast<size_t>(all_descriptors.cols()) &&
                     i < matches->size() * 2;
       ++i) {
    unsigned int index_a = distribution(generator);
    unsigned int index_b = distribution(generator);
    if (index_a == index_b) {
      continue;
    }
    non_matches->emplace_back(index_a, index_b);
  }
}

void BuildCovarianceMatricesOfMatchesAndNonMatches(
    unsigned int descriptor_size, const Eigen::MatrixXf& all_descriptors,
    const std::vector<Track>& tracks, unsigned int* sample_size_matches,
    unsigned int* sample_size_non_matches, Eigen::MatrixXf* cov_matches,
    Eigen::MatrixXf* cov_non_matches) {
  CHECK_NOTNULL(sample_size_matches);
  CHECK_NOTNULL(sample_size_non_matches);
  CHECK_NOTNULL(cov_matches);
  CHECK_NOTNULL(cov_non_matches);

  {  // Scope to limit memory usage.
    unsigned int too_short_tracks = 0;
    unsigned int long_enough_tracks = 0;
    constexpr size_t kMinTrackLength = 5;
    size_t number_of_used_tracks = 0;
    Eigen::MatrixXf sumMuMu;
    sumMuMu.setZero(descriptor_size, descriptor_size);

    std::vector<size_t> descriptors_from_tracks;
    descriptors_from_tracks.reserve(500);

    // Centering.
    constexpr int kMaxNumSamples = 50000;
    for (const Track& track : tracks) {
      if (track.size() < kMinTrackLength) {
        ++too_short_tracks;
        continue;
      }
      if (number_of_used_tracks >= kMaxNumSamples) {
        LOG(WARNING) << "Truncated descriptors to " << kMaxNumSamples << ".";
        break;
      }
      ++long_enough_tracks;

      Eigen::Matrix<float, Eigen::Dynamic, 1> mean;
      mean.resize(descriptor_size, Eigen::NoChange);
      mean.setZero();

      for (const size_t& descriptor_idx : track) {
        descriptors_from_tracks.push_back(descriptor_idx);
        mean += all_descriptors.block(0, descriptor_idx, descriptor_size, 1);
      }

      mean /= track.size();
      CHECK_LE(mean.maxCoeff(), 1.0);
      CHECK_GE(mean.minCoeff(), 0.0);

      Eigen::MatrixXf mu_sq_current = (mean * mean.transpose()).eval();

      sumMuMu += mu_sq_current * track.size();
      ++number_of_used_tracks;
    }

    VLOG(3) << "Got " << long_enough_tracks << " tracks out of "
            << tracks.size() << " (dropped " << too_short_tracks
            << " tracks because they were too short)";

    CHECK(!descriptors_from_tracks.empty());

    VLOG(3) << "Computing matches covariance from "
            << descriptors_from_tracks.size()
            << " matches (descriptor size: " << descriptor_size << ")";

    *sample_size_matches = descriptors_from_tracks.size();

    Eigen::MatrixXf matches;
    matches.resize(descriptor_size, descriptors_from_tracks.size());
    matches.setZero();

    int matched_idx = 0;
    for (const size_t& descriptor_idx : descriptors_from_tracks) {
      matches.block(0, matched_idx, descriptor_size, 1) =
          all_descriptors.block(0, descriptor_idx, descriptor_size, 1);
      ++matched_idx;
    }

    CHECK_GT(descriptors_from_tracks.size(), number_of_used_tracks);

    // Covariance computation for matches.
    cov_matches->noalias() =
        (matches * matches.transpose() - sumMuMu) * 2.0 /
        static_cast<float>(
            descriptors_from_tracks.size() - number_of_used_tracks);
  }  // Scope to limit memory usage.

  // Use all descriptors to estimate the non-matching covariance.
  *sample_size_non_matches = all_descriptors.cols();
  // Compute sample covariances non matched descriptors.
  ComputeCovariance(all_descriptors, cov_non_matches);

  *cov_non_matches *= 2.0f;
}

void ComputeProjectionMatrix(
    const Eigen::MatrixXf& cov_matches, const Eigen::MatrixXf& cov_non_matches,
    Eigen::MatrixXf* A) {
  CHECK_NOTNULL(A);

  const int dimensionality = cov_matches.cols();
  A->resize(dimensionality, dimensionality);

  Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov_matches, Eigen::ComputeFullV);

  CHECK_NE(svd.singularValues().minCoeff(), 0)
      << "Rank deficiency for matrix"
         " of samples detected. Probably too little matches.";

  Eigen::MatrixXf Av =
      svd.singularValues().cwiseSqrt().cwiseInverse().asDiagonal() *
      svd.matrixV().transpose();

  Eigen::JacobiSVD<Eigen::MatrixXf> svd_d(
      Av * cov_non_matches * Av.transpose(), Eigen::ComputeFullV);

  Eigen::MatrixXf singular_values_sqrt_inv =
      svd_d.singularValues().cwiseSqrt().cwiseInverse().asDiagonal();

  Eigen::MatrixXf eye;
  eye.resize(dimensionality, dimensionality);
  eye.setIdentity();

  *A = (eye - singular_values_sqrt_inv) * svd_d.matrixV().transpose() *
       svd.singularValues().cwiseInverse().asDiagonal() *
       svd.matrixV().transpose();
}
}  // namespace descriptor_projection
