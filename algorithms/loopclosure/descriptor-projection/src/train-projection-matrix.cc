#include "descriptor-projection/train-projection-matrix.h"

#include <iostream>  // NOLINT
#include <string>
#include <utility>
#include <vector>

#include <descriptor-projection/build-projection-matrix.h>
#include <descriptor-projection/descriptor-projection.h>
#include <descriptor-projection/flags.h>
#include <descriptor-projection/map-track-extractor.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <loopclosure-common/flags.h>
#include <loopclosure-common/types.h>
#include <maplab-common/binary-serialization.h>
#include <vi-map/vi-map.h>

namespace descriptor_projection {
struct SubSetCovariance {
  unsigned int sample_size_matches;
  unsigned int sample_size_non_matches;
  Eigen::MatrixXf cov_matches;
  Eigen::MatrixXf cov_non_matches;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

void TrainProjectionMatrix(const vi_map::VIMap& map) {
  CHECK_NE(FLAGS_lc_projection_matrix_filename, "")
      << "You have to provide a filename to write the projection matrix to.";

  Eigen::MatrixXf cov_matches;
  Eigen::MatrixXf cov_non_matches;
  double total_sample_size_matches = 0;
  double total_sample_size_non_matches = 0;

  // Get the descriptor-length.
  unsigned int descriptor_size = -1;
  unsigned int raw_descriptor_matching_threshold = 70;
  if (FLAGS_feature_descriptor_type == loop_closure::kFeatureDescriptorFREAK) {
    descriptor_size = loop_closure::kFreakDescriptorLengthBits;
  } else if (
      FLAGS_feature_descriptor_type == loop_closure::kFeatureDescriptorBRISK) {
    descriptor_size = loop_closure::kBriskDescriptorLengthBits;
  } else {
    CHECK(false) << "Unknown feature descriptor "
                 << FLAGS_feature_descriptor_type;
  }

  // A list to find the dataset with most data points.
  std::vector<std::pair<vi_map::MissionId, unsigned int> >
      match_counts_and_datasets;

  // For memory reasons we compute sample covariances on every dataset
  // and compute a weighted average afterwards.

  vi_map::MissionIdList all_mission_ids;
  map.getAllMissionIds(&all_mission_ids);

  std::vector<SubSetCovariance> covariances;
  for (const vi_map::MissionId& mission_id : all_mission_ids) {
    Eigen::MatrixXf all_descriptors;
    SubSetCovariance subset_covariance;
    std::vector<descriptor_projection::Track> tracks;

    using descriptor_projection::CollectAndConvertDescriptors;
    CollectAndConvertDescriptors(
        map, mission_id, descriptor_size, raw_descriptor_matching_threshold,
        &all_descriptors, &tracks);

    descriptor_projection::BuildCovarianceMatricesOfMatchesAndNonMatches(
        descriptor_size, all_descriptors, tracks,
        &subset_covariance.sample_size_matches,
        &subset_covariance.sample_size_non_matches,
        &subset_covariance.cov_matches, &subset_covariance.cov_non_matches);

    match_counts_and_datasets.emplace_back(
        mission_id, subset_covariance.sample_size_matches);

    total_sample_size_matches += subset_covariance.sample_size_matches;
    total_sample_size_non_matches += subset_covariance.sample_size_non_matches;
    covariances.push_back(subset_covariance);
  }

  CHECK(!covariances.empty());
  const int descriptor_dims = covariances[0].cov_matches.cols();
  cov_matches.resize(descriptor_dims, descriptor_dims);
  cov_matches.setZero();
  cov_non_matches.resize(descriptor_dims, descriptor_dims);
  cov_non_matches.setZero();

  for (const SubSetCovariance& subset_covariance : covariances) {
    cov_matches += subset_covariance.cov_matches *
                   subset_covariance.sample_size_matches /
                   total_sample_size_matches;
    cov_non_matches += subset_covariance.cov_non_matches *
                       subset_covariance.sample_size_non_matches /
                       total_sample_size_non_matches;
  }

  Eigen::MatrixXf A;
  descriptor_projection::ComputeProjectionMatrix(
      cov_matches, cov_non_matches, &A);

  VLOG(3) << "Projection matrix:";
  VLOG(3) << "\tcols: " << A.cols();
  VLOG(3) << "\trows: " << A.rows();
  VLOG(3) << "\tmin: " << A.minCoeff();
  VLOG(3) << "\tmax: " << A.maxCoeff();

  std::ofstream serializer(
      FLAGS_lc_projection_matrix_filename, std::ofstream::binary);
  common::Serialize(A, &serializer);
  serializer.flush();

  std::cout << "Stored projection matrix to "
            << FLAGS_lc_projection_matrix_filename << std::endl;
}
}  // namespace descriptor_projection
