#include "dense-mapping/dense-mapping-alignment.h"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <maplab-common/multi-threaded-progress-bar.h>
#include <registration-toolbox/common/base-controller.h>
#include <registration-toolbox/mock-controller.h>
#include <registration-toolbox/model/registration-result.h>
#include <registration-toolbox/pcl-icp-controller.h>

#include "dense-mapping/dense-mapping-parallel-process.h"

namespace dense_mapping {

AlignmentConfig AlignmentConfig::fromGflags() {
  AlignmentConfig config;
  // TODO(mfehr): implement

  return config;
}

static AlignmentCandidatePair candidatePairFromRegistrationResult(
    const AlignmentCandidatePair& candidate,
    const regbox::RegistrationResult& registration_result) {
  AlignmentCandidatePair aligned_candidate = candidate;

  aligned_candidate.success = registration_result.hasConverged();
  aligned_candidate.T_SB_SA_final = registration_result.get_T_target_source();
  aligned_candidate.T_SB_SA_final_covariance =
      registration_result.get_T_target_source_covariance();

  return aligned_candidate;
}

template <typename ResourceDataType>
static bool retrieveResourceForCandidate(
    const AlignmentCandidate candidate, const vi_map::VIMap& map,
    ResourceDataType* candidate_resource) {
  CHECK_NOTNULL(candidate_resource);
  const vi_map::VIMission& mission = map.getMission(candidate.mission_id);
  return map.getSensorResource<ResourceDataType>(
      mission, candidate.resource_type, candidate.sensor_id,
      candidate.timestamp_ns, candidate_resource);
}

template <>
bool computeAlignmentForCandidatePairsImpl<resources::PointCloud>(
    const vi_map::VIMap& map, const AlignmentCandidatePair& pair,
    std::shared_ptr<regbox::BaseController>* aligner_ptr,
    AlignmentCandidatePair* aligned_pair_ptr) {
  CHECK_NOTNULL(aligned_pair_ptr);
  CHECK_NOTNULL(aligner_ptr);

  // Initialize the aligner if we haven't already.
  if (!*aligner_ptr) {
    *aligner_ptr =
        regbox::BaseController::make(regbox::Aligner::PclGIcp, "ADMC Aligner");
  }
  CHECK(*aligner_ptr);

  // Extract depth resource.
  resources::PointCloud candidate_resource_A, candidate_resource_B;
  if (!retrieveResourceForCandidate(
          pair.candidate_A, map, &candidate_resource_A)) {
    LOG(ERROR) << "Unable to retrieve resource for candidate A.";
    return false;
  }
  if (!retrieveResourceForCandidate(
          pair.candidate_B, map, &candidate_resource_B)) {
    LOG(ERROR) << "Unable to retrieve resource for candidate B.";
    return false;
  }

  const regbox::RegistrationResult result =
      (*aligner_ptr)
          ->align(
              candidate_resource_B, candidate_resource_A, pair.T_SB_SA_init);
  *aligned_pair_ptr = candidatePairFromRegistrationResult(pair, result);
  return true;
}

bool computeAlignmentForCandidatePairs(
    const AlignmentConfig& config, const vi_map::VIMap& map,
    const AlignmentCandidatePairs& candidate_pairs,
    AlignmentCandidatePairs* aligned_candidate_pairs) {
  CHECK_NOTNULL(aligned_candidate_pairs);

  const size_t num_pairs = candidate_pairs.size();

  // Parallelization settings.
  const size_t num_threads = common::getNumHardwareThreads();

  // Per thread result vector, to be combined later.
  Aligned<std::vector, AlignmentCandidatePairs>
      aligned_candidate_pairs_per_thread(num_threads);
  Aligned<std::vector, std::shared_ptr<regbox::BaseController>>
      aligner_per_thread(num_threads);
  std::vector<size_t> processed_elements_per_thread(num_threads, 0u);
  std::vector<size_t> successful_alignments_per_thread(num_threads, 0u);

  common::MultiThreadedProgressBar progress_bar;

  auto alignment_function = [&config, &map, &num_pairs, &num_threads,
                             &progress_bar, &candidate_pairs,
                             &aligned_candidate_pairs_per_thread,
                             &processed_elements_per_thread,
                             &successful_alignments_per_thread,
                             &aligner_per_thread](
                                const size_t thread_idx, const size_t start_idx,
                                const size_t end_idx) {
    CHECK_LT(thread_idx, num_threads);

    // Get thread specific input and output variables.
    AlignmentCandidatePairs& aligned_candidate_pairs_for_thread =
        aligned_candidate_pairs_per_thread[thread_idx];
    std::shared_ptr<regbox::BaseController>& aligner_ptr =
        aligner_per_thread[thread_idx];

    // Introspection
    size_t processed_pairs = 0u;
    size_t successful_alignments = 0u;

    // Set number of elements processed in this thread.
    progress_bar.setNumElements(end_idx - start_idx);

    // Move iterator to the start idx, since for an unordered_map we cannot
    // access it by index directly.
    auto it = candidate_pairs.cbegin();
    for (size_t idx = 0u; idx < start_idx; ++idx) {
      ++it;
    }

    // Do the work.
    backend::ResourceType current_resource_type = backend::ResourceType::kCount;
    for (size_t idx = start_idx; idx < end_idx && it != candidate_pairs.cend();
         ++idx, ++it) {
      CHECK_LE(start_idx, end_idx);
      CHECK_LE(end_idx, num_pairs);

      const AlignmentCandidatePair& pair = *it;

      const backend::ResourceType resource_type_A =
          pair.candidate_A.resource_type;
      const backend::ResourceType resource_type_B =
          pair.candidate_B.resource_type;

      if (resource_type_A != resource_type_B) {
        LOG(FATAL)
            << "Alignment of different resource types is not supported, "
               "type A: '"
            << backend::ResourceTypeNames[static_cast<int>(resource_type_A)]
            << "(" << static_cast<int>(resource_type_A) << ")' vs. type B: '"
            << backend::ResourceTypeNames[static_cast<int>(resource_type_B)]
            << "(" << static_cast<int>(resource_type_B) << ")'.";
      }

      // Reset the aligner if we switch resource types, such that the
      // specific implementation for this resource type can chose its own
      // aligner type.
      if (current_resource_type != resource_type_A) {
        aligner_ptr.reset();
        current_resource_type = resource_type_A;
      }

      // Launch a resource type specific alignment.
      AlignmentCandidatePair processed_pair;
      bool aligned_without_error = false;
      switch (current_resource_type) {
        case backend::ResourceType::kPointCloudXYZ:
        // Fall through intended.
        case backend::ResourceType::kPointCloudXYZI:
        // Fall through intended.
        case backend::ResourceType::kPointCloudXYZRGBN:
          aligned_without_error =
              computeAlignmentForCandidatePairsImpl<resources::PointCloud>(
                  map, pair, &aligner_ptr, &processed_pair);
          break;
        default:
          LOG(FATAL) << "Alignment algorithm between resource types '"
                     << backend::ResourceTypeNames[static_cast<int>(
                            current_resource_type)]
                     << "(" << static_cast<int>(current_resource_type)
                     << ") is currently NOT implemented'!";
      }

      // In case there was an error inside the alignment function, we skip
      // the rest.
      if (!aligned_without_error) {
        LOG(WARNING) << "Alignment between candidate A ("
                     << aslam::time::timeNanosecondsToString(
                            pair.candidate_A.timestamp_ns)
                     << ") and candidate B ("
                     << aslam::time::timeNanosecondsToString(
                            pair.candidate_B.timestamp_ns)
                     << ") encountered and error!";
        progress_bar.update(++processed_pairs);
        continue;
      }

      if (processed_pair.success) {
        ++successful_alignments;
      }
      aligned_candidate_pairs_for_thread.emplace(processed_pair);

      progress_bar.update(++processed_pairs);
    }

    processed_elements_per_thread[thread_idx] = processed_pairs;
    successful_alignments_per_thread[thread_idx] = successful_alignments;
  };

  VLOG(1) << "Processing " << num_pairs << " alignment candidates in "
          << num_threads << " threads...";
  const size_t actual_number_of_threads = parallelProcess(
      alignment_function, 0u /*start idx*/, num_pairs /*end idx*/, num_threads);
  VLOG(1) << "Done. Actual number of threads used: "
          << actual_number_of_threads;

  // Combine results and compose print some statistics.
  size_t total_processed_pairs = 0u;
  size_t total_successful_pairs = 0u;
  for (size_t thread_idx = 0u; thread_idx < actual_number_of_threads;
       ++thread_idx) {
    const size_t processed_elements = processed_elements_per_thread[thread_idx];
    const size_t successful_alignments =
        successful_alignments_per_thread[thread_idx];

    total_processed_pairs += processed_elements;
    total_successful_pairs += successful_alignments;

    VLOG(1) << "Thread " << thread_idx << " successfully aligned "
            << successful_alignments << "/" << processed_elements
            << " candidates.";

    const AlignmentCandidatePairs& aligned_candidate_pairs_for_thread =
        aligned_candidate_pairs_per_thread[thread_idx];
    CHECK_EQ(aligned_candidate_pairs_for_thread.size(), successful_alignments);

    aligned_candidate_pairs->insert(
        aligned_candidate_pairs_for_thread.begin(),
        aligned_candidate_pairs_for_thread.end());
  }
  CHECK_EQ(total_processed_pairs, num_pairs);
  CHECK_EQ(aligned_candidate_pairs->size(), total_successful_pairs);

  return true;
}

}  // namespace dense_mapping
