#include "dense-mapping/dense-mapping-alignment.h"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <maplab-common/multi-threaded-progress-bar.h>
#include <maplab-common/progress-bar.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <registration-toolbox/common/base-controller.h>
#include <registration-toolbox/loam-feature-detector.h>
#include <registration-toolbox/mock-controller.h>
#include <registration-toolbox/model/registration-result.h>
#include <registration-toolbox/pcl-icp-controller.h>

#include "dense-mapping/dense-mapping-parallel-process.h"
#include "dense-mapping/dense-mapping-search.h"

namespace dense_mapping {

AlignmentConfig AlignmentConfig::fromGflags() {
  AlignmentConfig config;

  config.maximum_deviation_from_initial_guess_delta_position_m =
      FLAGS_dm_candidate_alignment_max_delta_position_to_initial_guess_m;
  config.maximum_deviation_from_initial_guess_delta_rotation_deg =
      FLAGS_dm_candidate_alignment_max_delta_rotation_to_initial_guess_deg;

  CHECK_GT(config.maximum_deviation_from_initial_guess_delta_position_m, 0.0);
  CHECK_GT(config.maximum_deviation_from_initial_guess_delta_rotation_deg, 0.0);
  return config;
}

bool alignmentDeviatesTooMuchFromInitialGuess(
    const AlignmentConfig& config, const AlignmentCandidatePair& pair) {
  // Only need to check pairs that have not already failed.
  if (!pair.success) {
    return false;
  }

  const aslam::Transformation T_SA_init_SA_final =
      pair.T_SB_SA_init.inverse() * pair.T_SB_SA_final;

  const double delta_position_m = T_SA_init_SA_final.getPosition().norm();
  static constexpr double kRadToDeg = 180.0 / M_PI;
  const double delta_rotation_deg =
      aslam::AngleAxis(T_SA_init_SA_final.getRotation()).angle() * kRadToDeg;
  const bool too_far_from_initial_guess =
      delta_rotation_deg >
          config.maximum_deviation_from_initial_guess_delta_rotation_deg ||
      delta_position_m >
          config.maximum_deviation_from_initial_guess_delta_position_m;
  return too_far_from_initial_guess;
}

static AlignmentCandidatePair candidatePairFromRegistrationResult(
    const AlignmentCandidatePair& pair,
    const regbox::RegistrationResult& registration_result) {
  AlignmentCandidatePair aligned_pair = pair;

  aligned_pair.success = registration_result.hasConverged();
  aligned_pair.T_SB_SA_final = registration_result.get_T_target_source();
  aligned_pair.T_SB_SA_final_covariance =
      registration_result.get_T_target_source_covariance();

  return aligned_pair;
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

      // Resegt the aligner if we switch resource types, such that the
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
          try {
            aligned_without_error =
                computeAlignmentForCandidatePairsImpl<resources::PointCloud>(
                    map, pair, &aligner_ptr, &processed_pair);
          } catch (const std::exception&) {
            aligned_without_error = false;
          }
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

      // Some simple sanity checks to overrule the success of the alignment if
      // necessary,
      if (alignmentDeviatesTooMuchFromInitialGuess(config, processed_pair)) {
        processed_pair.success = false;
      }

      // Only keep pair if successful.
      if (processed_pair.success) {
        aligned_candidate_pairs_for_thread.emplace(processed_pair);
        ++successful_alignments;
      }

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
  // Check that all pairs are processed and that all the successful alignments
  // have made it into the result vector.
  CHECK_EQ(total_processed_pairs, num_pairs);
  CHECK_LE(aligned_candidate_pairs->size(), total_successful_pairs);

  return true;
}

bool computeLoamAlignmentForCandidatePairs(
    const AlignmentConfig& config, const vi_map::VIMap& map,
    const AlignmentCandidatePairs& candidate_pairs,
    AlignmentCandidatePairs* aligned_candidate_pairs) {
  CHECK_NOTNULL(aligned_candidate_pairs);

  const size_t num_pairs = candidate_pairs.size();

  common::ProgressBar progress_bar(num_pairs);

  LOG(INFO) << "Processing " << num_pairs << " alignment candidates...";
  VLOG(1) << "Processing " << num_pairs << " alignment candidates...";

  // Introspection
  size_t processed_pairs = 0u;
  size_t successful_alignments = 0u;

  auto aligner = regbox::BaseController::make("regbox::LoamController", "Loam");
  auto feature_detector = regbox::LoamFeatureDetector();
  pcl::PointCloud<pcl::PointXYZI>::Ptr aggregated_loam_map(
      new pcl::PointCloud<pcl::PointXYZI>);
  AlignmentCandidate loam_map_base_candidate;
  aslam::Transformation T_map_last_successful_candidate;
  AlignmentCandidate last_successful_candidate;
  // Do the work.
  backend::ResourceType current_resource_type = backend::ResourceType::kCount;

  // Move iterator to the start idx, since for an unordered_map we cannot
  // access it by index directly.
  std::vector<AlignmentCandidatePair> temporally_ordered_pairs;
  for (auto it = candidate_pairs.cbegin(); it != candidate_pairs.cend(); ++it) {
    temporally_ordered_pairs.push_back(*it);
  }
  std::sort(
      temporally_ordered_pairs.begin(), temporally_ordered_pairs.end(),
      [](const AlignmentCandidatePair& a, const AlignmentCandidatePair& b) {
        return a.candidate_A.timestamp_ns < b.candidate_A.timestamp_ns;
      });
  for (auto it = temporally_ordered_pairs.cbegin();
       it != temporally_ordered_pairs.cend(); ++it) {
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

    if (current_resource_type != resource_type_A) {
      current_resource_type = resource_type_A;
    }

    // Launch a resource type specific alignment.
    AlignmentCandidatePair loam_pair;
    AlignmentCandidatePair candidate_to_last_successful_pair;
    pcl::PointCloud<pcl::PointXYZI>::Ptr new_aligned_features(
        new pcl::PointCloud<pcl::PointXYZI>);
    bool aligned_without_error = false;
    switch (current_resource_type) {
      case backend::ResourceType::kPointCloudXYZ:
      // Fall through intended.
      case backend::ResourceType::kPointCloudXYZI:
      // Fall through intended.
      case backend::ResourceType::kPointCloudXYZRGBN:
      case backend::ResourceType::kPointCloudXYZL:
        try {
          // Extract depth resource.
          resources::PointCloud candidate_resource_A, candidate_resource_B;
          if (aggregated_loam_map->empty()) {
            if (!retrieveResourceForCandidate(
                    pair.candidate_A, map, &candidate_resource_A)) {
              LOG(ERROR) << "Unable to retrieve resource for candidate A.";
              return false;
            }

            pcl::PointCloud<pcl::PointXYZI>::Ptr candidate_A_cloud(
                new pcl::PointCloud<pcl::PointXYZI>);
            for (size_t point_idx = 0u; point_idx < candidate_resource_A.size();
                 point_idx++) {
              pcl::PointXYZI point;
              point.x = candidate_resource_A.xyz[point_idx * 3];
              point.y = candidate_resource_A.xyz[point_idx * 3 + 1];
              point.z = candidate_resource_A.xyz[point_idx * 3 + 2];
              point.intensity =
                  static_cast<float>(candidate_resource_A.labels[point_idx]);
              candidate_A_cloud->push_back(point);
            }

            feature_detector.extractLoamFeaturesFromPointCloud(
                candidate_A_cloud, aggregated_loam_map);

            loam_map_base_candidate = pair.candidate_A;
            last_successful_candidate = loam_map_base_candidate;
          }

          if (!retrieveResourceForCandidate(
                  pair.candidate_B, map, &candidate_resource_B)) {
            LOG(ERROR) << "Unable to retrieve resource for candidate B.";
            return false;
          }

          pcl::PointCloud<pcl::PointXYZI>::Ptr candidate_B_cloud(
              new pcl::PointCloud<pcl::PointXYZI>);
          for (size_t point_idx = 0u; point_idx < candidate_resource_B.size();
               point_idx++) {
            pcl::PointXYZI point;
            point.x = candidate_resource_B.xyz[point_idx * 3];
            point.y = candidate_resource_B.xyz[point_idx * 3 + 1];
            point.z = candidate_resource_B.xyz[point_idx * 3 + 2];
            point.intensity =
                static_cast<float>(candidate_resource_B.labels[point_idx]);
            candidate_B_cloud->push_back(point);
          }

          pcl::PointCloud<pcl::PointXYZI>::Ptr candidate_B_feature_cloud(
              new pcl::PointCloud<pcl::PointXYZI>);
          feature_detector.extractLoamFeaturesFromPointCloud(
              candidate_B_cloud, candidate_B_feature_cloud);

          AlignmentCandidatePair loam_candidate_pair;
          createCandidatePair(
              pair.candidate_B, loam_map_base_candidate, &loam_candidate_pair);

          createCandidatePair(
              pair.candidate_B, last_successful_candidate,
              &candidate_to_last_successful_pair);
          loam_candidate_pair.T_SB_SA_init =
              T_map_last_successful_candidate *
              candidate_to_last_successful_pair.T_SB_SA_init;

          const regbox::RegistrationResult result = aligner->align(
              aggregated_loam_map, candidate_B_feature_cloud,
              loam_candidate_pair.T_SB_SA_init);
          *new_aligned_features = *result.getRegisteredCloud();
          const aslam::Transformation T_map_candidate_aligned =
              result.get_T_target_source();

          loam_pair =
              candidatePairFromRegistrationResult(loam_candidate_pair, result);

          candidate_to_last_successful_pair.T_SB_SA_final =
              T_map_last_successful_candidate.inverse() *
              T_map_candidate_aligned;
          candidate_to_last_successful_pair.success = loam_pair.success;
          aligned_without_error = loam_pair.success;
        } catch (const std::exception&) {
          aligned_without_error = false;
        }
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
      LOG(WARNING) << "Alignment between LOAM Map and candidate ("
                   << aslam::time::timeNanosecondsToString(
                          pair.candidate_B.timestamp_ns)
                   << ") encountered and error!";
      progress_bar.update(++processed_pairs);
      continue;
    }

    // Some simple sanity checks to overrule the success of the alignment if
    // necessary,
    if (alignmentDeviatesTooMuchFromInitialGuess(
            config, candidate_to_last_successful_pair)) {
      LOG(WARNING) << "Alignment between candidat for time "
                   << candidate_to_last_successful_pair.candidate_A.timestamp_ns
                   << " and candidates for time "
                   << candidate_to_last_successful_pair.candidate_B.timestamp_ns
                   << std::endl;

      candidate_to_last_successful_pair.success = false;
      loam_pair.success = false;
    }

    // Only keep pair if successful.
    if (loam_pair.success) {
      aligned_candidate_pairs->emplace(loam_pair);
      ++successful_alignments;
      last_successful_candidate = loam_pair.candidate_A;
      T_map_last_successful_candidate = loam_pair.T_SB_SA_final;

      float min_x = std::numeric_limits<float>::max();
      float min_y = std::numeric_limits<float>::max();
      float min_z = std::numeric_limits<float>::max();
      float max_x = -std::numeric_limits<float>::max();
      float max_y = -std::numeric_limits<float>::max();
      float max_z = -std::numeric_limits<float>::max();
      for (pcl::PointXYZI new_feature_point : *new_aligned_features) {
        min_x = std::min(min_x, new_feature_point.x);
        min_y = std::min(min_y, new_feature_point.y);
        min_z = std::min(min_z, new_feature_point.z);
        max_x = std::max(max_x, new_feature_point.x);
        max_y = std::max(max_y, new_feature_point.y);
        max_z = std::max(max_z, new_feature_point.z);
      }

      pcl::CropBox<pcl::PointXYZI> box_filter(true);
      pcl::PointCloud<pcl::PointXYZI> map_points_in_current_fov;
      box_filter.setMin(Eigen::Vector4f(min_x, min_y, min_z, 1.0));
      box_filter.setMax(Eigen::Vector4f(max_x, max_y, max_z, 1.0));
      box_filter.setInputCloud(aggregated_loam_map);
      box_filter.filter(map_points_in_current_fov);
      pcl::PointCloud<pcl::PointXYZI> map_points_outside_current_fov;
      const std::vector<int> indices = *(box_filter.getRemovedIndices());
      for (const int& idx : indices) {
        map_points_outside_current_fov.push_back((*aggregated_loam_map)[idx]);
      }
      *aggregated_loam_map = map_points_outside_current_fov;

      map_points_in_current_fov += *new_aligned_features;
      pcl::PointCloud<pcl::PointXYZI>::Ptr map_surface_points_in_current_fov(
          new pcl::PointCloud<pcl::PointXYZI>);
      pcl::PointCloud<pcl::PointXYZI>::Ptr map_edge_points_in_current_fov(
          new pcl::PointCloud<pcl::PointXYZI>);
      for (pcl::PointXYZI map_point_in_fov : map_points_in_current_fov) {
        if (map_point_in_fov.intensity == 0) {
          map_surface_points_in_current_fov->push_back(map_point_in_fov);
        } else {
          map_edge_points_in_current_fov->push_back(map_point_in_fov);
        }
      }

      pcl::PointCloud<pcl::PointXYZI>
          map_edge_points_in_current_fov_down_sampled;
      pcl::VoxelGrid<pcl::PointXYZI> edge_filter;

      edge_filter.setInputCloud(map_edge_points_in_current_fov);
      edge_filter.setLeafSize(0.2, 0.2, 0.2);
      edge_filter.filter(map_edge_points_in_current_fov_down_sampled);

      pcl::PointCloud<pcl::PointXYZI>
          map_surface_points_in_current_fov_down_sampled;
      pcl::VoxelGrid<pcl::PointXYZI> surface_filter;
      surface_filter.setInputCloud(map_surface_points_in_current_fov);
      surface_filter.setLeafSize(0.4, 0.4, 0.4);
      surface_filter.filter(map_surface_points_in_current_fov_down_sampled);

      *aggregated_loam_map += map_surface_points_in_current_fov_down_sampled +
                              map_edge_points_in_current_fov_down_sampled;
    }

    progress_bar.update(++processed_pairs);
  }

  VLOG(1) << "Done.";

  VLOG(1) << "Successfully aligned " << successful_alignments << "/"
          << num_pairs << " candidates.";

  return true;
}

}  // namespace dense_mapping
