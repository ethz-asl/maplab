#include "dense-mapping/dense-mapping-alignment.h"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <maplab-common/multi-threaded-progress-bar.h>
#include <maplab-common/progress-bar.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <registration-toolbox/common/base-controller.h>
#include <registration-toolbox/mock-controller.h>
#include <registration-toolbox/model/registration-result.h>
#include <registration-toolbox/pcl-icp-controller.h>
#include <visualization/rviz-visualization-sink.h>

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

template <typename ResourceDataType>
bool retrievePointCloudsForDenseSubmap(
    const vi_map::VIMap& map, const int64_t timestamp_ns,
    const vi_map::DenseSubmap& submap,
    const backend::ResourceType resource_type, ResourceDataType* submap_cloud) {
  CHECK_NOTNULL(submap_cloud);
  vi_map::StampedTransformationMap stamped_transforms;
  if (!submap.getStampedTransformsToResourceFrameAtTimestamp(
          timestamp_ns, &stamped_transforms)) {
    return false;
  }
  const vi_map::VIMission& mission = map.getMission(submap.getMissionId());
  std::vector<resources::PointCloud> submap_clouds(stamped_transforms.size());
  size_t idx = 0u;
  for (auto it = stamped_transforms.begin(); it != stamped_transforms.end();
       ++idx, ++it) {
    if (!map.getSensorResource<ResourceDataType>(
            mission, resource_type, submap.getSensorId(), it->first,
            &submap_clouds[idx])) {
      continue;
    }
    submap_clouds[idx].applyTransformation(it->second);
  }
  submap_cloud->append(submap_clouds);
  if (submap_cloud->empty()) {
    return false;
  }
  return true;
}
template <typename ResourceDataType>
bool candidatesQualifyForDenseSubmap(
    const AlignmentCandidate candidate_B, const AlignmentCandidate candidate_A,
    const vi_map::VIMap& map, ResourceDataType* candidate_resource) {
  vi_map::DenseSubmap submap;
  if (!map.getDenseSubmapManager().getClosestDenseSubmap(
          candidate_B.mission_id, candidate_B.timestamp_ns, &submap)) {
    return false;
  }
  int64_t min_timestamp_ns;
  int64_t max_timestamp_ns;
  submap.getMinAndMaxTimestampNs(&min_timestamp_ns, &max_timestamp_ns);
  if (min_timestamp_ns <= candidate_A.timestamp_ns &&
      max_timestamp_ns >= candidate_A.timestamp_ns) {
    return false;
  }
  return retrievePointCloudsForDenseSubmap(
      map, candidate_B.timestamp_ns, submap, candidate_B.resource_type,
      candidate_resource);
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
    if (FLAGS_dm_candidate_alignment_type == "PclIcp") {
      *aligner_ptr =
          regbox::BaseController::make(regbox::Aligner::PclIcp, "ADMC Aligner");
    } else if (FLAGS_dm_candidate_alignment_type == "PclGIcp") {
      *aligner_ptr = regbox::BaseController::make(
          regbox::Aligner::PclGIcp, "ADMC Aligner");
    } else if (FLAGS_dm_candidate_alignment_type == "LpmIcp") {
      *aligner_ptr =
          regbox::BaseController::make(regbox::Aligner::LpmIcp, "ADMC Aligner");
    } else {
      *aligner_ptr = regbox::BaseController::make(
          regbox::Aligner::PclGIcp, "ADMC Aligner");
      LOG(ERROR) << "Selected alignment for dense mapping constraints is not "
                    "supported. Choosing PclGIcp as default.";
    }
  }
  CHECK(*aligner_ptr);

  // Extract depth resource.
  resources::PointCloud candidate_resource_A, candidate_resource_B;
  if (!retrieveResourceForCandidate(
          pair.candidate_A, map, &candidate_resource_A)) {
    LOG(ERROR) << "Unable to retrieve resource for candidate A.";
    return false;
  }

  bool use_dense_submap = false;
  if (FLAGS_dm_candidate_alignment_use_incremental_submap_alignment) {
    use_dense_submap = candidatesQualifyForDenseSubmap(
        pair.candidate_B, pair.candidate_A, map, &candidate_resource_B);
  }
  if (!use_dense_submap) {
    if (!retrieveResourceForCandidate(
            pair.candidate_B, map, &candidate_resource_B)) {
      LOG(ERROR) << "Unable to retrieve resource for candidate B.";
      return false;
    }
  }

  const regbox::RegistrationResult result =
      (*aligner_ptr)
          ->align(
              candidate_resource_B, candidate_resource_A, pair.T_SB_SA_init);

  if (use_dense_submap && FLAGS_dm_visualize_incremental_submap) {
    sensor_msgs::PointCloud2 submap_points_msg;
    backend::convertPointCloudType(candidate_resource_B, &submap_points_msg);
    candidate_resource_B.appendTransformed(
        candidate_resource_A, result.get_T_target_source());
    submap_points_msg.header.frame_id = "submap";
    visualization::RVizVisualizationSink::publish(
        "retrieved_submaps", submap_points_msg);
  }
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
        case backend::ResourceType::kPointCloudXYZIRT:
        // Fall through intended.
        case backend::ResourceType::kPointCloudXYZRGBN:
          try {
            aligned_without_error =
                computeAlignmentForCandidatePairsImpl<resources::PointCloud>(
                    map, pair, &aligner_ptr, &processed_pair);
          } catch (const std::exception& e) {
            LOG(ERROR) << e.what();
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

void addSubmapAndDownsample(
    const resources::PointCloud& registered_candidate_cloud,
    resources::PointCloud* aggregated_filtered_map) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr registered_candidate_cloud_pcl(
      new pcl::PointCloud<pcl::PointXYZ>);
  backend::convertPointCloudType(
      registered_candidate_cloud, registered_candidate_cloud_pcl.get());
  pcl::PointCloud<pcl::PointXYZ> points_to_add_downsampled;
  pcl::VoxelGrid<pcl::PointXYZ> grid_filter;
  grid_filter.setInputCloud(registered_candidate_cloud_pcl);
  grid_filter.setLeafSize(
      regbox::FLAGS_regbox_pcl_downsample_leaf_size_m,
      regbox::FLAGS_regbox_pcl_downsample_leaf_size_m,
      regbox::FLAGS_regbox_pcl_downsample_leaf_size_m);
  grid_filter.filter(points_to_add_downsampled);

  pcl::PointCloud<pcl::PointXYZ>::Ptr aggregated_filtered_map_pcl(
      new pcl::PointCloud<pcl::PointXYZ>);
  backend::convertPointCloudType(
      *aggregated_filtered_map, &(*aggregated_filtered_map_pcl));

  float min_x = std::numeric_limits<float>::max();
  float min_y = std::numeric_limits<float>::max();
  float min_z = std::numeric_limits<float>::max();
  float max_x = -std::numeric_limits<float>::max();
  float max_y = -std::numeric_limits<float>::max();
  float max_z = -std::numeric_limits<float>::max();
  for (const pcl::PointXYZ& point_to_add : points_to_add_downsampled) {
    min_x = std::min(min_x, point_to_add.x);
    min_y = std::min(min_y, point_to_add.y);
    min_z = std::min(min_z, point_to_add.z);
    max_x = std::max(max_x, point_to_add.x);
    max_y = std::max(max_y, point_to_add.y);
    max_z = std::max(max_z, point_to_add.z);
  }

  pcl::CropBox<pcl::PointXYZ> box_filter(true);
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_points_in_current_fov(
      new pcl::PointCloud<pcl::PointXYZ>);
  box_filter.setMin(Eigen::Vector4f(min_x, min_y, min_z, 1.0));
  box_filter.setMax(Eigen::Vector4f(max_x, max_y, max_z, 1.0));
  box_filter.setInputCloud(aggregated_filtered_map_pcl);
  box_filter.filter(*map_points_in_current_fov);
  pcl::PointCloud<pcl::PointXYZ> map_points_outside_current_fov;
  const std::vector<int> indices = *(box_filter.getRemovedIndices());
  for (const int& idx : indices) {
    map_points_outside_current_fov.push_back(
        (*aggregated_filtered_map_pcl)[idx]);
  }

  *aggregated_filtered_map_pcl = map_points_outside_current_fov;

  *map_points_in_current_fov += points_to_add_downsampled;

  pcl::PointCloud<pcl::PointXYZ> map_points_in_current_fov_down_sampled;

  grid_filter.setInputCloud(map_points_in_current_fov);
  grid_filter.setLeafSize(
      regbox::FLAGS_regbox_pcl_downsample_leaf_size_m,
      regbox::FLAGS_regbox_pcl_downsample_leaf_size_m,
      regbox::FLAGS_regbox_pcl_downsample_leaf_size_m);
  grid_filter.filter(map_points_in_current_fov_down_sampled);
  *aggregated_filtered_map_pcl += map_points_in_current_fov_down_sampled;
  backend::convertPointCloudType(
      *aggregated_filtered_map_pcl, aggregated_filtered_map);
}

bool computeAlignmentForIncrementalSubmapCandidatePairs(
    const AlignmentConfig& config,
    const AlignmentCandidatePairs& candidate_pairs, vi_map::VIMap* vi_map_ptr,
    AlignmentCandidatePairs* aligned_candidate_pairs) {
  CHECK_NOTNULL(aligned_candidate_pairs);

  if (candidate_pairs.empty()) {
    VLOG(1) << "Incremental submap candidates are empty.";
    return false;
  }

  const size_t num_pairs = candidate_pairs.size();

  common::ProgressBar progress_bar(num_pairs);

  VLOG(1) << "Processing " << num_pairs << " submap alignment candidates...";

  // Introspection
  size_t processed_pairs = 0u;
  size_t successful_alignments = 0u;
  std::shared_ptr<regbox::BaseController> aligner;
  // Initialize the aligner if we haven't already.
  if (!aligner) {
    if (FLAGS_dm_candidate_alignment_type == "PclIcp") {
      aligner =
          regbox::BaseController::make(regbox::Aligner::PclIcp, "ADMC Aligner");
    } else if (FLAGS_dm_candidate_alignment_type == "PclGIcp") {
      aligner = regbox::BaseController::make(
          regbox::Aligner::PclGIcp, "ADMC Aligner");
    } else if (FLAGS_dm_candidate_alignment_type == "LpmIcp") {
      aligner =
          regbox::BaseController::make(regbox::Aligner::LpmIcp, "ADMC Aligner");
    } else {
      aligner = regbox::BaseController::make(
          regbox::Aligner::PclGIcp, "ADMC Aligner");
      LOG(ERROR) << "Selected alignment for dense mapping constraints is not "
                    "supported. Choosing PclGIcp as default.";
    }
  }

  std::vector<AlignmentCandidatePair> temporally_ordered_pairs;
  for (auto it = candidate_pairs.cbegin(); it != candidate_pairs.cend(); ++it) {
    temporally_ordered_pairs.push_back(*it);
  }

  // The candidates have to be sorted by increasing timestamp.
  std::sort(
      temporally_ordered_pairs.begin(), temporally_ordered_pairs.end(),
      [](const AlignmentCandidatePair& a, const AlignmentCandidatePair& b) {
        return a.candidate_A.timestamp_ns < b.candidate_A.timestamp_ns;
      });

  std::vector<std::vector<AlignmentCandidatePair>> candidates_per_dense_submap(
      1);
  size_t dense_submap_counter = 0u;
  size_t dense_submap_start_time_ns =
      temporally_ordered_pairs[0].candidate_A.timestamp_ns;
  for (const auto& pair : temporally_ordered_pairs) {
    if (pair.candidate_A.timestamp_ns - dense_submap_start_time_ns >
        kSecondsToNanoSeconds *
            FLAGS_dm_candidate_alignment_incremental_submap_length_s) {
      candidates_per_dense_submap.emplace_back();
      dense_submap_counter += 1;
      dense_submap_start_time_ns = pair.candidate_A.timestamp_ns;
    }
    candidates_per_dense_submap[dense_submap_counter].push_back(pair);
  }

  bool added_dense_incremental_submaps = false;
  for (const auto& submap_candidates : candidates_per_dense_submap) {
    std::unique_ptr<vi_map::DenseSubmap> dense_submap;

    // Filtered aggregated map for the incremental alignment.
    resources::PointCloud aggregated_filtered_map;
    // Aggregated map of all points using the transfomration of the incremental
    // alignment.
    resources::PointCloud aggregated_map;

    // The first successful candidate is used as the frame of the incremental
    // map.
    AlignmentCandidate map_base_candidate;
    // The last candidate that was successfully added to the incremental map.
    AlignmentCandidate last_successful_candidate;
    // The transformation of the last successful candidate point cloud
    // to the map
    aslam::Transformation T_map_last_successful_candidate;

    // Do the work.
    backend::ResourceType current_resource_type = backend::ResourceType::kCount;
    std::vector<pose_graph::VertexId> ids_in_submap;

    for (auto it = submap_candidates.cbegin(); it != submap_candidates.cend();
         ++it) {
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

      // Consecutive pair from candidate search.
      AlignmentCandidatePair submap_pair;
      // Pair between current candidate and last successful candidate.
      AlignmentCandidatePair candidate_to_last_successful_pair;
      // Pointcloud of current candidate.
      resources::PointCloud candidate_resource_B;
      bool aligned_without_error = false;

      // Launch a resource type specific alignment.
      switch (current_resource_type) {
        case backend::ResourceType::kPointCloudXYZ:
        // Fall through intended.
        case backend::ResourceType::kPointCloudXYZI:
        // Fall through intended.
        case backend::ResourceType::kPointCloudXYZIRT:
        // Fall through intended.
        case backend::ResourceType::kPointCloudXYZRGBN:
          try {
            // Extract depth resource.
            resources::PointCloud candidate_resource_A;
            if (aggregated_filtered_map.empty()) {
              if (!retrieveResourceForCandidate(
                      pair.candidate_A, *vi_map_ptr, &candidate_resource_A)) {
                LOG(ERROR) << "Unable to retrieve resource for candidate A.";
                continue;
              }
              dense_submap.reset(new vi_map::DenseSubmap(
                  pair.candidate_A.mission_id, pair.candidate_A.sensor_id));
              dense_submap->addTransformationToSubmap(
                  pair.candidate_A.timestamp_ns, aslam::Transformation());
              aggregated_filtered_map = candidate_resource_A;
              if (FLAGS_dm_visualize_incremental_submap) {
                aggregated_map = candidate_resource_A;
              }
              map_base_candidate = pair.candidate_A;
              ids_in_submap.push_back(map_base_candidate.closest_vertex_id);
              last_successful_candidate = map_base_candidate;
            }

            if (!retrieveResourceForCandidate(
                    pair.candidate_B, *vi_map_ptr, &candidate_resource_B)) {
              LOG(ERROR) << "Unable to retrieve resource for candidate B.";
              continue;
            }

            // Pair between base candidate and current candidate used for
            // constraint.
            AlignmentCandidatePair map_candidate_pair;
            createCandidatePair(
                pair.candidate_B, map_base_candidate,
                ConstraintType::incremental, &map_candidate_pair);

            // Pair between current and last successful candidate to retrieve
            // initial transform guess.
            createCandidatePair(
                pair.candidate_B, last_successful_candidate,
                ConstraintType::incremental,
                &candidate_to_last_successful_pair);

            map_candidate_pair.T_SB_SA_init =
                T_map_last_successful_candidate *
                candidate_to_last_successful_pair.T_SB_SA_init;

            const regbox::RegistrationResult result = aligner->align(
                aggregated_filtered_map, candidate_resource_B,
                map_candidate_pair.T_SB_SA_init);

            const aslam::Transformation T_map_candidate_aligned =
                result.get_T_target_source();

            submap_pair =
                candidatePairFromRegistrationResult(map_candidate_pair, result);

            // This transform is used to compare against the incremental initial
            // guess.
            candidate_to_last_successful_pair.T_SB_SA_final =
                T_map_last_successful_candidate.inverse() *
                T_map_candidate_aligned;

            candidate_to_last_successful_pair.success = submap_pair.success;
            aligned_without_error = submap_pair.success;
          } catch (const std::exception&) {
            aligned_without_error = false;
          }
          break;
        default:
          LOG(FATAL) << "Incremental Submap Alignment is not implemented for "
                     << "type "
                     << backend::ResourceTypeNames[static_cast<int>(
                            current_resource_type)];
          return false;
      }

      // In case there was an error inside the alignment function, we skip
      // the rest.
      if (!aligned_without_error) {
        LOG(WARNING) << "Alignment between Incremental Submap and candidate ("
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
        LOG(WARNING)
            << "Alignment between candidat for time "
            << candidate_to_last_successful_pair.candidate_A.timestamp_ns
            << " and candidates for time "
            << candidate_to_last_successful_pair.candidate_B.timestamp_ns
            << std::endl;

        candidate_to_last_successful_pair.success = false;
        submap_pair.success = false;
      }

      // Only keep pair if successful.
      if (submap_pair.success) {
        // Transform candidate cloud into incremental map frame.
        resources::PointCloud registered_candidate_cloud = candidate_resource_B;
        registered_candidate_cloud.applyTransformation(
            submap_pair.T_SB_SA_final);

        // We only add a new constraint if no constraint to this candidate
        // vertex exists yet to avoid multiple constraints between the same
        // vertices.
        if (std::find(
                ids_in_submap.begin(), ids_in_submap.end(),
                submap_pair.candidate_A.closest_vertex_id) ==
            ids_in_submap.end()) {
          ids_in_submap.push_back(submap_pair.candidate_A.closest_vertex_id);
          aligned_candidate_pairs->insert(submap_pair);
        }

        dense_submap->addTransformationToSubmap(
            submap_pair.candidate_A.timestamp_ns, submap_pair.T_SB_SA_final);
        ++successful_alignments;
        last_successful_candidate = submap_pair.candidate_A;
        T_map_last_successful_candidate = submap_pair.T_SB_SA_final;

        // Add the new points to the incremental submaps and voxel filter it.
        addSubmapAndDownsample(
            registered_candidate_cloud, &aggregated_filtered_map);

        if (FLAGS_dm_visualize_incremental_submap) {
          aggregated_map.append(registered_candidate_cloud);
          const std::string kSubMapFrame = "submap";
          sensor_msgs::PointCloud2 map_points_msg;

          backend::convertPointCloudType(aggregated_map, &map_points_msg);
          map_points_msg.header.frame_id = kSubMapFrame;
          visualization::RVizVisualizationSink::publish(
              "aggregated_points", map_points_msg);
          sensor_msgs::PointCloud2 odom_points_msg;
        }
      }
      progress_bar.update(++processed_pairs);
    }
    VLOG(1) << "Done.";

    if (dense_submap && dense_submap->size() > 2u) {
      vi_map_ptr->getDenseSubmapManager().addDenseSubmap(*dense_submap);
      VLOG(1) << "Successfully aligned " << successful_alignments << "/"
              << num_pairs << " candidates.";
      added_dense_incremental_submaps = true;
    }
  }
  if (added_dense_incremental_submaps) {
    return true;
  } else {
    VLOG(1) << "Incremental submap alignment did not succeed.";
    return false;
  }
}

}  // namespace dense_mapping
