#include "registration-toolbox-testing-plugin/test-registration-toolbox.h"

#include <aslam/common/timer.h>
#include <geometry_msgs/Pose.h>
#include <minkindr_conversions/kindr_msg.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <registration-toolbox/loam-feature-detector.h>
#include <visualization/rviz-visualization-sink.h>
DEFINE_int32(n_vertices_to_test, 50, "");
DEFINE_double(min_distance_short, 0.5, "");
DEFINE_double(min_distance_long, 4., "");
DEFINE_double(max_noise_x_m, 2., "");
DEFINE_double(max_noise_y_m, 2., "");
DEFINE_double(max_noise_yaw_deg, 30., "");
DEFINE_int32(n_noise_steps, 1., "");
DEFINE_double(max_score_distance, 0.5, "");
DEFINE_bool(pcl_gicp, true, "");
DEFINE_bool(pcl_icp, true, "");
DEFINE_bool(pcl_vgicp, true, "");
DEFINE_bool(lpm, true, "");
DEFINE_bool(loam, true, "");
DEFINE_double(max_success_delta_position, 0.3, "");
DEFINE_double(max_success_delta_rotation_deg, 5., "");
namespace registration_toolbox_testing {

struct RegistrationMetrics {
  double delta_position_m;
  double delta_rotation_deg;
  double fitness_score;
  double vertices_distance_m;
  double noise_x;
  double noise_y;
  double noise_yaw;
  double runtime_s;
};

void calculateRegistrationPerformance(
    const std::vector<RegistrationMetrics>& metrics) {
  int n_short = 0;
  int n_far = 0;
  int n_success_short[FLAGS_n_noise_steps + 1] = {};
  int n_success_short_all;
  int n_success_far[FLAGS_n_noise_steps + 1] = {};
  int n_success_far_all;
  double delta_p_short[FLAGS_n_noise_steps + 1] = {};
  double delta_p_short_all;
  double delta_p_far[FLAGS_n_noise_steps + 1] = {};
  double delta_p_far_all;
  double delta_rotation_short[FLAGS_n_noise_steps + 1] = {};
  double delta_rotation_short_all;
  double delta_rotation_far[FLAGS_n_noise_steps + 1] = {};
  double delta_rotation_far_all;
  double fitness_short[FLAGS_n_noise_steps + 1] = {};
  double fitness_short_all;
  double fitness_far[FLAGS_n_noise_steps + 1] = {};
  double fitness_far_all;
  double total_runtime;
  for (const RegistrationMetrics& metric : metrics) {
    bool cond_1 =
        metric.delta_rotation_deg >
            dense_mapping::
                FLAGS_dm_candidate_alignment_max_delta_rotation_to_initial_guess_deg ||  // NOLINT
        metric.delta_position_m >
            dense_mapping::
                FLAGS_dm_candidate_alignment_max_delta_position_to_initial_guess_m;  //  NOLINT
    bool cond_2 =
        metric.delta_position_m < FLAGS_max_success_delta_position &&
        metric.delta_rotation_deg < FLAGS_max_success_delta_rotation_deg;

    if (!cond_2 && (!cond_1)) {
      LOG(INFO) << "weird";
    }
    total_runtime += metric.runtime_s;
    int noise_factor =
        round(metric.noise_x / (FLAGS_max_noise_x_m / FLAGS_n_noise_steps));
    if (metric.vertices_distance_m < FLAGS_min_distance_long / 2.) {
      // LOG(INFO)<<"short";
      n_short += 1;
      if (metric.delta_position_m < FLAGS_max_success_delta_position &&
          metric.delta_rotation_deg < FLAGS_max_success_delta_rotation_deg) {
        n_success_short[noise_factor] += 1;
        n_success_short_all++;
        delta_p_short[noise_factor] += metric.delta_position_m;
        delta_p_short_all += metric.delta_position_m;
        delta_rotation_short[noise_factor] += metric.delta_rotation_deg;
        delta_rotation_short_all += metric.delta_rotation_deg;
        fitness_short[noise_factor] += metric.fitness_score;
        fitness_short_all += metric.fitness_score;
      }
    } else {
      n_far += 1;
      if (metric.delta_position_m < FLAGS_max_success_delta_position &&
          metric.delta_rotation_deg < FLAGS_max_success_delta_rotation_deg) {
        n_success_far[noise_factor] += 1;
        n_success_far_all++;
        delta_p_far[noise_factor] += metric.delta_position_m;
        delta_p_far_all += metric.delta_position_m;
        delta_rotation_far[noise_factor] += metric.delta_rotation_deg;
        delta_rotation_far_all += metric.delta_rotation_deg;
        fitness_far[noise_factor] += metric.fitness_score;
        fitness_far_all += metric.fitness_score;
      }
    }
  }

  const double total_success_rate = (n_success_short_all + n_success_far_all) /
                                    static_cast<double>(metrics.size());
  const double short_success_rate =
      n_success_short_all / static_cast<double>(n_short);
  const double far_success_rate =
      n_success_far_all / static_cast<double>(n_far);
  LOG(INFO) << "TOTAL SUCCESS RATE: " << total_success_rate;
  LOG(INFO) << "SHORT SUCCESS RATE: " << short_success_rate;
  LOG(INFO) << "FAR SUCCESS RATE: " << far_success_rate;

  for (int i = 0; i < FLAGS_n_noise_steps + 1; i++) {
    double success_rate_short_per_noise =
        n_success_short[i] /
        (static_cast<double>(n_short) / (FLAGS_n_noise_steps + 1.));
    double success_rate_far_per_noise =
        n_success_far[i] /
        (static_cast<double>(n_far) / (FLAGS_n_noise_steps + 1.));
    double delta_p_short_per_noise = delta_p_short[i] / n_success_short[i];
    double delta_p_far_per_noise = delta_p_far[i] / n_success_far[i];
    double delta_rotation_short_per_noise =
        delta_rotation_short[i] / n_success_short[i];
    double delta_rotation_far_per_noise =
        delta_rotation_far[i] / n_success_far[i];
    LOG(INFO) << "SUCCESS RATE SHORT LEVEL " << i << ": "
              << success_rate_short_per_noise;
    LOG(INFO) << "SUCCESS RATE FAR LEVEL " << i << ": "
              << success_rate_far_per_noise;
  }
}

double getEuclideanFitnessScore(
    const double max_range, const resources::PointCloud& target,
    const resources::PointCloud& source_transformed) {
  double fitness_score = 0.0;

  pcl::PointCloud<pcl::PointXYZ>::Ptr source_transformed_pcl(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_pcl(
      new pcl::PointCloud<pcl::PointXYZ>);

  backend::convertPointCloudType(
      source_transformed, &(*source_transformed_pcl));
  backend::convertPointCloudType(target, &(*target_pcl));

  std::vector<int> nn_indices(1);
  std::vector<float> nn_dists(1);
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;

  tree.setInputCloud(target_pcl);
  // For each point in the source dataset
  int nr = 0;
  for (size_t i = 0; i < source_transformed_pcl->points.size(); ++i) {
    // Find its nearest neighbor in the target
    tree.nearestKSearch(
        source_transformed_pcl->points[i], 1, nn_indices, nn_dists);

    // Deal with occlusions (incomplete targets)
    if (nn_dists[0] <= max_range) {
      // Add to the fitness score
      fitness_score += nn_dists[0];
      nr++;
    }
  }

  if (nr > 0) {
    return (fitness_score / nr);
  } else {
    return (std::numeric_limits<double>::max());
  }
}

Eigen::Matrix3d yawToRotationMatrix(const double& yaw) {
  double cyaw = cos(yaw);
  double syaw = sin(yaw);
  Eigen::Matrix3d R;
  R.setIdentity();
  R.row(0)(0) = cyaw;
  R.row(0)(1) = -syaw;
  R.row(1)(0) = syaw;
  R.row(1)(1) = cyaw;
  return R;
}

void findCandidates(
    const vi_map::VIMap& map, const vi_map::MissionIdList& mission_ids,
    const dense_mapping::Config& config,
    dense_mapping::AlignmentCandidatePairs* candidate_pairs_ptr) {
  dense_mapping::MissionToAlignmentCandidatesMap candidates_per_mission;
  try {
    dense_mapping::findAllAlignmentCandidates(
        config.search_config, map, mission_ids, &candidates_per_mission);
  } catch (std::exception& e) {
    LOG(ERROR) << "Finding alignment pairs failed. Aborting.";
    return;
  }
  dense_mapping::AlignmentCandidateList candidates =
      candidates_per_mission[mission_ids[0]];
  double mission_distance;
  map.getDistanceTravelledPerMission(mission_ids[0], &mission_distance);
  const double distance_between_vertices =
      mission_distance / FLAGS_n_vertices_to_test;

  const vi_map::VIMission& mission = map.getMission(mission_ids[0]);
  const pose_graph::VertexId& root_vertex_id = mission.getRootVertexId();
  if (!root_vertex_id.isValid()) {
    return;
  }

  pose_graph::VertexId current_vertex_id = root_vertex_id;
  pose_graph::VertexId next_vertex_id = root_vertex_id;
  pose_graph::VertexIdList base_ids;
  std::vector<double> distance_to_last_base_vertex;
  base_ids.push_back(root_vertex_id);
  distance_to_last_base_vertex.push_back(0.0);
  double distance = 0;
  do {
    const vi_map::Vertex& current_vertex = map.getVertex(current_vertex_id);
    const vi_map::Vertex& next_vertex = map.getVertex(next_vertex_id);

    CHECK_EQ(next_vertex.getMissionId(), current_vertex.getMissionId());

    distance += (next_vertex.get_p_M_I() - current_vertex.get_p_M_I()).norm();
    if (distance > distance_between_vertices) {
      distance = 0;
      base_ids.push_back(next_vertex_id);
    }
    current_vertex_id = next_vertex_id;
  } while (map.getNextVertex(
      current_vertex_id, map.getGraphTraversalEdgeType(mission_ids[0]),
      &next_vertex_id));

  std::vector<int> candidate_list_indices;
  for (const pose_graph::VertexId& vertex_id : base_ids) {
    const vi_map::Vertex& vertex = map.getVertex(vertex_id);
    const int64_t vertex_timestamp = vertex.getMinTimestampNanoseconds();
    int64_t min_diff = std::numeric_limits<int64_t>::max();
    size_t closest_candidate_idx;
    for (size_t idx = 0u; idx < candidates.size(); idx++) {
      const dense_mapping::AlignmentCandidate& candidate = candidates[idx];
      int64_t diff = abs(candidate.timestamp_ns - vertex_timestamp);
      if (diff < min_diff) {
        min_diff = diff;
        closest_candidate_idx = idx;
      }
    }
    candidate_list_indices.push_back(closest_candidate_idx);
  }

  dense_mapping::SearchConfig close_candidate_config = config.search_config;
  dense_mapping::SearchConfig far_candidate_config = config.search_config;
  far_candidate_config.consecutive_search_max_delta_position_m =
      FLAGS_min_distance_long;
  far_candidate_config.consecutive_search_max_delta_rotation_deg = 270.;
  close_candidate_config.consecutive_search_max_delta_position_m =
      FLAGS_min_distance_short;
  close_candidate_config.consecutive_search_max_delta_rotation_deg = 270.;
  for (const size_t& idx : candidate_list_indices) {
    const dense_mapping::AlignmentCandidate* candidate_A = nullptr;
    const dense_mapping::AlignmentCandidate* candidate_B = nullptr;
    bool close_found = false;
    size_t current_idx = idx;
    while (current_idx < candidates.size()) {
      // If we have not set a first candidate yet, we need to do now and move
      // on.
      if (candidate_A == nullptr) {
        CHECK_LT(current_idx, candidates.size());
        candidate_A = &(candidates[current_idx]);
        candidate_B = nullptr;
        VLOG(5) << aslam::time::timeNanosecondsToString(
                       candidate_A->timestamp_ns)
                << " - none (init)";

        ++current_idx;
        continue;
      }

      // Fetch current candidate for B.
      CHECK_LT(current_idx, candidates.size());
      const dense_mapping::AlignmentCandidate* current_candidate =
          &(candidates[current_idx]);

      // Now we should have two candidates to compare.
      CHECK_NOTNULL(current_candidate);
      CHECK_NOTNULL(candidate_A);

      VLOG(5) << aslam::time::timeNanosecondsToString(candidate_A->timestamp_ns)
              << " - "
              << aslam::time::timeNanosecondsToString(
                     current_candidate->timestamp_ns);

      // For consecutive constraints we don't want to mix sensors, since we
      // expect the candidates to be in temporal order for each sensor. The
      // candidate list however will include all sensor data for a single
      // sensor/type in temporal order, but not across sensor/type.
      if (candidate_A->sensor_id != current_candidate->sensor_id ||
          candidate_A->resource_type != current_candidate->resource_type) {
        candidate_A = current_candidate;
        candidate_B = nullptr;

        ++current_idx;
        continue;
      }
      CHECK_LT(candidate_A->timestamp_ns, current_candidate->timestamp_ns);

      // If the current B candidate tripped any of the criteria, we need to take
      // the previous B candidate, unless we don't have one, in which case we
      // move on.
      if (!close_found) {
        if (dense_mapping::candidatesAreSpatiallyTooFar(
                close_candidate_config, *candidate_A, *current_candidate)) {
          // If we alread had a candidate B, we take the two candidates and
          // create a candidate pair and move on.
          dense_mapping::addCandidatePair(
              *candidate_A, *current_candidate, candidate_pairs_ptr);
          close_found = true;
          ++current_idx;
          continue;
          // NOTE: Do not move the index, since we want to try candidate B
          // against the current candidate in the next iteration.
        } else {
          // Move candidate B one further, since we can skip the previous B
          // candidate and still fullfill all criteria.
          candidate_B = current_candidate;
          CHECK_NOTNULL(candidate_A);
          CHECK_NOTNULL(candidate_B);

          ++current_idx;
          continue;
        }
      } else {
        if (dense_mapping::candidatesAreSpatiallyTooFar(
                far_candidate_config, *candidate_A, *current_candidate)) {
          if (candidate_B == nullptr) {
            LOG(WARNING) << "Dense mapping consecutive candidate search config "
                         << "is too strict or there is not enough data, could "
                         << "not find corresponding candidate for map at "
                         << candidate_A->timestamp_ns << "ns. Skipping one "
                         << "dense (sub) map.";

            // If we haven't found a candidate B yet, then we likely cannot find
            // a match for candidate A. This means the search config is too
            // strict or there is not enough data! Move on to try to find a new
            // pair, with the current candidate as a basse.
            candidate_A = current_candidate;
            candidate_B = nullptr;
            CHECK_NOTNULL(candidate_A);

            ++current_idx;
            continue;
          }
          // If we alread had a candidate B, we take the two candidates and
          // create a candidate pair and move on.
          dense_mapping::addCandidatePair(
              *candidate_A, *current_candidate, candidate_pairs_ptr);
          break;
        } else {
          // Move candidate B one further, since we can skip the previous B
          // candidate and still fullfill all criteria.
          candidate_B = current_candidate;
          CHECK_NOTNULL(candidate_A);
          CHECK_NOTNULL(candidate_B);

          ++current_idx;
          continue;
        }
      }
    }
  }
}

bool alignCandidatePairs(
    const dense_mapping::AlignmentConfig& config, const vi_map::VIMap& map,
    const vi_map::MissionIdList& mission_ids,
    const dense_mapping::AlignmentCandidatePairs& candidate_pairs,
    const bool is_loam, const std::string& alignment_name,
    std::shared_ptr<regbox::BaseController>* aligner_ptr,
    dense_mapping::AlignmentCandidatePairs* aligned_candidate_pairs,
    std::vector<RegistrationMetrics>* result_metrics) {
  CHECK_NOTNULL(aligned_candidate_pairs);
  CHECK_NOTNULL(aligner_ptr);
  CHECK(*aligner_ptr);
  const vi_map::VIMission& mission = map.getMission(mission_ids[0]);
  const size_t num_pairs = candidate_pairs.size();

  size_t successful_alignments = 0;
  // Move iterator to the start idx, since for an unordered_map we cannot
  // access it by index directly.
  auto it = candidate_pairs.cbegin();

  timing::TimerImpl timer(alignment_name);
  // Do the work.
  backend::ResourceType current_resource_type = backend::ResourceType::kCount;
  for (size_t idx = 0; idx < num_pairs && it != candidate_pairs.cend();
       ++idx, ++it) {
    const dense_mapping::AlignmentCandidatePair& pair = *it;

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
      current_resource_type = resource_type_A;
    }

    // Launch a resource type specific alignment.
    dense_mapping::AlignmentCandidatePair processed_pair = pair;
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
          // Extract depth resource.
          resources::PointCloud candidate_resource_A, candidate_resource_B;
          if (!map.getSensorResource<resources::PointCloud>(
                  mission, pair.candidate_A.resource_type,
                  pair.candidate_A.sensor_id, pair.candidate_A.timestamp_ns,
                  &candidate_resource_A)) {
            LOG(ERROR) << "Unable to retrieve resource for candidate A.";
            return false;
          }
          if (!map.getSensorResource<resources::PointCloud>(
                  mission, pair.candidate_B.resource_type,
                  pair.candidate_B.sensor_id, pair.candidate_B.timestamp_ns,
                  &candidate_resource_B)) {
            LOG(ERROR) << "Unable to retrieve resource for candidate B.";
            return false;
          }
          if (is_loam) {
            auto feature_detector = regbox::LoamFeatureDetector();
            resources::PointCloud features_A, features_B;
            feature_detector.extractLoamFeaturesFromPointCloud(
                candidate_resource_A, &features_A);
            feature_detector.extractLoamFeaturesFromPointCloud(
                candidate_resource_B, &features_B);
            candidate_resource_A = features_A;
            candidate_resource_B = features_B;
          }

          for (size_t noise_idx = 0u; noise_idx <= FLAGS_n_noise_steps;
               noise_idx++) {
            RegistrationMetrics metrics;
            metrics.vertices_distance_m =
                processed_pair.T_SB_SA_init.getPosition().norm();

            const double noise_x =
                noise_idx * FLAGS_max_noise_x_m / FLAGS_n_noise_steps;
            const double noise_y =
                noise_idx * FLAGS_max_noise_y_m / FLAGS_n_noise_steps;
            const double noise_yaw = noise_idx * (M_PI / 180.) *
                                     FLAGS_max_noise_yaw_deg /
                                     FLAGS_n_noise_steps;
            metrics.noise_x = noise_x;
            metrics.noise_y = noise_y;
            metrics.noise_yaw = noise_yaw;
            Eigen::Vector3d p_SB_SA_init = pair.T_SB_SA_init.getPosition();
            p_SB_SA_init(0) += noise_x;
            p_SB_SA_init(1) += noise_y;

            const Eigen::Matrix3d R_SB_SA_init =
                pair.T_SB_SA_init.getRotationMatrix() *
                yawToRotationMatrix(noise_yaw);
            const aslam::Transformation T_SB_SA_init(
                p_SB_SA_init, aslam::Quaternion(R_SB_SA_init));

            timer.Start();
            const regbox::RegistrationResult result =
                (*aligner_ptr)
                    ->align(
                        candidate_resource_B, candidate_resource_A,
                        T_SB_SA_init);

            metrics.runtime_s = timer.Stop();
            processed_pair.success = result.hasConverged();
            processed_pair.T_SB_SA_final = result.get_T_target_source();
            processed_pair.T_SB_SA_final_covariance =
                result.get_T_target_source_covariance();
            aligned_without_error = processed_pair.success;

            const aslam::Transformation T_SA_init_SA_final =
                processed_pair.T_SB_SA_init.inverse() *
                processed_pair.T_SB_SA_final;

            metrics.delta_position_m = T_SA_init_SA_final.getPosition().norm();
            static constexpr double kRadToDeg = 180.0 / M_PI;
            metrics.delta_rotation_deg =
                aslam::AngleAxis(T_SA_init_SA_final.getRotation()).angle() *
                kRadToDeg;

            resources::PointCloud candidate_resource_A_aligned =
                candidate_resource_A;
            candidate_resource_A_aligned.applyTransformation(
                processed_pair.T_SB_SA_final);

            metrics.fitness_score = getEuclideanFitnessScore(
                FLAGS_max_score_distance, candidate_resource_B,
                candidate_resource_A_aligned);
            result_metrics->push_back(metrics);
            // LOG(INFO) << "Score: " << metrics.fitness_score;
            resources::PointCloud merged_point_cloud = candidate_resource_B;
            merged_point_cloud.append(candidate_resource_A_aligned);
            sensor_msgs::PointCloud2 points_msg;
            backend::convertPointCloudType(merged_point_cloud, &points_msg);
            points_msg.header.stamp.fromNSec(
                processed_pair.candidate_B.timestamp_ns);
            points_msg.header.frame_id = "debug";
            visualization::RVizVisualizationSink::publish(
                "/failed_registrations", points_msg);
            // sleep(1);
            geometry_msgs::PoseStamped init_pose;
            tf::poseKindrToMsg(pair.T_SB_SA_init, &init_pose.pose);
            init_pose.header.frame_id = "debug";

            init_pose.header.stamp.fromNSec(
                processed_pair.candidate_B.timestamp_ns);
            visualization::RVizVisualizationSink::publish(
                "/init_pose", init_pose);
            geometry_msgs::PoseStamped pose;
            tf::poseKindrToMsg(T_SB_SA_init, &pose.pose);
            pose.header.frame_id = "debug";

            pose.header.stamp.fromNSec(processed_pair.candidate_B.timestamp_ns);
            visualization::RVizVisualizationSink::publish("/noise_pose", pose);
            if (!processed_pair.success) {
              // LOG(INFO) << "Alignment failed";
            }
            // Only keep pair if successful.
            if (processed_pair.success) {
              aligned_candidate_pairs->emplace(processed_pair);
              ++successful_alignments;
            }
          }
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
  }

  VLOG(1) << "Successfully aligned " << successful_alignments << "/"
          << num_pairs << " candidates.";
  LOG(INFO) << timing::Timing::Print();
  return true;
}

bool testRegistrationToolboxWithMap(
    const vi_map::MissionIdList& mission_ids, vi_map::VIMap* vi_map_ptr) {
  CHECK_NOTNULL(vi_map_ptr);
  CHECK(!mission_ids.empty());

  dense_mapping::Config config = dense_mapping::Config::fromGflags();
  dense_mapping::AlignmentCandidatePairs candidates;
  findCandidates(*vi_map_ptr, mission_ids, config, &candidates);
  LOG(INFO) << "Found " << candidates.size() << " candidates.";

  std::shared_ptr<regbox::BaseController> aligner;
  dense_mapping::AlignmentCandidatePairs aligned_candidates;
  if (FLAGS_lpm) {
    aligner =
        regbox::BaseController::make(regbox::Aligner::LpmIcp, "ADMC Aligner");
    std::vector<RegistrationMetrics> lpm_metrics;
    if (!alignCandidatePairs(
            config.alignment_config, *vi_map_ptr, mission_ids, candidates,
            false, "LPM", &aligner, &aligned_candidates, &lpm_metrics)) {
      LOG(ERROR) << "Computing the alignment of the candidates failed!";
      return false;
    }
    calculateRegistrationPerformance(lpm_metrics);
  }
  if (FLAGS_pcl_icp) {
    aligner = regbox::BaseController::make(
        regbox::Aligner::PclIcp, "PCL ICP Aligner");
    aligned_candidates = dense_mapping::AlignmentCandidatePairs();
    std::vector<RegistrationMetrics> pcl_icp_metrics;
    if (!alignCandidatePairs(
            config.alignment_config, *vi_map_ptr, mission_ids, candidates,
            false, "PCL ICP", &aligner, &aligned_candidates,
            &pcl_icp_metrics)) {
      LOG(ERROR) << "Computing the alignment of the candidates failed!";
      return false;
    }
    calculateRegistrationPerformance(pcl_icp_metrics);
  }
  if (FLAGS_pcl_gicp) {
    aligner = regbox::BaseController::make(
        regbox::Aligner::PclGIcp, "PCL GICP Aligner");
    aligned_candidates = dense_mapping::AlignmentCandidatePairs();
    std::vector<RegistrationMetrics> pcl_gicp_metrics;
    if (!alignCandidatePairs(
            config.alignment_config, *vi_map_ptr, mission_ids, candidates,
            false, "PCL GICP", &aligner, &aligned_candidates,
            &pcl_gicp_metrics)) {
      LOG(ERROR) << "Computing the alignment of the candidates failed!";
      return false;
    }
    calculateRegistrationPerformance(pcl_gicp_metrics);
  }
  if (FLAGS_pcl_vgicp) {
    aligner = regbox::BaseController::make(
        regbox::Aligner::PclVGIcp, "PCL VGICP Aligner");
    aligned_candidates = dense_mapping::AlignmentCandidatePairs();
    std::vector<RegistrationMetrics> pcl_vgicp_metrics;
    if (!alignCandidatePairs(
            config.alignment_config, *vi_map_ptr, mission_ids, candidates,
            false, "PCL VGICP", &aligner, &aligned_candidates,
            &pcl_vgicp_metrics)) {
      LOG(ERROR) << "Computing the alignment of the candidates failed!";
      return false;
    }
    calculateRegistrationPerformance(pcl_vgicp_metrics);
  }
  if (FLAGS_loam) {
    aligner =
        regbox::BaseController::make(regbox::Aligner::Loam, "Loam Aligner");
    aligned_candidates = dense_mapping::AlignmentCandidatePairs();
    std::vector<RegistrationMetrics> loam_metrics;
    if (!alignCandidatePairs(
            config.alignment_config, *vi_map_ptr, mission_ids, candidates, true,
            "LOAM", &aligner, &aligned_candidates, &loam_metrics)) {
      LOG(ERROR) << "Computing the alignment of the candidates failed!";
      return false;
    }
    calculateRegistrationPerformance(loam_metrics);
  }
  return true;
}
}  // namespace registration_toolbox_testing
