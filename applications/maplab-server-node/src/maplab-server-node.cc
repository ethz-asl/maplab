#include "maplab-server-node/maplab-server-node.h"

#include <aslam/common/timer.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <landmark-triangulation/pose-interpolator.h>
#include <map-anchoring/map-anchoring.h>
#include <map-optimization/outlier-rejection-solver.h>
#include <map-optimization/solver-options.h>
#include <map-optimization/vi-map-optimizer.h>
#include <map-optimization/vi-optimization-builder.h>
#include <maplab-common/file-system-tools.h>
#include <maplab-common/sigint-breaker.h>
#include <maplab-common/threading-helpers.h>
#include <signal.h>
#include <vi-map-basic-plugin/vi-map-basic-plugin.h>
#include <vi-map-helpers/vi-map-landmark-quality-evaluation.h>
#include <vi-map/landmark-quality-metrics.h>

#include <atomic>
#include <memory>
#include <string>

DECLARE_bool(ros_free);
DECLARE_uint64(vi_map_landmark_quality_min_observers);

DEFINE_int32(
    maplab_server_submap_loading_thread_pool_size, 4,
    "Number of threads used to load and pre-process incoming submaps. These "
    "threads are different from the one thread that is merging and optimizing "
    "the global map.");

DEFINE_string(
    maplab_server_merged_map_folder, "",
    "Where the finished/intermediate maps should be stored. Not optional.");

DEFINE_string(
    maplab_server_resource_folder, "",
    "Where the resources of the merged map should be stored, if empty, the "
    "standard map resource folder is used.");

DEFINE_int32(
    maplab_server_backup_interval_s, 300,
    "Create a backup of the current map every n seconds. 0 = no backups.");

DEFINE_bool(
    maplab_server_enable_default_submap_processing, true,
    "If enabled, the submap processing commands are ignored and a default set "
    "of commands is executed on the submaps. These commands are landmark "
    "quality evaluation and optimization.");

DEFINE_bool(
    maplab_server_remove_outliers_in_absolute_pose_constraints, true,
    "If enabled, the submap processing will after optimization run "
    "RANSAC LSQ on the absolute pose constraints to remove outliers.");

DEFINE_int32(
    maplab_server_dense_map_resource_type, 21,
    "Type of resources that are used to compose the dense map, options are ["
    "kRawDepthMap = 8, kOptimizedDepthMap = 9, kPointCloudXYZ = 16, "
    "kPointCloudXYZRGBN = 17, kVoxbloxOccupancyMap = 20, kPointCloudXYZI = "
    "21]");

namespace maplab {
MaplabServerNode::MaplabServerNode(const MaplabServerNodeConfig& config)
    : config_(config),
      submap_loading_thread_pool_(
          FLAGS_maplab_server_submap_loading_thread_pool_size),
      base_console_("base_console", 0 /*argc*/, nullptr /*argv*/),
      plotter_(nullptr),
      is_running_(false),
      shut_down_requested_(false),
      merging_thread_busy_(false),
      time_of_last_map_backup_s_(0.0),
      duration_last_merging_loop_s_(0.0) {
  if (!FLAGS_ros_free) {
    visualization::RVizVisualizationSink::init();
    plotter_.reset(new visualization::ViwlsGraphRvizPlotter);
  }
}

MaplabServerNode::~MaplabServerNode() {
  shutdown();
}

void MaplabServerNode::start() {
  std::lock_guard<std::mutex> lock(mutex_);
  LOG(INFO) << "[MaplabServerNode] Starting...";

  if (shut_down_requested_.load()) {
    LOG(ERROR)
        << "[MaplabServerNode] Cannot start node (again), a shutdown has "
        << "already been requrested!";
    return;
  }

  LOG(INFO) << "[MaplabServerNode] launching MapMerging thread...";

  submap_merging_thread_ = std::thread([this]() {
    // Loop until shutdown is requested.
    bool received_first_submap = false;
    while (!shut_down_requested_.load()) {
      timing::TimerImpl map_merging_timer("map-merging");

      // Delete blacklisted submap mission, if no missions remain in the merged
      // map, it will return false and therefore reset the
      // 'received_first_submap' variable.
      received_first_submap &= deleteBlacklistedMissions();

      std::vector<std::string> all_map_keys;
      map_manager_.getAllMapKeys(&all_map_keys);

      // List all loaded maps.
      if (VLOG_IS_ON(1) && !all_map_keys.empty()) {
        std::stringstream ss;
        ss << "[MaplabServerNode] MapMerging - Loaded maps ("
           << all_map_keys.size() << " total):";
        std::sort(all_map_keys.begin(), all_map_keys.end());
        for (const std::string& key : all_map_keys) {
          ss << "\n  " << key;
        }
        VLOG(1) << ss.str();
      }

      if (!received_first_submap && all_map_keys.empty()) {
        VLOG(1) << "[MaplabServerNode] MapMerging - waiting for first "
                   "submap to be loaded...";

        map_merging_timer.Discard();

        std::this_thread::sleep_for(
            std::chrono::seconds(kSecondsToSleepBetweenAttempts));
        continue;
      }

      merging_thread_busy_ = true;

      received_first_submap |= appendAvailableSubmaps();

      if (received_first_submap) {
        VLOG(3) << "[MaplabServerNode] MapMerging - processing global map "
                << "commands on map with key '" << kMergedMapKey << "'";

        runOneIterationOfMapMergingCommands();

        publishDenseMap();

        publishMostRecentVertexPoseAndCorrection();

        saveMapEveryInterval();

        duration_last_merging_loop_s_.store(map_merging_timer.Stop());
      } else {
        map_merging_timer.Discard();
      }

      merging_thread_busy_ = false;
      std::this_thread::sleep_for(
          std::chrono::seconds(kSecondsToSleepBetweenAttempts));
    }
  });

  LOG(INFO) << "[MaplabServerNode] launching Status thread...";

  status_thread_ = std::thread([this]() {
    // Loop until shutdown is requested.
    while (!shut_down_requested_.load()) {
      printAndPublishServerStatus();
      std::this_thread::sleep_for(
          std::chrono::seconds(kSecondsToSleepBetweenStatus));
    }
  });

  is_running_ = true;
  LOG(INFO) << "[MaplabServerNode] MapMerging - thread launched.";
}

void MaplabServerNode::shutdown() {
  if (shut_down_requested_.load()) {
    // Already shut down.
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  LOG(INFO) << "[MaplabServerNode] Shutting down...";
  shut_down_requested_.store(true);

  LOG(INFO) << "[MaplabServerNode] Stopping MapMerging thread...";
  submap_merging_thread_.join();
  LOG(INFO) << "[MaplabServerNode] Done.";

  LOG(INFO) << "[MaplabServerNode] Stopping SubmapProcessing threads...";
  submap_loading_thread_pool_.stop();
  submap_loading_thread_pool_.waitForEmptyQueue();
  LOG(INFO) << "[MaplabServerNode] Done.";

  LOG(INFO) << "[MaplabServerNode] Stopping Status thread...";
  status_thread_.join();
  LOG(INFO) << "[MaplabServerNode] Done.";

  is_running_ = false;
}

bool MaplabServerNode::saveMap(const std::string& path) {
  std::lock_guard<std::mutex> lock(mutex_);
  LOG(INFO) << "[MaplabServerNode] Saving map to '" << path << "'.";
  if (map_manager_.hasMap(kMergedMapKey)) {
    return map_manager_.saveMapToFolder(
        kMergedMapKey, path, vi_map::parseSaveConfigFromGFlags());
  } else {
    return false;
  }
}

bool MaplabServerNode::isSubmapBlacklisted(const std::string& map_key) {
  CHECK(map_manager_.hasMap(map_key));

  vi_map::MissionId submap_mission_id;
  {
    vi_map::VIMapManager::MapReadAccess submap =
        map_manager_.getMapReadAccess(map_key);
    CHECK_EQ(submap->numMissions(), 1u);
    submap_mission_id = submap->getIdOfFirstMission();
  }

  bool mission_is_blacklisted = false;
  {
    std::lock_guard<std::mutex> lock(blacklisted_missions_mutex_);
    mission_is_blacklisted =
        blacklisted_missions_.count(submap_mission_id) > 0u;
  }
  if (mission_is_blacklisted) {
    return true;
  }
  return false;
}

bool MaplabServerNode::loadAndProcessSubmap(
    const std::string& robot_name, const std::string& submap_path) {
  CHECK(!submap_path.empty());
  CHECK(!robot_name.empty());

  std::lock_guard<std::mutex> lock(mutex_);

  if (shut_down_requested_.load()) {
    LOG(WARNING) << "[MaplabServerNode] shutdown was requrested, will ignore "
                 << " SubmapProcessing thread for submap at '" << submap_path
                 << "'.";
    return false;
  }

  VLOG(1) << "[MaplabServerNode] launching SubmapProcessing thread for "
          << "submap at '" << submap_path << "'.";

  std::lock_guard<std::mutex> submap_queue_lock(submap_processing_queue_mutex_);
  // Add new element at the back.
  submap_processing_queue_.emplace_back();

  SubmapProcess& submap_process = submap_processing_queue_.back();
  submap_process.path = submap_path;
  submap_process.robot_name = robot_name;
  submap_process.map_hash = std::hash<std::string>{}(submap_path);
  submap_process.map_key =
      submap_process.robot_name + "_" + std::to_string(submap_process.map_hash);

  // Start a thread that loads the map and updates the submap entry in the
  // queue when done. The submap mutex will be released in the meantime,
  // such that submaps that arrive later can be loaded and processed in
  // parallel.
  const size_t kSubmapLoadingExclusivityGroup =
      // std::hash<std::string>{}(submap_process.robot_name);
      aslam::ThreadPool::kGroupdIdNonExclusiveTask;
  submap_loading_thread_pool_.enqueueOrdered(
      kSubmapLoadingExclusivityGroup, [&submap_process, this]() {
        std::lock_guard<std::mutex> submap_process_lock(submap_process.mutex);

        timing::TimerImpl submap_processing_timer("submap_processing");

        VLOG(3) << "[MaplabServerNode] SubmapProcessing - loading and "
                << "processing submap from '" << submap_process.path << "'...";

        {
          std::lock_guard<std::mutex> command_lock(submap_commands_mutex_);
          submap_commands_[submap_process.map_hash] = "loading";
        }

        // TODO(mfehr): Make this more robust: in case of a submap failing to
        // load (submap is lost), treat subsequent submaps of the same robot
        // as new trajectory and start a new mission.

        if (map_manager_.hasMap(submap_process.map_key)) {
          // NOTE: this has never happened so far and should lead to an
          // immediate check fail, but in the interest of not loosing this
          // submap we will handle this softly.
          const std::string old_key = submap_process.map_key;
          submap_process.map_key += "_something_is_fishy";
          LOG(ERROR)
              << "[MaplabServerNode] There is already a map with this map "
              << "key in storage, something went wrong! key '" << old_key
              << "'. Changing the key to: '" << submap_process.map_key << "'.";
        }

        CHECK(map_manager_.loadMapFromFolder(
            submap_process.path, submap_process.map_key));

        submap_process.is_loaded = true;

        VLOG(3) << "[MaplabServerNode] SubmapProcessing - finished loading "
                   "submap with key '"
                << submap_process.map_key << "', starts processing...";

        if (isSubmapBlacklisted(submap_process.map_key)) {
          LOG(WARNING) << "[MaplabServerNode] SubmapProcessing - received a "
                          "blacklisted submap, skip processing...";

          {
            std::lock_guard<std::mutex> command_lock(submap_commands_mutex_);
            submap_commands_[submap_process.map_hash] = "blacklisted";
          }

          // We skip the processing part, the merging thread will then discard
          // the submap.
          submap_process.is_processed = true;
          return true;
        }

        extractLatestUnoptimizedPoseFromSubmap(submap_process);

        runSubmapProcessingCommands(submap_process);

        submap_process.is_processed = true;

        VLOG(3) << "[MaplabServerNode] SubmapProcessing - finished processing "
                   "submap with key '"
                << submap_process.map_key << "'.";
        return true;
      });

  VLOG(1) << "[MaplabServerNode] SubmapProcessing - thread launched.";
  return true;
}

bool MaplabServerNode::saveMap() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (FLAGS_maplab_server_merged_map_folder.empty()) {
    LOG(ERROR) << "[MaplabServerNode] Cannot save map because "
                  "--maplab_server_merged_map_folder is empty!";
    return false;
  }

  LOG(INFO) << "[MaplabServerNode] Saving map to '"
            << FLAGS_maplab_server_merged_map_folder << "'.";
  if (map_manager_.hasMap(kMergedMapKey)) {
    return map_manager_.saveMapToFolder(
        kMergedMapKey, FLAGS_maplab_server_merged_map_folder,
        vi_map::parseSaveConfigFromGFlags());
  } else {
    return false;
  }
}

void MaplabServerNode::visualizeMap() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (plotter_ != nullptr) {
    if (map_manager_.hasMap(kMergedMapKey)) {
      LOG(INFO) << "[MaplabServerNode] Visualizing map...";
      const vi_map::VIMapManager map_manager;
      vi_map::VIMapManager::MapReadAccess map =
          map_manager.getMapReadAccess(kMergedMapKey);
      plotter_->visualizeMap(*map);
    } else {
      LOG(WARNING) << "[MaplabServerNode] Could not visualize merged map, as "
                      "it doesn't exist yet!";
    }
  } else {
    LOG(WARNING) << "[MaplabServerNode] No plotter was added to the maplab "
                 << "server node, cannot visualize map!";
  }
}

MaplabServerNode::MapLookupStatus MaplabServerNode::mapLookup(
    const std::string& robot_name, const vi_map::SensorType sensor_type,
    const int64_t timestamp_ns, const Eigen::Vector3d& p_S,
    Eigen::Vector3d* p_G, Eigen::Vector3d* sensor_p_G) const {
  CHECK_NOTNULL(p_G);
  CHECK_NOTNULL(sensor_p_G);

  if (robot_name.empty()) {
    LOG(WARNING)
        << "[MaplabServerNode] Received map lookup with empty robot name!";
    return MapLookupStatus::kNoSuchMission;
  }
  vi_map::MissionId submap_mission_id;
  {
    std::lock_guard<std::mutex> lock(robot_to_mission_id_map_mutex_);
    if (robot_to_mission_id_map_.count(robot_name) == 0u) {
      LOG(WARNING) << "[MaplabServerNode] Received map lookup with invalid "
                      "robot name: "
                   << robot_name;
      return MapLookupStatus::kNoSuchMission;
    }

    const RobotMissionInformation& robot_info =
        robot_to_mission_id_map_.at(robot_name);

    submap_mission_id = robot_info.mission_ids.front();

    if (!submap_mission_id.isValid()) {
      LOG(ERROR)
          << "[MaplabServerNode] Received map lookup with valid robot name ("
          << robot_name
          << "), but an invalid mission id is associated with it!";
      return MapLookupStatus::kNoSuchMission;
    }
  }

  if (timestamp_ns < 0) {
    LOG(WARNING)
        << "[MaplabServerNode] Received map lookup with invalid timestamp: "
        << timestamp_ns << "ns";
    return MapLookupStatus::kPoseNeverAvailable;
  }

  CHECK(submap_mission_id.isValid());
  {
    vi_map::VIMapManager::MapReadAccess map =
        map_manager_.getMapReadAccess(kMergedMapKey);

    if (!map->hasMission(submap_mission_id)) {
      LOG(ERROR)
          << "[MaplabServerNode] Received map lookup with valid robot name ("
          << robot_name
          << "), but a mission id is associated with it that is not part of "
          << "the map (yet)!";
      return MapLookupStatus::kNoSuchMission;
    }

    const vi_map::VIMission& mission = map->getMission(submap_mission_id);

    aslam::SensorId sensor_id;
    if (sensor_type == vi_map::SensorType::kNCamera) {
      if (!mission.hasNCamera()) {
        LOG(WARNING) << "[MaplabServerNode] Received map lookup with NCamera "
                     << "sensor, but there is no such sensor in the map!";
        return MapLookupStatus::kNoSuchSensor;
      }
      sensor_id = mission.getNCameraId();
    } else if (sensor_type == vi_map::SensorType::kImu) {
      if (!mission.hasImu()) {
        LOG(WARNING) << "[MaplabServerNode] Received map lookup with IMU "
                     << "sensor, but there is no such sensor in the map!";
        return MapLookupStatus::kNoSuchSensor;
      }
      sensor_id = mission.getImuId();
    } else if (sensor_type == vi_map::SensorType::kLidar) {
      if (!mission.hasLidar()) {
        LOG(WARNING) << "[MaplabServerNode] Received map lookup with Lidar "
                     << "sensor, but there is no such sensor in the map!";
        return MapLookupStatus::kNoSuchSensor;
      }
      sensor_id = mission.getLidarId();
    } else if (sensor_type == vi_map::SensorType::kOdometry6DoF) {
      if (!mission.hasOdometry6DoFSensor()) {
        LOG(WARNING) << "[MaplabServerNode] Received map lookup with Odometry "
                     << "sensor, but there is no such sensor in the map!";
        return MapLookupStatus::kNoSuchSensor;
      }
      sensor_id = mission.getOdometry6DoFSensor();
    } else {
      LOG(WARNING)
          << "[MaplabServerNode] Received map lookup with invalid sensor!";
      return MapLookupStatus::kNoSuchSensor;
    }
    const aslam::Transformation T_B_S =
        map->getSensorManager().getSensor_T_B_S(sensor_id);

    const aslam::Transformation& T_G_M =
        map->getMissionBaseFrameForMission(submap_mission_id).get_T_G_M();

    landmark_triangulation::VertexToTimeStampMap vertex_to_time_map;
    int64_t min_timestamp_ns;
    int64_t max_timestamp_ns;
    const landmark_triangulation::PoseInterpolator pose_interpolator;
    pose_interpolator.getVertexToTimeStampMap(
        *map, submap_mission_id, &vertex_to_time_map, &min_timestamp_ns,
        &max_timestamp_ns);
    if (timestamp_ns < min_timestamp_ns) {
      LOG(WARNING) << "[MaplabServerNode] Received map lookup with timestamp "
                      "that is before the selected robot mission, this "
                      "position will never be available: "
                   << aslam::time::timeNanosecondsToString(timestamp_ns)
                   << " - earliest map time: "
                   << aslam::time::timeNanosecondsToString(min_timestamp_ns);
      return MapLookupStatus::kPoseNeverAvailable;
    } else if (timestamp_ns > max_timestamp_ns) {
      LOG(WARNING) << "[MaplabServerNode] Received map lookup with timestamp "
                      "that is not yet available: "
                   << aslam::time::timeNanosecondsToString(timestamp_ns)
                   << " - most recent map time: "
                   << aslam::time::timeNanosecondsToString(max_timestamp_ns);
      return MapLookupStatus::kPoseNotAvailableYet;
    }

    Eigen::Matrix<int64_t, 1, Eigen::Dynamic> timestamps_ns =
        Eigen::Matrix<int64_t, 1, 1>::Constant(timestamp_ns);

    aslam::TransformationVector T_M_B_vector;
    pose_interpolator.getPosesAtTime(
        *map, submap_mission_id, timestamps_ns, &T_M_B_vector);
    CHECK_EQ(static_cast<int>(T_M_B_vector.size()), timestamps_ns.cols());

    const aslam::Transformation T_G_B = T_G_M * T_M_B_vector[0];
    const aslam::Transformation T_G_S = T_G_B * T_B_S;

    *p_G = T_G_S * p_S;
    *sensor_p_G = T_G_S * Eigen::Vector3d::Zero();
  }
  return MapLookupStatus::kSuccess;
}

void MaplabServerNode::registerPoseCorrectionPublisherCallback(
    std::function<void(
        const int64_t, const std::string&, const aslam::Transformation&,
        const aslam::Transformation&, const aslam::Transformation&,
        const aslam::Transformation&)>
        callback) {
  CHECK(callback);
  pose_correction_publisher_callback_ = callback;
}

void MaplabServerNode::runOneIterationOfMapMergingCommands() {
  // Copy console to process the global map.
  MapLabConsole console(
      base_console_, "global_map_console", false /*disable plotter*/);

  // Select submap.
  console.setSelectedMapKey(kMergedMapKey);

  for (const std::string& command : config_.global_map_commands) {
    {
      std::lock_guard<std::mutex> merge_command_lock(
          current_merge_command_mutex_);
      current_merge_command_ = command;
    }
    VLOG(1) << "[MaplabServerNode] Running: " << command;
    if (console.RunCommand(command) != common::kSuccess) {
      LOG(WARNING)
          << "[MaplabServerNode] MapMerging - Command did not succeed: '"
          << command << "'.";
    } else {
      VLOG(3) << "[MaplabServerNode] MapMerging console command successful.";
    }
  }
  {
    std::lock_guard<std::mutex> merge_command_lock(
        current_merge_command_mutex_);
    current_merge_command_ = "";
  }
}

void MaplabServerNode::publishMostRecentVertexPoseAndCorrection() {
  vi_map::VIMapManager::MapReadAccess map =
      map_manager_.getMapReadAccess(kMergedMapKey);

  vi_map::MissionIdList mission_ids;
  map->getAllMissionIds(&mission_ids);
  if (!mission_ids.empty()) {
    std::lock_guard<std::mutex> lock(robot_to_mission_id_map_mutex_);
    for (const vi_map::MissionId& mission_id : mission_ids) {
      const std::string& robot_name = mission_id_to_robot_map_[mission_id];
      if (robot_name.empty()) {
        LOG(ERROR) << "[MaplabServerNode] MapMerging - No entry found in "
                      "mission id to robot name index for mission "
                   << mission_id << "!";
        continue;
      }
      if (robot_to_mission_id_map_.count(robot_name) == 0u) {
        LOG(ERROR) << "[MaplabServerNode] MapMerging - No entry found in "
                      "robot name to mission id map for robot "
                   << robot_name << "!";
        continue;
      }
      RobotMissionInformation& robot_info =
          robot_to_mission_id_map_[robot_name];

      // Get the latest vertex of the robot mission.
      const pose_graph::VertexId last_vertex_id =
          map->getLastVertexIdOfMission(mission_id);
      const vi_map::Vertex& last_vertex = map->getVertex(last_vertex_id);
      const aslam::Transformation& T_G_M_latest =
          map->getMissionBaseFrameForMission(mission_id).get_T_G_M();
      const aslam::Transformation& T_M_B_latest = last_vertex.get_T_M_I();
      const int64_t current_last_vertex_timestamp_ns =
          last_vertex.getMinTimestampNanoseconds();

      if (pose_correction_publisher_callback_) {
        const auto it_T_M_B = robot_info.T_M_B_submaps_input.find(
            current_last_vertex_timestamp_ns);
        const auto it_T_G_M = robot_info.T_G_M_submaps_input.find(
            current_last_vertex_timestamp_ns);
        if (it_T_M_B != robot_info.T_M_B_submaps_input.end() &&
            it_T_G_M != robot_info.T_G_M_submaps_input.end()) {
          const aslam::Transformation T_G_curr_B_curr =
              T_G_M_latest * T_M_B_latest;
          const aslam::Transformation T_G_curr_M_curr = T_G_M_latest;

          const aslam::Transformation& T_G_in_B_in =
              it_T_G_M->second * it_T_M_B->second;
          const aslam::Transformation& T_G_in_M_in = it_T_G_M->second;

          pose_correction_publisher_callback_(
              current_last_vertex_timestamp_ns, robot_name, T_G_curr_B_curr,
              T_G_curr_M_curr, T_G_in_B_in, T_G_in_M_in);
        } else {
          LOG(ERROR) << "[MaplabServerNode] MapMerging - Could not "
                     << "find corresponding original pose for the "
                     << "latest vertex (" << current_last_vertex_timestamp_ns
                     << ") of robot " << robot_name << "!";

          {
            std::stringstream ss;
            ss << "\nT_G_M_submaps_input:";
            for (auto entry : robot_info.T_G_M_submaps_input) {
              ss << " - " << entry.first << "ns\n";
            }
            LOG(INFO) << ss.str();
          }
          {
            std::stringstream ss;
            ss << "\nT_M_B_submaps_input:";
            for (auto entry : robot_info.T_M_B_submaps_input) {
              ss << " - " << entry.first << "ns\n";
            }
            LOG(INFO) << ss.str();
          }
        }
      }
    }
  }
}

void MaplabServerNode::saveMapEveryInterval() {
  const double time_now_s =
      aslam::time::nanoSecondsToSeconds(aslam::time::nanoSecondsSinceEpoch());

  if ((time_now_s - time_of_last_map_backup_s_) >
          FLAGS_maplab_server_backup_interval_s &&
      FLAGS_maplab_server_backup_interval_s > 0) {
    LOG(INFO) << "[MaplabServerNode] MapMerging - saving map as backup.";
    {
      std::lock_guard<std::mutex> merge_command_lock(
          current_merge_command_mutex_);
      current_merge_command_ = "save map";
    }
    saveMap();

    time_of_last_map_backup_s_ = time_now_s;
  }
}

bool MaplabServerNode::appendAvailableSubmaps() {
  bool found_new_submaps = false;

  std::lock_guard<std::mutex> lock(submap_processing_queue_mutex_);
  while (!submap_processing_queue_.empty() && !shut_down_requested_.load()) {
    SubmapProcess& submap_process = submap_processing_queue_.front();

    // Try to lock the submap_process struct, if we fail this means
    // something is still processing so we need to stop and try again
    // later.
    if (!submap_process.mutex.try_lock()) {
      break;
    }
    // Check if submap has finished loading.
    if (!submap_process.is_loaded) {
      // Give up and try again later.
      submap_process.mutex.unlock();
      break;
    }

    // Check if submap has finished processing as well.
    if (!submap_process.is_processed) {
      // Give up and try again later.
      submap_process.mutex.unlock();
      break;
    }

    // Check if submap is blacklisted and delete it.
    CHECK(!submap_process.map_key.empty());
    CHECK(map_manager_.hasMap(submap_process.map_key));
    if (isSubmapBlacklisted(submap_process.map_key)) {
      vi_map::MissionId submap_mission_id;
      {
        vi_map::VIMapManager::MapReadAccess submap =
            map_manager_.getMapReadAccess(submap_process.map_key);
        CHECK_EQ(submap->numMissions(), 1u);
        submap_mission_id = submap->getIdOfFirstMission();
      }

      LOG(WARNING) << "[MaplabServerNode] MapMerging - Received a new submap "
                   << "of deleted mission " << submap_mission_id
                   << ", will discard it.";

      // Delete map from manager.
      map_manager_.deleteMap(submap_process.map_key);

      // Erase submap processing status.
      {
        std::lock_guard<std::mutex> command_lock(submap_commands_mutex_);
        submap_commands_.erase(submap_process.map_hash);
      }

      // Unlock and delete the submap process struct.
      submap_process.mutex.unlock();
      submap_processing_queue_.pop_front();
      continue;
    }

    VLOG(3) << "[MaplabServerNode] MapMerging - submap with key '"
            << submap_process.map_key << "' is ready to be merged.";

    // If we don't have a merged map yet, simply rename the submap into
    // the merged map.
    {
      std::lock_guard<std::mutex> merge_command_lock(
          current_merge_command_mutex_);
      current_merge_command_ = "merging submap";
    }

    if (!map_manager_.hasMap(kMergedMapKey)) {
      VLOG(3) << "[MaplabServerNode] MapMerging - first submap is "
                 "used to initalize merged map with key '"
              << kMergedMapKey << "'.";
      map_manager_.renameMap(submap_process.map_key, kMergedMapKey);
      found_new_submaps = true;
    } else {
      LOG(INFO) << "[MaplabServerNode] MapMerging - merge submap '"
                << submap_process.map_key << "' into "
                << "merged map with key '" << kMergedMapKey << "'";

      // TODO(mfehr): make this more robust: if merging fails, either
      // try again later or load as new mission so we don't loose the
      // data.
      CHECK(map_manager_.mergeSubmapIntoBaseMap(
          kMergedMapKey, submap_process.map_key));
      // Remove submap.
      map_manager_.deleteMap(submap_process.map_key);
    }
    CHECK(map_manager_.hasMap(kMergedMapKey));
    CHECK(!map_manager_.hasMap(submap_process.map_key));

    submap_process.is_merged = true;

    // Unlock the submap process struct.
    submap_process.mutex.unlock();

    // Remove the struct from the list of processed submaps.
    submap_processing_queue_.pop_front();
  }

  return found_new_submaps;
}

void MaplabServerNode::printAndPublishServerStatus() {
  std::stringstream ss;

  ss << "\n=============================================================="
        "=="
     << "==\n";
  ss << "[MaplabServerNode] Status:\n";
  {
    std::lock_guard<std::mutex> lock(submap_processing_queue_mutex_);
    if (submap_processing_queue_.empty()) {
      ss << " - No submaps to process or merge...\n";
    } else {
      for (const SubmapProcess& submap_process : submap_processing_queue_) {
        ss << " - " << submap_process.robot_name << " - map '"
           << submap_process.map_key << "'\t: ";

        bool was_locked_by_other_process = false;
        if (submap_process.mutex.try_lock()) {
          ss << "(unlocked)";
          was_locked_by_other_process = false;
        } else {
          was_locked_by_other_process = true;
          ss << "(locked)";
        }

        if (submap_process.is_merged && was_locked_by_other_process) {
          ss << " merged\n";
        } else if (submap_process.is_merged && !was_locked_by_other_process) {
          LOG(ERROR) << "[MaplabServerNode] A submap process cannot be "
                        "merged and "
                     << "unlocked at the same time! Something is wrong!";
          ss << " ERROR!\n";
        } else if (submap_process.is_processed && was_locked_by_other_process) {
          ss << " merging...\n";
        } else if (
            submap_process.is_processed && !was_locked_by_other_process) {
          ss << " ready to merge\n";
        } else if (submap_process.is_loaded && was_locked_by_other_process) {
          ss << " processing...\n";
        } else if (submap_process.is_loaded && !was_locked_by_other_process) {
          ss << " queued for processing\n";
        } else if (!submap_process.is_loaded && was_locked_by_other_process) {
          ss << " loading...\n";
        } else if (!submap_process.is_loaded && !was_locked_by_other_process) {
          ss << " queued for loading\n";
        }

        if (!was_locked_by_other_process) {
          submap_process.mutex.unlock();
        }
      }
    }
  }
  ss << "================================================================"
     << "==\n";
  ss << " - Active submap threads: "
     << submap_loading_thread_pool_.numActiveThreads() << "/"
     << FLAGS_maplab_server_submap_loading_thread_pool_size << "\n";
  {
    std::lock_guard<std::mutex> command_lock(submap_commands_mutex_);
    for (const std::pair<const size_t, std::string>& comm : submap_commands_) {
      ss << "   - submap " << std::to_string(comm.first)
         << " - command: " << comm.second << "\n";
    }
  }

  ss << " - Active merging thread: ";
  if (merging_thread_busy_.load()) {
    ss << "yes\n";
    std::lock_guard<std::mutex> merge_command_lock(
        current_merge_command_mutex_);
    ss << "   - current command: " << current_merge_command_ << "\n";
  } else {
    ss << "no\n";
  }
  ss << "   - duration of last iteration: "
     << duration_last_merging_loop_s_.load() << "s\n";
  ss << "================================================================"
     << "==\n";
  {
    std::lock_guard<std::mutex> lock(robot_to_mission_id_map_mutex_);
    ss << "Robot to mission map: ";
    for (const std::pair<const std::string, RobotMissionInformation>& pair :
         robot_to_mission_id_map_) {
      const RobotMissionInformation& robot_info = pair.second;
      ss << "\n - " << pair.first;
      for (const vi_map::MissionId& mission_id : robot_info.mission_ids) {
        ss << "\n\t - " << mission_id;
      }
    }
  }
  ss << "\n=============================================================="
        "=="
     << "==\n";
  const std::string status_string = ss.str();
  LOG(INFO) << status_string;

  if (status_publisher_callback_) {
    status_publisher_callback_(status_string);
  }
}

void MaplabServerNode::extractLatestUnoptimizedPoseFromSubmap(
    const SubmapProcess& submap_process) {
  vi_map::VIMapManager::MapReadAccess map =
      map_manager_.getMapReadAccess(submap_process.map_key);

  vi_map::MissionIdList mission_ids;
  map->getAllMissionIds(&mission_ids);
  if (mission_ids.size() == 1u) {
    const vi_map::MissionId& submap_mission_id = mission_ids[0];
    CHECK(submap_mission_id.isValid());

    const pose_graph::VertexId last_vertex_id =
        map->getLastVertexIdOfMission(submap_mission_id);

    const vi_map::Vertex& last_vertex = map->getVertex(last_vertex_id);

    const aslam::Transformation& T_G_M_submap =
        map->getMissionBaseFrameForMission(submap_mission_id).get_T_G_M();
    const aslam::Transformation& T_M_B_last_vertex = last_vertex.get_T_M_I();
    const int64_t last_vertex_timestamp_ns =
        last_vertex.getMinTimestampNanoseconds();

    {
      std::lock_guard<std::mutex> lock(robot_to_mission_id_map_mutex_);

      if (!submap_process.robot_name.empty()) {
        RobotMissionInformation& robot_info =
            robot_to_mission_id_map_[submap_process.robot_name];

        if (robot_info.mission_ids.empty()) {
          robot_info.mission_ids.push_front(submap_mission_id);
        } else if (robot_info.mission_ids.front() != submap_mission_id) {
          robot_info.mission_ids.push_front(submap_mission_id);
        }

        mission_id_to_robot_map_[submap_mission_id] = submap_process.robot_name;

        robot_info.T_G_M_submaps_input[last_vertex_timestamp_ns] = T_G_M_submap;
        robot_info.T_M_B_submaps_input[last_vertex_timestamp_ns] =
            T_M_B_last_vertex;

      } else {
        LOG(ERROR) << "[MaplabServerNode] Submap with key "
                   << submap_process.map_key
                   << " does not have a robot name associated with it! "
                   << "This might make some service calls such as the map "
                   << "lookup service impossible.";
      }
    }
  } else {
    LOG(ERROR) << "[MaplabServerNode] SubmapProcessing - submap loaded from "
               << "'" << submap_process.path << "' contains "
               << mission_ids.size() << " missions, but should contain one!";
  }
}

void MaplabServerNode::runSubmapProcessingCommands(
    const SubmapProcess& submap_process) {
  if (FLAGS_maplab_server_enable_default_submap_processing) {
    // We only want to get these once, such that if the gflags get modified
    // later the optimization settings for the submaps remain the same.
    static map_optimization::ViProblemOptions options =
        map_optimization::ViProblemOptions::initFromGFlags();
    static map_optimization::OutlierRejectionSolverOptions
        outlier_rejection_options =
            map_optimization::OutlierRejectionSolverOptions::initFromFlags();

    vi_map::VIMapManager map_manager;
    vi_map::VIMapManager::MapWriteAccess map =
        map_manager.getMapWriteAccess(submap_process.map_key);
    vi_map::MissionIdList missions_to_optimize_list;
    map->getAllMissionIds(&missions_to_optimize_list);

    // ELQ
    {
      std::lock_guard<std::mutex> command_lock(submap_commands_mutex_);
      submap_commands_[submap_process.map_hash] = "elq";
    }
    if (FLAGS_vi_map_landmark_quality_min_observers > 2) {
      LOG(WARNING) << "[[MaplabServerNode] Minimum required landmark observers "
                   << "is set to "
                   << FLAGS_vi_map_landmark_quality_min_observers
                   << ",  this might be too stricht if keyframing is enabled.";
    }
    vi_map_helpers::evaluateLandmarkQuality(
        missions_to_optimize_list, map.get());

    // OPTVI
    {
      std::lock_guard<std::mutex> command_lock(submap_commands_mutex_);
      submap_commands_[submap_process.map_hash] = "optvi";
    }
    const vi_map::MissionIdSet missions_to_optimize(
        missions_to_optimize_list.begin(), missions_to_optimize_list.end());
    map_optimization::VIMapOptimizer optimizer(
        nullptr /*no plotter*/, false /*signal handler enabled*/);
    optimizer.optimize(
        options, missions_to_optimize, &outlier_rejection_options, map.get());

    if (FLAGS_maplab_server_remove_outliers_in_absolute_pose_constraints) {
      std::lock_guard<std::mutex> command_lock(submap_commands_mutex_);
      submap_commands_[submap_process.map_hash] =
          "remove_abs_constraint_outlier";
      map_anchoring::removeOutliersInAbsolute6DoFConstraints(map.get());
    }

  } else {
    // Copy console to process the global map.
    const std::string console_name =
        "submap_processing_console_" + submap_process.map_key;
    MapLabConsole console(
        base_console_, console_name, false /*disable plotter*/);

    // Select submap.
    console.setSelectedMapKey(submap_process.map_key);

    for (const std::string& command : config_.submap_commands) {
      {
        std::lock_guard<std::mutex> command_lock(submap_commands_mutex_);
        submap_commands_[submap_process.map_hash] = command;
      }
      VLOG(3) << "[MaplabServerNode] SubmapProcessing console command: "
              << command;
      if (console.RunCommand(command) != common::kSuccess) {
        LOG(ERROR) << "[MaplabServerNode] SubmapProcessing - failed to run "
                      "command: '"
                   << command << "' on submap '" << submap_process.map_key
                   << "'.";
      } else {
        VLOG(3) << "[MaplabServerNode] SubmapProcessing console command "
                   "successful.";
      }

      if (shut_down_requested_.load()) {
        LOG(WARNING) << "[MaplabServerNode] SubmapProcessing - shutdown was "
                        "requested, aborting processing of submap with key '"
                     << submap_process.map_key << "'...";
        break;
      }
    }
  }
  {
    std::lock_guard<std::mutex> command_lock(submap_commands_mutex_);
    submap_commands_.erase(submap_process.map_hash);
  }
}

void MaplabServerNode::registerStatusCallback(
    std::function<void(const std::string&)> callback) {
  CHECK(callback);
  status_publisher_callback_ = callback;
}

void MaplabServerNode::publishDenseMap() {
  if (!map_manager_.hasMap(kMergedMapKey)) {
    return;
  }
  vi_map::VIMapManager::MapReadAccess map =
      map_manager_.getMapReadAccess(kMergedMapKey);
  std::unordered_map<std::string, vi_map::MissionIdList>
      robot_to_mission_id_map;
  {
    std::lock_guard<std::mutex> lock(robot_to_mission_id_map_mutex_);
    for (const auto& kv : mission_id_to_robot_map_) {
      robot_to_mission_id_map[kv.second].push_back(kv.first);
    }
  }
  visualization::visualizeReprojectedDepthResourcePerRobot(
      static_cast<backend::ResourceType>(
          FLAGS_maplab_server_dense_map_resource_type),
      robot_to_mission_id_map, *map);
}

bool MaplabServerNode::deleteMission(
    const std::string& partial_mission_id_string, std::string* status_message) {
  CHECK_NOTNULL(status_message);

  std::stringstream ss;

  const uint32_t kMinMissionIdHashLength = 4u;
  const uint32_t partial_mission_id_string_size =
      partial_mission_id_string.size();

  if (partial_mission_id_string_size < kMinMissionIdHashLength) {
    ss << "Mission id hash is too short (length "
       << std::to_string(partial_mission_id_string_size)
       << " is smaller than 4)";
    *status_message = ss.str();
    LOG(ERROR) << "[MaplabServerNode] " << *status_message;
    return false;
  }

  // Retrieve full mission id.
  uint32_t num_matching_missions = 0u;
  vi_map::MissionId mission_to_delete;
  std::string robot_name_of_mission;
  {
    std::lock_guard<std::mutex> lock(robot_to_mission_id_map_mutex_);
    for (const auto& kv : mission_id_to_robot_map_) {
      const std::string mission_id_string = kv.first.hexString();
      if (mission_id_string.substr(0, partial_mission_id_string_size) ==
          partial_mission_id_string) {
        mission_to_delete = kv.first;
        robot_name_of_mission = kv.second;
        ++num_matching_missions;
      }
    }
  }

  if (num_matching_missions == 0u) {
    ss << "No mission matches the provided (partial) mission id hashid hash "
       << "('" << partial_mission_id_string << "')";
    *status_message = ss.str();
    LOG(ERROR) << "[MaplabServerNode] " << *status_message;
    return false;
  }

  if (num_matching_missions > 1u) {
    ss << "Multiple missions are matching the provided (partial) mission id "
       << "hashid hash ('" << partial_mission_id_string_size
       << "'). Try providing the full hash.";
    *status_message = ss.str();
    LOG(ERROR) << "[MaplabServerNode] " << *status_message;
    return false;
  }
  CHECK_EQ(num_matching_missions, 1u);
  CHECK(mission_to_delete.isValid());

  // Blacklist the mission, this will delete it at the end of the merging
  // iteration and prevent subsequent submap updates of this mission from
  // being merged.
  {
    std::lock_guard<std::mutex> lock(blacklisted_missions_mutex_);
    blacklisted_missions_[mission_to_delete] = robot_name_of_mission;

    ss << "Will delete and blacklist mission " << mission_to_delete.hexString();
    *status_message = ss.str();
    LOG(INFO) << "[MaplabServerNode] " << *status_message;
    return true;
  }
}

bool MaplabServerNode::deleteAllRobotMissions(
    const std::string& robot_name, std::string* status_message) {
  CHECK_NOTNULL(status_message);

  std::stringstream ss;

  if (robot_name.empty()) {
    ss << "Robot name is empty, cannot find associated missions to delete "
       << "them!";
    *status_message = ss.str();
    LOG(ERROR) << "[MaplabServerNode] " << *status_message;
    return false;
  }

  // Retrieve all missions associated with this robot.
  uint32_t num_matching_missions = 0u;
  std::unordered_set<vi_map::MissionId> missions_to_delete;
  {
    std::lock_guard<std::mutex> lock(robot_to_mission_id_map_mutex_);
    for (const auto& kv : mission_id_to_robot_map_) {
      if (kv.second == robot_name) {
        CHECK(kv.first.isValid());
        missions_to_delete.insert(kv.first);
        ++num_matching_missions;
      }
    }
  }

  if (num_matching_missions == 0u) {
    ss << "No mission matches the provided robot name "
       << "('" << robot_name << "')";
    *status_message = ss.str();
    LOG(ERROR) << "[MaplabServerNode] " << *status_message;
    return false;
  }

  // Blacklist the mission, this will delete it at the end of the merging
  // iteration and prevent subsequent submap updates of this mission from
  // being merged.
  {
    std::lock_guard<std::mutex> lock(blacklisted_missions_mutex_);
    ss << "Will delete and blacklist all missions of robot " << robot_name
       << ": ";
    for (const vi_map::MissionId& mission_to_delete : missions_to_delete) {
      *status_message += mission_to_delete.hexString() + " ";
      blacklisted_missions_[mission_to_delete] = robot_name;
    }
    *status_message = ss.str();
    LOG(INFO) << "[MaplabServerNode] " << *status_message;
    return true;
  }
}

bool MaplabServerNode::deleteBlacklistedMissions() {
  if (!map_manager_.hasMap(kMergedMapKey)) {
    return false;
  }
  uint32_t num_missions_in_merged_map_after_deletion;
  {
    vi_map::VIMapManager::MapWriteAccess merged_map =
        map_manager_.getMapWriteAccess(kMergedMapKey);

    // Make copy of blacklisted missions to avoid repeatedly locking a
    // potentially changing blacklist.
    std::unordered_map<vi_map::MissionId, std::string>
        blacklisted_missions_copy;
    {
      std::lock_guard<std::mutex> lock(blacklisted_missions_mutex_);
      blacklisted_missions_copy = blacklisted_missions_;
    }

    if (blacklisted_missions_copy.empty()) {
      // Nothing todo here.
      return true;
    }

    // Actually delete the mission from the merge map, if present.

    vi_map::MissionIdList mission_ids;
    merged_map->getAllMissionIds(&mission_ids);

    for (const vi_map::MissionId& mission_id : mission_ids) {
      CHECK(mission_id.isValid());
      const bool mission_is_blacklisted =
          blacklisted_missions_copy.count(mission_id) > 0u;
      if (!mission_is_blacklisted) {
        continue;
      }

      LOG(INFO) << "[MaplabServerNode] Deleting blacklisted mission "
                << mission_id << " from the merged map.";
      merged_map->removeMission(mission_id, true /*remove baseframe*/);
    }

    // Cleanup bookeeping (robot to mission, mission to robot).
    {
      std::lock_guard<std::mutex> lock(robot_to_mission_id_map_mutex_);

      for (const auto& blacklisted_mission_id_and_robot_name :
           blacklisted_missions_copy) {
        const vi_map::MissionId& blacklisted_mission_id =
            blacklisted_mission_id_and_robot_name.first;
        const std::string& robot_name =
            blacklisted_mission_id_and_robot_name.second;

        // Check mission to robot map.
        if (mission_id_to_robot_map_.count(blacklisted_mission_id) > 0u) {
          // Copy intended, to avoid invaliding the refrence when deleting the
          // element.
          const std::string robot_name =
              mission_id_to_robot_map_[blacklisted_mission_id];
          mission_id_to_robot_map_.erase(blacklisted_mission_id);
        }

        // Check robot to robot mission info.
        RobotMissionInformation& robot_mission_info =
            robot_to_mission_id_map_[robot_name];
        auto it = std::find(
            robot_mission_info.mission_ids.begin(),
            robot_mission_info.mission_ids.end(), blacklisted_mission_id);
        const bool has_mission = it != robot_mission_info.mission_ids.end();
        if (has_mission) {
          robot_mission_info.mission_ids.erase(it);
        }

        // If this was the only/last mission of that robot, remove the entry and
        // also publish an empty point cloud to the dense map topic.
        if (robot_mission_info.mission_ids.empty()) {
          robot_to_mission_id_map_.erase(robot_name);

          std::unordered_map<std::string, vi_map::MissionIdList>
              empty_robot_mission_id_list;
          empty_robot_mission_id_list[robot_name] = vi_map::MissionIdList();
          visualization::visualizeReprojectedDepthResourcePerRobot(
              static_cast<backend::ResourceType>(
                  FLAGS_maplab_server_dense_map_resource_type),
              empty_robot_mission_id_list, *merged_map);
        }
      }

    }  // Limits the scope of the lock on the robot to mission id bookkeeping

    num_missions_in_merged_map_after_deletion = merged_map->numMissions();

  }  // Limits the scope of the lock on the merged map, such that it can
     // be deleted down below.

  // If we deleted all of the missions, we need to reset the state of the merged
  // map.
  if (num_missions_in_merged_map_after_deletion == 0u) {
    LOG(INFO) << "[MaplabServerNode] Merged map is empty after deleting "
              << "mission, delete merged map as well.";
    map_manager_.deleteMap(kMergedMapKey);

    // Return false to reset the 'received_first_submap' variable.
    return false;
  }
  return true;
}

}  // namespace maplab
