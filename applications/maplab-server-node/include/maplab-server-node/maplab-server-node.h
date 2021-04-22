#ifndef MAPLAB_SERVER_NODE_MAPLAB_SERVER_NODE_H_
#define MAPLAB_SERVER_NODE_MAPLAB_SERVER_NODE_H_

#include <atomic>
#include <deque>
#include <list>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <aslam/common/thread-pool.h>
#include <map-manager/map-manager.h>
#include <resources-common/point-cloud.h>
#include <vi-map/vi-map.h>
#include <visualization/resource-visualization.h>
#include <visualization/viwls-graph-plotter.h>

#include "maplab-server-node/robot_missions_information.pb.h"

namespace maplab {

struct SubmapProcess {
  // Name of the agent
  std::string robot_name;

  // Path to the map on the file system.
  std::string path;

  // Is true if the map has been loaded into the map manager already.
  bool is_loaded = false;

  // Map key of the map in the map manager.
  std::string map_key;

  // A unique hash to allow for quick lookup when multiple processes need to be
  // kept track of.
  size_t map_hash;

  // Is true if the map has been processed, i.e. all the submap processing steps
  // have been applied to the map.
  bool is_processed = false;

  // Is true if submap has been merged into global map.
  bool is_merged = false;

  mutable std::mutex mutex;
};

class MaplabServerNode final {
 public:
  MaplabServerNode();

  ~MaplabServerNode();

  // Once the node is started, the configuration cannot be changed anymore.
  void start();
  void shutdown();

  bool loadAndProcessSubmap(
      const std::string& robot_name, const std::string& submap_path);

  // Save the map to disk.
  bool saveMap(const std::string& path);
  bool saveMap();

  enum class MapLookupStatus : int {
    kSuccess = 0,
    kNoSuchMission = 1,
    kNoSuchSensor = 2,
    kPoseNotAvailableYet = 3,
    kPoseNeverAvailable = 4
  };
  MapLookupStatus mapLookup(
      const std::string& robot_name, const vi_map::SensorType sensor_type,
      const int64_t timestamp_ns, const Eigen::Vector3d& p_S,
      Eigen::Vector3d* p_G, Eigen::Vector3d* sensor_p_G) const;

  // Initially blacklists the mission, the merging thread will then remove it
  // within one iteration. All new submaps of this mission that arrive will be
  // discarded. The mission can be identified with a partial hash of length 4 or
  // more.
  bool deleteMission(
      const std::string& partial_mission_id_string,
      std::string* status_message);

  // Initially blacklists the missions of this robot, the merging thread will
  // then remove them within one iteration. All new submaps of these missions
  // that arrive will be discarded.
  bool deleteAllRobotMissions(
      const std::string& robot_name, std::string* status_message);

  // Returns an accumulation of the dense map data in global frame within a
  // radius around a center.
  bool getDenseMapInRange(
      const backend::ResourceType resource_type,
      const Eigen::Vector3d& center_G, const double radius_m,
      resources::PointCloud* point_cloud_G);

  void visualizeMap();

  void registerPoseCorrectionPublisherCallback(
      std::function<void(
          const int64_t, const std::string&, const aslam::Transformation&,
          const aslam::Transformation&, const aslam::Transformation&,
          const aslam::Transformation&)>
          callback);

  void registerStatusCallback(std::function<void(const std::string&)> callback);

 protected:
  // Status thread functions:
  void printAndPublishServerStatus();

  // Submap processing functions:
  void updateRobotInfoBasedOnSubmap(const SubmapProcess& submap_process);
  void runSubmapProcessing(const SubmapProcess& submap_process);

  // Map merging function:

  // Deletes missions from the merged map that have been blacklisted. Returns
  // false if no missions are left in the merged map, which also deletes it from
  // the map manager.
  bool deleteBlacklistedMissions();

  bool appendAvailableSubmaps();

  void saveMapEveryInterval();

  void runOneIterationOfMapMergingAlgorithms();

  void publishDenseMap();

  void publishMostRecentVertexPoseAndCorrection();

  bool isSubmapBlacklisted(const std::string& map_key);

  bool saveRobotMissionsInfo(const backend::SaveConfig& config);

  struct RobotMissionInformation {
    explicit RobotMissionInformation(
        const maplab_server_node::proto::RobotMissionInfo&
            robot_mission_information_proto);
    RobotMissionInformation() {}
    void serialize(maplab_server_node::proto::RobotMissionInfo*
                       robot_mission_information_proto) const;
    bool addSubmapKey(
        const vi_map::MissionId& mission_id, const std::string& submap_key);

    std::string robot_name;
    // Contains the mission ids and whether the baseframe is known of this
    // robot, the most recent mission is at the front of the vector.
    std::list<std::pair<vi_map::MissionId, bool>>
        mission_ids_with_baseframe_status;

    // These keep track of the end/start poses of submaps as they came in
    // and the most recent submap end pose in the optimized map. This is used
    // to compute the correction T_B_old_B_new that is published by the
    // server. This correction can then be used to coorect any poses that were
    // expressed in the odometry frame that was used to build the map
    // initially.
    std::map<int64_t, aslam::Transformation> T_M_B_submaps_input;
    std::map<int64_t, aslam::Transformation> T_G_M_submaps_input;

    std::unordered_map<vi_map::MissionId, std::vector<std::string>>
        mission_ids_to_submap_keys;
  };

 private:
  // Threads
  //////////
  // Threadpool to individuall and concurrently process incomming submaps.
  aslam::ThreadPool submap_loading_thread_pool_;
  // Single merging thread that attaches processed submaps to the global map and
  // optimizes it.
  std::thread submap_merging_thread_;
  // Fast status loop that reads current the thread status from merging and
  // submap thread and summarizes it.
  std::thread status_thread_;

  // Map management
  /////////////////
  const std::string kMergedMapKey = "merged_map";
  const std::string kRobotMissionsInfoFileName = "robot_missions_info";
  // Stores all submaps and the merged map.
  vi_map::VIMapManager map_manager_;
  // Map visualization
  std::unique_ptr<visualization::ViwlsGraphRvizPlotter> plotter_;

  // Concurrently accessed variables
  //////////////////////////////////
  // Accessed by all threads to allow them to aboart early if a shutdown is
  // requested.
  std::atomic<bool> shut_down_requested_;

  // Submap processing thread status variables.
  // Accessed by submap and status threads.
  std::mutex running_submap_process_mutex_;
  std::map<size_t, std::string> running_submap_process_;

  // Merging thread status variables
  // Accessed by merging and status thread.
  std::atomic<bool> merging_thread_busy_;
  std::mutex running_merging_process_mutex_;
  std::string running_merging_process_;
  std::atomic<double> duration_last_merging_loop_s_;
  std::atomic<double> optimization_trust_region_radius_;
  // Keep strack of the total number of merged submaps into the global map.
  std::atomic<uint32_t> total_num_merged_submaps_;

  // Server status and map management variables
  // Accessed by all threads to map between robot names and missions.
  mutable std::mutex robot_to_mission_id_map_mutex_;
  std::unordered_map<std::string, RobotMissionInformation>
      robot_to_mission_id_map_;
  std::unordered_map<vi_map::MissionId, std::string> mission_id_to_robot_map_;
  // Accessed by main thread and submapping threads, used to blacklist missions
  // after deleting them, such that submaps arriving late from the same mapping
  // session of that robot are ignored.
  mutable std::mutex blacklisted_missions_mutex_;
  std::unordered_map<vi_map::MissionId, std::string> blacklisted_missions_;
  // Accessed by the main thread, the submap processing threads and the merging
  // thread. Queue at the interface between submap processing and merging
  // threads. The submap processes are launched based on submaps loaded into
  // this queue. After the completion of the submap processing, the merging
  // threads extracts the finished submaps and adds them to the global map.
  std::mutex submap_processing_queue_mutex_;
  std::deque<SubmapProcess> submap_processing_queue_;

  // Callbacks
  ////////////
  // Callback that is called at the end of every merging thread loop and provide
  // an updated set of transformations related to the initial value and the most
  // recent optmized value of the newest pose and baseframe of every robot.
  std::function<void(
      const int64_t, const std::string&, const aslam::Transformation&,
      const aslam::Transformation&, const aslam::Transformation&,
      const aslam::Transformation&)>
      pose_correction_publisher_callback_;
  // Callback that is called in every iteration of the status thread to forward
  // the server status summary.
  std::function<void(const std::string&)> status_publisher_callback_;

  // Server settings and status.
  //////////////////////////////
  bool is_running_ = false;
  const int kSecondsToSleepBetweenAttempts = 1;
  const int kSecondsToSleepBetweenStatus = 1;

  // Exclusively accessed by the merging thread, to keep track of how often it
  // should save the map.
  double time_of_last_map_backup_s_;
  // Exclusively accessed by the merging thread, keeps track of number of
  // submaps at the the last time the trust region has been
  // reset.
  uint32_t num_submaps_at_last_trust_region_reset = 0u;

  // Protects the whole server from concurrent access from the outside.
  mutable std::mutex mutex_;

  // Number of full map merging processings
  uint32_t num_full_map_merging_processings = 0u;

  std::string initial_map_path_;
};

}  // namespace maplab

#endif  // MAPLAB_SERVER_NODE_MAPLAB_SERVER_NODE_H_
