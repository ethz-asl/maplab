#include "maplab-node/map-builder-flow.h"

#include <functional>
#include <landmark-triangulation/landmark-triangulation.h>
#include <localization-summary-map/localization-summary-map-creation.h>
#include <localization-summary-map/localization-summary-map.h>
#include <map-sparsification-plugin/map-sparsification-plugin.h>
#include <maplab-common/file-logger.h>
#include <maplab-common/map-manager-config.h>
#include <mapping-workflows-plugin/localization-map-creation.h>
#include <online-map-builders/stream-map-builder.h>
#include <sensors/sensor-types.h>
#include <vi-map-helpers/vi-map-landmark-quality-evaluation.h>
#include <vi-map-helpers/vi-map-manipulation.h>
#include <vi-map/check-map-consistency.h>
#include <vi-map/vi-map-serialization.h>
#include <vi-map/vi-map.h>
#include <visualization/viwls-graph-plotter.h>

#include "maplab-node/odometry-estimate.h"

DEFINE_double(
    localization_map_keep_landmark_fraction, 0.0,
    "Fraction of landmarks to keep when creating a localization summary map.");
DECLARE_bool(visualize_map);

DEFINE_bool(
    map_split_map_into_submaps_when_saving_periodically, true,
    "If the map is saved periodically, it will automatically discard the part "
    "of the map that has been saved. This way every saved map will represent "
    "only a submap and can later be assembled again.");

DEFINE_bool(
    map_initialize_and_triangulate_landmarks_when_saving, true,
    "If enabled, the landmarks will be initialize and triangulated before "
    "saving. If localization map creation is enabled, this will always "
    "happen, regardless of the value of this flag. ");

DEFINE_int32(
    map_save_every_n_sec, 0,
    "If set to a value larger than 0s, it will save the map periodically to "
    "the file system.");

DEFINE_bool(
    map_run_keyframing_when_saving, false,
    "If enabled, the map will be key-framed before saving.");

DEFINE_bool(
    map_remove_bad_landmarks, false,
    "If enabled, the bad landmarks will be removed before saving. IMPORTANT: "
    "if keyframing is enabled it is highly recommended to set "
    "--vi_map_landmark_quality_min_observers to 2.");

DEFINE_int32(
    map_add_odometry_edges_if_less_than_n_common_landmarks, -1,
    "Add odometry 6DoF edges/constraints based on the incoming odometry "
    "poses, but only if there are less common landmarks than this threshold. "
    "If set to 0, edges are added between all vertices, if set to -1, no "
    "odometry edges are added.");

namespace maplab {
MapBuilderFlow::MapBuilderFlow(
    const vi_map::SensorManager& sensor_manager,
    const std::string& save_map_folder,
    const vio_common::PoseLookupBuffer& T_M_B_buffer)
    : sensor_manager_(sensor_manager),
      map_with_mutex_(aligned_shared<VIMapWithMutex>()),
      last_vertex_of_previous_map_saving_(),
      mapping_terminated_(false),
      map_update_builder_(T_M_B_buffer),
      external_resource_folder_(),
      stream_map_builder_(sensor_manager_, &map_with_mutex_->vi_map) {
  if (!save_map_folder.empty()) {
    VLOG(1) << "Set VIMap folder to: " << save_map_folder;
    map_with_mutex_->vi_map.setMapFolder(save_map_folder);

    // If we save periodically, we should keep the resources in an external
    // folder, such that once the map is saved no new resources will be stored
    // into that folder. When saving a map, the resources that belong to the map
    // are either copied or move to the new map folder, such that it is
    // self-contained again.
    if (FLAGS_map_save_every_n_sec > 0) {
      external_resource_folder_ = save_map_folder + "_resource_folder";
      VLOG(1) << "Saving resources in external folder: "
              << external_resource_folder_;
      map_with_mutex_->vi_map.useExternalResourceFolder(
          external_resource_folder_);
    }
  }
  CHECK(!last_vertex_of_previous_map_saving_.isValid());
}

void MapBuilderFlow::attachToMessageFlow(message_flow::MessageFlow* flow) {
  CHECK_NOTNULL(flow);
  const message_flow::DeliveryOptions delivery_options;
  static constexpr char kSubscriberNodeName[] = "MapBuilder";
  std::function<void(const VIMapWithMutex::ConstPtr&)> map_publish_function =
      flow->registerPublisher<message_flow_topics::RAW_VIMAP>();
  CHECK(map_publish_function);
  flow->registerSubscriber<message_flow_topics::MAP_UPDATES>(
      kSubscriberNodeName, delivery_options,
      [this, map_publish_function](const vio::MapUpdate::ConstPtr& vio_update) {
        CHECK(vio_update != nullptr);
        {
          std::lock_guard<std::mutex> lock(map_with_mutex_->mutex);
          if (mapping_terminated_) {
            return;
          }

          // WARNING: The tracker is updating the track information in the
          // current and previous frame; therefore the most recent
          // VisualNFrame added to the map might still be modified.
          constexpr bool kDeepCopyNFrame = false;
          stream_map_builder_.apply(*vio_update, kDeepCopyNFrame);

          VLOG(3) << "[MaplabNode-MapBuilder] Applied VIO update to map.";
        }
        map_publish_function(map_with_mutex_);
      });

  flow->registerSubscriber<message_flow_topics::SYNCED_LIDAR_MEASUREMENTS>(
      kSubscriberNodeName, delivery_options,
      [this](const vi_map::RosLidarMeasurement::ConstPtr& lidar_measurement) {
        CHECK(lidar_measurement);
        {
          if (!FLAGS_map_builder_save_point_clouds_as_resources) {
            return;
          }

          std::lock_guard<std::mutex> lock(map_with_mutex_->mutex);
          if (mapping_terminated_) {
            return;
          }

          stream_map_builder_.attachLidarMeasurement(*lidar_measurement);

          VLOG(3) << "[MaplabNode-MapBuilder] Attached raw lidar measurement "
                  << "to map.";
        }
      });

  flow->registerSubscriber<message_flow_topics::SYNCED_ABSOLUTE_6DOF>(
      kSubscriberNodeName, delivery_options,
      [this](const vi_map::Absolute6DoFMeasurement::Ptr&
                 absolute_6dof_measurement) {
        CHECK(absolute_6dof_measurement);
        {
          std::lock_guard<std::mutex> lock(map_with_mutex_->mutex);
          if (mapping_terminated_) {
            return;
          }

          stream_map_builder_.bufferAbsolute6DoFConstraint(
              absolute_6dof_measurement);

          VLOG(3)
              << "[MaplabNode-MapBuilder] Attached absolute 6DoF constraint "
              << "to map.";
        }
      });

  flow->registerSubscriber<message_flow_topics::SYNCED_LOOP_CLOSURE>(
      kSubscriberNodeName, delivery_options,
      [this](const vi_map::LoopClosureMeasurement::ConstPtr&
                 loop_closure_measurement) {
        CHECK(loop_closure_measurement);
        {
          std::lock_guard<std::mutex> lock(map_with_mutex_->mutex);
          if (mapping_terminated_) {
            return;
          }

          stream_map_builder_.bufferLoopClosureConstraint(
              loop_closure_measurement);

          VLOG(3) << "[MaplabNode-MapBuilder] Attached loop closure constraint "
                  << "to map.";
        }
      });

  flow->registerSubscriber<message_flow_topics::SYNCED_WHEEL_ODOMETRY>(
      kSubscriberNodeName, delivery_options,
      [this](const vi_map::WheelOdometryMeasurement::ConstPtr&
                 wheel_odometry_measurement) {
        CHECK(wheel_odometry_measurement);
        {
          std::lock_guard<std::mutex> lock(map_with_mutex_->mutex);
          if (mapping_terminated_) {
            return;
          }

          stream_map_builder_.bufferWheelOdometryConstraint(
              wheel_odometry_measurement);

          VLOG(4)
              << "[MaplabNode-MapBuilder] Attached wheel odometry constraint "
              << "to map.";
        }
      });
  flow->registerSubscriber<message_flow_topics::SYNCED_POINTCLOUD_MAP>(
      kSubscriberNodeName, delivery_options,
      [this](const vi_map::RosPointCloudMapSensorMeasurement::ConstPtr&
                 pointcloud_map_measurement) {
        CHECK(pointcloud_map_measurement);
        {
          if (!FLAGS_map_builder_save_point_cloud_maps_as_resources) {
            LOG(WARNING) << "[MaplabNode-MapBuilder] Point cloud map is "
                         << "discarded, because it is not "
                         << "attached to the map. Set "
                         << "--map_builder_save_point_cloud_maps_as_resources="
                         << "true to enable atachment.";
            return;
          }

          std::lock_guard<std::mutex> lock(map_with_mutex_->mutex);
          if (mapping_terminated_) {
            return;
          }

          stream_map_builder_.attachPointCloudMap(*pointcloud_map_measurement);

          VLOG(3) << "[MaplabNode-MapBuilder] Attached point cloud map "
                  << "to map.";
        }
      });

  std::function<void(const vio::MapUpdate::ConstPtr&)>
      map_update_builder_publisher =
          flow->registerPublisher<message_flow_topics::MAP_UPDATES>();
  map_update_builder_.registerMapUpdatePublishFunction(
      map_update_builder_publisher);
  flow->registerSubscriber<message_flow_topics::TRACKED_NFRAMES>(
      kSubscriberNodeName, delivery_options,
      [this](const vio::SynchronizedNFrame::ConstPtr& synchronized_nframe_imu) {
        if (mapping_terminated_) {
          return;
        }
        map_update_builder_.processTrackedNFrame(synchronized_nframe_imu);
      });
  flow->registerSubscriber<message_flow_topics::FUSED_LOCALIZATION_RESULT>(
      kSubscriberNodeName, delivery_options,
      std::bind(
          &MapUpdateBuilder::processLocalizationResult, &map_update_builder_,
          std::placeholders::_1));
}

bool MapBuilderFlow::saveMapAndOptionallyOptimize(
    const std::string& path, const bool overwrite_existing_map,
    const bool process_to_localization_map, const bool stop_mapping) {
  CHECK(!path.empty());
  CHECK(map_with_mutex_);

  std::lock_guard<std::mutex> lock(map_with_mutex_->mutex);

  if (stop_mapping) {
    mapping_terminated_ = true;
  }

  // Early exit if the map is empty.
  if (map_with_mutex_->vi_map.numVertices() < 3u) {
    LOG(WARNING)
        << "[MaplabNode-MapBuilder] Map is empty; nothing will be saved.";
    return false;
  }

  visualization::ViwlsGraphRvizPlotter::UniquePtr plotter;
  if (FLAGS_visualize_map) {
    plotter = aligned_unique<visualization::ViwlsGraphRvizPlotter>();
  }

  // There should only be one mission in the map.
  vi_map::MissionIdList mission_ids;
  map_with_mutex_->vi_map.getAllMissionIds(&mission_ids);
  CHECK_EQ(mission_ids.size(), 1u);
  const vi_map::MissionId& id_of_first_mission = mission_ids.front();

  if (FLAGS_map_initialize_and_triangulate_landmarks_when_saving ||
      process_to_localization_map) {
    VLOG(1) << "[MaplabNode-MapBuilder] Initializing landmarks of created map.";
    vi_map_helpers::VIMapManipulation landmark_manipulation(
        &map_with_mutex_->vi_map);
    if (!last_vertex_of_previous_map_saving_.isValid()) {
      landmark_manipulation.initializeLandmarksFromUnusedFeatureTracksOfMission(
          id_of_first_mission);
      landmark_triangulation::retriangulateLandmarksOfMission(
          id_of_first_mission, &map_with_mutex_->vi_map);
      landmark_triangulation::retriangulateLidarLandmarksOfMission(
          id_of_first_mission, &map_with_mutex_->vi_map);
    } else {
      landmark_manipulation.initializeLandmarksFromUnusedFeatureTracksOfMission(
          id_of_first_mission, last_vertex_of_previous_map_saving_);
      landmark_triangulation::retriangulateLandmarksAlongMissionAfterVertex(
          id_of_first_mission, last_vertex_of_previous_map_saving_,
          &map_with_mutex_->vi_map);
      landmark_triangulation::retriangulateLidarLandmarksAlongMissionAfterVertex(
          id_of_first_mission, last_vertex_of_previous_map_saving_,
          &map_with_mutex_->vi_map);
    }
  }
  last_vertex_of_previous_map_saving_ =
      map_with_mutex_->vi_map.getLastVertexIdOfMission(id_of_first_mission);

  if (FLAGS_map_run_keyframing_when_saving) {
    using map_sparsification::KeyframingHeuristicsOptions;
    KeyframingHeuristicsOptions options =
        KeyframingHeuristicsOptions::initializeFromGFlags();
    VLOG(1) << "[MaplabNode-MapBuilder] Keyframing map.";
    if (map_sparsification_plugin::keyframeMapBasedOnHeuristics(
            options, id_of_first_mission, plotter.get(),
            &map_with_mutex_->vi_map) != common::kSuccess) {
      LOG(ERROR) << "[MaplabNode-MapBuilder] Keyframing of mission "
                 << id_of_first_mission << " failed.";
    }
  }

  vi_map_helpers::VIMapManipulation manipulation(&map_with_mutex_->vi_map);

  if (FLAGS_map_add_odometry_edges_if_less_than_n_common_landmarks >= 0) {
    const uint32_t num_odom_edges_added =
        manipulation.addOdometryEdgesBetweenVertices(
            FLAGS_map_add_odometry_edges_if_less_than_n_common_landmarks);
    LOG(INFO) << "[MaplabNode-MapBuilder] Added " << num_odom_edges_added
              << " odometry edges to map.";
  }

  if (FLAGS_map_remove_bad_landmarks) {
    LOG(INFO) << "[MaplabNode-MapBuilder] Removing bad landmarks.";

    const size_t num_removed = manipulation.removeBadLandmarks();
    LOG(INFO) << "[MaplabNode-MapBuilder] Removed " << num_removed
              << " bad landmark(s).";
  }

  backend::SaveConfig save_config;
  {
    save_config.overwrite_existing_files = overwrite_existing_map;

    save_config.migrate_resources_settings = backend::SaveConfig::
        MigrateResourcesSettings::kMigrateResourcesToMapFolder;

    // If we split into submaps, all resources currently in the map are
    // exclusively owned by this map, hence we can move instead of copy them. If
    // we don't split, each partial map needs a copy of every resource in the
    // map at the time of map saving.
    if (FLAGS_map_split_map_into_submaps_when_saving_periodically) {
      save_config.move_resources_when_migrating = true;
    } else {
      save_config.move_resources_when_migrating = false;
    }

    if (!vi_map::serialization::saveMapToFolder(
            path, save_config, &map_with_mutex_->vi_map)) {
      LOG(ERROR) << "[MaplabNode-MapBuilder] failed to save map to '" << path
                 << "'";
      return false;
    }
    VLOG(1) << "[MaplabNode-MapBuilder] Saved map to: " << path;

    if (FLAGS_map_save_every_n_sec) {
      // We need to set the external resource folder again, since saving the map
      // will automatically set the resource folder to the new folder.
      map_with_mutex_->vi_map.useExternalResourceFolder(
          external_resource_folder_);
    }
  }

  // Visualize at the time of saving.
  if (plotter) {
    plotter->visualizeMap(map_with_mutex_->vi_map);
  }

  if (process_to_localization_map) {
    VLOG(1) << "[MaplabNode-MapBuilder] Map is being processed into a "
            << "localization map... please wait.";

    map_sparsification::KeyframingHeuristicsOptions keyframe_options =
        map_sparsification::KeyframingHeuristicsOptions::initializeFromGFlags();
    constexpr bool kInitializeLandmarks = false;
    mapping_workflows_plugin::processVIMapToLocalizationMap(
        kInitializeLandmarks, keyframe_options, &map_with_mutex_->vi_map,
        plotter.get());

    // Create localization summary map that has pre-projected descriptors.
    std::unique_ptr<summary_map::LocalizationSummaryMap> localization_map(
        new summary_map::LocalizationSummaryMap);
    vi_map_helpers::evaluateLandmarkQuality(&map_with_mutex_->vi_map);

    if (FLAGS_localization_map_keep_landmark_fraction > 0.0) {
      summary_map::createLocalizationSummaryMapForSummarizedLandmarks(
          map_with_mutex_->vi_map,
          FLAGS_localization_map_keep_landmark_fraction,
          localization_map.get());
    } else {
      summary_map::createLocalizationSummaryMapForWellConstrainedLandmarks(
          map_with_mutex_->vi_map, localization_map.get());
    }

    std::string localization_map_path = path + "_localization";
    localization_map->saveToFolder(localization_map_path, save_config);
    VLOG(1) << "[MaplabNode-MapBuilder] Localization summary map saved to: "
            << localization_map_path;
  }

  if (FLAGS_map_split_map_into_submaps_when_saving_periodically) {
    VLOG(1) << "[MaplabNode-MapBuilder] Dropping map data up to last vertex...";
    vi_map_helpers::VIMapManipulation drop_data_manipulation(
        &map_with_mutex_->vi_map);

    drop_data_manipulation.dropMapDataBeforeVertex(
        id_of_first_mission, last_vertex_of_previous_map_saving_,
        false /* delete resources from file system */);

    // Even though dropping all the map data up until the current vertex removes
    // all resources up to the vertex, there might still be some resources that
    // are ahead of the current map. So we delete them all to be sure we catch
    // those as well.
    map_with_mutex_->vi_map.deleteAllSensorResources(
        id_of_first_mission, false);

    stream_map_builder_.updateMapDependentData();
  }
  return true;
}
}  // namespace maplab
