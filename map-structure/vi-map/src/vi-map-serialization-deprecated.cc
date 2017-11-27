#include "vi-map/deprecated/vi-map-serialization-deprecated.h"

#include <functional>
#include <mutex>
#include <string>
#include <thread>

#include <aslam-serialization/camera-serialization.h>
#include <glog/logging.h>
#include <map-resources/resource-map-serialization.h>
#include <maplab-common/eigen-proto.h>
#include <maplab-common/file-system-tools.h>
#include <maplab-common/map-manager-config.h>
#include <maplab-common/multi-threaded-progress-bar.h>
#include <maplab-common/parallel-process.h>
#include <maplab-common/proto-serialization-helper.h>

#include "vi-map/deprecated/gps-data-storage.h"
#include "vi-map/deprecated/optional-sensor-extrinsics.h"
#include "vi-map/vi-map.h"

namespace vi_map {
namespace serialization_deprecated {

void deserializeCompleteMap(
    const vi_map_deprecated::proto::VIMap& proto, vi_map::VIMap* map) {
  CHECK_NOTNULL(map);
  deserializeMissionsAndBaseFrames(proto, map);
  deserializeVertices(proto, map);
  deserializeEdges(proto, map);
  deserializeLandmarkIndex(proto, map);
  deserializeOtherFields(proto, map);
}

void deserializeVertices(
    const vi_map_deprecated::proto::VIMap& proto, vi_map::VIMap* map) {
  CHECK_NOTNULL(map);
  CHECK_EQ(proto.vertex_ids_size(), proto.vertices_size());
  for (int i = 0; i < proto.vertex_ids_size(); ++i) {
    pose_graph::VertexId id;
    id.deserialize(proto.vertex_ids(i));
    vi_map::Vertex* vertex(new vi_map::Vertex);
    vertex->deserialize(id, proto.vertices(i));

    const vi_map::MissionId& mission_id = vertex->getMissionId();
    CHECK(map->hasMission(mission_id));
    aslam::NCamera::Ptr ncamera =
        map->getSensorManager().getNCameraSharedForMission(mission_id);
    CHECK(ncamera);
    vertex->setNCameras(ncamera);

    map->addVertex(vi_map::Vertex::UniquePtr(vertex));
  }
}

void deserializeEdges(
    const vi_map_deprecated::proto::VIMap& proto, vi_map::VIMap* map) {
  CHECK_NOTNULL(map);
  CHECK_EQ(proto.edge_ids_size(), proto.edges_size());
  for (int i = 0; i < proto.edge_ids_size(); ++i) {
    pose_graph::EdgeId id;
    id.deserialize(proto.edge_ids(i));
    map->addEdge(vi_map::Edge::deserialize(id, proto.edges(i)));
  }
}

void serializeImuSigmas(
    const ImuSigmas& imu_sigmas, vi_map_deprecated::proto::ImuSigmas* proto) {
  CHECK_NOTNULL(proto);
  proto->set_gyro_noise(imu_sigmas.gyro_noise_density);
  proto->set_gyro_bias(imu_sigmas.gyro_bias_random_walk_noise_density);
  proto->set_acc_noise(imu_sigmas.acc_noise_density);
  proto->set_acc_bias(imu_sigmas.acc_bias_random_walk_noise_density);
}

void deserializeImuSigmas(
    const vi_map_deprecated::proto::ImuSigmas& proto, ImuSigmas* imu_sigmas) {
  CHECK(proto.has_gyro_noise());
  CHECK(proto.has_gyro_bias());
  CHECK(proto.has_acc_noise());
  CHECK(proto.has_acc_bias());
  imu_sigmas->gyro_noise_density = proto.gyro_noise();
  imu_sigmas->gyro_bias_random_walk_noise_density = proto.gyro_bias();
  imu_sigmas->acc_noise_density = proto.acc_noise();
  imu_sigmas->acc_bias_random_walk_noise_density = proto.acc_bias();
}

void deserializeMissionsAndBaseFrames(
    const vi_map_deprecated::proto::VIMap& proto, vi_map::VIMap* map) {
  CHECK_NOTNULL(map);
  CHECK_EQ(proto.mission_ids_size(), proto.missions_size());
  CHECK_EQ(
      proto.mission_base_frame_ids_size(), proto.mission_base_frames_size());
  CHECK_EQ(proto.missions_size(), proto.mission_base_frames_size());
  constexpr char kImuHardwareId[] = "imu0";
  for (int i = 0; i < proto.missions_size(); ++i) {
    MissionId mission_id;
    mission_id.deserialize(proto.mission_ids(i));
    VIMission* mission(new VIMission);
    mission->deserializeDeprecated(mission_id, proto.missions(i));

    MissionBaseFrameId mission_frame_id;
    mission_frame_id.deserialize(proto.mission_base_frame_ids(i));
    MissionBaseFrame mission_frame;
    mission_frame.deserialize(mission_frame_id, proto.mission_base_frames(i));

    const vi_map_deprecated::proto::Mission& proto_mission = proto.missions(i);
    CHECK(proto_mission.has_n_camera());
    aslam::NCamera::Ptr n_camera;
    aslam::serialization::deserializeNCamera(
        proto_mission.n_camera(), &n_camera);
    CHECK(n_camera);

    map->addNewMissionWithBaseframe(
        VIMission::UniquePtr(mission), n_camera,  mission_frame);

    CHECK(proto_mission.has_imu_sigmas());
    vi_map::ImuSigmas imu_sigmas;
    deserializeImuSigmas(proto_mission.imu_sigmas(), &imu_sigmas);

    SensorId imu_sensor_id;
    common::generateId(&imu_sensor_id);
    vi_map::Imu::UniquePtr imu = aligned_unique<vi_map::Imu>(
        imu_sensor_id, static_cast<std::string>(kImuHardwareId));
    imu->setImuSigmas(imu_sigmas);
    map->getSensorManager().addSensor(std::move(imu), mission_id);

    if (i == 0) {
      SensorSystem::UniquePtr sensor_system =
          aligned_unique<SensorSystem>(imu_sensor_id);
      map->getSensorManager().addSensorSystem(std::move(sensor_system));
    }
  }
}

void deserializeLandmarkIndex(
    const vi_map_deprecated::proto::VIMap& proto, vi_map::VIMap* map) {
  CHECK_NOTNULL(map);
  for (int i = 0; i < proto.landmark_index_size(); ++i) {
    LandmarkId landmark_id;
    landmark_id.deserialize(proto.landmark_index(i).landmark_id());
    pose_graph::VertexId storing_vertex_id;
    storing_vertex_id.deserialize(proto.landmark_index(i).vertex_id());

    if (map->hasLandmark(landmark_id)) {
      CHECK_EQ(storing_vertex_id, map->getLandmarkStoreVertexId(landmark_id));
    } else {
      map->addLandmarkIndexReference(landmark_id, storing_vertex_id);
    }
  }

  if (proto.landmark_index_ids_size() > 0) {
    LOG(WARNING) << "Legacy map with store/global landmark ids found. "
                 << "Cleaning up the ids.";
    CHECK_EQ(proto.landmark_index_size(), proto.landmark_index_ids_size());

    // Go over all the landmarks and change the ids in the visualframes
    // to the actual landmark id (instead of the old global landmark id).
    vi_map::LandmarkIdList landmark_ids;
    map->getAllLandmarkIds(&landmark_ids);
    for (const vi_map::LandmarkId& actual_landmark_id : landmark_ids) {
      const vi_map::Landmark& landmark = map->getLandmark(actual_landmark_id);
      landmark.forEachObservation(
          [&](const vi_map::KeypointIdentifier& keypoint_id) {
            vi_map::Vertex& vertex =
                map->getVertex(keypoint_id.frame_id.vertex_id);
            vertex.setObservedLandmarkId(keypoint_id, actual_landmark_id);
          });
    }
  }
}

void deserializeOtherFields(
    const vi_map_deprecated::proto::VIMap& proto, vi_map::VIMap* map) {
  CHECK_NOTNULL(map);

  GPSDataStorage gps_data_storage;
  gps_data_storage.deserialize(proto.gps_data_storage());

  const GPSDataStorage::UTMContainer& utm_container =
      gps_data_storage.getUTMContainer();

  // First, find the sensor id <-> mission id associations.
  std::unordered_map<vi_map::SensorId, vi_map::MissionIdSet>
      sensor_id_to_mission_id_map;

  pose_graph::EdgeIdList edge_ids;
  map->getAllEdgeIds(&edge_ids);
  for (const pose_graph::EdgeId& edge_id : edge_ids) {
    CHECK(edge_id.isValid());
    const vi_map::Edge::EdgeType edge_type = map->getEdgeType(edge_id);
    if (edge_type == vi_map::Edge::EdgeType::kOdometry) {
      const vi_map::TransformationEdge& edge =
          map->getEdgeAs<vi_map::TransformationEdge>(edge_id);
      const vi_map::SensorId& sensor_id = edge.getSensorId();
      CHECK(sensor_id.isValid());
      const vi_map::MissionId& mission_id_from =
          map->getMissionIdForVertex(edge.from());
      CHECK_EQ(mission_id_from, map->getMissionIdForVertex(edge.to()));
      std::unordered_map<vi_map::SensorId, vi_map::MissionIdSet>::iterator
          sensor_mission_iterator = sensor_id_to_mission_id_map.find(sensor_id);
      if (sensor_mission_iterator == sensor_id_to_mission_id_map.end()) {
        sensor_id_to_mission_id_map[sensor_id] = {mission_id_from};
      } else {
        sensor_mission_iterator->second.emplace(mission_id_from);
      }
    }
  }
  for (const GPSDataStorage::UTMContainer::value_type&
      mission_id_utm_data_pair : utm_container) {
    const vi_map::MissionId& mission_id = mission_id_utm_data_pair.first;
    CHECK(map->hasMission(mission_id));

    const vi_map::GPSDataStorage::UTMBuffer& utm_measurements =
        mission_id_utm_data_pair.second;
    for (const vi_map::GPSDataStorage::UTMBuffer::BufferType::value_type&
        time_with_utm_measurement_deprecated :
        utm_measurements.buffered_values()) {
      const vi_map::GPSMeasurementUTM& utm_measurement_deprecated =
          time_with_utm_measurement_deprecated.second;
      CHECK_EQ(time_with_utm_measurement_deprecated.first,
               utm_measurement_deprecated.timestamp);
      const SensorId& sensor_id = utm_measurement_deprecated.sensor_id;
      CHECK(sensor_id.isValid());

      std::unordered_map<vi_map::SensorId, vi_map::MissionIdSet>::iterator
          sensor_mission_iterator = sensor_id_to_mission_id_map.find(sensor_id);
      if (sensor_mission_iterator == sensor_id_to_mission_id_map.end()) {
        sensor_id_to_mission_id_map[sensor_id] = {mission_id};
      } else {
        sensor_mission_iterator->second.emplace(mission_id);
      }
    }
  }

  const GPSDataStorage::WGSContainer& wgs_container =
      gps_data_storage.getWGSContainer();

  for (const GPSDataStorage::WGSContainer::value_type&
      mission_id_wgs_data_pair : wgs_container) {
    const vi_map::MissionId& mission_id = mission_id_wgs_data_pair.first;
    CHECK(map->hasMission(mission_id));

    const vi_map::GPSDataStorage::WGSBuffer& wgs_measurements =
        mission_id_wgs_data_pair.second;
    for (const vi_map::GPSDataStorage::WGSBuffer::BufferType::value_type&
        time_with_wgs_measurement_deprecated :
        wgs_measurements.buffered_values()) {
      const vi_map::GPSMeasurementWGS& wgs_measurement_deprecated =
          time_with_wgs_measurement_deprecated.second;
      CHECK_EQ(time_with_wgs_measurement_deprecated.first,
               wgs_measurement_deprecated.timestamp);
      const SensorId& sensor_id = wgs_measurement_deprecated.sensor_id;
      CHECK(sensor_id.isValid());

      std::unordered_map<vi_map::SensorId, vi_map::MissionIdSet>::iterator
          sensor_mission_iterator = sensor_id_to_mission_id_map.find(sensor_id);
      if (sensor_mission_iterator == sensor_id_to_mission_id_map.end()) {
        sensor_id_to_mission_id_map[sensor_id] = {mission_id};
      } else {
        sensor_mission_iterator->second.emplace(mission_id);
      }
    }
  }

  // Now, add the sensors.
  for (const vi_map_deprecated::proto::OptionalSensorExtrinsics&
           proto_optional_sensor_extrinsics :
       proto.optional_sensors_extrinsics()) {
    vi_map::OptionalSensorType sensor_type =
        static_cast<vi_map::OptionalSensorType>(
            proto_optional_sensor_extrinsics.sensor_type());
    aslam::Transformation T_S_I;
    common::eigen_proto::deserialize(
        proto_optional_sensor_extrinsics.t_s_i(), &T_S_I);
    aslam::TransformationCovariance T_S_I_covariance;
    common::eigen_proto::deserialize(
        proto_optional_sensor_extrinsics.t_s_i_covariance(), &T_S_I_covariance);

    SensorId sensor_id;
    sensor_id.deserialize(proto_optional_sensor_extrinsics.id());
    CHECK(sensor_id.isValid());

    // Find the associated mission.
    std::unordered_map<vi_map::SensorId, vi_map::MissionIdSet>::iterator
        sensor_mission_iterator = sensor_id_to_mission_id_map.find(sensor_id);
    if (sensor_mission_iterator == sensor_id_to_mission_id_map.end()) {
      LOG(WARNING)
          << "Sensor with id " << sensor_id.hexString() << " of type "
          << convertOptionalSensorTypeToString(sensor_type)
          << " does not have any data associated with any mission. This "
          << "sensor is not added to the map and discarded.";
      continue;
    }

    const vi_map::MissionIdSet& mission_ids = sensor_mission_iterator->second;
    CHECK(!mission_ids.empty());

    vi_map::MissionIdSet::const_iterator mission_iterator = mission_ids.begin();
    CHECK(mission_iterator != mission_ids.end());
    vi_map::SensorManager& sensor_manager = map->getSensorManager();
    switch (sensor_type) {
      case vi_map::OptionalSensorType::kGPSUTM: {
        constexpr char kDefaultHardwareId[] = "gps_utm";
        GpsUtm::UniquePtr sensor = aligned_unique<GpsUtm>(
            sensor_id, static_cast<std::string>(kDefaultHardwareId));
        sensor_manager.addSensor(std::move(sensor), *mission_iterator);
        break;
      }
      case vi_map::OptionalSensorType::kGPSWGS: {
        constexpr char kDefaultHardwareId[] = "gps_wgs";
        GpsWgs::UniquePtr sensor = aligned_unique<GpsWgs>(
            sensor_id, static_cast<std::string>(kDefaultHardwareId));
        sensor_manager.addSensor(std::move(sensor), *mission_iterator);
        break;
      }
      case vi_map::OptionalSensorType::kWheelOdometry: {
        constexpr char kDefaultHardwareId[] = "wheel_odometry";
        Relative6DoFPose::UniquePtr sensor = aligned_unique<Relative6DoFPose>(
            sensor_id, static_cast<std::string>(kDefaultHardwareId),
            aslam::TransformationCovariance::Identity());
        sensor_manager.addSensor(std::move(sensor), *mission_iterator);
        break;
      }
      default:
        LOG(FATAL) << "Unknown deprecated optional sensor extrinsics type: "
                   << static_cast<int>(sensor_type);
        break;
    }
    ++mission_iterator;
    while (mission_iterator != mission_ids.end()) {
      CHECK(map->hasMission(*mission_iterator));
      sensor_manager.associateExistingSensorWithMission(
          sensor_id, *mission_iterator);
      ++mission_iterator;
    }
    sensor_manager.setSensor_T_R_S(sensor_id, T_S_I.inverse());
  }

  // Add the measurements.
  for (const GPSDataStorage::UTMContainer::value_type&
      mission_id_utm_data_pair : utm_container) {
    const vi_map::MissionId& mission_id = mission_id_utm_data_pair.first;
    CHECK(map->hasMission(mission_id));

    const vi_map::GPSDataStorage::UTMBuffer& utm_measurements =
        mission_id_utm_data_pair.second;
    for (const vi_map::GPSDataStorage::UTMBuffer::BufferType::value_type&
        time_with_utm_measurement_deprecated :
        utm_measurements.buffered_values()) {
      const vi_map::GPSMeasurementUTM& utm_measurement_deprecated =
          time_with_utm_measurement_deprecated.second;
      CHECK_EQ(time_with_utm_measurement_deprecated.first,
               utm_measurement_deprecated.timestamp);
      const vi_map::GpsUtmMeasurement utm_measurement(
          utm_measurement_deprecated.sensor_id,
          utm_measurement_deprecated.timestamp,
          utm_measurement_deprecated.T_R_S, vi_map::UtmZone::createInvalid());

      map->addOptionalSensorMeasurement(utm_measurement, mission_id);
    }
  }

  for (const GPSDataStorage::WGSContainer::value_type&
      mission_id_wgs_data_pair : wgs_container) {
    const vi_map::MissionId& mission_id = mission_id_wgs_data_pair.first;
    CHECK(map->hasMission(mission_id));

    const vi_map::GPSDataStorage::WGSBuffer& wgs_measurements =
        mission_id_wgs_data_pair.second;
    for (const vi_map::GPSDataStorage::WGSBuffer::BufferType::value_type&
        time_with_wgs_measurement_deprecated :
        wgs_measurements.buffered_values()) {
      const vi_map::GPSMeasurementWGS& wgs_measurement_deprecated =
          time_with_wgs_measurement_deprecated.second;
      CHECK_EQ(time_with_wgs_measurement_deprecated.first,
               wgs_measurement_deprecated.timestamp);
      const vi_map::GpsWgsMeasurement wgs_measurement(
          wgs_measurement_deprecated.sensor_id,
          wgs_measurement_deprecated.timestamp,
          wgs_measurement_deprecated.latitude_deg,
          wgs_measurement_deprecated.longitude_deg,
          wgs_measurement_deprecated.altitude_meters);

      map->addOptionalSensorMeasurement(wgs_measurement, mission_id);
    }
  }
}

namespace internal {

size_t numberOfProtos(
    const VIMap& map, const backend::SaveConfig& save_config) {
  const size_t num_vertices = map.numVertices();
  const size_t num_vertices_protos =
      (save_config.vertices_per_proto_file - 1u + num_vertices) /
      save_config.vertices_per_proto_file;
  return internal::kMinNumProtos + num_vertices_protos;
}

}  // namespace internal

void deserializeFromListOfProtos(
    const std::vector<vi_map_deprecated::proto::VIMap>& list_of_protos,
    vi_map::VIMap* map) {
  CHECK_NOTNULL(map);
  CHECK_GE(list_of_protos.size(), internal::kMinNumProtos);

  deserializeMissionsAndBaseFrames(
      list_of_protos[internal::kProtoListMissionsIndex], map);

  // Deserialize vertices first. This is because other fields might depend on
  // edges an can't be inserted into the map without the corresponding vertices
  // being present.
  for (size_t vertices_index = internal::kProtoListVerticesStartIndex;
       vertices_index < list_of_protos.size(); ++vertices_index) {
    deserializeVertices(list_of_protos[vertices_index], map);
  }

  deserializeEdges(list_of_protos[internal::kProtoListEdgesIndex], map);
  deserializeLandmarkIndex(
      list_of_protos[internal::kProtoListLandmarkIndexIndex], map);
  deserializeOtherFields(
      list_of_protos[internal::kProtoListOtherFieldsIndex], map);
}

bool getListOfExistingMapFiles(
    const std::string& map_folder,
    std::vector<std::string>* list_of_map_files) {
  CHECK_NOTNULL(list_of_map_files);
  if (!common::pathExists(map_folder)) {
    return false;
  }

  bool minimum_required_files_exist =
      backend::resource_map_serialization::getListOfExistingMapFiles(
          map_folder, list_of_map_files);

  std::string path_to_vi_map_file;
  std::string vi_map_file;
  std::string path_to_vi_map_folder;
  common::concatenateFolderAndFileName(
      map_folder, internal::kFolderName, &path_to_vi_map_folder);

  for (const std::string& map_file : internal::kMinimumVIMapFiles) {
    common::concatenateFolderAndFileName(
        path_to_vi_map_folder, map_file, &path_to_vi_map_file);
    if (common::fileExists(path_to_vi_map_file)) {
      common::concatenateFolderAndFileName(
          internal::kFolderName, map_file, &vi_map_file);
      list_of_map_files->emplace_back(vi_map_file);
    } else {
      minimum_required_files_exist = false;
    }
  }

  size_t number_of_vertex_files = 0u;
  std::string file_name =
      internal::kFileNameVertices + std::to_string(number_of_vertex_files);

  common::concatenateFolderAndFileName(
      path_to_vi_map_folder, file_name, &path_to_vi_map_file);
  while (common::fileExists(path_to_vi_map_file)) {
    common::concatenateFolderAndFileName(
        internal::kFolderName, file_name, &vi_map_file);
    list_of_map_files->emplace_back(vi_map_file);
    ++number_of_vertex_files;
    file_name =
        internal::kFileNameVertices + std::to_string(number_of_vertex_files);
    common::concatenateFolderAndFileName(
        path_to_vi_map_folder, file_name, &path_to_vi_map_file);
  }
  return minimum_required_files_exist;
}

/*
bool hasMapOnFileSystem(const std::string& folder_path) {
  std::vector<std::string> list_of_map_files;
  return getListOfExistingMapFiles(folder_path, &list_of_map_files);
}*/

std::string getFileNameFromIndex(const size_t index) {
  std::string file_name;
  switch (index) {
    case internal::kProtoListMissionsIndex:
      file_name = internal::kFileNameMissions;
      break;
    case internal::kProtoListEdgesIndex:
      file_name = internal::kFileNameEdges;
      break;
    case internal::kProtoListLandmarkIndexIndex:
      file_name = internal::kFileNameLandmarkIndex;
      break;
    case internal::kProtoListOtherFieldsIndex:
      file_name = internal::kFileNameOtherFields;
      break;
    default: {
      // Vertex files.
      CHECK_GE(index, internal::kProtoListVerticesStartIndex);
      file_name =
          internal::kFileNameVertices +
          std::to_string(index - internal::kProtoListVerticesStartIndex);
      break;
    }
  }
  return file_name;
}

bool loadMapFromFolder(const std::string& folder_path, vi_map::VIMap* map) {
  CHECK_NOTNULL(map);
  std::vector<std::string> list_of_map_files;
  if (!common::pathExists(folder_path) ||
      !getListOfExistingMapFiles(folder_path, &list_of_map_files)) {
    LOG(ERROR) << "Map under \"" << folder_path << "\" does not exist!";
    return false;
  }

  // Load metadata first.
  CHECK(
      backend::resource_map_serialization::loadMetaDataFromFolder(
          folder_path, map));

  // Since we load this map from what obviously is the map folder, we can
  // replace the previous map folder (and resource folder) with the path of the
  // loaded map. This is important, because someone might have move the map on
  // the file system and those paths might be wrong now.
  map->replaceMapFolder(folder_path);

  const size_t kNumResourceFiles =
      backend::resource_map_serialization::internal::kResourceFiles.size();
  const size_t number_of_protos = list_of_map_files.size() - kNumResourceFiles;
  std::vector<vi_map_deprecated::proto::VIMap> list_of_protos(number_of_protos);
  common::MultiThreadedProgressBar progress_bar;
  std::string path_to_map_file;
  common::concatenateFolderAndFileName(
      folder_path, internal::kFolderName, &path_to_map_file);

  std::function<void(const std::vector<size_t>)> load_function =
      [&](const std::vector<size_t>& range) {
        progress_bar.setNumElements(range.size());
        size_t num_processed_tasks = 0u;
        for (const size_t& task_idx : range) {
          const std::string file_name =
              list_of_map_files[kNumResourceFiles + task_idx].substr(
                  static_cast<std::string>(internal::kFolderName).size() + 1u);
          vi_map_deprecated::proto::VIMap& proto = list_of_protos[task_idx];

          // If the map was saved in an old map format, the file may not exist.
          // This is ok to do here since we already check for all essential
          // files above with hasMapOnFileSystem.
          std::string complete_path_to_file;
          common::concatenateFolderAndFileName(
              path_to_map_file, file_name, &complete_path_to_file);
          if (common::fileExists(complete_path_to_file)) {
            CHECK(!file_name.empty());
            CHECK(
                common::proto_serialization_helper::parseProtoFromFile(
                    path_to_map_file, file_name, &proto));
          }
          progress_bar.update(++num_processed_tasks);
        }
      };

  constexpr bool kAlwaysParallelize = true;
  const size_t num_threads = common::getNumHardwareThreads();
  common::ParallelProcess(
      number_of_protos, load_function, kAlwaysParallelize, num_threads);

  deserializeFromListOfProtos(list_of_protos, map);

  CHECK(
      backend::resource_map_serialization::loadMapFromFolder(folder_path, map));

  LOG(INFO) << "Loaded VIMap from \"" << folder_path << "\".";
  return true;
}

void deserializeFromRawArray(
    const network::RawMessageDataList& raw_data, const size_t start_index,
    vi_map::VIMap* map) {
  CHECK_NOTNULL(map);
  CHECK_GE(raw_data.size(), start_index + internal::kMinNumProtos);

  std::vector<vi_map_deprecated::proto::VIMap> proto_list(
      raw_data.size() - start_index);
  constexpr bool kAlwaysParallelize = true;
  const size_t num_threads = common::getNumHardwareThreads();
  common::ParallelProcess(
      proto_list.size(),
      [&raw_data, &proto_list, &start_index](const std::vector<size_t>& range) {
        for (size_t task_idx : range) {
          const size_t raw_data_idx = start_index + task_idx;
          common::proto_serialization_helper::deserializeFromArray(
              raw_data[raw_data_idx].first, raw_data[raw_data_idx].second,
              &proto_list[task_idx]);
        }
      },
      kAlwaysParallelize, num_threads);

  deserializeFromListOfProtos(proto_list, map);
}

}  // namespace serialization_deprecated

}  // namespace vi_map
