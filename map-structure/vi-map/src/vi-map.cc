#include "vi-map/vi-map.h"

#include <limits>
#include <queue>

#include <aslam/common/memory.h>
#include <aslam/common/time.h>
#include <map-resources/resource_metadata.pb.h>
#include <maplab-common/file-system-tools.h>

#include "vi-map/deprecated/vi-map-serialization-deprecated.h"
#include "vi-map/semantics-manager.h"
#include "vi-map/vertex.h"
#include "vi-map/vi-map-serialization.h"

namespace vi_map {

VIMap::VIMap(const std::string& map_folder)
    : backend::ResourceMap(map_folder),
      const_this(this),
      generator_(static_cast<int>(
          std::chrono::system_clock::now().time_since_epoch().count())) {}

VIMap::VIMap(const metadata::proto::MetaData& metadata_proto)
    : backend::ResourceMap(metadata_proto),
      const_this(this),
      generator_(static_cast<int>(
          std::chrono::system_clock::now().time_since_epoch().count())) {}

VIMap::VIMap()
    : backend::ResourceMap(),
      const_this(this),
      generator_(static_cast<int>(
          std::chrono::system_clock::now().time_since_epoch().count())) {}

VIMap::~VIMap() {}

void VIMap::deepCopy(const VIMap& other) {
  clear();
  mergeAllMissionsFromMapWithoutResources(other);
  ResourceMap::deepCopyFrom(other);
}

void VIMap::mergeAllMissionsFromMapWithoutResources(
    const vi_map::VIMap& other) {
  const SensorManager& other_sensor_manager = other.getSensorManager();

  // Get all missions from old map and add them into the new map.
  vi_map::MissionIdList other_mission_ids;
  other.getAllMissionIds(&other_mission_ids);
  vi_map::MissionBaseFrameIdList mission_base_frame_ids;
  other.getAllMissionBaseFrameIds(&mission_base_frame_ids);

  for (const vi_map::MissionId& other_mission_id : other_mission_ids) {
    CHECK(other_mission_id.isValid());
    const vi_map::VIMission& other_mission = other.getMission(other_mission_id);
    const vi_map::MissionBaseFrame& original_mission_base_frame =
        other.getMissionBaseFrameForMission(other_mission_id);

    const aslam::NCamera& other_mission_ncamera =
        other_sensor_manager.getNCameraForMission(other_mission_id);

    addNewMissionWithBaseframe(
        other_mission_id, original_mission_base_frame.get_T_G_M(),
        original_mission_base_frame.get_T_G_M_Covariance(),
        other_mission_ncamera.cloneToShared(), other_mission.backboneType());

    SensorIdSet other_mission_sensor_ids;
    other_sensor_manager.getAllSensorIdsAssociatedWithMission(
        other_mission_id, &other_mission_sensor_ids);
    for (const SensorId& other_mission_sensor_id : other_mission_sensor_ids) {
      CHECK(other_mission_sensor_id.isValid());
      if (sensor_manager_.hasSensor(other_mission_sensor_id)) {
        sensor_manager_.associateExistingSensorWithMission(
            other_mission_sensor_id, other_mission_id);
      } else {
        sensor_manager_.addSensor(
            other_sensor_manager.getSensor(other_mission_sensor_id).clone(),
            other_mission_id);
      }
    }

    vi_map::Mission& copied_mission = getMission(other_mission_id);
    copied_mission.setRootVertexId(other_mission.getRootVertexId());
  }

  // Get all vertices and add them into the new map.
  pose_graph::VertexIdList vertex_ids;
  other.getAllVertexIds(&vertex_ids);

  for (const pose_graph::VertexId& vertex_id : vertex_ids) {
    const vi_map::Vertex& original_vertex = other.getVertex(vertex_id);
    vi_map::Vertex::UniquePtr copied_vertex =
        aligned_unique<vi_map::Vertex>(original_vertex);
    addVertex(std::move(copied_vertex));
  }

  // Add all edges into the new map.
  pose_graph::EdgeIdList edge_ids;
  other.getAllEdgeIds(&edge_ids);

  for (const pose_graph::EdgeId& edge_id : edge_ids) {
    const vi_map::Edge& original_edge = other.getEdgeAs<vi_map::Edge>(edge_id);
    vi_map::Edge* copied_edge;
    original_edge.copyEdgeInto(&copied_edge);
    addEdge(vi_map::Edge::UniquePtr(copied_edge));
  }

  // Add landmarks into copy of map.
  const LandmarkIndex& original_landmark_index = other.landmark_index;
  vi_map::LandmarkIdList landmarks_ids;
  original_landmark_index.getAllLandmarkIds(&landmarks_ids);
  for (const vi_map::LandmarkId& landmark_id : landmarks_ids) {
    CHECK(landmark_id.isValid());
    const pose_graph::VertexId& original_landmark_store_vertex_id =
        original_landmark_index.getStoringVertexId(landmark_id);
    CHECK(hasVertex(original_landmark_store_vertex_id));
    landmark_index.addLandmarkAndVertexReference(
        landmark_id, original_landmark_store_vertex_id);
  }

  for (const OptionalSensorDataMap::value_type& other_optional_sensor_data :
      other.optional_sensor_data_map_) {
    const MissionId& mission_id = other_optional_sensor_data.first;
    CHECK(hasMission(mission_id));
    CHECK(optional_sensor_data_map_.emplace(
        std::piecewise_construct,
        std::forward_as_tuple(mission_id),
        std::forward_as_tuple(other_optional_sensor_data.second)).second);
  }
}

void VIMap::mergeAllMissionsFromMap(const vi_map::VIMap& other) {
  VLOG(1) << "Merging from VI-Map.";
  mergeAllMissionsFromMapWithoutResources(other);

  VLOG(1) << "Copying metadata and resource infos.";
  ResourceMap::mergeFromMap(other);
}

void VIMap::swap(VIMap* other) {
  CHECK_NOTNULL(other);
  posegraph.swap(&other->posegraph);
  missions.swap(other->missions);
  mission_base_frames.swap(other->mission_base_frames);
  landmark_index.swap(&other->landmark_index);
  optional_sensor_data_map_.swap(other->optional_sensor_data_map_);
}

bool VIMap::hexStringToMissionIdIfValid(
    const std::string& map_mission_id_string,
    vi_map::MissionId* mission_id) const {
  CHECK_NOTNULL(mission_id);
  mission_id->fromHexString(map_mission_id_string);

  if (!mission_id->isValid() || !hasMission(*mission_id)) {
    return false;
  }
  return true;
}

bool VIMap::ensureMissionIdValid(
    const std::string& map_mission_id_string,
    vi_map::MissionId* mission_id) const {
  CHECK_NOTNULL(mission_id);

  if (numMissions() == 0) {
    LOG(ERROR) << "No missions in database.";
    return false;
  }

  if (!hexStringToMissionIdIfValid(map_mission_id_string, mission_id)) {
    *mission_id = getIdOfFirstMission();
    LOG(WARNING) << "Mission ID not set or invalid, taking 1st mission ID: "
                 << mission_id->hexString();
    return false;
  }
  return true;
}

template <>
void VIMap::getAllIds(pose_graph::VertexIdList* all_vertex_ids) const {
  return getAllVertexIds(all_vertex_ids);
}
template <>
void VIMap::getAllIds(LandmarkIdList* all_store_landmark_ids) const {
  return getAllLandmarkIds(all_store_landmark_ids);
}

template <>
void VIMap::getAllIdsInMission(
    const MissionId& mission, pose_graph::VertexIdList* all_ids_of_type) const {
  return getAllVertexIdsInMission(mission, all_ids_of_type);
}
template <>
void VIMap::getAllIdsInMission(
    const MissionId& mission, LandmarkIdList* all_ids_of_type) const {
  return getAllLandmarkIdsInMission(mission, all_ids_of_type);
}

template <>
void VIMap::getMissionIds(
    const pose_graph::VertexId& object_id,
    vi_map::MissionIdSet* mission_ids) const {
  CHECK_NOTNULL(mission_ids)->clear();
  mission_ids->insert(getMissionIdForVertex(object_id));
}

template <>
void VIMap::getMissionIds(
    const LandmarkId& object_id, vi_map::MissionIdSet* mission_ids) const {
  CHECK_NOTNULL(mission_ids)->clear();
  getLandmarkObserverMissions(object_id, mission_ids);
}

template <>
Eigen::Vector3d VIMap::get_p_G(const pose_graph::VertexId& id) const {
  return getVertex_G_p_I(id);
}
template <>
Eigen::Vector3d VIMap::get_p_G(const LandmarkId& id) const {
  return getLandmark_G_p_fi(id);
}
template <>
vi_map::MissionId VIMap::getMissionId(const pose_graph::VertexId& id) const {
  return getMissionIdForVertex(id);
}

template <>
vi_map::MissionId VIMap::getMissionId(const LandmarkId& id) const {
  return getLandmarkStoreVertex(id).getMissionId();
}

template <>
void VIMap::getMissionIds(
    const pose_graph::VertexIdList& object_ids,
    vi_map::MissionIdSet* mission_ids) const {
  CHECK_NOTNULL(mission_ids)->clear();
  for (const pose_graph::VertexId& vertex_id : object_ids) {
    mission_ids->insert(getMissionId(vertex_id));
  }
}

template <>
void VIMap::getMissionIds(
    const pose_graph::VertexIdSet& object_ids,
    vi_map::MissionIdSet* mission_ids) const {
  CHECK_NOTNULL(mission_ids)->clear();
  for (const pose_graph::VertexId& vertex_id : object_ids) {
    mission_ids->insert(getMissionId(vertex_id));
  }
}

template <>
void VIMap::getMissionIds(
    const LandmarkIdList& object_ids, vi_map::MissionIdSet* mission_ids) const {
  CHECK_NOTNULL(mission_ids)->clear();
  for (const LandmarkId& store_landmark_id : object_ids) {
    vi_map::MissionIdSet current_mission_ids;
    getMissionIds(store_landmark_id, &current_mission_ids);
    mission_ids->insert(current_mission_ids.begin(), current_mission_ids.end());
  }
}

pose_graph::Edge::EdgeType VIMap::getGraphTraversalEdgeType(
    const vi_map::MissionId& mission_id) const {
  Mission::BackBone back_bone_type = getMission(mission_id).backboneType();
  switch (back_bone_type) {
    case Mission::BackBone::kViwls:
      return pose_graph::Edge::EdgeType::kViwls;
    case Mission::BackBone::kOdometry:
      return pose_graph::Edge::EdgeType::kOdometry;
    default:
      LOG(FATAL) << "Unknown edge type: " << static_cast<int>(back_bone_type);
      return pose_graph::Edge::EdgeType::kViwls;
  }
}

pose_graph::VertexId VIMap::getLandmarkStoreVertexId(
    const vi_map::LandmarkId& id) const {
  CHECK(id.isValid());
  return landmark_index.getStoringVertexId(id);
}

void VIMap::sparsifyMission(
    const vi_map::MissionId& mission_id, int every_nth_vertex_to_keep) {
  CHECK_GT(every_nth_vertex_to_keep, 0);
  CHECK(hasMission(mission_id)) << "The mission " << mission_id << " is not "
                                << "present or selected.";

  const vi_map::VIMission& mission = getMission(mission_id);
  const pose_graph::VertexId& root_vertex_id = mission.getRootVertexId();

  pose_graph::VertexId kept_vertex_id = root_vertex_id;
  pose_graph::VertexId current_vertex_id;

  do {
    current_vertex_id = kept_vertex_id;
    VLOG(4) << "Vertex " << kept_vertex_id.hexString() << " : landmark count: "
            << getVertex(kept_vertex_id).getLandmarks().size();
    for (int i = 0; i < (every_nth_vertex_to_keep - 1); ++i) {
      if (getNextVertex(
              kept_vertex_id, getGraphTraversalEdgeType(mission_id),
              &current_vertex_id)) {
        VLOG(4) << "Merging " << i << "th vertex "
                << current_vertex_id.hexString() << " into "
                << kept_vertex_id.hexString();
        mergeNeighboringVertices(kept_vertex_id, current_vertex_id);
      } else {
        break;
      }
    }
  } while (getNextVertex(
      kept_vertex_id, getGraphTraversalEdgeType(mission_id), &kept_vertex_id));
}

void VIMap::getStatisticsOfMission(
    const vi_map::MissionId& mission_id,
    std::vector<size_t>* num_good_landmarks_per_camera,
    std::vector<size_t>* num_bad_landmarks_per_camera,
    std::vector<size_t>* num_unknown_landmarks_per_camera,
    std::vector<size_t>* total_num_landmarks_per_camera, size_t* num_landmarks,
    size_t* num_vertices, size_t* num_observations, double* duration_s,
    int64_t* start_time_ns, int64_t* end_time_ns) const {
  CHECK_NOTNULL(num_good_landmarks_per_camera)->clear();
  CHECK_NOTNULL(num_bad_landmarks_per_camera)->clear();
  CHECK_NOTNULL(num_unknown_landmarks_per_camera)->clear();
  CHECK_NOTNULL(total_num_landmarks_per_camera)->clear();
  CHECK_NOTNULL(num_observations);
  CHECK_NOTNULL(duration_s);
  CHECK_NOTNULL(start_time_ns);
  CHECK_NOTNULL(end_time_ns);
  CHECK(mission_id.isValid());
  *num_observations = 0u;
  *num_vertices = 0u;
  *num_landmarks = 0u;

  const aslam::NCameraId& ncamera_id =
      sensor_manager_.getNCameraForMission(mission_id).getId();
  CHECK(ncamera_id.isValid());
  const aslam::NCamera& ncamera = sensor_manager_.getNCamera(ncamera_id);
  const size_t num_cameras = ncamera.numCameras();
  num_good_landmarks_per_camera->resize(num_cameras, 0u);
  num_bad_landmarks_per_camera->resize(num_cameras, 0u);
  num_unknown_landmarks_per_camera->resize(num_cameras, 0u);
  total_num_landmarks_per_camera->resize(num_cameras, 0u);

  pose_graph::VertexIdList vertex_ids;
  getAllVertexIdsInMissionAlongGraph(mission_id, &vertex_ids);
  for (const pose_graph::VertexId& vertex_id : vertex_ids) {
    const vi_map::Vertex& vertex = getVertex(vertex_id);
    ++(*num_vertices);
    *num_landmarks += vertex.getLandmarks().size();
    for (const vi_map::Landmark& landmark : vertex.getLandmarks()) {
      const KeypointIdentifierList& observations = landmark.getObservations();
      if (!observations.empty()) {
        const size_t first_observation_frame_idx =
            observations.front().frame_id.frame_index;
        CHECK_LT(first_observation_frame_idx, num_cameras);

        if (landmark.getQuality() == vi_map::Landmark::Quality::kUnknown) {
          ++((*num_unknown_landmarks_per_camera)[first_observation_frame_idx]);
        } else if (landmark.getQuality() == vi_map::Landmark::Quality::kBad) {
          ++((*num_bad_landmarks_per_camera)[first_observation_frame_idx]);
        } else if (landmark.getQuality() == vi_map::Landmark::Quality::kGood) {
          ++((*num_good_landmarks_per_camera)[first_observation_frame_idx]);
        }
        ++((*total_num_landmarks_per_camera)[first_observation_frame_idx]);
      }
    }

    const unsigned int num_frames = vertex.numFrames();
    for (unsigned int frame_idx = 0; frame_idx < num_frames; ++frame_idx) {
      if (vertex.isVisualFrameSet(frame_idx) &&
          vertex.isVisualFrameValid(frame_idx)) {
        *num_observations +=
            vertex.getVisualFrame(frame_idx).getNumKeypointMeasurements();
      }
    }
  }

  *duration_s = 0.0;
  *start_time_ns = 0;
  *end_time_ns = 0;
  if (!vertex_ids.empty()) {
    const vi_map::Vertex& first_vertex = getVertex(vertex_ids.front());
    const vi_map::Vertex& last_vertex = getVertex(vertex_ids.back());
    const unsigned int kFirstFrameIndex = 0u;
    if (first_vertex.isFrameIndexValid(kFirstFrameIndex) &&
        last_vertex.isFrameIndexValid(kFirstFrameIndex)) {
      *start_time_ns = first_vertex.getVisualFrame(kFirstFrameIndex)
                           .getTimestampNanoseconds();
      *end_time_ns = last_vertex.getVisualFrame(kFirstFrameIndex)
                         .getTimestampNanoseconds();
      *duration_s =
          aslam::time::nanoSecondsToSeconds(*end_time_ns - *start_time_ns);
    }
  }
}

std::string VIMap::printMapStatistics(
    const vi_map::MissionId& mission_id, const unsigned int mission_number,
    const SemanticsManager& semantics) const {
  std::stringstream stats_text;

  static constexpr int kMaxLength = 20;

  auto print_aligned = [&stats_text](
      const std::string& key, const std::string& value, int indent) {
    for (int i = 0; i < indent; ++i) {
      stats_text << "\t";
    }
    stats_text.width(static_cast<std::streamsize>(kMaxLength));
    stats_text.setf(std::ios::left, std::ios::adjustfield);
    stats_text << key << "\t";
    stats_text.width(25);
    stats_text << value << std::endl;
  };

  std::string name = semantics.getNameOfMission(mission_id);
  std::vector<size_t> num_good_landmarks_per_camera;
  std::vector<size_t> num_bad_landmarks_per_camera;
  std::vector<size_t> num_unknown_landmarks_per_camera;
  std::vector<size_t> total_num_landmarks_per_camera;
  size_t num_landmarks = 0u;
  size_t num_vertices = 0u;
  size_t num_observations = 0u;
  double duration_s = 0.0;
  int64_t start_time_ns = 0u;
  int64_t end_time_ns = 0u;
  getStatisticsOfMission(
      mission_id, &num_good_landmarks_per_camera, &num_bad_landmarks_per_camera,
      &num_unknown_landmarks_per_camera, &total_num_landmarks_per_camera,
      &num_landmarks, &num_vertices, &num_observations, &duration_s,
      &start_time_ns, &end_time_ns);

  size_t num_wheel_odometry_edges = 0u;
  size_t num_imu_edges = 0u;
  size_t num_loop_closure_edges = 0u;

  pose_graph::EdgeIdList edge_ids;
  getAllEdgeIdsInMissionAlongGraph(mission_id, &edge_ids);
  for (const pose_graph::EdgeId& edge_id : edge_ids) {
    CHECK(edge_id.isValid());
    switch (getEdgeType(edge_id)) {
      case pose_graph::Edge::EdgeType::kOdometry:
        ++num_wheel_odometry_edges;
        break;
      case pose_graph::Edge::EdgeType::kLoopClosure:
        ++num_loop_closure_edges;
        break;
      case pose_graph::Edge::EdgeType::kViwls:
        ++num_imu_edges;
        break;
      default:
        break;
    }
  }

  const vi_map::VIMission& mission = getMission(mission_id);
  stats_text << std::endl;

  std::string mission_type = "Unknown";
  switch (mission.backboneType()) {
    case Mission::BackBone::kViwls: {
      mission_type = "Viwls";
      break;
    }
    case Mission::BackBone::kOdometry: {
      mission_type = "Odometry";
      break;
    }
    default: {
      LOG(FATAL) << "Unknown backbone type "
                 << static_cast<int>(mission.backboneType());
      break;
    }
  }

  print_aligned(
      "Mission " + std::to_string(mission_number) + ":",
      "\t" + mission.id().hexString() + "\t" + mission_type, 0);
  if (name != "") {
    print_aligned("Name:", name, 1);
  }

  const aslam::NCameraId& ncamera_id =
      sensor_manager_.getNCameraForMission(mission_id).getId();
  const aslam::NCamera& ncamera = sensor_manager_.getNCamera(ncamera_id);
  const size_t num_cameras = ncamera.numCameras();
  print_aligned("NCamera Sensor: ", ncamera_id.hexString(), 1);

  SensorIdSet imu_sensors_ids;
  sensor_manager_.getAllSensorIdsOfTypeAssociatedWithMission(
      SensorType::kImu, mission_id, &imu_sensors_ids);
  for (const SensorId& sensor_id : imu_sensors_ids) {
    CHECK(sensor_id.isValid());
    print_aligned("IMU Sensor: ", sensor_id.hexString(), 1);
  }

  SensorIdSet relative_6dof_pose_sensors_ids;
  sensor_manager_.getAllSensorIdsOfTypeAssociatedWithMission(
      SensorType::kRelative6DoFPose, mission_id,
      &relative_6dof_pose_sensors_ids);
  for (const SensorId& sensor_id : relative_6dof_pose_sensors_ids) {
    CHECK(sensor_id.isValid());
    print_aligned("Rel.6DoF-P. Sensor: ", sensor_id.hexString(), 1);
  }

  SensorIdSet gps_utm_sensors_ids;
  sensor_manager_.getAllSensorIdsOfTypeAssociatedWithMission(
      SensorType::kGpsUtm, mission_id, &gps_utm_sensors_ids);
  for (const SensorId& sensor_id : gps_utm_sensors_ids) {
    CHECK(sensor_id.isValid());
    print_aligned("GPS UTM Sensor: ", sensor_id.hexString(), 1);
  }

  SensorIdSet gps_wgs_sensors_ids;
  sensor_manager_.getAllSensorIdsOfTypeAssociatedWithMission(
      SensorType::kGpsWgs, mission_id, &gps_wgs_sensors_ids);
  for (const SensorId& sensor_id : gps_wgs_sensors_ids) {
    CHECK(sensor_id.isValid());
    print_aligned("GPS WGS Sensor: ", sensor_id.hexString(), 1);
  }

  print_aligned("Vertices:", std::to_string(num_vertices), 1);

  print_aligned("Landmarks:", std::to_string(num_landmarks), 1);
  print_aligned("Landmarks by first observer backlink:", "", 1);
  for (size_t camera_idx = 0u; camera_idx < num_cameras; ++camera_idx) {
    print_aligned(
        "Camera " + std::to_string(camera_idx) + ":",
        std::to_string(total_num_landmarks_per_camera[camera_idx]) +
            " (g:" + std::to_string(num_good_landmarks_per_camera[camera_idx]) +
            " b:" + std::to_string(num_bad_landmarks_per_camera[camera_idx]) +
            " u:" +
            std::to_string(num_unknown_landmarks_per_camera[camera_idx]) + ")",
        1);
  }
  print_aligned("Observations:", std::to_string(num_observations), 1);
  print_aligned("Num edges by type: ", "", 1);
  print_aligned("IMU: ", std::to_string(num_imu_edges), 1);
  print_aligned(
      "Wheel-odometry: ", std::to_string(num_wheel_odometry_edges), 1);
  print_aligned("Loop-closure: ", std::to_string(num_loop_closure_edges), 1);
  double distance = 0;
  getDistanceTravelledPerMission(mission_id, &distance);
  print_aligned("Distance travelled [m]:", std::to_string(distance), 1);

  if (!selected_missions_.empty()) {
    const bool is_selected = (selected_missions_.count(mission_id) > 0);
    print_aligned("Selected:", std::to_string(is_selected), 1);
  }

  if (num_vertices > 0) {
    time_t start_time(aslam::time::nanoSecondsToSeconds(start_time_ns));
    std::string start_time_str = common::generateDateString(&start_time);

    time_t end_time(aslam::time::nanoSecondsToSeconds(end_time_ns));
    std::string end_time_str = common::generateDateString(&end_time);
    print_aligned(
        "Start to end time: ", start_time_str + " to " + end_time_str, 1);
  } else {
    print_aligned("Mission has no vertices!", "", 1);
  }

  const vi_map::MissionBaseFrame& base_frame =
      getMissionBaseFrame(mission.getBaseFrameId());
  print_aligned("T_G_M", base_frame.is_T_G_M_known() ? "known" : "unknown", 1);

  if (hasOptionalSensorData(mission_id)) {
    const OptionalSensorData& optional_sensor_data =
        getOptionalSensorData(mission_id);

    vi_map::SensorIdSet sensor_ids;
    optional_sensor_data.getAllSensorIds(&sensor_ids);
    for (const SensorId& sensor_id : sensor_ids) {
      if (gps_utm_sensors_ids.count(sensor_id)) {
        const MeasurementBuffer<GpsUtmMeasurement>& utm_measurements =
            optional_sensor_data.getMeasurements<GpsUtmMeasurement>(sensor_id);
        print_aligned(
            "Num GPS UTM measurements (Sensor " + sensor_id.shortHex() +
                "..): ",
            std::to_string(utm_measurements.size()), 1);
      } else if (gps_wgs_sensors_ids.count(sensor_id)) {
        const MeasurementBuffer<GpsWgsMeasurement>& wgs_measurements =
            optional_sensor_data.getMeasurements<GpsWgsMeasurement>(sensor_id);
        print_aligned(
            "Num GPS WGS measurements (Sensor " + sensor_id.shortHex() +
                "..): ",
            std::to_string(wgs_measurements.size()), 1);
      }
    }
  }

  return stats_text.str();
}

std::string VIMap::printMapStatistics(void) const {
  vi_map::MissionIdList all_missions;
  getAllMissionIdsSortedByTimestamp(&all_missions);

  std::stringstream stats_text;
  stats_text << "Mission statistics: " << std::endl;
  unsigned int mission_number = 0u;
  for (const vi_map::MissionId& mission_id : all_missions) {
    stats_text << printMapStatistics(
        mission_id, mission_number, vi_map::SemanticsManager());
    ++mission_number;
  }
  return stats_text.str();
}

std::string VIMap::printMapAccumulatedStatistics() const {
  vi_map::MissionIdList all_missions;
  getAllMissionIdsSortedByTimestamp(&all_missions);

  double total_distance_travelled = 0.0;
  size_t total_num_vertices = 0u;
  size_t total_num_landmarks = 0u;
  size_t total_num_observations = 0u;
  size_t total_num_good_landmarks = 0u;
  size_t total_num_bad_landmarks = 0u;
  size_t total_num_unknown_landmarks = 0u;
  double total_duration_s = 0.0;

  for (const vi_map::MissionId& mission_id : all_missions) {
    std::vector<size_t> num_good_landmarks_per_camera;
    std::vector<size_t> num_bad_landmarks_per_camera;
    std::vector<size_t> num_unknown_landmarks_per_camera;
    std::vector<size_t> total_num_landmarks_per_camera;
    size_t num_landmarks;
    size_t num_vertices;
    size_t num_observations;
    double duration_s;
    int64_t start_time_ns;
    int64_t end_time_ns;
    getStatisticsOfMission(
        mission_id, &num_good_landmarks_per_camera,
        &num_bad_landmarks_per_camera, &num_unknown_landmarks_per_camera,
        &total_num_landmarks_per_camera, &num_landmarks, &num_vertices,
        &num_observations, &duration_s, &start_time_ns, &end_time_ns);

    total_num_good_landmarks += std::accumulate(
        num_good_landmarks_per_camera.begin(),
        num_good_landmarks_per_camera.end(), 0u);
    total_num_bad_landmarks += std::accumulate(
        num_bad_landmarks_per_camera.begin(),
        num_bad_landmarks_per_camera.end(), 0u);
    total_num_unknown_landmarks += std::accumulate(
        num_unknown_landmarks_per_camera.begin(),
        num_unknown_landmarks_per_camera.end(), 0u);
    total_num_landmarks += num_landmarks;
    total_num_observations += num_observations;
    total_num_vertices += num_vertices;

    total_duration_s += duration_s;

    double distance_travelled;
    getDistanceTravelledPerMission(mission_id, &distance_travelled);
    total_distance_travelled += distance_travelled;
  }

  std::stringstream stats_text;
  static constexpr int kMaxLength = 20;
  auto print_aligned = [&stats_text](
      const std::string& key, const std::string& value, int indent) {
    for (int i = 0; i < indent; ++i) {
      stats_text << "\t";
    }
    stats_text.width(static_cast<std::streamsize>(kMaxLength));
    stats_text.setf(std::ios::left, std::ios::adjustfield);
    stats_text << key << "\t";
    stats_text.width(25);
    stats_text << value << std::endl;
  };

  stats_text << "Accumulated statistics over all missions:" << std::endl;
  print_aligned("Number of missions:", std::to_string(numMissions()), 1);
  print_aligned("Vertices:", std::to_string(total_num_vertices), 1);
  print_aligned(
      "Landmarks:", std::to_string(total_num_landmarks) + " (g:" +
                        std::to_string(total_num_good_landmarks) + " b:" +
                        std::to_string(total_num_bad_landmarks) + " u:" +
                        std::to_string(total_num_unknown_landmarks) + ")",
      1);
  print_aligned("Observations:", std::to_string(total_num_observations), 1);
  print_aligned(
      "Distance travelled [m]:", std::to_string(total_distance_travelled), 1);
  print_aligned("Duration [s]: ", std::to_string(total_duration_s), 1);
  return stats_text.str();
}

void VIMap::getDistanceTravelledPerMission(
    const vi_map::MissionId& mission_id, double* distance) const {
  CHECK_NOTNULL(distance);
  *distance = 0.;
  const vi_map::VIMission& mission = getMission(mission_id);
  const pose_graph::VertexId& root_vertex_id = mission.getRootVertexId();
  if (!root_vertex_id.isValid()) {
    return;
  }

  pose_graph::VertexId current_vertex_id = root_vertex_id;
  pose_graph::VertexId next_vertex_id = root_vertex_id;

  do {
    const vi_map::Vertex& current_vertex = getVertex(current_vertex_id);
    const vi_map::Vertex& next_vertex = getVertex(next_vertex_id);

    CHECK_EQ(next_vertex.getMissionId(), current_vertex.getMissionId());

    *distance += (next_vertex.get_p_M_I() - current_vertex.get_p_M_I()).norm();

    current_vertex_id = next_vertex_id;
  } while (getNextVertex(
      current_vertex_id, getGraphTraversalEdgeType(mission_id),
      &next_vertex_id));
}

void VIMap::addNewMissionWithBaseframe(
    const vi_map::MissionId& mission_id, const pose::Transformation& T_G_M,
    const Eigen::Matrix<double, 6, 6>& T_G_M_covariance,
    const aslam::NCamera::Ptr& ncamera, Mission::BackBone backbone_type) {
  CHECK(mission_id.isValid());
  CHECK(ncamera);
  addNewMissionWithBaseframe(
      mission_id, T_G_M, T_G_M_covariance, backbone_type);
  const aslam::NCameraId& ncamera_id = ncamera->getId();
  if (sensor_manager_.hasNCamera(ncamera_id)) {
    sensor_manager_.associateExistingNCameraWithMission(ncamera_id, mission_id);
  } else {
    sensor_manager_.addNCamera(ncamera, mission_id);
  }
}

void VIMap::addNewMissionWithBaseframe(
    vi_map::VIMission::UniquePtr mission,
    const aslam::NCamera::Ptr& ncamera,
    const vi_map::MissionBaseFrame& mission_base_frame) {
  CHECK(ncamera);
  const MissionId& mission_id = mission->id();
  CHECK(mission_id.isValid());
  sensor_manager_.addNCamera(ncamera, mission_id);
  addNewMissionWithBaseframe(std::move(mission), mission_base_frame);
}

void VIMap::addNewMissionWithBaseframe(
    vi_map::VIMission::UniquePtr mission,
    const vi_map::MissionBaseFrame& mission_base_frame) {
  const MissionId& mission_id = mission->id();
  CHECK(mission_id.isValid());
  const MissionBaseFrameId& mission_base_frame_id = mission_base_frame.id();
  CHECK(mission_base_frame_id.isValid());
  CHECK_EQ(mission->getBaseFrameId(), mission_base_frame_id);
  missions.emplace(mission_id, std::move(mission));
  mission_base_frames.emplace(mission_base_frame_id, mission_base_frame);
}

void VIMap::addNewMissionWithBaseframe(
    const vi_map::MissionId& mission_id, const pose::Transformation& T_G_M,
    const Eigen::Matrix<double, 6, 6>& T_G_M_covariance,
    Mission::BackBone backbone_type) {
  // Create and add new mission base frame.
  vi_map::MissionBaseFrameId mission_baseframe_id =
      common::createRandomId<vi_map::MissionBaseFrameId>();
  MissionBaseFrame new_mission_base_frame(
      mission_baseframe_id, T_G_M, T_G_M_covariance);

  mission_base_frames.emplace(mission_baseframe_id, new_mission_base_frame);

  // Create and add new mission.
  VIMission* new_mission(new VIMission(
      mission_id, mission_baseframe_id, backbone_type));

  missions.emplace(mission_id, VIMission::UniquePtr(new_mission));

  if (!selected_missions_.empty()) {
    selected_missions_.insert(mission_id);
  }
}

size_t VIMap::numLandmarks() const {
  size_t result = 0u;
  pose_graph::VertexIdList vertex_ids;
  getAllVertexIds(&vertex_ids);
  for (const pose_graph::VertexId& vertex_id : vertex_ids) {
    result += getVertex(vertex_id).getLandmarks().size();
  }
  return result;
}

void VIMap::addNewLandmark(
    const vi_map::Landmark& landmark,
    const pose_graph::VertexId& keypoint_and_store_vertex_id,
    unsigned int frame_index, unsigned int keypoint_index) {
  CHECK(!hasLandmark(landmark.id()))
      << "Landmark " << landmark.id() << " already exists in map.";
  CHECK(hasVertex(keypoint_and_store_vertex_id));
  CHECK_EQ(0u, landmark.numberOfObserverVertices())
      << "The new landmark shouldn't contain any backlinks as this may cause "
         "inconsistencies.";

  getVertex(keypoint_and_store_vertex_id).getLandmarks().addLandmark(landmark);

  const vi_map::LandmarkId& landmark_id = landmark.id();
  landmark_index.addLandmarkAndVertexReference(
      landmark_id, keypoint_and_store_vertex_id);
  CHECK(hasLandmark(landmark_id));
  associateKeypointWithExistingLandmark(
      keypoint_and_store_vertex_id, frame_index, keypoint_index, landmark_id);
}

void VIMap::addNewLandmark(
    const LandmarkId& predefined_landmark_id,
    const KeypointIdentifier& first_observation) {
  CHECK(!hasLandmark(predefined_landmark_id));
  vi_map::Landmark landmark;
  landmark.setId(predefined_landmark_id);
  addNewLandmark(
      landmark, first_observation.frame_id.vertex_id,
      first_observation.frame_id.frame_index, first_observation.keypoint_index);
}

void VIMap::associateKeypointWithExistingLandmark(
    const pose_graph::VertexId& keypoint_vertex_id, unsigned int frame_index,
    unsigned int keypoint_index, const vi_map::LandmarkId& landmark_id) {
  CHECK(hasVertex(keypoint_vertex_id));
  CHECK(hasLandmark(landmark_id))
      << "No landmark " << landmark_id << " in the map!";

  vi_map::Vertex& vertex = getVertex(keypoint_vertex_id);
  CHECK_LT(keypoint_index, vertex.observedLandmarkIdsSize(frame_index));
  const LandmarkId existing_id =
      vertex.getObservedLandmarkId(frame_index, keypoint_index);
  if (existing_id.isValid()) {
    CHECK_EQ(existing_id, landmark_id)
        << "Cannot reassociate a keypoint that already points to an existing "
        << "store landmark. Dereference it first.";
  }

  // Update vertex.
  vertex.setObservedLandmarkId(frame_index, keypoint_index, landmark_id);

  // Update landmark.
  vi_map::Landmark& landmark = getLandmark(landmark_id);
  landmark.addObservation(keypoint_vertex_id, frame_index, keypoint_index);
}

void VIMap::associateKeypointWithExistingLandmark(
    const KeypointIdentifier& keypoint_id, const LandmarkId& landmark_id) {
  associateKeypointWithExistingLandmark(
      keypoint_id.frame_id.vertex_id, keypoint_id.frame_id.frame_index,
      keypoint_id.keypoint_index, landmark_id);
}

void VIMap::getAllVertex_p_G_I(
    const MissionId& mission_id, Eigen::Matrix3Xd* result) const {
  CHECK_NOTNULL(result);
  CHECK(hasMission(mission_id));
  pose_graph::VertexIdList mission_vertex_ids;
  getAllVertexIdsInMission(mission_id, &mission_vertex_ids);
  Eigen::Matrix3Xd p_M_I(3, mission_vertex_ids.size());
  int i = 0;
  for (const pose_graph::VertexId& vertex_id : mission_vertex_ids) {
    const Vertex& vertex = getVertex(vertex_id);
    p_M_I.col(i) = vertex.get_p_M_I();
    ++i;
  }
  *result = getMissionBaseFrameForMission(mission_id)
                .get_T_G_M()
                .transformVectorized(p_M_I);
}

void VIMap::getVertex_p_G_I_ForVertexSet(
    const pose_graph::VertexIdSet& vertex_ids, Eigen::Matrix3Xd* p_G_Is) const {
  CHECK_NOTNULL(p_G_Is);

  const size_t num_vertices = vertex_ids.size();
  p_G_Is->resize(Eigen::NoChange, num_vertices);

  size_t vertex_idx = 0u;
  for (const pose_graph::VertexId& vertex_id : vertex_ids) {
    p_G_Is->col(vertex_idx) = getVertex_G_p_I(vertex_id);
    ++vertex_idx;
  }
}

void VIMap::addLandmarkIndexReference(
    const vi_map::LandmarkId& landmark_id,
    const pose_graph::VertexId& storing_vertex_id) {
  CHECK(landmark_id.isValid());
  CHECK(getVertex(storing_vertex_id).hasStoredLandmark(landmark_id))
      << "Landmark " << landmark_id << " not found in vertex "
      << storing_vertex_id << "!";

  landmark_index.addLandmarkAndVertexReference(landmark_id, storing_vertex_id);
}

void VIMap::moveLandmarksToOtherVertex(
    const pose_graph::VertexId& vertex_id_from,
    const pose_graph::VertexId& vertex_id_to) {
  CHECK(hasVertex(vertex_id_from));
  CHECK(hasVertex(vertex_id_to));
  CHECK_NE(vertex_id_from.hexString(), vertex_id_to.hexString());

  vi_map::Vertex& vertex_from = getVertex(vertex_id_from);
  vi_map::Vertex& vertex_to = getVertex(vertex_id_to);

  vi_map::LandmarkStore& landmark_store_from = vertex_from.getLandmarks();
  vi_map::LandmarkStore& landmark_store_to = vertex_to.getLandmarks();

  pose::Transformation T_I_M = vertex_to.get_T_M_I().inverse();

  vi_map::LandmarkIdSet landmarks_to_be_removed;
  for (const vi_map::Landmark& landmark : landmark_store_from) {
    // This should not be a reference! Otherwise, when moving the object,
    // it will get invalidated and will cause random segfaults.
    vi_map::LandmarkId landmark_id = landmark.id();
    const Eigen::Vector3d& p_G_fi = getLandmark_G_p_fi(landmark_id);
    const vi_map::MissionBaseFrame& mission_baseframe_to =
        getMissionBaseFrame(getMissionForVertex(vertex_id_to).getBaseFrameId());

    const Eigen::Vector3d p_M_fi =
        mission_baseframe_to.transformPointInGlobalFrameToMissionFrame(p_G_fi);
    const Eigen::Vector3d p_I_fi = T_I_M * p_M_fi;

    CHECK(!landmark_store_to.hasLandmark(landmark_id));
    landmark_store_to.addLandmark(landmark);
    vi_map::Landmark& new_landmark = landmark_store_to.getLandmark(landmark_id);
    new_landmark.set_p_B(pose::Position3D(p_I_fi));

    CHECK_EQ(getLandmarkStoreVertexId(landmark_id), vertex_id_from);
    landmark_index.updateVertexOfLandmark(landmark_id, vertex_id_to);

    landmarks_to_be_removed.insert(landmark_id);
  }

  for (const vi_map::LandmarkId& landmark_id : landmarks_to_be_removed) {
    landmark_store_from.removeLandmark(landmark_id);
  }
}

// Currently assumes vertex_from is the next after vertex_to.
void VIMap::mergeNeighboringVertices(
    const pose_graph::VertexId& merge_into_vertex_id,
    const pose_graph::VertexId& next_vertex_id) {
  CHECK(hasVertex(merge_into_vertex_id));
  CHECK(hasVertex(next_vertex_id));

  vi_map::MissionId mission_id = getVertex(merge_into_vertex_id).getMissionId();

  pose_graph::VertexId check_vertex_id;
  getNextVertex(
      merge_into_vertex_id, getGraphTraversalEdgeType(mission_id),
      &check_vertex_id);
  CHECK_EQ(check_vertex_id, next_vertex_id)
      << "Vertices should be neighboring and vertex_from should be the next"
      << " after next_vertex_id.";

  VLOG(4) << "Merging vertices: " << next_vertex_id << " into "
          << merge_into_vertex_id;

  // Move landmarks.
  moveLandmarksToOtherVertex(next_vertex_id, merge_into_vertex_id);

  std::unordered_set<pose_graph::EdgeId> incoming, outgoing;
  vi_map::Vertex& next_vertex = getVertex(next_vertex_id);

  next_vertex.getOutgoingEdges(&outgoing);
  next_vertex.getIncomingEdges(&incoming);
  // It's possible that next vertex is the last vertex in the mission so
  // we need to handle no outgoing edge case.
  size_t num_incoming_viwls_edges = 0u, num_outgoing_viwls_edges = 0u;
  vi_map::ViwlsEdge* edge_between_vertices = nullptr;
  vi_map::ViwlsEdge* edge_after_next_vertex = nullptr;
  for (const pose_graph::EdgeId& incoming_edge : incoming) {
    if (getEdgeType(incoming_edge) == pose_graph::Edge::EdgeType::kViwls) {
      edge_between_vertices = getEdgePtrAs<vi_map::ViwlsEdge>(incoming_edge);
      ++num_incoming_viwls_edges;
    }
  }
  for (const pose_graph::EdgeId& outgoing_edge : outgoing) {
    if (getEdgeType(outgoing_edge) == pose_graph::Edge::EdgeType::kViwls) {
      edge_after_next_vertex = getEdgePtrAs<vi_map::ViwlsEdge>(outgoing_edge);
      ++num_outgoing_viwls_edges;
    }
  }
  CHECK_LE(num_outgoing_viwls_edges, 1u)
      << "A vertex can have only one outgoing edge in VIWLS graph";
  CHECK_EQ(1u, num_incoming_viwls_edges)
      << "A vertex can have only one incoming edge in VIWLS graph";
  CHECK_NOTNULL(edge_between_vertices);

  // Remove all other edges from vertex.
  for (const pose_graph::EdgeId& incoming_edge : incoming) {
    if (getEdgeType(incoming_edge) != pose_graph::Edge::EdgeType::kViwls) {
      posegraph.removeEdge(incoming_edge);
    }
  }
  for (const pose_graph::EdgeId& outgoing_edge : outgoing) {
    if (getEdgeType(outgoing_edge) != pose_graph::Edge::EdgeType::kViwls) {
      posegraph.removeEdge(outgoing_edge);
    }
  }

  if (edge_after_next_vertex != nullptr) {
    posegraph.mergeNeighboringViwlsEdges(
        merge_into_vertex_id, *edge_between_vertices, *edge_after_next_vertex);
  } else {
    // We should remove the edge linking the two vertices.
    posegraph.removeEdge(edge_between_vertices->id());
  }

  const unsigned int num_frames = next_vertex.numFrames();
  // We need to track landmarks that get removed in the following loop. This
  // prevents a crash in the case where a landmark is only observed from one
  // vertex, but from multiple frames within this vertex. For the first frame,
  // the landmark will be removed from the map in this case. All subsequent
  // frames try to also access this landmark, which will fail because it has
  // been deleted.
  std::unordered_set<LandmarkId> deleted_landmarks;
  for (unsigned int frame_idx = 0u; frame_idx < num_frames; ++frame_idx) {
    if (!next_vertex.isVisualFrameSet(frame_idx)) {
      continue;
    }
    const size_t num_of_keypoints =
        next_vertex.observedLandmarkIdsSize(frame_idx);
    for (size_t i = 0u; i < num_of_keypoints; ++i) {
      vi_map::LandmarkId landmark_id =
          next_vertex.getObservedLandmarkId(frame_idx, i);

      if (landmark_id.isValid() && deleted_landmarks.count(landmark_id) == 0u) {
        vi_map::Landmark& landmark = getLandmark(landmark_id);

        // Check for orphaned landmarks.
        if (landmark.numberOfObserverVertices() == 1u) {
          // Our merged vertex is the only vertex seeing this landmark.
          removeLandmark(landmark_id);
          deleted_landmarks.emplace(landmark_id);
        } else {
          // Remove vertex visibility in landmarks.
          landmark.removeAllObservationsOfVertex(next_vertex_id);
        }
      }
    }
  }

  // Remove the vertex.
  posegraph.removeVertex(next_vertex_id);
}

void VIMap::mergeLandmarks(
    const vi_map::LandmarkId landmark_id_to_merge,
    const vi_map::LandmarkId& landmark_id_into) {
  CHECK_NE(landmark_id_to_merge, landmark_id_into);
  CHECK(hasLandmark(landmark_id_to_merge));
  CHECK(hasLandmark(landmark_id_into));

  const unsigned int landmark_index_size_before = landmark_index.numLandmarks();

  vi_map::Vertex& landmark_vertex_to_merge =
      getLandmarkStoreVertex(landmark_id_to_merge);
  vi_map::Vertex& landmark_vertex_into =
      getLandmarkStoreVertex(landmark_id_into);

  // Check if vertices exist, they most probably are, but may belong to
  // a mission that is not selected.
  CHECK(hasVertex(landmark_vertex_to_merge.id()));
  CHECK(hasVertex(landmark_vertex_into.id()));

  CHECK(landmark_vertex_to_merge.getLandmarks().hasLandmark(
      landmark_id_to_merge));
  CHECK(landmark_vertex_into.getLandmarks().hasLandmark(landmark_id_into));

  vi_map::Landmark& landmark_into =
      landmark_vertex_into.getLandmarks().getLandmark(landmark_id_into);
  const vi_map::Landmark& landmark_to_merge =
      landmark_vertex_to_merge.getLandmarks().getLandmark(landmark_id_to_merge);

  // Move backlinks in landmark object to the new landmark.
  landmark_into.addObservations(landmark_to_merge.getObservations());

  if (landmark_into.getQuality() != Landmark::Quality::kGood) {
    if (landmark_to_merge.getQuality() == Landmark::Quality::kGood) {
      landmark_into.setQuality(Landmark::Quality::kGood);
    } else {
      landmark_into.setQuality(Landmark::Quality::kUnknown);
    }
  }

  // Update the observed landmark IDs in observer vertices.
  landmark_to_merge.forEachObservation(
      [&](const KeypointIdentifier& observation) {
        vi_map::Vertex& vertex = getVertex(observation.frame_id.vertex_id);
        const unsigned int frame_idx = observation.frame_id.frame_index;
        CHECK(vertex.isVisualFrameSet(frame_idx));
        vertex.setObservedLandmarkId(observation, landmark_id_into);
        vertex.updateIdInObservedLandmarkIdList(
            landmark_id_to_merge, landmark_id_into);
      });

  // Remove landmark object stored in a vertex.
  landmark_vertex_to_merge.getLandmarks().removeLandmark(landmark_id_to_merge);
  CHECK(!landmark_vertex_to_merge.getLandmarks().hasLandmark(
      landmark_id_to_merge));

  // Remove the landmark from the landmark index.
  landmark_index.removeLandmark(landmark_id_to_merge);

  // After the merge, we should have minus 1 entry in the landmark index.
  CHECK_EQ(landmark_index_size_before - 1, landmark_index.numLandmarks());
}

void VIMap::duplicateMission(const vi_map::MissionId& source_mission_id) {
  CHECK(hasMission(source_mission_id));

  const vi_map::VIMission& source_mission = getMission(source_mission_id);
  const vi_map::MissionBaseFrame& source_baseframe =
      getMissionBaseFrame(source_mission.getBaseFrameId());

  VLOG(1) << "Adding baseframe and mission.";
  // Copy baseframe.
  vi_map::MissionBaseFrame baseframe;
  vi_map::MissionBaseFrameId baseframe_id = source_baseframe.id();
  common::generateId(&baseframe_id);

  baseframe.setId(baseframe_id);
  baseframe.set_p_G_M(source_baseframe.get_p_G_M());
  baseframe.set_q_G_M(source_baseframe.get_q_G_M());
  baseframe.set_T_G_M_Covariance(source_baseframe.get_T_G_M_Covariance());
  mission_base_frames.emplace(baseframe_id, baseframe);

  // Copy mission.
  vi_map::VIMission::UniquePtr mission_ptr(new vi_map::VIMission);
  vi_map::MissionId duplicated_mission_id = source_mission.id();
  common::generateId(&duplicated_mission_id);

  mission_ptr->setId(duplicated_mission_id);
  mission_ptr->setBackboneType(source_mission.backboneType());
  mission_ptr->setBaseFrameId(baseframe_id);

  missions.emplace(mission_ptr->id(), std::move(mission_ptr));
  VLOG(1) << "New mission id: " << duplicated_mission_id;

  // Copy posegraph.
  typedef std::unordered_map<pose_graph::VertexId, pose_graph::VertexId>
      VertexIdToVertexIdMap;
  typedef std::unordered_map<pose_graph::EdgeId, pose_graph::EdgeId>
      EdgeIdToEdgeIdMap;
  typedef std::unordered_map<vi_map::LandmarkId, vi_map::LandmarkId>
      LandmarkIdToLandmarkIdMap;

  VertexIdToVertexIdMap source_to_dest_vertex_id_map;
  EdgeIdToEdgeIdMap source_to_dest_edge_id_map;
  LandmarkIdToLandmarkIdMap source_to_dest_landmark_id_map;
  LandmarkIdSet landmarks_not_to_duplicate;

  typedef VertexIdToVertexIdMap::iterator VertexIdToVertexIdIterator;
  typedef LandmarkIdToLandmarkIdMap::iterator LandmarkIdToLandmarkIdIterator;

  pose_graph::VertexIdList all_source_vertices;
  getAllVertexIds(&all_source_vertices);

  aslam::NCamera::Ptr duplicated_ncamera =
      sensor_manager_.getNCameraForMission(source_mission_id).cloneToShared();
  CHECK(duplicated_ncamera);

  // Generating new ids.
  aslam::NCameraId duplicated_ncamera_id;
  common::generateId(&duplicated_ncamera_id);
  duplicated_ncamera->setId(duplicated_ncamera_id);
  for (size_t camera_idx = 0u; camera_idx < duplicated_ncamera->numCameras();
  ++camera_idx) {
    aslam::CameraId camera_id;
    common::generateId(&camera_id);
    duplicated_ncamera->getCameraShared(camera_idx)->setId(camera_id);
  }
  sensor_manager_.addNCamera(duplicated_ncamera, duplicated_mission_id);

  SensorIdSet source_mission_sensor_ids;
  sensor_manager_.getAllSensorIdsAssociatedWithMission(
      source_mission_id, &source_mission_sensor_ids);
  for (const SensorId& source_mission_sensor_id : source_mission_sensor_ids) {
    CHECK(source_mission_sensor_id.isValid());
    Sensor::UniquePtr cloned_sensor =
        sensor_manager_.getSensor(source_mission_sensor_id).clone();
    CHECK(cloned_sensor);
    SensorId sensor_id;
    common::generateId(&sensor_id);
    cloned_sensor->setId(sensor_id);
    sensor_manager_.addSensor(std::move(cloned_sensor), duplicated_mission_id);
  }

  VLOG(1) << "Adding vertices.";
  for (const pose_graph::VertexId& vertex_id : all_source_vertices) {
    const vi_map::Vertex& source_vertex = getVertex(vertex_id);
    if (source_vertex.getMissionId() == source_mission_id) {
      pose_graph::VertexId new_vertex_id = vertex_id;
      common::generateId(&new_vertex_id);

      Eigen::Matrix<double, 6, 1> imu_ba_bw;
      imu_ba_bw << source_vertex.getAccelBias(), source_vertex.getGyroBias();

      std::vector<LandmarkIdList> landmarks_seen_by_vertex;
      const size_t num_frames = source_vertex.numFrames();
      landmarks_seen_by_vertex.resize(num_frames);
      for (size_t frame_idx = 0u; frame_idx < source_vertex.numFrames();
           ++frame_idx) {
        if (!source_vertex.isVisualFrameSet(frame_idx)) {
          continue;
        }
        for (size_t landmark_idx = 0u;
             landmark_idx < source_vertex.observedLandmarkIdsSize(frame_idx);
             ++landmark_idx) {
          const vi_map::LandmarkId& current_landmark_id =
              source_vertex.getObservedLandmarkId(frame_idx, landmark_idx);
          vi_map::LandmarkId new_landmark_id = current_landmark_id;
          if (!current_landmark_id.isValid()) {
            new_landmark_id.setInvalid();
          } else {
            bool should_duplicate = true;
            if (landmarks_not_to_duplicate.count(current_landmark_id) > 0u) {
              should_duplicate = false;
            } else {
              const bool is_stored_in_source_mission =
                  (getLandmarkStoreVertex(current_landmark_id).getMissionId() ==
                   source_mission_id);

              if (!is_stored_in_source_mission) {
                should_duplicate = false;
              } else {
                vi_map::MissionIdSet observer_missions;
                getLandmarkObserverMissions(
                    current_landmark_id, &observer_missions);
                CHECK(!observer_missions.empty())
                    << "Landmark should have at "
                    << "least one observer (= one observer mission).";
                const bool is_observed_by_source_mission_only =
                    (observer_missions.size() == 1u &&
                     observer_missions.count(source_mission_id) > 0u);

                if (!is_observed_by_source_mission_only) {
                  should_duplicate = false;
                }
              }
            }

            if (!should_duplicate) {
              // The landmark comes from some other mission or is observed
              // by other missions. We will not generate a new LandmarkId,
              // but just add observations.
              new_landmark_id = current_landmark_id;

              // Add a backlink to the landmark from some other mission.
              getLandmark(current_landmark_id)
                  .addObservation(new_vertex_id, frame_idx, landmark_idx);

              landmarks_not_to_duplicate.emplace(current_landmark_id);
            } else {
              // This landmark comes from the source mission and is only
              // observed by the source mission. It should be duplicated.
              const LandmarkIdToLandmarkIdIterator it =
                  source_to_dest_landmark_id_map.find(current_landmark_id);
              if (it != source_to_dest_landmark_id_map.end()) {
                new_landmark_id = it->second;
              } else {
                common::generateId(&new_landmark_id);
                source_to_dest_landmark_id_map.emplace(
                    current_landmark_id, new_landmark_id);
              }
            }
          }
          landmarks_seen_by_vertex[frame_idx].push_back(new_landmark_id);
        }
      }

      aslam::VisualNFrame::Ptr n_frame(new aslam::VisualNFrame(
          duplicated_ncamera));
      CHECK_EQ(duplicated_ncamera->getNumCameras(), num_frames);
      for (size_t frame_idx = 0u; frame_idx < num_frames; ++frame_idx) {
        if (source_vertex.isVisualFrameSet(frame_idx)) {
          const aslam::VisualFrame& source_frame =
              source_vertex.getVisualFrame(frame_idx);
          aslam::FrameId frame_id = source_frame.getId();
          common::generateId(&frame_id);
          aslam::VisualFrame::Ptr visual_frame(new aslam::VisualFrame);
          visual_frame->setId(frame_id);
          visual_frame->setTimestampNanoseconds(
              source_frame.getTimestampNanoseconds());
          visual_frame->setCameraGeometry(
              duplicated_ncamera->getCameraShared(frame_idx));
          visual_frame->setKeypointMeasurements(
              source_frame.getKeypointMeasurements());
          visual_frame->setKeypointMeasurementUncertainties(
              source_frame.getKeypointMeasurementUncertainties());
          visual_frame->setDescriptors(source_frame.getDescriptors());
          if (source_frame.hasTrackIds()) {
            visual_frame->setTrackIds(source_frame.getTrackIds());
          }

          n_frame->setFrame(frame_idx, visual_frame);
        }
      }

      vi_map::Vertex* vertex_ptr(
          new vi_map::Vertex(
              new_vertex_id, imu_ba_bw, n_frame, landmarks_seen_by_vertex,
              duplicated_mission_id));

      vertex_ptr->set_p_M_I(source_vertex.get_p_M_I());
      vertex_ptr->set_q_M_I(source_vertex.get_q_M_I().normalized());
      vertex_ptr->set_v_M(source_vertex.get_v_M());

      vertex_ptr->setFrameResourceMap(source_vertex.getFrameResourceMap());

      posegraph.addVertex(vi_map::Vertex::UniquePtr(vertex_ptr));

      // Our local old-to-new VertexId map.
      source_to_dest_vertex_id_map.emplace(vertex_id, new_vertex_id);
    }
  }

  VLOG(1) << "Adding edges";
  pose_graph::EdgeIdList all_source_edges;
  getAllEdgeIds(&all_source_edges);
  for (const pose_graph::EdgeId& edge_id : all_source_edges) {
    const vi_map::Edge& source_edge = getEdgeAs<vi_map::Edge>(edge_id);

    const MissionId& mission_id_from =
        getMissionIdForVertex(source_edge.from());
    const MissionId& mission_id_to = getMissionIdForVertex(source_edge.to());
    if (mission_id_from == source_mission_id ||
        mission_id_to == source_mission_id) {
      vi_map::Edge* copied_edge = nullptr;
      source_edge.copyEdgeInto(&copied_edge);
      CHECK_NOTNULL(copied_edge);

      pose_graph::EdgeId new_edge_id;
      common::generateId(&new_edge_id);

      copied_edge->setId(new_edge_id);

      VertexIdToVertexIdIterator it_from;
      it_from = source_to_dest_vertex_id_map.find(source_edge.from());
      CHECK(it_from != source_to_dest_vertex_id_map.end());
      copied_edge->setFrom(it_from->second);

      VertexIdToVertexIdIterator it_to;
      it_to = source_to_dest_vertex_id_map.find(source_edge.to());
      CHECK(it_to != source_to_dest_vertex_id_map.end());
      copied_edge->setTo(it_to->second);

      posegraph.addEdge(pose_graph::Edge::UniquePtr(copied_edge));

      // Our local old-to-new EdgeId map.
      source_to_dest_edge_id_map.insert(std::make_pair(edge_id, new_edge_id));
    }
  }

  VLOG(1) << "Setting root vertex.";
  // Set root vertex.
  const VertexIdToVertexIdIterator it =
      source_to_dest_vertex_id_map.find(source_mission.getRootVertexId());
  CHECK(it != source_to_dest_vertex_id_map.end());
  getMission(duplicated_mission_id).setRootVertexId(it->second);
  VLOG(1) << "Set root vertex in the new mission: " << it->second;

  VLOG(1) << "Adding landmarks.";
  for (const pose_graph::VertexId& vertex_id : all_source_vertices) {
    const vi_map::Vertex& source_vertex = getVertex(vertex_id);
    if (source_vertex.getMissionId() == source_mission_id) {
      VertexIdToVertexIdIterator it;
      it = source_to_dest_vertex_id_map.find(vertex_id);
      CHECK(it != source_to_dest_vertex_id_map.end());
      vi_map::Vertex& new_vertex = getVertex(it->second);

      const vi_map::LandmarkStore& source_landmark_store =
          source_vertex.getLandmarks();
      for (const vi_map::Landmark& landmark : source_landmark_store) {
        bool should_add_new_landmark =
            !(landmarks_not_to_duplicate.count(landmark.id()) > 0u);

        vi_map::Landmark new_landmark;
        if (should_add_new_landmark) {
          new_landmark.set_p_B(landmark.get_p_B());
          Eigen::Matrix3d covariance;
          if (landmark.get_p_B_Covariance(&covariance)) {
            new_landmark.set_p_B_Covariance(covariance);
          }

          vi_map::LandmarkId new_landmark_id;
          const LandmarkIdToLandmarkIdIterator it =
              source_to_dest_landmark_id_map.find(landmark.id());
          CHECK(it != source_to_dest_landmark_id_map.end())
              << "Landmark with "
              << "id" << landmark.id() << " is not present in the landmark to "
              << "landmark duplication map. Possibly it has no observations?";

          new_landmark_id = it->second;
          new_landmark.setId(new_landmark_id);
          new_landmark.setQuality(landmark.getQuality());
        }

        vi_map::Landmark& nonconst_landmark = getLandmark(landmark.id());

        landmark.forEachObservation([&](const KeypointIdentifier& observation) {
          if (getVertex(observation.frame_id.vertex_id).getMissionId() ==
              source_mission_id) {
            VertexIdToVertexIdIterator observer_vertex_it =
                source_to_dest_vertex_id_map.find(
                    observation.frame_id.vertex_id);
            CHECK(observer_vertex_it != source_to_dest_vertex_id_map.end());

            if (should_add_new_landmark) {
              new_landmark.addObservation(
                  observer_vertex_it->second, observation.frame_id.frame_index,
                  observation.keypoint_index);
            } else {
              nonconst_landmark.addObservation(
                  observer_vertex_it->second, observation.frame_id.frame_index,
                  observation.keypoint_index);
            }
          }
        });

        if (should_add_new_landmark) {
          new_vertex.getLandmarks().addLandmark(new_landmark);
        }
      }
    }
  }

  VLOG(1) << "Updating landmark index tables.";
  for (const LandmarkIdToLandmarkIdMap::value_type& source_to_dest_landmark :
       source_to_dest_landmark_id_map) {
    CHECK(source_to_dest_landmark.first.isValid());
    CHECK(source_to_dest_landmark.second.isValid());
    CHECK_NE(source_to_dest_landmark.first, source_to_dest_landmark.second);

    CHECK(hasLandmark(source_to_dest_landmark.first));
    const pose_graph::VertexId& old_store_vertex_id =
        getLandmarkStoreVertexId(source_to_dest_landmark.first);

    vi_map::LandmarkId new_landmark_id = source_to_dest_landmark.second;

    VertexIdToVertexIdIterator it_vertex;
    it_vertex = source_to_dest_vertex_id_map.find(old_store_vertex_id);
    CHECK(it_vertex != source_to_dest_vertex_id_map.end());
    pose_graph::VertexId new_store_vertex_id = it_vertex->second;

    landmark_index.addLandmarkAndVertexReference(
        new_landmark_id, new_store_vertex_id);
  }
  CHECK(checkMapConsistency(*this));

  if (!selected_missions_.empty()) {
    VLOG(1) << "Adding to VIMap selected missions set.";
    selected_missions_.insert(duplicated_mission_id);
  }

  VLOG(1) << "Done.";
}

unsigned int VIMap::numExpectedLandmarkObserverMissions(
    const vi_map::LandmarkId& landmark_id) const {
  CHECK(landmark_id.isValid());
  CHECK(hasLandmark(landmark_id));

  vi_map::MissionIdSet observer_missions;
  getLandmarkObserverMissions(landmark_id, &observer_missions);

  vi_map::MissionIdList candidate_mission_ids;
  getAllMissionIds(&candidate_mission_ids);
  vi_map::MissionIdSet candidate_missions(
      candidate_mission_ids.begin(), candidate_mission_ids.end());
  for (const vi_map::MissionId& observer_mission_id : observer_missions) {
    candidate_missions.erase(observer_mission_id);
  }

  // Get context landmarks and count observations.
  typedef std::unordered_map<vi_map::LandmarkId, unsigned int>
      ContextLandmarkObservationCountMap;
  ContextLandmarkObservationCountMap context_landmarks;
  const vi_map::Landmark& landmark = getLandmark(landmark_id);

  landmark.forEachObservation([&](const KeypointIdentifier& observation) {
    const vi_map::Vertex vertex = getVertex(observation.frame_id.vertex_id);
    const unsigned int frame_idx = observation.frame_id.frame_index;
    if (vertex.isVisualFrameSet(frame_idx)) {
      for (size_t j = 0; j < vertex.observedLandmarkIdsSize(frame_idx); ++j) {
        vi_map::LandmarkId observed_landmark_id =
            vertex.getObservedLandmarkId(frame_idx, j);
        if (observed_landmark_id.isValid()) {
          ++context_landmarks[observed_landmark_id];
        }
      }
    }
  });

  // The considered landmark got also added, let's remove it.
  context_landmarks.erase(landmark_id);

  static constexpr unsigned int kMinObservationsPerContextLandmark = 2;
  typedef std::unordered_map<vi_map::MissionId, unsigned int>
      MissionContextObservationCountMap;
  MissionContextObservationCountMap mission_context_landmark_observations;
  for (const ContextLandmarkObservationCountMap::value_type& context_landmark :
       context_landmarks) {
    // Check if this context landmark was observed more than
    // kMinObservationsPerContextLandmark times.
    if (context_landmark.second > kMinObservationsPerContextLandmark) {
      std::unordered_set<vi_map::MissionId> context_landmark_observer_missions;
      getLandmarkObserverMissions(
          context_landmark.first, &context_landmark_observer_missions);

      for (const vi_map::MissionId& mission_id :
           context_landmark_observer_missions) {
        // This is not a candidate mission, skip.
        if (candidate_missions.count(mission_id) == 0u) {
          continue;
        }

        ++mission_context_landmark_observations[mission_id];
      }
    }
  }

  static constexpr double kMinContextLandmarksCoverage = 0.2;
  unsigned int num_expected_observer_missions = 0;
  for (const MissionContextObservationCountMap::value_type& candidate_mission :
       mission_context_landmark_observations) {
    if (candidate_mission.second >
        kMinContextLandmarksCoverage * context_landmarks.size()) {
      ++num_expected_observer_missions;
    }
  }
  CHECK_LE(
      num_expected_observer_missions + observer_missions.size(), numMissions());
  return num_expected_observer_missions + observer_missions.size();
}

void VIMap::removeMission(
    const vi_map::MissionId& mission_id, bool remove_baseframe) {
  // Delete all the vertices and edges in the mission.
  CHECK(mission_id.isValid());
  CHECK(hasMission(mission_id));

  vi_map::VIMission& mission = getMission(mission_id);
  pose_graph::VertexIdList vertices;
  getAllVertexIdsInMission(mission_id, &vertices);

  for (pose_graph::VertexId vertex_id : vertices) {
    vi_map::Vertex* vertex = getVertexPtr(vertex_id);
    // Delete all edges.
    pose_graph::EdgeIdSet incoming_edges;
    vertex->getIncomingEdges(&incoming_edges);
    for (pose_graph::EdgeId incoming_edge_id : incoming_edges) {
      if (hasEdge(incoming_edge_id)) {
        removeEdge(incoming_edge_id);
      }
    }
    pose_graph::EdgeIdSet outgoing_edges;
    vertex->getOutgoingEdges(&outgoing_edges);
    for (pose_graph::EdgeId outgoing_edge_id : outgoing_edges) {
      if (hasEdge(outgoing_edge_id)) {
        removeEdge(outgoing_edge_id);
      }
    }

    // Delete all landmarks.
    vi_map::LandmarkIdList landmark_ids;
    vertex->getStoredLandmarkIdList(&landmark_ids);
    for (const vi_map::LandmarkId& landmark_id : landmark_ids) {
      removeLandmark(landmark_id);
    }
    CHECK_EQ(0u, vertex->getLandmarks().size());

    // Remove references to global landmarks and landmark backlinks.
    for (unsigned int frame_idx = 0u; frame_idx < vertex->numFrames();
         ++frame_idx) {
      vi_map::LandmarkIdList frame_landmark_ids;
      vertex->getFrameObservedLandmarkIds(frame_idx, &frame_landmark_ids);

      for (size_t i = 0; i < frame_landmark_ids.size(); ++i) {
        if (frame_landmark_ids[i].isValid()) {
          const vi_map::LandmarkId& landmark_id = frame_landmark_ids[i];

          vi_map::Landmark& landmark = getLandmark(landmark_id);
          landmark.removeAllObservationsOfVertex(vertex_id);

          // Also clean up the global landmark id list in the current vertex
          // so that we feed a consistent state to the removeVertex method
          // below.
          vi_map::LandmarkId invalid_landmark_id;
          invalid_landmark_id.setInvalid();
          vertex->setObservedLandmarkId(frame_idx, i, invalid_landmark_id);

          // If the current vertex was the only observer of the landmark stored
          // in some other mission, we should remove this orphaned landmark.
          if (getMissionIdForLandmark(landmark_id) != mission_id &&
              !landmark.hasObservations()) {
            removeLandmark(landmark.id());
          }
        }
      }
    }

    removeVertex(vertex_id);
  }

  // Set the root vertex to invalid. (Also it's anyway already deleted.)
  pose_graph::VertexId invalid_vertex_id;
  invalid_vertex_id.setInvalid();
  mission.setRootVertexId(invalid_vertex_id);

  // Then remove all references to this mission object.
  removeMissionObject(mission_id, remove_baseframe);
}

void VIMap::removeMissionObject(
    const vi_map::MissionId& mission_id, bool remove_baseframe) {
  CHECK(mission_id.isValid());
  CHECK(hasMission(mission_id));

  const vi_map::VIMission& mission = getMission(mission_id);
  CHECK(!mission.getRootVertexId().isValid())
      << "Root vertex ID of a mission you want to delete should be invalid.";

  if (remove_baseframe) {
    mission_base_frames.erase(mission.getBaseFrameId());
  }
  missions.erase(mission_id);
  selected_missions_.erase(mission_id);
}

void VIMap::moveLandmarkToOtherVertex(
    const LandmarkId& landmark_id, const pose_graph::VertexId& vertex_id_to) {
  CHECK(landmark_id.isValid());
  CHECK(vertex_id_to.isValid());
  CHECK(hasLandmark(landmark_id));
  CHECK(hasVertex(vertex_id_to));

  vi_map::Vertex& vertex_to = getVertex(vertex_id_to);
  vi_map::LandmarkStore& landmark_store_to = vertex_to.getLandmarks();

  vi_map::Vertex& vertex_from =
      getVertex(landmark_index.getStoringVertexId(landmark_id));
  CHECK(hasVertex(vertex_from.id()))
      << "Store landmark " << landmark_id
      << " is stored in a mission that is not selected.";
  vi_map::LandmarkStore& landmark_store_from = vertex_from.getLandmarks();

  pose::Transformation T_I_M = vertex_to.get_T_M_I().inverse();

  const Eigen::Vector3d& p_G_fi = getLandmark_G_p_fi(landmark_id);
  const vi_map::MissionBaseFrame& mission_baseframe_to =
      getMissionBaseFrame(getMissionForVertex(vertex_id_to).getBaseFrameId());

  const Eigen::Vector3d p_M_fi =
      mission_baseframe_to.transformPointInGlobalFrameToMissionFrame(p_G_fi);
  const Eigen::Vector3d p_I_fi = T_I_M * p_M_fi;

  CHECK(!landmark_store_to.hasLandmark(landmark_id));
  const vi_map::Landmark& landmark =
      landmark_store_from.getLandmark(landmark_id);
  landmark_store_to.addLandmark(landmark);

  vi_map::Landmark& new_landmark = landmark_store_to.getLandmark(landmark_id);
  new_landmark.set_p_B(pose::Position3D(p_I_fi));

  updateLandmarkIndexReference(landmark_id, vertex_id_to);
  landmark_store_from.removeLandmark(landmark_id);
}

unsigned int VIMap::getVertexCountInMission(
    const vi_map::MissionId& mission_id) const {
  CHECK(hasMission(mission_id));

  const vi_map::VIMission& mission = getMission(mission_id);
  pose_graph::VertexId current_vertex_id = mission.getRootVertexId();

  unsigned int vertex_count = 0;
  do {
    ++vertex_count;
  } while (getNextVertex(
      current_vertex_id, getGraphTraversalEdgeType(mission_id),
      &current_vertex_id));
  return vertex_count;
}

void VIMap::getVertexIdsByMission(
    vi_map::MissionVertexIdList* mission_to_vertex_ids_map) const {
  CHECK_NOTNULL(mission_to_vertex_ids_map);
  mission_to_vertex_ids_map->clear();
  MissionIdList mission_ids;
  getAllMissionIds(&mission_ids);
  for (const MissionId mission_id : mission_ids) {
    pose_graph::VertexIdList vertex_ids;
    getAllVertexIdsInMission(mission_id, &vertex_ids);
    (*mission_to_vertex_ids_map)[mission_id] = vertex_ids;
  }
}

void VIMap::getAllVertexIdsInMission(
    const vi_map::MissionId& mission_id,
    pose_graph::VertexIdList* vertices) const {
  getAllVertexIdsInMissionAlongGraph(mission_id, vertices);
}

void VIMap::getAllVertexIdsInMissionAlongGraph(
    const vi_map::MissionId& mission_id,
    pose_graph::VertexIdList* vertices) const {
  CHECK_NOTNULL(vertices);
  CHECK(hasMission(mission_id));
  vertices->clear();

  const vi_map::VIMission& mission = getMission(mission_id);
  pose_graph::VertexId current_vertex_id = mission.getRootVertexId();

  // Early exit if the root vertex hasn't been set yet.
  if (!current_vertex_id.isValid()) {
    return;
  }

  do {
    vertices->push_back(current_vertex_id);
  } while (getNextVertex(
      current_vertex_id, getGraphTraversalEdgeType(mission_id),
      &current_vertex_id));
}

void VIMap::getAllVertexIdsAlongGraphsSortedByTimestamp(
    pose_graph::VertexIdList* vertices) const {
  CHECK_NOTNULL(vertices)->clear();
  vertices->reserve(numVertices());
  MissionIdList all_mission_ids;
  getAllMissionIdsSortedByTimestamp(&all_mission_ids);
  CHECK(!all_mission_ids.empty());
  for (const vi_map::MissionId& mission_id : all_mission_ids) {
    pose_graph::VertexIdList mission_all_vertex_ids;
    getAllVertexIdsInMissionAlongGraph(mission_id, &mission_all_vertex_ids);
    vertices->insert(
        vertices->end(), mission_all_vertex_ids.cbegin(),
        mission_all_vertex_ids.cend());
  }
}

void VIMap::getAllEdgeIdsInMissionAlongGraph(
    const vi_map::MissionId& mission_id, pose_graph::EdgeIdList* edges) const {
  CHECK_NOTNULL(edges);
  CHECK(hasMission(mission_id))
      << "Mission " << mission_id.hexString() << " does not exist";
  edges->clear();

  const vi_map::VIMission& mission = getMission(mission_id);
  pose_graph::VertexId current_vertex_id = mission.getRootVertexId();

  // Early exit if the root vertex hasn't been set yet.
  if (!current_vertex_id.isValid()) {
    return;
  }

  do {
    pose_graph::EdgeIdSet outgoing_edges;
    getVertex(current_vertex_id).getOutgoingEdges(&outgoing_edges);
    edges->insert(edges->end(), outgoing_edges.begin(), outgoing_edges.end());
  } while (getNextVertex(
      current_vertex_id, getGraphTraversalEdgeType(mission_id),
      &current_vertex_id));
}

void VIMap::getAllEdgeIdsInMissionAlongGraph(
    const vi_map::MissionId& mission_id, pose_graph::Edge::EdgeType edge_type,
    pose_graph::EdgeIdList* edges) const {
  CHECK_NOTNULL(edges);
  CHECK(hasMission(mission_id))
      << "Mission " << mission_id.hexString() << " does not exist";
  edges->clear();

  const vi_map::VIMission& mission = getMission(mission_id);
  pose_graph::VertexId current_vertex_id = mission.getRootVertexId();

  // Early exit if the root vertex hasn't been set yet.
  if (!current_vertex_id.isValid()) {
    return;
  }

  std::function<bool(pose_graph::EdgeId&)> is_edge_type_different =  // NOLINT
      [&](const pose_graph::EdgeId& edge_id) {
        return getEdgeType(edge_id) != edge_type;
      };

  do {
    pose_graph::EdgeIdSet outgoing_edges;
    getVertex(current_vertex_id).getOutgoingEdges(&outgoing_edges);
    edges->insert(edges->end(), outgoing_edges.begin(), outgoing_edges.end());
    edges->erase(
        std::remove_if(edges->begin(), edges->end(), is_edge_type_different),
        edges->end());
  } while (getNextVertex(
      current_vertex_id, getGraphTraversalEdgeType(mission_id),
      &current_vertex_id));
}

bool VIMap::hasEdgesOfType(pose_graph::Edge::EdgeType edge_type) const {
  pose_graph::EdgeIdList edges;
  getAllEdgeIds(&edges);

  for (const pose_graph::EdgeId edge_id : edges) {
    if (getEdgeType(edge_id) == edge_type) {
      return true;
    }
  }
  return false;
}

void VIMap::getAllLandmarkIdsInMission(
    const MissionId& mission_id, LandmarkIdList* landmarks) const {
  CHECK_NOTNULL(landmarks)->clear();
  CHECK(hasMission(mission_id));

  pose_graph::VertexIdList mission_vertex_ids;
  getAllVertexIdsInMission(mission_id, &mission_vertex_ids);

  for (const pose_graph::VertexId& vertex_id : mission_vertex_ids) {
    LandmarkIdList vertex_landmarks;
    const LandmarkStore& store = getVertex(vertex_id).getLandmarks();
    for (const Landmark& landmark : store) {
      landmarks->push_back(landmark.id());
    }
  }
}

void VIMap::getOutgoingOfType(
    const pose_graph::Edge::EdgeType ref_edge_type,
    const pose_graph::VertexId& current_vertex_id,
    pose_graph::EdgeIdList* edge_ids) const {
  CHECK(hasVertex(current_vertex_id));
  CHECK_NOTNULL(edge_ids);
  edge_ids->clear();

  pose_graph::EdgeIdSet outgoing_edges;
  getVertex(current_vertex_id).getOutgoingEdges(&outgoing_edges);
  for (const pose_graph::EdgeId& edge_id : outgoing_edges) {
    pose_graph::Edge::EdgeType edge_type =
        getEdgePtrAs<vi_map::Edge>(edge_id)->getType();
    if (edge_type == ref_edge_type) {
      edge_ids->push_back(edge_id);
    }
  }
}

void VIMap::getIncomingOfType(
    const pose_graph::Edge::EdgeType ref_edge_type,
    const pose_graph::VertexId& current_vertex_id,
    pose_graph::EdgeIdList* edge_ids) const {
  CHECK(hasVertex(current_vertex_id));
  CHECK_NOTNULL(edge_ids);
  edge_ids->clear();

  pose_graph::EdgeIdSet incoming_edges;
  getVertex(current_vertex_id).getIncomingEdges(&incoming_edges);
  for (const pose_graph::EdgeId& edge_id : incoming_edges) {
    if (getEdgeType(edge_id) == ref_edge_type) {
      edge_ids->push_back(edge_id);
    }
  }
}

void VIMap::getRandomVertexId(pose_graph::VertexId* vertex_id) const {
  CHECK_NOTNULL(vertex_id);

  pose_graph::VertexIdList vertices;
  getAllVertexIds(&vertices);
  CHECK(!vertices.empty());
  std::uniform_int_distribution<int> distribution(
      0, static_cast<int>(vertices.size() - 1));
  int rand_int = distribution(generator_);
  *vertex_id = vertices[rand_int];
}

void VIMap::getRandomVertexIdInMission(
    const vi_map::MissionId& mission_id,
    pose_graph::VertexId* vertex_id) const {
  CHECK(hasMission(mission_id));
  CHECK_NOTNULL(vertex_id);

  pose_graph::VertexIdList vertices;
  getAllVertexIdsInMissionAlongGraph(mission_id, &vertices);
  CHECK(!vertices.empty());
  std::uniform_int_distribution<int> distribution(
      0, static_cast<int>(vertices.size() - 1));
  int rand_int = distribution(generator_);
  *vertex_id = vertices[rand_int];
}

pose_graph::VertexId VIMap::getLastVertexIdOfMission(
    const vi_map::MissionId& mission_id) const {
  CHECK(mission_id.isValid());

  pose_graph::VertexIdList vertices;
  getAllVertexIdsInMissionAlongGraph(mission_id, &vertices);
  CHECK(!vertices.empty());
  return vertices.back();
}

void VIMap::setRandIntGeneratorSeed(int seed) {
  generator_.seed(seed);
}

void VIMap::selectMissions(
    const vi_map::MissionIdSet& selected_missions_ids) const {
  selected_missions_ = selected_missions_ids;
}

void VIMap::deselectMission(const vi_map::MissionId& mission_id) const {
  selected_missions_.erase(mission_id);
}

void VIMap::resetMissionSelection() const {
  selected_missions_.clear();
}

unsigned int VIMap::numSelectedMissions() const {
  return selected_missions_.size();
}

const vi_map::MissionIdSet& VIMap::getSelectedMissions() const {
  return selected_missions_;
}

void VIMap::forEachMission(
    const std::function<void(const MissionId&)>& action) const {
  MissionIdList all_mission_ids;
  getAllMissionIds(&all_mission_ids);
  for (const MissionId& id : all_mission_ids) {
    action(id);
  }
}

bool VIMap::checkResourceConsistency() const {
  bool consistent = true;
  pose_graph::VertexIdList vertices;
  getAllVertexIds(&vertices);
  std::unordered_set<backend::ResourceId> unique_resources;
  for (const pose_graph::VertexId& vertex_id : vertices) {
    const vi_map::Vertex& vertex = getVertex(vertex_id);
    const vi_map::Vertex::FrameResourceMap& vertex_resource_info =
        vertex.getFrameResourceMap();
    for (unsigned int frame_idx = 0u; frame_idx < vertex.numFrames();
         ++frame_idx) {
      if (vertex.isVisualFrameSet(frame_idx)) {
        const backend::ResourceTypeToIdsMap& frame_resource_info =
            vertex_resource_info[frame_idx];
        for (const backend::ResourceTypeToIdsMap::value_type&
                 type_to_resource_ids : frame_resource_info) {
          for (const backend::ResourceId& resource_id :
               type_to_resource_ids.second) {
            const std::unordered_set<backend::ResourceId>::const_iterator it =
                unique_resources.find(resource_id);
            if (it != unique_resources.end()) {
              LOG(ERROR) << "Resource " << resource_id << " is not unique!";
              consistent = false;
            } else {
              unique_resources.insert(resource_id);
            }
            backend::ResourceType type = type_to_resource_ids.first;
            switch (type) {
              case backend::ResourceType::kRawImage:
              case backend::ResourceType::kUndistortedImage:
              case backend::ResourceType::kRectifiedImage:
              case backend::ResourceType::kImageForDepthMap:
              case backend::ResourceType::kRawColorImage:
              case backend::ResourceType::kUndistortedColorImage:
              case backend::ResourceType::kRectifiedColorImage:
              case backend::ResourceType::kColorImageForDepthMap:
              case backend::ResourceType::kRawDepthMap:
              case backend::ResourceType::kOptimizedDepthMap:
              case backend::ResourceType::kDisparityMap:
                if (!checkResource<cv::Mat>(resource_id, type)) {
                  LOG(ERROR) << "Resource " << resource_id
                             << " is in an inconsistent state!";
                  consistent = false;
                }
                break;
              case backend::ResourceType::kPointCloudXYZ:
                if (!checkResource<resources::PointCloud>(resource_id, type)) {
                  LOG(ERROR) << "Resource " << resource_id
                             << " is in an inconsistent state!";
                  consistent = false;
                }
                break;
              case backend::ResourceType::kPointCloudXYZRGBN:
                if (!checkResource<resources::PointCloud>(resource_id, type)) {
                  LOG(ERROR) << "Resource " << resource_id
                             << " is in an inconsistent state!";
                  consistent = false;
                }
                break;
              default:
                LOG(FATAL) << "Unknown frame resource type: "
                           << static_cast<int>(type);
            }
          }
        }
      }
    }
  }

  vi_map::MissionIdList mission_ids;
  getAllMissionIds(&mission_ids);
  for (const vi_map::MissionId& mission_id : mission_ids) {
    const vi_map::VIMission& mission = getMission(mission_id);
    backend::ResourceIdSet tsdf_resources;
    mission.getAllResourceIds(
        backend::ResourceType::kTsdfGridPath, &tsdf_resources);
    for (const backend::ResourceId& resource_id : tsdf_resources) {
      consistent &= checkResource<std::string>(
          resource_id, backend::ResourceType::kTsdfGridPath);
    }
    backend::ResourceIdSet occupancy_resources;
    mission.getAllResourceIds(
        backend::ResourceType::kOccupancyGridPath, &occupancy_resources);
    for (const backend::ResourceId& resource_id : occupancy_resources) {
      consistent &= checkResource<std::string>(
          resource_id, backend::ResourceType::kOccupancyGridPath);
    }
    backend::ResourceIdSet reconstruction_resources;
    mission.getAllResourceIds(
        backend::ResourceType::kPmvsReconstructionPath,
        &reconstruction_resources);
    for (const backend::ResourceId& resource_id : reconstruction_resources) {
      consistent &= checkResource<std::string>(
          resource_id, backend::ResourceType::kPmvsReconstructionPath);
    }
  }

  return consistent;
}

bool VIMap::hasMissionResource(
    const backend::ResourceType& resource_type,
    const MissionIdList& involved_mission_ids) const {
  std::lock_guard<std::recursive_mutex> lock(resource_mutex_);
  backend::ResourceId resource_id;
  return getResourceIdForMissions(
      resource_type, involved_mission_ids, &resource_id);
}

bool VIMap::getResourceIdForMissions(
    const backend::ResourceType& resource_type,
    const MissionIdList& involved_mission_ids,
    backend::ResourceId* resource_id) const {
  CHECK_NOTNULL(resource_id);
  CHECK(!involved_mission_ids.empty());

  typedef std::unordered_map<backend::ResourceId, MissionIdList>
      ResourceToMissionsMap;
  ResourceToMissionsMap resource_to_mission_map;
  getResourceIdToMissionsMap(resource_type, &resource_to_mission_map);

  for (ResourceToMissionsMap::value_type& resource : resource_to_mission_map) {
    VLOG(1) << "Found resource for " << resource.second.size() << " missions.";
    if (resource.second.size() == involved_mission_ids.size()) {
      std::sort(resource.second.begin(), resource.second.end());
      MissionIdList involved_mission_ids_sorted = involved_mission_ids;
      std::sort(
          involved_mission_ids_sorted.begin(),
          involved_mission_ids_sorted.end());
      bool equal = true;
      for (unsigned int i = 0; i < involved_mission_ids_sorted.size(); ++i) {
        equal &= (resource.second[i] == involved_mission_ids_sorted[i]);
      }
      if (equal) {
        *resource_id = resource.first;
        return true;
      }
    }
  }
  return false;
}

void VIMap::getResourceIdToMissionsMap(
    const backend::ResourceType& resource_type,
    std::unordered_map<backend::ResourceId, MissionIdList>*
        resource_to_mission_map) const {
  CHECK_NOTNULL(resource_to_mission_map)->clear();

  MissionIdList all_mission_ids;
  getAllMissionIds(&all_mission_ids);
  for (const MissionId& mission_id : all_mission_ids) {
    backend::ResourceIdSet resource_ids;
    getMission(mission_id).getAllResourceIds(resource_type, &resource_ids);
    for (const backend::ResourceId& resource_id : resource_ids) {
      (*resource_to_mission_map)[resource_id].push_back(mission_id);
    }
  }
}

void VIMap::addResourceIdForMissions(
    const backend::ResourceType& resource_type,
    const MissionIdList& involved_mission_ids,
    const backend::ResourceId& resource_id) {
  // Search for an existing mission set resource first.
  CHECK(!hasMissionResource(resource_type, involved_mission_ids))
      << "There is already a resource of type "
      << static_cast<unsigned int>(resource_type) << " for these missions!";

  for (const MissionId& mission_id : involved_mission_ids) {
    getMission(mission_id).addResourceId(resource_id, resource_type);
  }
}

void VIMap::deleteResourceIdForMissions(
    const backend::ResourceType& resource_type,
    const MissionIdList& involved_mission_ids,
    const backend::ResourceId& resource_id) {
  // Search for an existing path resource first.
  CHECK(hasMissionResource(resource_type, involved_mission_ids))
      << "There is no path resource of type "
      << static_cast<unsigned int>(resource_type) << " for these missions!";

  for (const MissionId& mission_id : involved_mission_ids) {
    getMission(mission_id).deleteResourceId(resource_id, resource_type);
  }
}

// NOTE: [ADD_RESOURCE_TYPE] [ADD_RESOURCE_DATA_TYPE] Add a switch case if the
// resource is a frame resource.
void VIMap::deleteAllFrameResources(
    const unsigned int frame_idx, Vertex* vertex_ptr) {
  CHECK_NOTNULL(vertex_ptr);
  CHECK_LT(static_cast<unsigned int>(frame_idx), vertex_ptr->numFrames());

  using backend::ResourceType;
  const backend::ResourceTypeToIdsMap& resource_map =
      vertex_ptr->getFrameResourceMap()[frame_idx];
  for (const backend::ResourceTypeToIdsMap::value_type& resource_ids_per_type :
       resource_map) {
    const ResourceType& type = resource_ids_per_type.first;
    for (const backend::ResourceId& resource_id :
         resource_ids_per_type.second) {
      switch (type) {
        case ResourceType::kRawImage:
        case ResourceType::kUndistortedImage:
        case ResourceType::kRectifiedImage:
        case ResourceType::kImageForDepthMap:
        case ResourceType::kRawColorImage:
        case ResourceType::kUndistortedColorImage:
        case ResourceType::kRectifiedColorImage:
        case ResourceType::kColorImageForDepthMap:
        case ResourceType::kRawDepthMap:
        case ResourceType::kOptimizedDepthMap:
        case ResourceType::kDisparityMap:
          deleteResource<cv::Mat>(resource_id, type);
          break;
        default:
          LOG(FATAL) << "Unsupported frame resource data type: "
                     << backend::ResourceTypeNames[static_cast<int>(type)]
                     << "! Implement a case here.";
      }
    }
  }
  vertex_ptr->deleteAllFrameResourceInfo();
}

std::string VIMap::getSubFolderName() {
  return serialization::getSubFolderName();
}

bool VIMap::getListOfExistingMapFiles(
    const std::string& map_folder, std::string* sensors_filepath,
    std::vector<std::string>* list_of_resource_files,
    std::vector<std::string>* list_of_map_proto_files) {
  return serialization::getListOfExistingMapFiles(
      map_folder, sensors_filepath, list_of_resource_files,
      list_of_map_proto_files);
}

bool VIMap::hasMapOnFileSystem(const std::string& map_folder) {
  return serialization::hasMapOnFileSystem(map_folder);
}

bool VIMap::loadFromFolder(const std::string& map_folder) {
  return serialization::loadMapFromFolder(map_folder, this);
}

bool VIMap::loadFromFolderDeprecated(const std::string& map_folder) {
  return serialization_deprecated::loadMapFromFolder(map_folder, this);
}

bool VIMap::saveToFolder(
    const std::string& folder_path, const backend::SaveConfig& config) {
  return serialization::saveMapToFolder(folder_path, config, this);
}

bool VIMap::saveToMapFolder(const backend::SaveConfig& config) {
  return saveToFolder(getMapFolder(), config);
}

bool VIMap::hasOptionalCameraResource(
    const VIMission& mission, const backend::ResourceType& type,
    const aslam::CameraId& camera_id, const int64_t timestamp_ns) const {
  return mission.hasOptionalCameraResourceId(type, camera_id, timestamp_ns);
}

bool VIMap::findAllCloseOptionalCameraResources(
    const VIMission& mission, const backend::ResourceType& type,
    const int64_t timestamp_ns, const int64_t tolerance_ns,
    aslam::CameraIdList* camera_ids,
    std::vector<int64_t>* closest_timestamps_ns) const {
  CHECK_NOTNULL(camera_ids);
  CHECK_NOTNULL(closest_timestamps_ns);
  return mission.findAllCloseOptionalCameraResources(
      type, timestamp_ns, tolerance_ns, camera_ids, closest_timestamps_ns);
}

OptionalSensorData& VIMap::getOptionalSensorData(const MissionId& mission_id) {
  CHECK(hasMission(mission_id));
  return common::getChecked(optional_sensor_data_map_, mission_id);
}

const OptionalSensorData& VIMap::getOptionalSensorData(
    const MissionId& mission_id) const {
  CHECK(hasMission(mission_id));
  return common::getChecked(optional_sensor_data_map_, mission_id);
}

bool VIMap::hasOptionalSensorData(const MissionId& mission_id) const {
  CHECK(hasMission(mission_id));
  return optional_sensor_data_map_.count(mission_id) > 0u;
}

}  // namespace vi_map
