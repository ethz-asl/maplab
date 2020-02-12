#include "vi-map/vi-map.h"

#include <limits>
#include <queue>

#include <aslam/common/memory.h>
#include <aslam/common/time.h>
#include <map-resources/resource_metadata.pb.h>
#include <maplab-common/file-system-tools.h>

#include "vi-map/sensor-manager.h"
#include "vi-map/sensor-utils.h"
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
  CHECK(mergeAllMissionsFromMapWithoutResources(other));
  ResourceMap::deepCopy(other);
  CHECK(checkMapConsistency(other));
}

bool VIMap::mergeAllMissionsFromMapWithoutResources(
    const vi_map::VIMap& other) {
  // Get all missions from old map and add them into the new map.
  vi_map::MissionIdList other_mission_ids;
  other.getAllMissionIds(&other_mission_ids);
  vi_map::MissionBaseFrameIdList mission_base_frame_ids;
  other.getAllMissionBaseFrameIds(&mission_base_frame_ids);

  for (const vi_map::MissionId& other_mission_id : other_mission_ids) {
    CHECK(other_mission_id.isValid());

    if (hasMission(other_mission_id)) {
      LOG(ERROR) << "Cannot merge these maps, because mission "
                 << other_mission_id << " is present in both maps!";
      return false;
    }

    const vi_map::VIMission& other_mission = other.getMission(other_mission_id);
    const vi_map::MissionBaseFrame& original_mission_base_frame =
        other.getMissionBaseFrameForMission(other_mission_id);

    addNewMissionWithBaseframe(
        aligned_unique<VIMission>(other_mission), original_mission_base_frame);

    // This will add copies of all sensors of the other sensor manager to this
    // one.
    sensor_manager_.merge(other.getSensorManager());

    vi_map::Mission& copied_mission = getMission(other_mission_id);
    copied_mission.setRootVertexId(other_mission.getRootVertexId());
  }

  // Get all vertices and add them into the new map.
  vi_map::MissionVertexIdList mission_to_vertex_ids_map;
  other.getVertexIdsByMission(&mission_to_vertex_ids_map);

  for (const std::pair<const MissionId, pose_graph::VertexIdList>
           mission_to_vertices : mission_to_vertex_ids_map) {
    const VIMission& other_mission =
        other.getMission(mission_to_vertices.first);

    // If it is a mission with ncamera, the vertices need to have VisualNFrames,
    // so we need to copy them properly by pointing to the camera in the current
    // sensor manager.
    if (other_mission.hasNCamera()) {
      aslam::NCamera::Ptr ncamera_ptr =
          sensor_manager_.getSensorPtr<aslam::NCamera>(
              other_mission.getNCameraId());
      CHECK(ncamera_ptr);
      for (const pose_graph::VertexId& vertex_id : mission_to_vertices.second) {
        const vi_map::Vertex& original_vertex = other.getVertex(vertex_id);

        // We could probably relax this check in the future, but for now we
        // don't have a use case for this so let's keep it.
        CHECK(original_vertex.hasVisualNFrame())
            << "Inconsistent state: Mission " << mission_to_vertices.first
            << " has an NCamera, but vertex " << vertex_id
            << " does not have a VisualNFrame!";

        vi_map::Vertex::UniquePtr copied_vertex(
            original_vertex.cloneWithVisualNFrame(ncamera_ptr));
        addVertex(std::move(copied_vertex));
      }
      // If the mission has no NCamera, the vertices can't have VisualNFrames!
    } else {
      for (const pose_graph::VertexId& vertex_id : mission_to_vertices.second) {
        const vi_map::Vertex& original_vertex = other.getVertex(vertex_id);

        CHECK(!original_vertex.hasVisualNFrame())
            << "Inconsistent state: Mission " << mission_to_vertices.first
            << " has no NCamera, but vertex " << vertex_id
            << " has a VisualNFrame!";

        vi_map::Vertex::UniquePtr copied_vertex(
            original_vertex.cloneWithoutVisualNFrame());
        addVertex(std::move(copied_vertex));
      }
    }
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

  return true;
}

bool VIMap::mergeAllMissionsFromMap(const vi_map::VIMap& other) {
  VLOG(1) << "Merging from VI-Map.";
  if (!mergeAllMissionsFromMapWithoutResources(other)) {
    return false;
  }

  VLOG(1) << "Copying metadata and resource infos.";
  ResourceMap::mergeFromMap(other);

  CHECK(checkMapConsistency(*this));
  return true;
}

bool VIMap::mergeAllSubmapsFromMap(const vi_map::VIMap& submap) {
  VLOG(1) << "Merging submaps into base map.";
  if (!mergeAllSubmapsFromMapWithoutResources(submap)) {
    return false;
  }

  VLOG(1) << "Copying metadata and resource infos.";
  ResourceMap::mergeFromMap(submap);
  return true;
}

void VIMap::swap(VIMap* other) {
  CHECK_NOTNULL(other);
  posegraph.swap(&other->posegraph);
  missions.swap(other->missions);
  mission_base_frames.swap(other->mission_base_frames);
  landmark_index.swap(&other->landmark_index);
  sensor_manager_.swap(&other->sensor_manager_);
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
  getObserverMissionsForLandmark(object_id, mission_ids);
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
    case Mission::BackBone::kWheelOdometry:
      return pose_graph::Edge::EdgeType::kWheelOdometry;
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

bool VIMap::getEarliestMissionStartTimeNs(int64_t* start_time_ns) const {
  CHECK(start_time_ns);

  vi_map::MissionIdList sorted_missions;
  getAllMissionIdsSortedByTimestamp(&sorted_missions);
  if (!sorted_missions.empty()) {
    const vi_map::VIMission& first_mission = getMission(sorted_missions[0]);
    const pose_graph::VertexId& starting_vertex_id =
        first_mission.getRootVertexId();
    if (starting_vertex_id.isValid()) {
      *start_time_ns =
          getVertex(starting_vertex_id).getMinTimestampNanoseconds();
      return true;
    }
  }
  return false;
}

// TODO(mfehr): split into visual stuff and rest.
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

  const aslam::NCameraId& ncamera_id = getMission(mission_id).getNCameraId();
  CHECK(ncamera_id.isValid());
  const aslam::NCamera& ncamera =
      sensor_manager_.getSensor<aslam::NCamera>(ncamera_id);

  *num_observations = 0u;
  *num_vertices = 0u;
  *num_landmarks = 0u;

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

    *start_time_ns = first_vertex.getMinTimestampNanoseconds();
    *end_time_ns = last_vertex.getMinTimestampNanoseconds();
    *duration_s =
        aslam::time::nanoSecondsToSeconds(*end_time_ns - *start_time_ns);
  }
}

void VIMap::printNumSensorResourcesOfMission(
    const VIMission& mission,
    const std::function<void(const std::string&, const std::string&, int)>&
        print_aligned_function) const {
  assert(
      static_cast<int>(backend::ResourceType::kCount) ==
      static_cast<int>(backend::ResourceTypeNames.size()));
  print_aligned_function("Sensor Resources:", "", 1);
  for (int resource_idx = 0;
       resource_idx < static_cast<int>(backend::ResourceType::kCount);
       ++resource_idx) {
    const std::string kResourceName = backend::ResourceTypeNames[resource_idx];
    CHECK(!kResourceName.empty());
    const typename std::unordered_map<
        aslam::SensorId, backend::TemporalResourceIdBuffer>*
        sensor_resources_map = mission.getAllSensorResourceIdsOfType(
            static_cast<backend::ResourceType>(resource_idx));

    if (sensor_resources_map != nullptr) {
      for (const typename std::unordered_map<
               aslam::SensorId, backend::TemporalResourceIdBuffer>::value_type&
               sensor_id_resource_ids_pair : *sensor_resources_map) {
        const aslam::SensorId& sensor_id = sensor_id_resource_ids_pair.first;
        CHECK(sensor_id.isValid());
        const size_t num_measurements =
            sensor_id_resource_ids_pair.second.size();

        print_aligned_function(
            " - " + kResourceName + " (Sensor " + sensor_id.shortHex() +
                "...): ",
            std::to_string(num_measurements), 1);
      }
    }
  }
}

std::string VIMap::printMapStatistics(
    const vi_map::MissionId& mission_id,
    const unsigned int mission_number) const {
  std::stringstream stats_text;
  stats_text << std::endl;

  static constexpr int kMaxLength = 20;
  const std::function<void(const std::string&, const std::string&, int)>
      print_aligned =
          [&stats_text](
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

  const vi_map::VIMission& mission = getMission(mission_id);

  // Determine primary backbone type.
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

  // Start printing mission.
  print_aligned(
      "Mission " + std::to_string(mission_number) + ":",
      "\t" + mission.id().hexString() + "\t" + mission_type, 0);

  // Print visual information if present.
  if (mission.hasNCamera()) {
    aslam::SensorId ncamera_id = mission.getNCameraId();
    const aslam::NCamera& ncamera =
        sensor_manager_.getSensor<aslam::NCamera>(ncamera_id);
    const size_t num_cameras = ncamera.numCameras();

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
        mission_id, &num_good_landmarks_per_camera,
        &num_bad_landmarks_per_camera, &num_unknown_landmarks_per_camera,
        &total_num_landmarks_per_camera, &num_landmarks, &num_vertices,
        &num_observations, &duration_s, &start_time_ns, &end_time_ns);

    CHECK_EQ(num_good_landmarks_per_camera.size(), num_cameras);
    CHECK_EQ(num_bad_landmarks_per_camera.size(), num_cameras);
    CHECK_EQ(num_unknown_landmarks_per_camera.size(), num_cameras);
    CHECK_EQ(total_num_landmarks_per_camera.size(), num_cameras);

    stats_text << std::endl;
    print_aligned("NCamera Sensor: ", ncamera_id.hexString(), 1);

    print_aligned(" - Landmarks: ", std::to_string(num_landmarks), 1);
    print_aligned(" - Observations:", std::to_string(num_observations), 1);
    print_aligned(" - Landmarks by first observer backlink:", "", 1);
    for (size_t camera_idx = 0u; camera_idx < num_cameras; ++camera_idx) {
      print_aligned(
          "   - Camera " + std::to_string(camera_idx) + ":",
          std::to_string(total_num_landmarks_per_camera[camera_idx]) + " (g:" +
              std::to_string(num_good_landmarks_per_camera[camera_idx]) +
              " b:" + std::to_string(num_bad_landmarks_per_camera[camera_idx]) +
              " u:" +
              std::to_string(num_unknown_landmarks_per_camera[camera_idx]) +
              ")",
          1);
    }
  }

  // Count number of edges
  size_t num_imu_edges = 0u;
  size_t num_loop_closure_edges = 0u;
  size_t num_wheel_odometry_edges = 0u;
  size_t num_absolute_6dof_constraints = 0u;
  pose_graph::EdgeIdList edge_ids;
  getAllEdgeIdsInMissionAlongGraph(mission_id, &edge_ids);
  for (const pose_graph::EdgeId& edge_id : edge_ids) {
    CHECK(edge_id.isValid());
    switch (getEdgeType(edge_id)) {
      case pose_graph::Edge::EdgeType::kWheelOdometry:
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
  num_absolute_6dof_constraints =
      getNumAbsolute6DoFMeasurementsInMission(mission_id);

  // Print imu sensor/constraint if present.
  if (mission.hasImu()) {
    aslam::SensorId imu_id = mission.getImuId();
    // const Imu& imu = sensor_manager_.getSensor<Imu>(imu_id);
    stats_text << std::endl;
    print_aligned("IMU Sensor: ", imu_id.hexString(), 1);

    print_aligned(" - IMU Edges: ", std::to_string(num_imu_edges), 1);
  }

  // Print loop closure sensor/constraint info if present.
  if (mission.hasLoopClosureSensor()) {
    aslam::SensorId loop_closure_sensor_id = mission.getLoopClosureSensor();
    // const LoopClosure& loop_closure =
    //     sensor_manager_.getSensor<LoopClosure>(loop_closure_sensor_id);
    stats_text << std::endl;
    print_aligned(
        "Loop Closure Sensor: ", loop_closure_sensor_id.hexString(), 1);

    print_aligned(
        " - Loop Closure Edges: ", std::to_string(num_loop_closure_edges), 1);
  }

  // Print wheel odometry sensor/constraint info if present.
  if (mission.hasWheelOdometrySensor()) {
    aslam::SensorId wheel_odometry_sensor_id = mission.getWheelOdometrySensor();
    // const WheelOdometry& wheel_odometry =
    //     sensor_manager_.getSensor<WheelOdometry>(wheel_odometry_sensor_id);
    stats_text << std::endl;
    print_aligned(
        "Wheel odometry Sensor: ", wheel_odometry_sensor_id.hexString(), 1);

    print_aligned(
        " - Wheel odometry Edges: ", std::to_string(num_wheel_odometry_edges),
        1);
  }

  // Print absolute 6dof sensor/constraint info if present.
  if (mission.hasAbsolute6DoFSensor()) {
    aslam::SensorId absolute_6dof_sensor_id = mission.getAbsolute6DoFSensor();
    // const Absolute6DoF& absolute_6dof =
    //     sensor_manager_.getSensor<Absolute6DoF>(absolute_6dof_sensor_id);
    stats_text << std::endl;
    print_aligned(
        "Absolute-6DoF Sensor: ", absolute_6dof_sensor_id.hexString(), 1);

    print_aligned(
        " - Absolute-6DoF Constraints: ",
        std::to_string(num_absolute_6dof_constraints), 1);
  }

  // Print odometry 6dof sensor/constraint info if present
  if (mission.hasOdometry6DoFSensor()) {
    aslam::SensorId odometry_6dof_id = mission.getOdometry6DoFSensor();
    // const Odometry6DoF& odometry_6dof =
    //     sensor_manager_.getSensor<Odometry6DoF>(odometry_6dof_id);
    stats_text << std::endl;
    print_aligned("Odometry-6DoF Sensor: ", odometry_6dof_id.hexString(), 1);

    // TODO(mfehr): ADD ALL Odometry.6DoF-P. Sensor SPECIFIC STUFF
  }

  // Print Lidar sensor/constraint info if present
  if (mission.hasLidar()) {
    aslam::SensorId lidar_id = mission.getLidarId();
    // const Lidar& lidar = sensor_manager_.getSensor<Lidar>(lidar_id);
    stats_text << std::endl;
    print_aligned("Lidar Sensor: ", lidar_id.hexString(), 1);

    // TODO(mfehr): ADD ALL LIDAR Sensor SPECIFIC STUFF
  }

  stats_text << std::endl;
  print_aligned("General:", "", 1);
  const size_t num_vertices = numVerticesInMission(mission_id);
  if (num_vertices == 0) {
    print_aligned(" - Mission has no vertices!", "", 1);
  } else {
    print_aligned(" - Vertices:", std::to_string(num_vertices), 1);
    print_aligned(
        " - Loop-closure edges: ", std::to_string(num_loop_closure_edges), 1);
    double distance = 0;
    getDistanceTravelledPerMission(mission_id, &distance);
    print_aligned(" - Distance:", std::to_string(distance) + "m", 1);

    // Compute time spend in mission.
    double duration_s = 0.0;
    int64_t start_time_ns = 0;
    int64_t end_time_ns = 0;
    pose_graph::VertexIdList vertex_ids;
    getAllVertexIdsInMissionAlongGraph(mission_id, &vertex_ids);
    if (!vertex_ids.empty()) {
      const vi_map::Vertex& first_vertex = getVertex(vertex_ids.front());
      const vi_map::Vertex& last_vertex = getVertex(vertex_ids.back());

      start_time_ns = first_vertex.getMinTimestampNanoseconds();
      end_time_ns = last_vertex.getMinTimestampNanoseconds();
      duration_s =
          aslam::time::nanoSecondsToSeconds(end_time_ns - start_time_ns);
    }
    const time_t start_time_s(
        static_cast<time_t>(aslam::time::nanoSecondsToSeconds(start_time_ns)));
    const time_t end_time_s(
        static_cast<time_t>(aslam::time::nanoSecondsToSeconds(end_time_ns)));
    const std::string start_time_str =
        common::generateDateString(&start_time_s);
    const std::string end_time_str = common::generateDateString(&end_time_s);
    print_aligned(
        " - Start to end time: ", start_time_str + " to " + end_time_str, 1);
    print_aligned(" - Duration: ", std::to_string(duration_s) + "s", 1);
  }

  const vi_map::MissionBaseFrame& base_frame =
      getMissionBaseFrame(mission.getBaseFrameId());
  print_aligned(
      " - T_G_M", base_frame.is_T_G_M_known() ? "known" : "unknown", 1);

  if (!selected_missions_.empty()) {
    const bool is_selected = (selected_missions_.count(mission_id) > 0);
    print_aligned(" - Is selected:", std::to_string(is_selected), 1);
  }

  stats_text << std::endl;
  printNumSensorResourcesOfMission(mission, print_aligned);

  return stats_text.str();
}

std::string VIMap::printMapStatistics(void) const {
  vi_map::MissionIdList all_missions;
  getAllMissionIdsSortedByTimestamp(&all_missions);

  std::stringstream stats_text;
  stats_text << "Mission statistics: " << std::endl;
  unsigned int mission_number = 0u;
  for (const vi_map::MissionId& mission_id : all_missions) {
    stats_text << printMapStatistics(mission_id, mission_number);
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
                           const std::string& key, const std::string& value,
                           int indent) {
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
      "Visual Landmarks:",
      std::to_string(total_num_landmarks) +
          " (g:" + std::to_string(total_num_good_landmarks) +
          " b:" + std::to_string(total_num_bad_landmarks) +
          " u:" + std::to_string(total_num_unknown_landmarks) + ")",
      1);
  print_aligned(
      "Visual Observations:", std::to_string(total_num_observations), 1);
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
      aslam::createRandomId<vi_map::MissionBaseFrameId>();
  MissionBaseFrame new_mission_base_frame(
      mission_baseframe_id, T_G_M, T_G_M_covariance);

  mission_base_frames.emplace(mission_baseframe_id, new_mission_base_frame);

  // Create and add new mission.
  VIMission* new_mission(
      new VIMission(mission_id, mission_baseframe_id, backbone_type));

  missions.emplace(mission_id, VIMission::UniquePtr(new_mission));

  if (!selected_missions_.empty()) {
    selected_missions_.insert(mission_id);
  }
}

void VIMap::associateMissionSensors(
    const aslam::SensorIdSet& sensor_ids, const vi_map::MissionId& id) {
  VIMission& mission = getMission(id);

  // Cannot use unordered_map with the enum as key, as it is not supported prior
  // to cpp14, hence it will not compile on xenial and older.
  std::unordered_map<uint8_t, std::set<aslam::SensorId>> sensors_of_type;
  for (const aslam::SensorId& sensor_id : sensor_ids) {
    const SensorType sensor_type = sensor_manager_.getSensorType(sensor_id);
    std::set<aslam::SensorId>& sensor_ids =
        sensors_of_type[static_cast<uint8_t>(sensor_type)];
    sensor_ids.insert(sensor_id);
  }

  auto retrieve_unique_sensor_id_of_type =
      [](const std::string& id_flag, const std::string& flag_name,
         const std::string& sensor_name,
         const std::set<aslam::SensorId>& ids_of_type) -> aslam::SensorId {
    aslam::SensorId sensor_id;
    if (ids_of_type.size() > 1u) {
      CHECK(
          !id_flag.empty() && sensor_id.fromHexString(id_flag) &&
          (ids_of_type.count(sensor_id) > 0u))
          << "If more than one " << sensor_name
          << " is provided in the sensor manager, "
          << "use --" << flag_name << " to select which one to use.";
    } else {
      sensor_id = *(ids_of_type.begin());
    }
    CHECK(sensor_id.isValid());
    return sensor_id;
  };

  for (const auto& sensor_of_type : sensors_of_type) {
    CHECK(!sensor_of_type.second.empty());
    if (static_cast<SensorType>(sensor_of_type.first) == SensorType::kNCamera) {
      CHECK(!mission.hasNCamera()) << "There shouldn't be a NCamera sensor "
                                   << "associated yet with this mission!";
      const aslam::SensorId sensor_id = retrieve_unique_sensor_id_of_type(
          FLAGS_selected_ncamera_sensor_id, "selected_ncamera_sensor_id",
          "NCamera", sensor_of_type.second);
      mission.setNCameraId(sensor_id);
    } else if (
        static_cast<SensorType>(sensor_of_type.first) == SensorType::kImu) {
      CHECK(!mission.hasImu()) << "There shouldn't be a IMU sensor associated "
                               << "yet with this mission!";
      const aslam::SensorId sensor_id = retrieve_unique_sensor_id_of_type(
          FLAGS_selected_imu_sensor_id, "selected_imu_sensor_id", "IMU",
          sensor_of_type.second);
      mission.setImuId(sensor_id);
    } else if (
        static_cast<SensorType>(sensor_of_type.first) == SensorType::kLidar) {
      CHECK(!mission.hasLidar()) << "There shouldn't be a Lidar sensor "
                                 << "associated yet with this mission!";
      const aslam::SensorId sensor_id = retrieve_unique_sensor_id_of_type(
          FLAGS_selected_lidar_sensor_id, "selected_lidar_sensor_id", "Lidar",
          sensor_of_type.second);
      mission.setLidarId(sensor_id);
    } else if (
        static_cast<SensorType>(sensor_of_type.first) ==
        SensorType::kOdometry6DoF) {
      CHECK(!mission.hasOdometry6DoFSensor())
          << "There shouldn't be a Odometry6DoF sensor associated yet with "
          << "this mission!";
      const aslam::SensorId sensor_id = retrieve_unique_sensor_id_of_type(
          FLAGS_selected_odometry_6dof_sensor_id,
          "selected_odometry_6dof_sensor_id", "Odometry6DoF",
          sensor_of_type.second);
      mission.setOdometry6DoFSensor(sensor_id);
    } else if (
        static_cast<SensorType>(sensor_of_type.first) ==
        SensorType::kLoopClosureSensor) {
      CHECK(!mission.hasLoopClosureSensor())
          << "Can not handle more than one "
             "loop closure 6DOF sensor per mission";
      const aslam::SensorId sensor_id = retrieve_unique_sensor_id_of_type(
          FLAGS_selected_loop_closure_sensor_id,
          "selected_loop_closure_sensor_id", "LoopClosureSensor",
          sensor_of_type.second);
      mission.setLoopClosureSensor(sensor_id);
    } else if (
        static_cast<SensorType>(sensor_of_type.first) ==
        SensorType::kWheelOdometry) {
      CHECK(!mission.hasWheelOdometrySensor())
          << "Can not handle more than one wheel odometry sensor per mission";
      const aslam::SensorId sensor_id = retrieve_unique_sensor_id_of_type(
          FLAGS_selected_wheel_odometry_sensor_id,
          "selected_wheel_odometry_sensor_id", "WheelOdometrySensor",
          sensor_of_type.second);
      mission.setWheelOdometrySensor(sensor_id);
    } else if (
        static_cast<SensorType>(sensor_of_type.first) ==
        SensorType::kAbsolute6DoF) {
      CHECK(!mission.hasAbsolute6DoFSensor())
          << "There shouldn't be an Absolute6DoF sensor associated yet with "
          << "this mission!";
      const aslam::SensorId sensor_id = retrieve_unique_sensor_id_of_type(
          FLAGS_selected_absolute_6dof_sensor_id,
          "selected_absolute_6dof_sensor_id", "Absolute6DoF",
          sensor_of_type.second);
      mission.setAbsolute6DoFSensor(sensor_id);
    } else if (
        static_cast<SensorType>(sensor_of_type.first) ==
        SensorType::kPointCloudMapSensor) {
      // NOTE: this sensor type does not need to be associated with the VIMap,
      // as it is only used to store sensor resources anyways.
    } else {
      LOG(FATAL)
          << "Trying to associate an unknown sensor type with the VIMap! Type: "
          << sensor_of_type.first;
    }
  }
}

void VIMap::getAllAssociatedMissionSensorIds(
    const vi_map::MissionId& id, aslam::SensorIdSet* sensor_ids) const {
  CHECK_NOTNULL(sensor_ids)->clear();
  CHECK(id.isValid());
  CHECK(hasMission(id));
  const VIMission& mission = getMission(id);

  if (mission.hasNCamera()) {
    sensor_ids->insert(mission.getNCameraId());
  }
  if (mission.hasImu()) {
    sensor_ids->insert(mission.getImuId());
  }
  if (mission.hasLidar()) {
    sensor_ids->insert(mission.getLidarId());
  }
  if (mission.hasOdometry6DoFSensor()) {
    sensor_ids->insert(mission.getOdometry6DoFSensor());
  }
  if (mission.hasLoopClosureSensor()) {
    sensor_ids->insert(mission.getLoopClosureSensor());
  }
  if (mission.hasAbsolute6DoFSensor()) {
    sensor_ids->insert(mission.getAbsolute6DoFSensor());
  }
  if (mission.hasWheelOdometrySensor()) {
    sensor_ids->insert(mission.getWheelOdometrySensor());
  }
}

void VIMap::associateMissionNCamera(
    const aslam::SensorId& ncamera_id, const vi_map::MissionId& id) {
  getMission(id).setNCameraId(ncamera_id);
}

void VIMap::associateMissionImu(
    const aslam::SensorId& imu_id, const vi_map::MissionId& id) {
  getMission(id).setImuId(imu_id);
}

const aslam::NCamera& VIMap::getMissionNCamera(
    const vi_map::MissionId& id) const {
  const VIMission& mission = getMission(id);
  CHECK(mission.hasNCamera());
  return sensor_manager_.getSensor<aslam::NCamera>(mission.getNCameraId());
}

aslam::NCamera::Ptr VIMap::getMissionNCameraPtr(
    const vi_map::MissionId& id) const {
  return sensor_manager_.getSensorPtr<aslam::NCamera>(
      getMission(id).getNCameraId());
}

const vi_map::Imu& VIMap::getMissionImu(const vi_map::MissionId& id) const {
  return sensor_manager_.getSensor<vi_map::Imu>(getMission(id).getImuId());
}

vi_map::Imu::Ptr VIMap::getMissionImuPtr(const vi_map::MissionId& id) const {
  return sensor_manager_.getSensorPtr<vi_map::Imu>(getMission(id).getImuId());
}

const vi_map::Lidar& VIMap::getMissionLidar(const vi_map::MissionId& id) const {
  return sensor_manager_.getSensor<vi_map::Lidar>(getMission(id).getLidarId());
}

vi_map::Lidar::Ptr VIMap::getMissionLidarPtr(
    const vi_map::MissionId& id) const {
  return sensor_manager_.getSensorPtr<vi_map::Lidar>(
      getMission(id).getLidarId());
}

const vi_map::Odometry6DoF& VIMap::getMissionOdometry6DoFSensor(
    const vi_map::MissionId& id) const {
  return sensor_manager_.getSensor<vi_map::Odometry6DoF>(
      getMission(id).getOdometry6DoFSensor());
}

vi_map::Odometry6DoF::Ptr VIMap::getMissionOdometry6DoFSensorPtr(
    const vi_map::MissionId& id) const {
  return sensor_manager_.getSensorPtr<vi_map::Odometry6DoF>(
      getMission(id).getOdometry6DoFSensor());
}

const vi_map::LoopClosureSensor& VIMap::getMissionLoopClosureSensor(
    const vi_map::MissionId& id) const {
  return sensor_manager_.getSensor<vi_map::LoopClosureSensor>(
      getMission(id).getLoopClosureSensor());
}

vi_map::LoopClosureSensor::Ptr VIMap::getMissionLoopClosureSensorPtr(
    const vi_map::MissionId& id) const {
  return sensor_manager_.getSensorPtr<vi_map::LoopClosureSensor>(
      getMission(id).getLoopClosureSensor());
}

const vi_map::Absolute6DoF& VIMap::getMissionAbsolute6DoFSensor(
    const vi_map::MissionId& id) const {
  return sensor_manager_.getSensor<vi_map::Absolute6DoF>(
      getMission(id).getAbsolute6DoFSensor());
}

vi_map::Absolute6DoF::Ptr VIMap::getMissionAbsolute6DoFSensorPtr(
    const vi_map::MissionId& id) const {
  return sensor_manager_.getSensorPtr<vi_map::Absolute6DoF>(
      getMission(id).getAbsolute6DoFSensor());
}

const vi_map::WheelOdometry& VIMap::getMissionWheelOdometrySensor(
    const vi_map::MissionId& id) const {
  return sensor_manager_.getSensor<vi_map::WheelOdometry>(
      getMission(id).getWheelOdometrySensor());
}

vi_map::WheelOdometry::Ptr VIMap::getMissionWheelOdometrySensorPtr(
    const vi_map::MissionId& id) const {
  return sensor_manager_.getSensorPtr<vi_map::WheelOdometry>(
      getMission(id).getWheelOdometrySensor());
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
    const pose_graph::VertexId& vertex_to_merge, bool merge_viwls_edges,
    bool merge_odometry_edges, bool merge_wheel_odometry_edges) {
  CHECK(hasVertex(merge_into_vertex_id));
  CHECK(hasVertex(vertex_to_merge));

  const vi_map::MissionId& mission_id =
      getVertex(merge_into_vertex_id).getMissionId();

  pose_graph::VertexId check_vertex_id;
  getNextVertex(
      merge_into_vertex_id, getGraphTraversalEdgeType(mission_id),
      &check_vertex_id);
  CHECK_EQ(check_vertex_id, vertex_to_merge)
      << "Vertices should be neighboring and vertex_to_merge should be the "
      << "next after merge_into_vertex_id.";

  VLOG(4) << "Merging vertex: " << vertex_to_merge << " into "
          << merge_into_vertex_id;
  // Move landmarks.
  moveLandmarksToOtherVertex(vertex_to_merge, merge_into_vertex_id);

  pose_graph::EdgeIdSet next_vertex_incoming_edge_ids,
      next_vertex_outgoing_edge_ids;
  vi_map::Vertex& next_vertex = getVertex(vertex_to_merge);
  next_vertex.getOutgoingEdges(&next_vertex_outgoing_edge_ids);
  next_vertex.getIncomingEdges(&next_vertex_incoming_edge_ids);
  // Remove all other edges from vertex.
  for (const pose_graph::EdgeId& incoming_edge :
       next_vertex_incoming_edge_ids) {
    const pose_graph::Edge::EdgeType edge_type = getEdgeType(incoming_edge);
    bool keepViwlsEdge =
        edge_type == pose_graph::Edge::EdgeType::kViwls && merge_viwls_edges;
    bool keepOdometryEdge =
        edge_type == pose_graph::Edge::EdgeType::kOdometry &&
        merge_odometry_edges;
    bool keepWheelOdometryEdge =
        edge_type == pose_graph::Edge::EdgeType::kWheelOdometry &&
        merge_wheel_odometry_edges;

    if (!keepViwlsEdge && !keepOdometryEdge && !keepWheelOdometryEdge) {
      posegraph.removeEdge(incoming_edge);
    }
  }
  for (const pose_graph::EdgeId& outgoing_edge :
       next_vertex_outgoing_edge_ids) {
    const pose_graph::Edge::EdgeType edge_type = getEdgeType(outgoing_edge);
    bool keepViwlsEdge =
        edge_type == pose_graph::Edge::EdgeType::kViwls && merge_viwls_edges;
    bool keepOdometryEdge =
        edge_type == pose_graph::Edge::EdgeType::kOdometry &&
        merge_odometry_edges;
    bool keepWheelOdometryEdge =
        edge_type == pose_graph::Edge::EdgeType::kWheelOdometry &&
        merge_wheel_odometry_edges;

    if (!keepViwlsEdge && !keepOdometryEdge && !keepWheelOdometryEdge) {
      posegraph.removeEdge(outgoing_edge);
    }
  }
  aslam::SensorIdSet sensor_id_set;
  sensor_manager_.getAllSensorIds(&sensor_id_set);

  size_t num_outgoing_viwls_edges = 0;
  if (merge_viwls_edges) {
    num_outgoing_viwls_edges =
        mergeEdgesOfNeighboringVertices<ViwlsEdge, Edge::EdgeType::kViwls>(
            merge_into_vertex_id, vertex_to_merge);
  }
  CHECK(
      getMission(mission_id).backboneType() != Mission::BackBone::kViwls ||
      num_outgoing_viwls_edges == 1u);

  size_t num_outgoing_odometry_edges = 0;
  if (merge_odometry_edges) {
    num_outgoing_odometry_edges = mergeEdgesOfNeighboringVertices<
        TransformationEdge, Edge::EdgeType::kOdometry>(
        merge_into_vertex_id, vertex_to_merge);
  }
  CHECK(
      getMission(mission_id).backboneType() != Mission::BackBone::kOdometry ||
      num_outgoing_odometry_edges == 1u);

  size_t num_outgoing_wheel_odometry_edges = 0;
  if (merge_wheel_odometry_edges) {
    num_outgoing_wheel_odometry_edges = mergeEdgesOfNeighboringVertices<
        TransformationEdge, Edge::EdgeType::kWheelOdometry>(
        merge_into_vertex_id, vertex_to_merge);
  }
  CHECK(
      getMission(mission_id).backboneType() !=
          Mission::BackBone::kWheelOdometry ||
      num_outgoing_wheel_odometry_edges == 1u);

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
          landmark.removeAllObservationsOfVertex(vertex_to_merge);
        }
      }
    }
  }
  // Remove the vertex.
  posegraph.removeVertex(vertex_to_merge);
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

const vi_map::MissionId VIMap::duplicateMission(
    const vi_map::MissionId& source_mission_id) {
  CHECK(hasMission(source_mission_id));

  const vi_map::VIMission& source_mission = getMission(source_mission_id);
  const vi_map::MissionBaseFrame& source_baseframe =
      getMissionBaseFrame(source_mission.getBaseFrameId());

  VLOG(1) << "Adding baseframe and mission.";
  // Copy baseframe.
  vi_map::MissionBaseFrame baseframe;
  vi_map::MissionBaseFrameId baseframe_id = source_baseframe.id();
  aslam::generateId(&baseframe_id);

  baseframe.setId(baseframe_id);
  baseframe.set_p_G_M(source_baseframe.get_p_G_M());
  baseframe.set_q_G_M(source_baseframe.get_q_G_M());
  baseframe.set_T_G_M_Covariance(source_baseframe.get_T_G_M_Covariance());
  mission_base_frames.emplace(baseframe_id, baseframe);

  // Copy mission.
  vi_map::VIMission::UniquePtr mission_ptr(new vi_map::VIMission);
  vi_map::MissionId duplicated_mission_id;
  aslam::generateId(&duplicated_mission_id);

  mission_ptr->setId(duplicated_mission_id);
  mission_ptr->setBackboneType(source_mission.backboneType());
  mission_ptr->setBaseFrameId(baseframe_id);

  VLOG(1) << "Cloning all sensors with new ids.";
  // Duplicate mission sensors with new ids.
  if (source_mission.hasNCamera()) {
    const aslam::SensorId source_sensor_id = source_mission.getNCameraId();

    aslam::NCamera* cloned_ncamera =
        CHECK_NOTNULL(getMissionNCamera(source_mission_id).cloneWithNewIds());
    const aslam::SensorId new_sensor_id = cloned_ncamera->getId();
    const aslam::Transformation& T_B_S =
        getSensorManager().getSensor_T_B_S(source_sensor_id);
    const aslam::SensorId& base_sensor_id =
        getSensorManager().getBaseSensorId(source_sensor_id);
    getSensorManager().addSensor<aslam::NCamera>(
        aslam::NCamera::UniquePtr(cloned_ncamera), base_sensor_id, T_B_S);

    CHECK(mission_ptr);
    mission_ptr->setNCameraId(new_sensor_id);
  }

  if (source_mission.hasImu()) {
    const aslam::SensorId source_sensor_id = source_mission.getImuId();

    vi_map::Imu* cloned_imu =
        CHECK_NOTNULL(getMissionImu(source_mission_id).cloneWithNewIds());
    const aslam::SensorId new_sensor_id = cloned_imu->getId();
    const aslam::Transformation& T_B_S =
        getSensorManager().getSensor_T_B_S(source_sensor_id);
    const aslam::SensorId& base_sensor_id =
        getSensorManager().getBaseSensorId(source_sensor_id);
    getSensorManager().addSensor<vi_map::Imu>(
        vi_map::Imu::UniquePtr(cloned_imu), base_sensor_id, T_B_S);

    CHECK(mission_ptr);
    mission_ptr->setImuId(new_sensor_id);
  }

  if (source_mission.hasLidar()) {
    const aslam::SensorId source_sensor_id = source_mission.getLidarId();

    vi_map::Lidar* cloned_lidar =
        CHECK_NOTNULL(getMissionLidar(source_mission_id).cloneWithNewIds());
    const aslam::SensorId new_sensor_id = cloned_lidar->getId();
    const aslam::Transformation& T_B_S =
        getSensorManager().getSensor_T_B_S(source_sensor_id);
    const aslam::SensorId& base_sensor_id =
        getSensorManager().getBaseSensorId(source_sensor_id);
    getSensorManager().addSensor<vi_map::Lidar>(
        vi_map::Lidar::UniquePtr(cloned_lidar), base_sensor_id, T_B_S);

    CHECK(mission_ptr);
    mission_ptr->setLidarId(new_sensor_id);
  }

  if (source_mission.hasOdometry6DoFSensor()) {
    const aslam::SensorId source_sensor_id =
        source_mission.getOdometry6DoFSensor();

    vi_map::Odometry6DoF* cloned_odom_sensor = CHECK_NOTNULL(
        getMissionOdometry6DoFSensor(source_mission_id).cloneWithNewIds());
    CHECK(cloned_odom_sensor);
    const aslam::SensorId new_sensor_id = cloned_odom_sensor->getId();
    const aslam::Transformation& T_B_S =
        getSensorManager().getSensor_T_B_S(source_sensor_id);
    const aslam::SensorId& base_sensor_id =
        getSensorManager().getBaseSensorId(source_sensor_id);
    getSensorManager().addSensor<vi_map::Odometry6DoF>(
        vi_map::Odometry6DoF::UniquePtr(cloned_odom_sensor), base_sensor_id,
        T_B_S);

    CHECK(mission_ptr);
    mission_ptr->setOdometry6DoFSensor(new_sensor_id);
  }

  if (source_mission.hasLoopClosureSensor()) {
    const aslam::SensorId source_sensor_id =
        source_mission.getLoopClosureSensor();

    vi_map::LoopClosureSensor* cloned_loop_closure_sensor = CHECK_NOTNULL(
        getMissionLoopClosureSensor(source_mission_id).cloneWithNewIds());
    const aslam::SensorId new_sensor_id = cloned_loop_closure_sensor->getId();
    const aslam::Transformation& T_B_S =
        getSensorManager().getSensor_T_B_S(source_sensor_id);
    const aslam::SensorId& base_sensor_id =
        getSensorManager().getBaseSensorId(source_sensor_id);
    getSensorManager().addSensor<vi_map::LoopClosureSensor>(
        vi_map::LoopClosureSensor::UniquePtr(cloned_loop_closure_sensor),
        base_sensor_id, T_B_S);

    CHECK(mission_ptr);
    mission_ptr->setLoopClosureSensor(new_sensor_id);
  }

  if (source_mission.hasWheelOdometrySensor()) {
    const aslam::SensorId source_sensor_id =
        source_mission.getWheelOdometrySensor();

    vi_map::WheelOdometry* cloned_wheel_odometry_sensor = CHECK_NOTNULL(
        getMissionWheelOdometrySensor(source_mission_id).cloneWithNewIds());
    const aslam::SensorId new_sensor_id = cloned_wheel_odometry_sensor->getId();
    const aslam::Transformation& T_B_S =
        getSensorManager().getSensor_T_B_S(source_sensor_id);
    const aslam::SensorId& base_sensor_id =
        getSensorManager().getBaseSensorId(source_sensor_id);
    getSensorManager().addSensor<vi_map::WheelOdometry>(
        vi_map::WheelOdometry::UniquePtr(cloned_wheel_odometry_sensor),
        base_sensor_id, T_B_S);

    CHECK(mission_ptr);
    mission_ptr->setWheelOdometrySensor(new_sensor_id);
  }

  // Insert new mission
  missions.emplace(mission_ptr->id(), std::move(mission_ptr));
  const VIMission& new_mission = getMission(duplicated_mission_id);
  aslam::NCamera::Ptr new_ncamera;
  if (new_mission.hasNCamera()) {
    new_ncamera = getMissionNCameraPtr(duplicated_mission_id);
  }
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
  getAllVertexIdsInMission(source_mission_id, &all_source_vertices);

  VLOG(1) << "Adding vertices.";
  for (const pose_graph::VertexId& vertex_id : all_source_vertices) {
    const vi_map::Vertex& source_vertex = getVertex(vertex_id);
    CHECK_EQ(source_vertex.getMissionId(), source_mission_id);
    pose_graph::VertexId new_vertex_id = vertex_id;
    aslam::generateId(&new_vertex_id);

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
              getObserverMissionsForLandmark(
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
              aslam::generateId(&new_landmark_id);
              source_to_dest_landmark_id_map.emplace(
                  current_landmark_id, new_landmark_id);
            }
          }
        }
        landmarks_seen_by_vertex[frame_idx].push_back(new_landmark_id);
      }
    }

    aslam::VisualNFrame::Ptr n_frame(
        new aslam::VisualNFrame(source_vertex.getVisualNFrame()));

    // Overwrite ncamera with new cloned ncamera.
    n_frame->setNCameras(new_ncamera);

    aslam::NFramesId n_frame_id;
    aslam::generateId(&n_frame_id);
    n_frame->setId(n_frame_id);

    for (size_t frame_idx = 0u; frame_idx < num_frames; ++frame_idx) {
      if (n_frame->isFrameSet(frame_idx)) {
        aslam::FrameId frame_id;
        aslam::generateId(&frame_id);
        n_frame->getFrameShared(frame_idx)->setId(frame_id);
      }
    }

    vi_map::Vertex::UniquePtr vertex_ptr(new vi_map::Vertex(
        new_vertex_id, imu_ba_bw, n_frame, landmarks_seen_by_vertex,
        duplicated_mission_id));

    vertex_ptr->set_p_M_I(source_vertex.get_p_M_I());
    vertex_ptr->set_q_M_I(source_vertex.get_q_M_I().normalized());
    vertex_ptr->set_v_M(source_vertex.get_v_M());

    vertex_ptr->setFrameResourceMap(source_vertex.getFrameResourceMap());

    posegraph.addVertex(std::move(vertex_ptr));

    // Our local old-to-new VertexId map.
    source_to_dest_vertex_id_map.emplace(vertex_id, new_vertex_id);
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
      aslam::generateId(&new_edge_id);

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

  return duplicated_mission_id;
}

unsigned int VIMap::numExpectedLandmarkObserverMissions(
    const vi_map::LandmarkId& landmark_id) const {
  CHECK(landmark_id.isValid());
  CHECK(hasLandmark(landmark_id));

  vi_map::MissionIdSet observer_missions;
  getObserverMissionsForLandmark(landmark_id, &observer_missions);

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
    const vi_map::Vertex& vertex = getVertex(observation.frame_id.vertex_id);
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
      getObserverMissionsForLandmark(
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

          // If the current vertex was the only observer of the landmark
          // stored in some other mission, we should remove this orphaned
          // landmark.
          if (getMissionIdForLandmark(landmark_id) != mission_id &&
              !landmark.hasObservations()) {
            removeLandmark(landmark.id());
          }
        }
      }
    }

    removeVertex(vertex_id);
  }

  // Remove mission sensors that are no longer needed.
  // TODO(smauq)

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
  CHECK_NOTNULL(vertices)->clear();
  CHECK(hasMission(mission_id));

  const vi_map::VIMission& mission = getMission(mission_id);
  const pose_graph::VertexId& starting_vertex_id = mission.getRootVertexId();
  if (starting_vertex_id.isValid()) {
    getAllVertexIdsInMissionAlongGraph(
        mission_id, starting_vertex_id, vertices);
  }
}

void VIMap::getAllVertexIdsInMissionAlongGraph(
    const vi_map::MissionId& mission_id,
    const pose_graph::VertexId& starting_vertex_id,
    pose_graph::VertexIdList* vertices) const {
  CHECK_NOTNULL(vertices)->clear();
  CHECK(starting_vertex_id.isValid());
  CHECK(hasMission(mission_id));

  pose_graph::VertexId current_vertex_id = starting_vertex_id;
  CHECK_EQ(getVertex(current_vertex_id).getMissionId(), mission_id);

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
    if (getEdgeType(edge_id) == ref_edge_type) {
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
    mission.getAllMissionResourceIds(
        backend::ResourceType::kTsdfGridPath, &tsdf_resources);
    for (const backend::ResourceId& resource_id : tsdf_resources) {
      consistent &= checkResource<std::string>(
          resource_id, backend::ResourceType::kTsdfGridPath);
    }
    backend::ResourceIdSet occupancy_resources;
    mission.getAllMissionResourceIds(
        backend::ResourceType::kOccupancyGridPath, &occupancy_resources);
    for (const backend::ResourceId& resource_id : occupancy_resources) {
      consistent &= checkResource<std::string>(
          resource_id, backend::ResourceType::kOccupancyGridPath);
    }
    backend::ResourceIdSet reconstruction_resources;
    mission.getAllMissionResourceIds(
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
    VLOG(3) << "Found resource for " << resource.second.size() << " missions.";
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
    getMission(mission_id)
        .getAllMissionResourceIds(resource_type, &resource_ids);
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
    getMission(mission_id).addMissionResourceId(resource_id, resource_type);
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
    getMission(mission_id).deleteMissionResourceId(resource_id, resource_type);
  }
}

// NOTE: [ADD_RESOURCE_TYPE] [ADD_RESOURCE_DATA_TYPE] Add a switch case if the
// resource is a frame resource.
void VIMap::deleteAllFrameResources(
    const unsigned int frame_idx, const bool delete_from_file_system,
    Vertex* vertex_ptr) {
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
          deleteResource<cv::Mat>(resource_id, type, !delete_from_file_system);
          break;
        default:
          LOG(FATAL) << "Unsupported frame resource data type: "
                     << backend::ResourceTypeNames[static_cast<int>(type)]
                     << "! Implement a case here.";
      }
    }
  }
  vertex_ptr->deleteAllFrameResourceInfo(frame_idx);
}

void VIMap::deleteAllSensorResourcesBeforeTime(
    const vi_map::MissionId& mission_id, int64_t timestamp_ns,
    const bool delete_from_file_system) {
  vi_map::VIMission& mission = getMission(mission_id);

  // Remove all sensor resource before that vertex.
  backend::ResourceTypeToSensorIdToResourcesMap&
      resource_types_to_sensor_to_resource_map =
          mission.getAllSensorResourceIds();
  for (backend::ResourceTypeToSensorIdToResourcesMap::value_type&
           resource_type_to_sensor_to_resource_map :
       resource_types_to_sensor_to_resource_map) {
    for (std::pair<const aslam::SensorId, backend::TemporalResourceIdBuffer>&
             sensor_to_resource_map :
         resource_type_to_sensor_to_resource_map.second) {
      backend::TemporalResourceIdBuffer& resource_map =
          sensor_to_resource_map.second;
      std::vector<backend::ResourceId> resource_ids;
      resource_map.extractItemsBeforeIncluding(timestamp_ns, &resource_ids);

      for (const backend::ResourceId& resource_id : resource_ids) {
        deleteResourceNoDataType(
            resource_id, resource_type_to_sensor_to_resource_map.first,
            !delete_from_file_system);
      }
    }
  }
}

std::string VIMap::getSubFolderName() {
  return serialization::getSubFolderName();
}

bool VIMap::hasMapOnFileSystem(const std::string& map_folder) {
  return serialization::hasMapOnFileSystem(map_folder);
}

bool VIMap::loadFromFolder(const std::string& map_folder) {
  return serialization::loadMapFromFolder(map_folder, this);
}

bool VIMap::saveToFolder(
    const std::string& folder_path, const backend::SaveConfig& config) {
  return serialization::saveMapToFolder(folder_path, config, this);
}

bool VIMap::saveToMapFolder(const backend::SaveConfig& config) {
  return saveToFolder(getMapFolder(), config);
}

bool VIMap::hasSensorResource(
    const VIMission& mission, const backend::ResourceType& type,
    const aslam::SensorId& sensor_id, const int64_t timestamp_ns) const {
  return mission.hasSensorResourceId(type, sensor_id, timestamp_ns);
}

bool VIMap::findAllCloseSensorResources(
    const VIMission& mission, const backend::ResourceType& type,
    const int64_t timestamp_ns, const int64_t tolerance_ns,
    std::vector<aslam::SensorId>* sensor_ids,
    std::vector<int64_t>* closest_timestamps_ns) const {
  CHECK_NOTNULL(sensor_ids)->clear();
  CHECK_NOTNULL(closest_timestamps_ns);
  return mission.findAllCloseSensorResources(
      type, timestamp_ns, tolerance_ns, sensor_ids, closest_timestamps_ns);
}

bool VIMap::hasFrameResource(
    const Vertex& vertex, const unsigned int frame_idx,
    const backend::ResourceType& resource_type) const {
  std::lock_guard<std::recursive_mutex> lock(resource_mutex_);
  return vertex.hasFrameResourceOfType(frame_idx, resource_type);
}

size_t VIMap::getNumAbsolute6DoFMeasurementsInMission(
    const vi_map::MissionId& mission_id) const {
  CHECK(mission_id.isValid());
  pose_graph::VertexIdList vertex_ids;
  getAllVertexIdsInMissionAlongGraph(mission_id, &vertex_ids);

  size_t num_constraints = 0u;

  for (const pose_graph::VertexId& vertex_id : vertex_ids) {
    const vi_map::Vertex& vertex = getVertex(vertex_id);
    num_constraints += vertex.getNumAbsolute6DoFMeasurements();
  }

  return num_constraints;
}

bool VIMap::mergeAllSubmapsFromMapWithoutResources(
    const vi_map::VIMap& submap) {
  // Get all missions from old map and add them into the new map.
  vi_map::MissionIdList submap_mission_ids;
  submap.getAllMissionIds(&submap_mission_ids);

  CHECK(submap_mission_ids.size() == 1u)
      << "mergeAllSubmapsFromMapWithoutResources currently only supports "
         "merging a map with a single mission into the current map.";

  const vi_map::MissionId& submap_and_base_mission_id = submap_mission_ids[0];
  CHECK(submap_and_base_mission_id.isValid());

  if (!hasMission(submap_and_base_mission_id)) {
    LOG(WARNING)
        << "Could not find a matching mission id in the other map that "
        << "could be merged as a submap! Adding as new mission without "
        << "attaching to existing missions!";
    return mergeAllMissionsFromMap(submap);
  }
  vi_map::VIMission& base_mission = getMission(submap_and_base_mission_id);
  const vi_map::VIMission& submap_mission =
      submap.getMission(submap_and_base_mission_id);

  // Merge sensor resources:
  base_mission.mergeAllSensorResources(submap_mission);

  const pose_graph::VertexId& first_submap_vertex_id =
      submap_mission.getRootVertexId();
  const pose_graph::VertexId& last_base_vertex_id =
      getLastVertexIdOfMission(submap_and_base_mission_id);
  CHECK(first_submap_vertex_id.isValid());
  CHECK(last_base_vertex_id.isValid());

  if (first_submap_vertex_id != last_base_vertex_id) {
    LOG(ERROR) << "The first vertex of the submap and the last vertex of the "
                  "base map are not the same!";
    // TODO(mfehr): In order to prevent loosing this submap in case we have
    // lost an intermediate submap (so attaching is not possible), we could
    // simply duplicate the submap mission (= gets new ids) and add it as a
    // new mission.
    return false;
  }

  VLOG(1) << "Attaching submap at common vertex " << last_base_vertex_id;

  // Get pose correction based on the difference between the two vertices.
  vi_map::Vertex& base_last_vertex = getVertex(last_base_vertex_id);
  const vi_map::Vertex& submap_first_vertex =
      submap.getVertex(first_submap_vertex_id);
  const aslam::Transformation& T_Mbase_B = base_last_vertex.get_T_M_I();
  const aslam::Transformation& T_Msubmap_B = submap_first_vertex.get_T_M_I();
  const aslam::Transformation T_Mbase_Msubmap =
      T_Mbase_B * T_Msubmap_B.inverse();

  // If there is an NCamera we need to update the link in the new vertices to
  // the ncamera inside the sensor manager of the base map.
  aslam::NCamera::Ptr ncamera_base;
  if (base_mission.hasNCamera()) {
    ncamera_base = sensor_manager_.getSensorPtr<aslam::NCamera>(
        base_mission.getNCameraId());
  }

  // Check if the first submap vertex owns different landmarks for the same
  // observations as the last vertex in the base map. We need to apply a lot
  // of transfering and reassociating of landmarks to fix this otherwise we
  // will end up loosing constraints at this junction of submaps or get an
  // inconsistent map.
  std::unordered_map<vi_map::LandmarkId, vi_map::LandmarkId>
      submap_to_base_landmark_id_map;
  std::unordered_set<vi_map::LandmarkId> remapped_landmark_base_ids;
  std::unordered_set<vi_map::LandmarkId> remapped_landmark_submap_ids;
  if (base_last_vertex.hasVisualNFrame()) {
    CHECK(submap_first_vertex.hasVisualNFrame());
    CHECK(ncamera_base);
    aslam::VisualNFrame& base_nframe = base_last_vertex.getVisualNFrame();
    const aslam::VisualNFrame& submap_nframe =
        submap_first_vertex.getVisualNFrame();

    const size_t num_frames = base_nframe.getNumFrames();
    CHECK_EQ(num_frames, submap_nframe.getNumFrames());
    for (size_t frame_idx = 0; frame_idx < num_frames; ++frame_idx) {
      const size_t num_base_keypoints =
          base_last_vertex.observedLandmarkIdsSize(frame_idx);
      const size_t num_submap_keypoints =
          submap_first_vertex.observedLandmarkIdsSize(frame_idx);
      CHECK_EQ(num_base_keypoints, num_submap_keypoints);

      for (size_t observation_idx = 0; observation_idx < num_base_keypoints;
           ++observation_idx) {
        const vi_map::LandmarkId& base_landmark_id =
            base_last_vertex.getObservedLandmarkId(frame_idx, observation_idx);
        const vi_map::LandmarkId& submap_landmark_id =
            submap_first_vertex.getObservedLandmarkId(
                frame_idx, observation_idx);
        if (!submap_landmark_id.isValid()) {
          // Nothing to do, no landmark to unify or move to the new vertex.
          continue;
        }

        // Check storing vertex of submap landmark.
        CHECK(submap.landmark_index.hasLandmark(submap_landmark_id));
        const pose_graph::VertexId& storing_vertex_id_submap =
            submap.landmark_index.getStoringVertexId(submap_landmark_id);
        CHECK_EQ(storing_vertex_id_submap, first_submap_vertex_id);
        CHECK(submap.hasVertex(storing_vertex_id_submap));
        CHECK(submap.getVertex(storing_vertex_id_submap)
                  .hasStoredLandmark(submap_landmark_id));
        CHECK(submap.getLandmark(submap_landmark_id)
                  .hasObservation(
                      first_submap_vertex_id, frame_idx, observation_idx));

        // Two options left, either there are valid landmark observations in
        // both vertices or only in the new one.
        if (base_landmark_id.isValid()) {
          // Check storing vertex of base landmark.
          CHECK(landmark_index.hasLandmark(base_landmark_id));
          const pose_graph::VertexId& storing_vertex_id_base_map =
              landmark_index.getStoringVertexId(base_landmark_id);
          CHECK(hasVertex(storing_vertex_id_base_map));
          CHECK(getVertex(storing_vertex_id_base_map)
                    .hasStoredLandmark(base_landmark_id));
          CHECK_GE(
              base_last_vertex.getNumLandmarkObservations(base_landmark_id),
              1u);
          CHECK(getLandmark(base_landmark_id)
                    .hasObservation(
                        last_base_vertex_id, frame_idx, observation_idx));

          // Both vertices have a valid landmark id associated with this
          // observation, hence we need to change the association of all
          // future observations of this landmark to the one in the base map.
          submap_to_base_landmark_id_map[submap_landmark_id] = base_landmark_id;
          remapped_landmark_base_ids.insert(base_landmark_id);
          remapped_landmark_submap_ids.insert(submap_landmark_id);
        } else {
          CHECK(
              !base_last_vertex.getLandmarks().hasLandmark(submap_landmark_id));

          // If the same vertex in the submap owns a new landmark, we
          // transfer it to the base map vertex.
          vi_map::Landmark submap_landmark =
              submap_first_vertex.getLandmarks().getLandmark(
                  submap_landmark_id);

          // Clear all observations and rebuild them later when adding the
          // vertices.
          base_last_vertex.getLandmarks().addLandmark(submap_landmark);
          landmark_index.addLandmarkAndVertexReference(
              submap_landmark_id, last_base_vertex_id);
          base_last_vertex.setObservedLandmarkId(
              frame_idx, observation_idx, submap_landmark_id);

          // Check storing vertex of submap landmark after insertion into the
          // base map.
          CHECK(landmark_index.hasLandmark(submap_landmark_id));
          const pose_graph::VertexId& storing_vertex_id_base_map =
              landmark_index.getStoringVertexId(submap_landmark_id);
          CHECK_EQ(storing_vertex_id_base_map, last_base_vertex_id);
          CHECK(hasVertex(storing_vertex_id_base_map));
          CHECK(getVertex(storing_vertex_id_base_map)
                    .hasStoredLandmark(submap_landmark_id));
          CHECK_GE(
              base_last_vertex.getNumLandmarkObservations(submap_landmark_id),
              1u);
          CHECK(getLandmark(submap_landmark_id)
                    .hasObservation(
                        last_base_vertex_id, frame_idx, observation_idx));
        }
      }
    }

    // After keyframing the vertices might store landmarks they do not observe
    // themselves, this is shit, but we need to deal with it by updating the
    // global landmark index and adding it to the base map vertex.
    LandmarkIdList owned_landmark_ids;
    submap_first_vertex.getStoredLandmarkIdList(&owned_landmark_ids);
    for (const LandmarkId& owned_landmark_id : owned_landmark_ids) {
      if (submap_first_vertex.getNumLandmarkObservations(owned_landmark_id) >
          0u) {
        // Nothing todo here, this landmark should already be moved or unified
        // with the base map.

        // Check if this landmark has really been stored correctly in the base
        // map.
        // If this landmark has been remapped to a new one, we should check
        // the base map for the presence of the remapped one.
        LandmarkId remapped_landmark_id = owned_landmark_id;
        if (remapped_landmark_submap_ids.count(owned_landmark_id) > 0u) {
          remapped_landmark_id =
              submap_to_base_landmark_id_map[owned_landmark_id];
        }
        CHECK(landmark_index.hasLandmark(remapped_landmark_id));
        const pose_graph::VertexId& storing_vertex_id_base_map =
            landmark_index.getStoringVertexId(remapped_landmark_id);
        CHECK(hasVertex(storing_vertex_id_base_map));
        CHECK(getVertex(storing_vertex_id_base_map)
                  .hasStoredLandmark(remapped_landmark_id));
        CHECK_GE(
            base_last_vertex.getNumLandmarkObservations(remapped_landmark_id),
            1u);
        continue;
      }

      // This landmark should not be in the base map index yet.
      CHECK(!landmark_index.hasLandmark(owned_landmark_id));

      // Save the landmark id so we can adapt the landmark observation
      // back-link later when adding vertices that observe this landmark.
      submap_to_base_landmark_id_map[owned_landmark_id] = owned_landmark_id;
      remapped_landmark_base_ids.insert(owned_landmark_id);
      remapped_landmark_submap_ids.insert(owned_landmark_id);

      // Transfer the landmark with cleared observations and rebuild them
      // later when adding the vertices.
      vi_map::Landmark submap_landmark =
          submap_first_vertex.getLandmarks().getLandmark(owned_landmark_id);
      base_last_vertex.getLandmarks().addLandmark(submap_landmark);
      landmark_index.addLandmarkAndVertexReference(
          owned_landmark_id, last_base_vertex_id);
    }

    base_last_vertex.checkConsistencyOfVisualObservationContainers();
  }

  pose_graph::VertexIdList submap_vertex_ids;
  submap.getAllVertexIdsInMissionAlongGraph(
      submap_and_base_mission_id, &submap_vertex_ids);
  CHECK_GE(submap_vertex_ids.size(), 1u);
  CHECK_EQ(submap_vertex_ids[0], first_submap_vertex_id);

  // Add the corrected vertices of the submap to the base map.
  for (size_t idx = 1; idx < submap_vertex_ids.size(); ++idx) {
    const pose_graph::VertexId& submap_vertex_id = submap_vertex_ids[idx];
    const vi_map::Vertex& submap_vertex = submap.getVertex(submap_vertex_id);

    vi_map::Vertex::UniquePtr vertex_copy;
    if (submap_vertex.hasVisualNFrame()) {
      CHECK(ncamera_base);
      vertex_copy.reset(submap_vertex.cloneWithVisualNFrame(ncamera_base));

      // After keyframing the vertices might store landmarks they do not
      // observe themselves, this is shit, but we need to deal with it by
      // updating the global landmark index or, if the landmark has been
      // associated with a base map landmark, delete it.
      LandmarkIdList owned_landmark_ids;
      vertex_copy->getStoredLandmarkIdList(&owned_landmark_ids);
      for (const LandmarkId& owned_landmark_id : owned_landmark_ids) {
        CHECK(vertex_copy->getLandmarks().hasLandmark(owned_landmark_id));

        // If this landmark has been remapped to a landmark in the base map we
        // can delete it or if it doesn't have any observations. Otherwise we
        // need to update the global landmark index of the base map.
        if (remapped_landmark_submap_ids.count(owned_landmark_id) > 0u ||
            !vertex_copy->getLandmarks()
                 .getLandmark(owned_landmark_id)
                 .hasObservations()) {
          vertex_copy->getLandmarks().removeLandmark(owned_landmark_id);
          LandmarkId invalid;
          vertex_copy->updateIdInObservedLandmarkIdList(
              owned_landmark_id, invalid);
        } else {
          landmark_index.addLandmarkAndVertexReference(
              owned_landmark_id, submap_vertex_id);
        }
      }

      // Loop over all observed landmarks and update the bookkeeping. We need
      // to turn the observed_landmark_ids into a set to because the same
      // landmark can be observed multiple times but we only need to perform
      // these corrections once per landmark.
      LandmarkIdList observed_landmark_ids;
      vertex_copy->getAllObservedLandmarkIds(&observed_landmark_ids);
      std::unordered_set<LandmarkId> unique_landmark_ids(
          observed_landmark_ids.begin(), observed_landmark_ids.end());
      for (const LandmarkId& submap_landmark_id : unique_landmark_ids) {
        if (!submap_landmark_id.isValid()) {
          continue;
        }

        CHECK_GE(
            vertex_copy->getNumLandmarkObservations(submap_landmark_id), 1u);

        LandmarkId corresponding_base_landmark_id;
        pose_graph::VertexId storing_vertex_id;
        // The observed landmark is either one that is only present in the
        // submap or it is one that needs to be remapped to a landmark in the
        // base map.
        if (remapped_landmark_submap_ids.count(submap_landmark_id) > 0u) {
          corresponding_base_landmark_id =
              submap_to_base_landmark_id_map[submap_landmark_id];

          // Sanity checks on the presence of the corresponding landmark in
          // the base map.
          CHECK(landmark_index.hasLandmark(corresponding_base_landmark_id));
          storing_vertex_id =
              landmark_index.getStoringVertexId(corresponding_base_landmark_id);
          CHECK(hasVertex(storing_vertex_id));
          CHECK(getVertex(storing_vertex_id)
                    .hasStoredLandmark(corresponding_base_landmark_id));

          if (submap_landmark_id != corresponding_base_landmark_id) {
            // Update the list of observed landmark ids. This will update all
            // occurences of the landmark in the observer list, which is why
            // we needed to make sure only every landmark is executing this
            // loop once.
            vertex_copy->updateIdInObservedLandmarkIdList(
                submap_landmark_id, corresponding_base_landmark_id);
            CHECK_EQ(
                vertex_copy->getNumLandmarkObservations(submap_landmark_id),
                0u);
          }

          CHECK_GE(
              vertex_copy->getNumLandmarkObservations(
                  corresponding_base_landmark_id),
              1u);

          // Should have been deleted already when it was detected in the
          // first vertex of the submap.
          CHECK(!vertex_copy->hasStoredLandmark(submap_landmark_id));
        } else {
          corresponding_base_landmark_id = submap_landmark_id;

          CHECK(submap.landmark_index.hasLandmark(submap_landmark_id));
          storing_vertex_id =
              submap.landmark_index.getStoringVertexId(submap_landmark_id);
          CHECK(submap.hasVertex(storing_vertex_id));
          CHECK(submap.getVertex(storing_vertex_id)
                    .hasStoredLandmark(submap_landmark_id));
        }
        CHECK(storing_vertex_id.isValid());
        CHECK(corresponding_base_landmark_id.isValid());

        if (landmark_index.hasLandmark(corresponding_base_landmark_id)) {
          landmark_index.updateVertexOfLandmark(
              corresponding_base_landmark_id, storing_vertex_id);
        } else {
          landmark_index.addLandmarkAndVertexReference(
              corresponding_base_landmark_id, storing_vertex_id);
        }

        CHECK(hasLandmarkIdInLandmarkIndex(corresponding_base_landmark_id))
            << "Landmark " << corresponding_base_landmark_id
            << " is not in the global map index of the base map!";
      }

      // Loop over all keypoints and for the ones that have been associated
      // with a base map landmark, add this keypoint observation to the
      // landmark back-link.
      vertex_copy->forEachKeypoint([&vertex_copy, &remapped_landmark_base_ids,
                                    &remapped_landmark_submap_ids, &submap,
                                    &submap_to_base_landmark_id_map, this](
                                       const KeypointIdentifier& identifier) {
        const LandmarkId& observed_landmark_id =
            vertex_copy->getObservedLandmarkId(identifier);
        if (!observed_landmark_id.isValid()) {
          return;
        }
        if (remapped_landmark_base_ids.count(observed_landmark_id) > 0u) {
          CHECK(this->landmark_index.hasLandmark(observed_landmark_id));
          const pose_graph::VertexId& storing_vertex_id_base_map =
              this->landmark_index.getStoringVertexId(observed_landmark_id);
          CHECK(this->hasVertex(storing_vertex_id_base_map));
          CHECK(this->getVertex(storing_vertex_id_base_map)
                    .hasStoredLandmark(observed_landmark_id));

          // Retrieve landmark in base map and add observation of new
          // vertex.
          Landmark& landmark = this->getLandmark(observed_landmark_id);
          if (!landmark.hasObservation(identifier)) {
            landmark.addObservation(identifier);
          }
        } else if (
            remapped_landmark_submap_ids.count(observed_landmark_id) > 0u) {
          const LandmarkId& corresponding_base_landmark_id =
              submap_to_base_landmark_id_map[observed_landmark_id];
          vertex_copy->setObservedLandmarkId(
              identifier, corresponding_base_landmark_id);

          Landmark& landmark =
              this->getLandmark(corresponding_base_landmark_id);
          if (!landmark.hasObservation(identifier)) {
            landmark.addObservation(identifier);
          }
        }
      });
      vertex_copy->checkConsistencyOfVisualObservationContainers();
    } else {
      vertex_copy.reset(submap_vertex.cloneWithoutVisualNFrame());
    }

    // Correct vertex position and velocity based on where the submap is
    // attached to, which might have changed since the time the map was split.
    vertex_copy->set_T_M_I(T_Mbase_Msubmap * vertex_copy->get_T_M_I());
    vertex_copy->set_v_M(T_Mbase_Msubmap * vertex_copy->get_v_M());
    addVertex(std::move(vertex_copy));
  }

  // Copy all the edges.
  pose_graph::EdgeIdList submap_edges;
  submap.getAllEdgeIds(&submap_edges);
  for (const pose_graph::EdgeId& edge_id : submap_edges) {
    const vi_map::Edge& submap_edge = submap.getEdgeAs<vi_map::Edge>(edge_id);
    vi_map::Edge* edge_copy;
    submap_edge.copyEdgeInto(&edge_copy);
    addEdge(vi_map::Edge::UniquePtr(edge_copy));
  }
  return true;
}

}  // namespace vi_map
