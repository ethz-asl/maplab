#ifndef VI_MAP_VI_MAP_H_
#define VI_MAP_VI_MAP_H_

#include <Eigen/Dense>
#include <Eigen/SparseCore>
#include <aslam/common/memory.h>
#include <aslam/common/pose-types.h>
#include <aslam/common/unique-id.h>
#include <map-resources/resource-common.h>
#include <map-resources/resource-map.h>
#include <maplab-common/file-serializable.h>
#include <maplab-common/macros.h>
#include <maplab-common/map-manager-config.h>
#include <maplab-common/map-traits.h>
#include <memory>
#include <mutex>
#include <posegraph/pose-graph.h>
#include <posegraph/unique-id.h>
#include <queue>
#include <random>
#include <sensors/absolute-6dof-pose.h>
#include <sensors/external-features.h>
#include <sensors/imu.h>
#include <sensors/lidar.h>
#include <sensors/loop-closure-sensor.h>
#include <sensors/odometry-6dof-pose.h>
#include <sensors/pointcloud-map-sensor.h>
#include <sensors/wheel-odometry-sensor.h>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "vi-map/cklam-edge.h"
#include "vi-map/landmark-index.h"
#include "vi-map/landmark.h"
#include "vi-map/laser-edge.h"
#include "vi-map/loopclosure-edge.h"
#include "vi-map/mission-baseframe.h"
#include "vi-map/mission.h"
#include "vi-map/pose-graph.h"
#include "vi-map/sensor-manager.h"
#include "vi-map/structure-loopclosure-edge.h"
#include "vi-map/trajectory-edge.h"
#include "vi-map/transformation-edge.h"
#include "vi-map/unique-id.h"
#include "vi-map/vertex.h"
#include "vi-map/vi_map.pb.h"
#include "vi-map/viwls-edge.h"

DECLARE_bool(disable_consistency_check);

class LoopClosureHandlerTest;

namespace vi_map {
class VIMap;
namespace proto {
class VIMap;
}  // namespace proto
}  // namespace vi_map

namespace metadata {
namespace proto {
class MetaData;
}  // namespace proto
}  // namespace metadata

namespace vi_map {
class SixDofVIMapGenerator;

namespace serialization {
void deserializeMissionsAndBaseframes(
    const proto::VIMap& proto, vi_map::VIMap* map);
}  // namespace serialization

typedef std::unordered_map<backend::ResourceId, MissionIdList>
    ResourceIdToMissionsMap;

typedef std::unordered_map<MissionId, pose_graph::VertexIdList>
    MissionVertexIdList;

typedef std::pair<MissionId, pose_graph::VertexIdList> MissionVertexIdPair;

class VIMap : public backend::ResourceMap,
              public backend::MapInterface<vi_map::VIMap> {
  friend ::LoopClosureHandlerTest;       // Test.
  friend class MapConsistencyCheckTest;  // Test.
  friend class SixDofVIMapGenerator;     // Test.
  friend bool checkMapConsistency(const VIMap&);
  friend class VIMapStats;

 public:
  MAPLAB_POINTER_TYPEDEFS(VIMap);
  typedef Eigen::Matrix<unsigned char, Eigen::Dynamic, 1> DescriptorType;
  typedef Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>
      DescriptorsType;
  typedef std::unordered_map<vi_map::MissionId, unsigned int>
      MissionToLandmarkCountMap;
  friend void serialization::deserializeMissionsAndBaseframes(
      const proto::VIMap& proto, vi_map::VIMap* map);

  explicit VIMap(const std::string& map_folder);
  explicit VIMap(const metadata::proto::MetaData& metadata_proto);
  VIMap();
  virtual ~VIMap();

  // Discards any data that is not stored in MappedContainerBase-s.
  void deepCopy(const VIMap& other) override;
  void swap(VIMap* other);  // NOLINT

  bool hexStringToMissionIdIfValid(
      const std::string& map_mission_id_string,
      vi_map::MissionId* mission_id) const;
  // Returns first mission if string not valid.
  bool ensureMissionIdValid(
      const std::string& map_mission_id_string,
      vi_map::MissionId* mission_id) const;

  // Generalized accessors.
  template <typename IdType>
  void getAllIds(std::vector<IdType>* all_ids_of_type) const;
  template <typename IdType>
  void getAllIdsInMission(
      const MissionId& mission, std::vector<IdType>* all_ids_of_type) const;
  template <typename IdType>
  void getMissionIds(
      const IdType& object_id, vi_map::MissionIdSet* mission_ids) const;
  template <typename IdType>
  void getMissionIds(
      const std::vector<IdType>& object_ids,
      vi_map::MissionIdSet* mission_ids) const;
  template <typename IdType>
  void getMissionIds(
      const std::unordered_set<IdType>& object_ids,
      vi_map::MissionIdSet* mission_ids) const;
  template <typename IdType>
  Eigen::Vector3d get_p_G(const IdType& id) const;
  template <typename IdType>
  vi_map::MissionId getMissionId(const IdType& id) const;

  inline const SensorManager& getSensorManager() const;
  inline SensorManager& getSensorManager();

  inline size_t numVertices() const;
  inline size_t numVerticesInMission(const vi_map::MissionId& mission_id) const;
  inline bool hasVertex(const pose_graph::VertexId& id) const;
  inline vi_map::Vertex& getVertex(const pose_graph::VertexId& id);
  inline const vi_map::Vertex& getVertex(const pose_graph::VertexId& id) const;
  inline vi_map::Vertex* getVertexPtr(const pose_graph::VertexId& id);
  inline const vi_map::Vertex* getVertexPtr(
      const pose_graph::VertexId& id) const;

  inline size_t numEdges() const;
  inline bool hasEdge(const pose_graph::EdgeId& id) const;
  template <typename EdgeType>
  EdgeType& getEdgeAs(const pose_graph::EdgeId& id);
  template <typename EdgeType>
  const EdgeType& getEdgeAs(const pose_graph::EdgeId& id) const;
  template <typename EdgeType>
  EdgeType* getEdgePtrAs(const pose_graph::EdgeId& id);
  template <typename EdgeType>
  const EdgeType* getEdgePtrAs(const pose_graph::EdgeId& id) const;

  /// Creates and adds a new MissionBaseFrame and a new Mission. It does
  /// not set the mission root vertex id, so make sure to set this using
  /// mission->setRootVertexId(id) once the first vertex is created.
  void addNewMissionWithBaseframe(
      const vi_map::MissionId& mission_id, const pose::Transformation& T_G_M,
      const Eigen::Matrix<double, 6, 6>& T_G_M_covariance,
      Mission::BackBone backbone_type);
  void addNewMissionWithBaseframe(
      vi_map::VIMission::UniquePtr mission,
      const vi_map::MissionBaseFrame& mission_base_frame);

  inline size_t numMissions() const;
  inline bool hasMission(const vi_map::MissionId& id) const;
  inline vi_map::VIMission& getMission(const vi_map::MissionId& id);
  inline const vi_map::VIMission& getMission(const vi_map::MissionId& id) const;

  /*void addMissionSensors(
      const std::vector<aslam::Sensor::Ptr>& sensors,
      const vi_map::MissionId& id);*/
  void associateMissionSensors(
      const aslam::SensorIdSet& sensor_ids, const vi_map::MissionId& id);
  void associateMissionNCamera(
      const aslam::SensorId& ncamera_id, const vi_map::MissionId& id);
  void associateMissionImu(
      const aslam::SensorId& imu_id, const vi_map::MissionId& id);
  void getAllAssociatedMissionSensorIds(
      const vi_map::MissionId& id, aslam::SensorIdSet* sensor_ids) const;

  const aslam::NCamera& getMissionNCamera(const vi_map::MissionId& id) const;
  aslam::NCamera::Ptr getMissionNCameraPtr(const vi_map::MissionId& id) const;
  const vi_map::Imu& getMissionImu(const vi_map::MissionId& id) const;
  vi_map::Imu::Ptr getMissionImuPtr(const vi_map::MissionId& id) const;
  const vi_map::Lidar& getMissionLidar(const vi_map::MissionId& id) const;
  vi_map::Lidar::Ptr getMissionLidarPtr(const vi_map::MissionId& id) const;
  const vi_map::Odometry6DoF& getMissionOdometry6DoFSensor(
      const vi_map::MissionId& id) const;
  vi_map::Odometry6DoF::Ptr getMissionOdometry6DoFSensorPtr(
      const vi_map::MissionId& id) const;
  const vi_map::LoopClosureSensor& getMissionLoopClosureSensor(
      const vi_map::MissionId& id) const;
  vi_map::LoopClosureSensor::Ptr getMissionLoopClosureSensorPtr(
      const vi_map::MissionId& id) const;
  const vi_map::Absolute6DoF& getMissionAbsolute6DoFSensor(
      const vi_map::MissionId& id) const;
  vi_map::Absolute6DoF::Ptr getMissionAbsolute6DoFSensorPtr(
      const vi_map::MissionId& id) const;
  const vi_map::WheelOdometry& getMissionWheelOdometrySensor(
      const vi_map::MissionId& id) const;
  vi_map::WheelOdometry::Ptr getMissionWheelOdometrySensorPtr(
      const vi_map::MissionId& id) const;

  inline size_t numMissionBaseFrames() const;
  inline bool hasMissionBaseFrame(const vi_map::MissionBaseFrameId& id) const;
  inline vi_map::MissionBaseFrame& getMissionBaseFrame(
      const vi_map::MissionBaseFrameId& id);
  inline const vi_map::MissionBaseFrame& getMissionBaseFrame(
      const vi_map::MissionBaseFrameId& id) const;
  inline vi_map::MissionBaseFrame& getMissionBaseFrameForMission(
      const vi_map::MissionId& id);
  inline const vi_map::MissionBaseFrame& getMissionBaseFrameForMission(
      const vi_map::MissionId& id) const;

  size_t getNumAbsolute6DoFMeasurementsInMission(
      const vi_map::MissionId& mission_id) const;

  inline size_t numLandmarksInIndex() const;
  size_t numLandmarks() const;
  inline bool hasLandmark(const vi_map::LandmarkId& id) const;

  inline vi_map::Landmark& getLandmark(const vi_map::LandmarkId& id);
  inline const vi_map::Landmark& getLandmark(
      const vi_map::LandmarkId& id) const;

  // Add a new reference from the provided global landmark to the store landmark
  // stored in the storing_vertex_id.
  void addLandmarkIndexReference(
      const vi_map::LandmarkId& landmark_id,
      const pose_graph::VertexId& storing_vertex_id);

  /// Set the storing information of a landmark id.
  inline void updateLandmarkIndexReference(
      const vi_map::LandmarkId& landmark_id,
      const pose_graph::VertexId& storing_vertex_id);

  /// Retrieve the vertex that given landmark is stored in.
  inline const vi_map::Vertex& getLandmarkStoreVertex(
      const vi_map::LandmarkId& id) const;
  /// Retrieve the vertex that given landmark is stored in.
  inline vi_map::Vertex& getLandmarkStoreVertex(const vi_map::LandmarkId& id);

  /// Retrieve the id of the vertex that given landmark is stored in.
  pose_graph::VertexId getLandmarkStoreVertexId(
      const vi_map::LandmarkId& id) const;

  inline void getAllLandmarkIds(LandmarkIdSet* landmark_ids) const;
  inline void getAllLandmarkIds(LandmarkIdList* landmark_ids) const;

  /// Executes the provided function on every landmark in the map.
  /// Landmarks should not be deleted using this method.
  inline void forEachLandmark(
      const std::function<void(const Landmark&)>& action) const;
  inline void forEachLandmark(
      const std::function<void(Landmark*)>& action);  // NOLINT

  inline void forEachVertex(
      const std::function<void(const Vertex&)>& action) const;
  inline void forEachVertex(
      const std::function<void(Vertex*)>& action);  // NOLINT

  /// Executes the provided function on every landmark in the map.
  /// Landmarks should not be deleted using this method.
  inline void forEachLandmark(
      const std::function<void(
          const LandmarkId&, const Landmark&, const Vertex&,
          const MissionBaseFrame&, size_t landmark_counter)>& action) const;

  /// Returns the main graph edge type which connects all vertices. This is
  /// usually IMU or odometry data. This is the edge type which is used to
  /// traverse the graph to reach all vertices.
  pose_graph::Edge::EdgeType getGraphTraversalEdgeType(
      const MissionId& mission_id) const;

  /// Get all the mission ids that contain frames that observe a given landmark.
  inline void getObserverMissionsForLandmark(
      const vi_map::LandmarkId& landmark_id,
      vi_map::MissionIdSet* missions) const;

  /// Get all vertices which contain frames that observe a given landmark.
  inline void getObserverVerticesForLandmark(
      const vi_map::LandmarkId& landmark_id,
      pose_graph::VertexIdSet* observer_vertices) const;

  /// Get all frames that observe a given landmark.
  inline void getVisualFrameIdentifiersForLandmark(
      const vi_map::LandmarkId& landmark_id,
      vi_map::VisualFrameIdentifierSet* observer_frames) const;

  /// Get the number of missions that contain frames that observe a given
  /// landmark.
  inline unsigned int numLandmarkObserverMissions(
      const vi_map::LandmarkId& landmark_id) const;

  // TODO(dymczykm) This method is very specialized and should be moved out of
  // the vi-map.
  /// Return the number of missions that we expect to see this landmark
  /// given that they also see co-observed landmarks from other frames.
  unsigned int numExpectedLandmarkObserverMissions(
      const vi_map::LandmarkId& landmark_id) const;

  /// Add a new landmark to the map by providing the vertex, frame and keypoint
  /// information of the first observation.
  void addNewLandmark(
      const vi_map::Landmark& landmark,
      const pose_graph::VertexId& keypoint_and_store_vertex_id,
      unsigned int frame_index, unsigned int keypoint_index);
  void addNewLandmark(
      const LandmarkId& predefined_landmark_id,
      const KeypointIdentifier& first_observation,
      FeatureType feature_type = kBinary);

  /// Add a new observation to an existing landmark by providing information
  /// about the vertex, frame and keypoint index of the observation.
  void associateKeypointWithExistingLandmark(
      const KeypointIdentifier& keypoint_id, const LandmarkId& landmark_id);
  void associateKeypointWithExistingLandmark(
      const pose_graph::VertexId& keypoint_vertex_id, unsigned int frame_index,
      unsigned int keypoint_index, const LandmarkId& landmark_id);

  inline const vi_map::MissionId& getMissionIdForVertex(
      const pose_graph::VertexId& id) const;
  inline const vi_map::VIMission& getMissionForVertex(
      const pose_graph::VertexId& id) const;
  inline vi_map::VIMission& getMissionForVertex(const pose_graph::VertexId& id);

  /// Get the mission id of the vertex that this landmark is stored in.
  inline const vi_map::MissionId& getMissionIdForLandmark(
      const vi_map::LandmarkId& id) const;

  /// Get the mission of the vertex that this landmark is stored in.
  inline const vi_map::VIMission& getMissionForLandmark(
      const vi_map::LandmarkId& id) const;
  /// Get the mission of the vertex that this landmark is stored in.
  inline vi_map::VIMission& getMissionForLandmark(const vi_map::LandmarkId& id);

  /// Get the baseframe of the mission that a given vertex belongs to.
  inline vi_map::MissionBaseFrame& getMissionBaseFrameForVertex(
      const pose_graph::VertexId& id);
  /// Get the baseframe of the mission that a given vertex belongs to.
  inline const vi_map::MissionBaseFrame& getMissionBaseFrameForVertex(
      const pose_graph::VertexId& id) const;

  inline Eigen::Vector3d getLandmark_LM_p_fi(
      const vi_map::LandmarkId& id) const;

  inline void setLandmark_LM_p_fi(
      const vi_map::LandmarkId& landmark_id, const Eigen::Vector3d& LM_p_fi);
  inline Eigen::Vector3d getLandmark_G_p_fi(const vi_map::LandmarkId& id) const;

  inline Eigen::Vector3d getLandmark_p_I_fi(
      const vi_map::LandmarkId& landmark_id,
      const vi_map::Vertex& observer_vertex) const;
  inline Eigen::Vector3d getLandmark_p_C_fi(
      const LandmarkId& landmark_id, const Vertex& observer_vertex,
      unsigned int frame_idx) const;

  inline Eigen::Vector3d getVertex_G_p_I(
      const pose_graph::VertexId& vertex_id) const;
  inline Eigen::Quaterniond getVertex_G_q_I(
      const pose_graph::VertexId& vertex_id) const;
  inline pose::Transformation getVertex_T_G_I(
      const pose_graph::VertexId& vertex_id) const;
  void getAllVertex_p_G_I(
      const MissionId& mission_id, Eigen::Matrix3Xd* result) const;
  void getVertex_p_G_I_ForVertexSet(
      const pose_graph::VertexIdSet& vertex_ids,
      Eigen::Matrix3Xd* p_G_Is) const;

  inline void getLandmarkDescriptors(
      const LandmarkId& id, DescriptorsType* result) const;

  void getMapFeatureTypes(std::set<FeatureType>* feature_types) const;
  void getMissionFeatureTypes(
      const vi_map::MissionId& mission_id,
      std::set<FeatureType>* feature_types) const;

  /// Return a mapping from mission id to number of landmarks stored in vertices
  /// from this mission.
  inline void getMissionLandmarkCounts(
      MissionToLandmarkCountMap* mission_to_landmark_count) const;

  /// Provide the next vertex id by following the graph-traversal edge type of
  /// the graph.
  inline bool getNextVertex(
      const pose_graph::VertexId& current_vertex_id,
      pose_graph::Edge::EdgeType edge_type,
      pose_graph::VertexId* next_vertex_id) const;

  /// Provide the previous vertex id by following the graph-traversal edge type
  /// of the graph.
  inline bool getPreviousVertex(
      const pose_graph::VertexId& current_vertex_id,
      pose_graph::Edge::EdgeType edge_type,
      pose_graph::VertexId* previous_vertex_id) const;

  inline void getAllVertexIds(pose_graph::VertexIdList* vertices) const;

  inline void forEachVisualFrame(const std::function<void(
                                     const aslam::VisualFrame&, Vertex&, size_t,
                                     const MissionBaseFrame&)>& action);
  inline void forEachVisualFrame(
      const std::function<void(
          const aslam::VisualFrame&, const Vertex&, size_t)>& action) const;
  inline void forEachVisualFrame(
      const std::function<void(const VisualFrameIdentifier&)>& action) const;
  inline void forEachListedVisualFrame(
      const VisualFrameIdentifierList& list,
      const std::function<void(
          const aslam::VisualFrame&, const Vertex&, const size_t,
          const MissionBaseFrame&)>& action) const;

  unsigned int getVertexCountInMission(
      const vi_map::MissionId& mission_id) const;

  /// Get all the vertex ids in the order that they appear when traversing the
  /// pose-graph of the provided mission.
  void getAllVertexIdsInMissionAlongGraph(
      const vi_map::MissionId& mission_id,
      pose_graph::VertexIdList* vertices) const;
  void getAllVertexIdsInMissionAlongGraph(
      const vi_map::MissionId& mission_id,
      const pose_graph::VertexId& starting_verte_id,
      pose_graph::VertexIdList* vertices) const;

  /// Get all the vertex ids in the order that they appear when traversing the
  /// pose-graphs of all missions (sorted by timestamp).
  void getAllVertexIdsAlongGraphsSortedByTimestamp(
      pose_graph::VertexIdList* vertices) const;
  void getVertexIdsByMission(
      vi_map::MissionVertexIdList* mission_to_vertex_ids_map) const;
  void getAllVertexIdsInMission(
      const vi_map::MissionId& mission_id,
      pose_graph::VertexIdList* vertices) const;
  void getAllEdgeIdsInMissionAlongGraph(
      const vi_map::MissionId& mission_id, pose_graph::EdgeIdList* edges) const;
  void getAllEdgeIdsInMissionAlongGraph(
      const vi_map::MissionId& mission_id, pose_graph::Edge::EdgeType edge_type,
      pose_graph::EdgeIdList* edges) const;
  void getAllLandmarkIdsInMission(
      const MissionId& mission_id, LandmarkIdList* landmarks) const;

  inline void getAllEdgeIds(pose_graph::EdgeIdList* edges) const;

  inline void getAllMissionIds(vi_map::MissionIdList* mission_ids) const;
  inline void getAllMissionIds(vi_map::MissionIdSet* mission_ids) const;
  inline void getAllMissionIdsSortedByTimestamp(
      vi_map::MissionIdList* sorted_mission_ids) const;
  void forEachMission(
      const std::function<void(const MissionId&)>& action) const;

  inline void getAllMissionBaseFrameIds(
      vi_map::MissionBaseFrameIdList* frames) const;

  inline void getAllLandmarkIdsObservedAtVertices(
      const pose_graph::VertexIdSet& vertex_ids,
      vi_map::LandmarkIdSet* observed_landmarks) const;

  inline void removeLandmark(const LandmarkId landmark_id);

  inline void addVertex(vi_map::Vertex::UniquePtr vertex_ptr);
  inline void addEdge(vi_map::Edge::UniquePtr edge_ptr);
  inline pose_graph::Edge::EdgeType getEdgeType(
      pose_graph::EdgeId edge_id) const;

  inline void removeVertex(pose_graph::VertexId vertex_id);
  inline void removeEdge(pose_graph::EdgeId edge_id);

  inline size_t removeLoopClosureEdges();
  bool hasEdgesOfType(pose_graph::Edge::EdgeType edge_type) const;

  inline vi_map::MissionId getIdOfFirstMission() const;

  /// Mission selection allows to limit all map operations to consider only
  /// data from a subset of missions (the selected missions).
  /// Until mission-selection is reset the map will appear as if it would only
  /// contain data from the selected missions.

  /// Select the missions provided in the set of missions to select.
  void selectMissions(const vi_map::MissionIdSet& selected_missions_ids) const;

  /// Remove a provided mission from the set of selected missions.
  void deselectMission(const vi_map::MissionId& mission_id) const;

  /// Reset the mission selection: All missions in the map are considered.
  void resetMissionSelection() const;
  unsigned int numSelectedMissions() const;
  const vi_map::MissionIdSet& getSelectedMissions() const;

  /// Merge the two provided landmarks by changing all the back-references
  /// pointing to the "to-merge" landmark to point now to the "into"
  /// landmark. Then the "to-merge" landmark is deleted.
  void mergeLandmarks(
      const vi_map::LandmarkId landmark_id_to_merge,
      const vi_map::LandmarkId& landmark_id_into);

  /// Moves a given landmark to be stored in the "to" vertex
  /// and updating all the references to it.
  void moveLandmarkToOtherVertex(
      const LandmarkId& landmark_id, const pose_graph::VertexId& vertex_id_to);

  /// Move all landmarks stored in the "from" vertex to the "to" vertex
  /// and updating all the references to it.
  void moveLandmarksToOtherVertex(
      const pose_graph::VertexId& vertex_id_from,
      const pose_graph::VertexId& vertex_id_to);

  // Removes every but the nth vertex to compress the map.
  void sparsifyMission(
      const vi_map::MissionId& id, int every_nth_vertex_to_keep);

  void getDistanceTravelledPerMission(
      const vi_map::MissionId& id, double* distance) const;

  bool getEarliestMissionStartTimeNs(int64_t* start_time_ns) const;

  void getStatisticsOfMission(
      const vi_map::MissionId& mission_id,
      const std::set<FeatureType>& feature_types,
      std::vector<std::map<int, size_t>>* num_good_landmarks_per_camera,
      std::vector<std::map<int, size_t>>* num_bad_landmarks_per_camera,
      std::vector<std::map<int, size_t>>* num_unknown_landmarks_per_camera,
      std::vector<std::map<int, size_t>>* total_num_landmarks_per_camera,
      std::map<int, size_t>* num_landmarks,
      std::map<int, size_t>* num_observations, size_t* num_vertices,
      double* duration_s, int64_t* start_time, int64_t* end_time) const;

  std::string printMapStatistics(
      const vi_map::MissionId& mission,
      const unsigned int mission_number) const;

  std::string printMapStatistics(void) const;
  std::string printMapAccumulatedStatistics() const;
  void printNumSensorResourcesOfMission(
      const VIMission& mission,
      const std::function<void(const std::string&, const std::string&, int)>&
          print_aligned_function) const;

  /// Merge two vertices which are directly connected in the pose-graph
  /// (neighbors) by moving the landmarks from the "from" vertex to the "to"
  /// vertex. The "from" vertex is then deleted.
  void mergeNeighboringVertices(
      const pose_graph::VertexId& vertex_id_from,
      const pose_graph::VertexId& vertex_id_to, bool merge_viwls_edges = true,
      bool merge_odometry_edges = true, bool merge_wheel_odometry_edges = true);

  // Returns the number of edges of the given type between merge_into_vertex_id
  // and vertex_to_merge.
  template <typename Edge, vi_map::Edge::EdgeType EdgeType>
  size_t mergeEdgesOfNeighboringVertices(
      const pose_graph::VertexId& merge_into_vertex_id,
      const pose_graph::VertexId& vertex_to_merge);

  // Duplicate the source mission, changing the map object ids such that they
  // can coexist in the same map.
  const vi_map::MissionId duplicateMission(
      const vi_map::MissionId& source_mission_id);

  // Removes references to the mission object - assumes mission is empty.
  void removeMissionObject(
      const vi_map::MissionId& mission_id, bool remove_baseframe);
  // Remove vertices and edges associated with the mission as well as the
  // mission object.
  void removeMission(
      const vi_map::MissionId& mission_id, bool remove_baseframe);

  /// Get the ids of outgoing edges to the provided vertex if they are of the
  /// provided type.
  void getOutgoingOfType(
      const pose_graph::Edge::EdgeType edge_type,
      const pose_graph::VertexId& current_vertex_id,
      pose_graph::EdgeIdList* edge_ids) const;

  /// Get the ids of incoming edges to the provided vertex if they are of the
  /// provided type.
  void getIncomingOfType(
      const pose_graph::Edge::EdgeType ref_edge_type,
      const pose_graph::VertexId& current_vertex_id,
      pose_graph::EdgeIdList* edge_ids) const;

  /// Get an id of a random vertex in the graph, random generator can be
  /// controlled with the seed below.
  void getRandomVertexId(pose_graph::VertexId* vertex_id) const;

  /// Get an id of a random vertex in the mission, random generator can be
  /// controlled with the seed below.
  void getRandomVertexIdInMission(
      const vi_map::MissionId& mission_id,
      pose_graph::VertexId* vertex_id) const;

  pose_graph::VertexId getLastVertexIdOfMission(
      const vi_map::MissionId& mission_id) const;

  /// Set the seed for random selection methods in this class.
  void setRandIntGeneratorSeed(int seed);

  // =========
  // RESOURCES
  // =========

  // Check if all resource ids are unique and if the associated resources exist.
  bool checkResourceConsistency() const;

  // Mission-set-based resources
  // ===========================
  // A set of missions can own a single resource of every type. Like all
  // resources, this data is serialized and deserialzed (with cache) upon
  // accessing the data.

  bool hasMissionResource(
      const backend::ResourceType& resource_type,
      const MissionIdList& involved_mission_ids) const;

  template <typename DataType>
  bool getMissionResource(
      const backend::ResourceType& type,
      const MissionIdList& involved_mission_ids, DataType* resource) const;

  template <typename DataType>
  void storeMissionResource(
      const backend::ResourceType& type, const DataType& resource,
      const MissionIdList& involved_mission_ids);

  template <typename DataType>
  void deleteMissionResource(
      const backend::ResourceType& resource_type,
      const MissionIdList& involved_mission_ids);

  template <typename DataType>
  void replaceMissionResource(
      const backend::ResourceType& resource_type, const DataType& resource,
      const MissionIdList& involved_mission_ids);

  // VisualFrame-based resources
  // ===========================
  // A resource that is associated with a visual frame defined through frame_idx
  // and vertex_ptr. Like all resources, this data is serialized and deserialzed
  // (with cache) upon accessing the data.

  template <typename DataType>
  void storeFrameResourceToFolder(
      const DataType& resource, const std::string& resource_folder,
      const unsigned int frame_idx, const backend::ResourceType& type,
      Vertex* vertex_ptr);
  template <typename DataType>
  void storeFrameResource(
      const DataType& resource, const unsigned int frame_idx,
      const backend::ResourceType& type, Vertex* vertex_ptr);
  template <typename DataType>
  bool getFrameResource(
      const Vertex& vertex, const unsigned int frame_idx,
      const backend::ResourceType& type, DataType* resource) const;
  bool hasFrameResource(
      const Vertex& vertex, const unsigned int frame_idx,
      const backend::ResourceType& type) const;

  template <typename DataType>
  void replaceFrameResource(
      const DataType& resource, const unsigned int frame_idx,
      const backend::ResourceType& type, Vertex* vertex_ptr);

  template <typename DataType>
  void deleteFrameResourcesOfType(
      const unsigned int frame_idx, const backend::ResourceType& type,
      Vertex* vertex_ptr);
  void deleteAllFrameResources(
      const unsigned int frame_idx, const bool delete_from_file_system,
      Vertex* vertex_ptr);

  // Sensor resources
  // ================
  // Timestamped resources that are only associated with a mission and a sensor.
  // These resources need to be timestamped based on a common clock, but not
  // necessarily synchronized with the vertex timestamp. Like all resources,
  // this data is serialized and deserialzed (with cache) upon storing/accessing
  // the data.

  bool hasSensorResource(
      const VIMission& mission, const backend::ResourceType& type,
      const aslam::SensorId& sensor_id, const int64_t timestamp_ns) const;
  bool hasSensorResource(
      const MissionIdList& involved_mission_ids,
      const backend::ResourceType& type) const;

  template <typename DataType>
  bool getSensorResource(
      const VIMission& mission, const backend::ResourceType& type,
      const aslam::SensorId& sensor_id, const int64_t timestamp_ns,
      DataType* resource) const;

  template <typename DataType>
  bool getClosestSensorResource(
      const VIMission& mission, const backend::ResourceType& type,
      const aslam::SensorId& sensor_id, const int64_t timestamp_ns,
      const int64_t tolerance_ns, DataType* resource,
      int64_t* closest_timestamp_ns) const;

  bool findAllCloseSensorResources(
      const VIMission& mission, const backend::ResourceType& type,
      const int64_t timestamp_ns, const int64_t tolerance_ns,
      std::vector<aslam::SensorId>* sensor_ids,
      std::vector<int64_t>* closest_timestamps_ns) const;

  template <typename DataType>
  bool getAllCloseSensorResources(
      const VIMission& mission, const backend::ResourceType& type,
      const int64_t timestamp_ns, const int64_t tolerance_ns,
      std::vector<aslam::SensorId>* sensor_ids,
      std::vector<int64_t>* closest_timestamps_ns,
      std::vector<DataType>* resources) const;

  template <typename DataType>
  void addSensorResource(
      const backend::ResourceType& type, const aslam::SensorId& sensor_id,
      const int64_t timestamp_ns, const DataType& resource, VIMission* mission);

  template <typename DataType>
  bool deleteSensorResource(
      const backend::ResourceType& type, const aslam::SensorId& sensor_id,
      const int64_t timestamp_ns, const bool keep_resource_file,
      VIMission* mission);

  void deleteAllSensorResourcesBeforeTime(
      const vi_map::MissionId& mission_id, int64_t timestamp_ns,
      const bool delete_from_file_system);
  void deleteAllSensorResources(
      const vi_map::MissionId& mission_id, const bool delete_from_file_system);

  // Map interface (for map manager)
  // ===============================
  bool mergeAllMissionsFromMap(const vi_map::VIMap& other) override;
  bool mergeAllSubmapsFromMap(const vi_map::VIMap& submap) override;
  static std::string getSubFolderName();

  static bool hasMapOnFileSystem(const std::string& map_folder);
  bool loadFromFolder(const std::string& map_folder) override;
  bool loadFromFolderDeprecated(const std::string& map_folder);
  bool saveToFolder(
      const std::string& folder_path,
      const backend::SaveConfig& config) override;
  bool saveToMapFolder(const backend::SaveConfig& config);

  inline bool hasLandmarkIdInLandmarkIndex(const vi_map::LandmarkId& id) const {
    return landmark_index.hasLandmark(id);
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  // Functions to retrieve and modify the resource ids associated with a set of
  // missions of this VIMap.These functinos are NOT threadsafe and should only
  // be used by the public mission resource functions defined above.
  bool getResourceIdForMissions(
      const backend::ResourceType& resource_type,
      const MissionIdList& involved_mission_ids,
      backend::ResourceId* resource_id) const;
  void getResourceIdToMissionsMap(
      const backend::ResourceType& resource_type,
      std::unordered_map<backend::ResourceId, MissionIdList>*
          path_resource_list) const;
  void addResourceIdForMissions(
      const backend::ResourceType& resource_type,
      const MissionIdList& involved_mission_ids,
      const backend::ResourceId& resource_id);
  void deleteResourceIdForMissions(
      const backend::ResourceType& resource_type,
      const MissionIdList& involved_mission_ids,
      const backend::ResourceId& resource_id);

  VIMap(const VIMap&) = delete;
  VIMap& operator=(const VIMap&) = delete;

  inline void clear();

  // Merges only the part inside the VIMap, not the objects related to the
  // ResourceMap.
  bool mergeAllMissionsFromMapWithoutResources(const vi_map::VIMap& source_map);
  // Append matching submaps (mission id is the same) of the other map to
  // the base map. If they don't match, return false. Currently assumes there is
  // only one mission in the map to be merged, but can support multiple missions
  // in the base map.
  bool mergeAllSubmapsFromMapWithoutResources(const vi_map::VIMap& source_map);

  // To force const accessors in non-const methods.
  const VIMap* const const_this;
  PoseGraph posegraph;
  VIMissionMap missions;
  MissionBaseFrameMap mission_base_frames;
  LandmarkIndex landmark_index;
  SensorManager sensor_manager_;

  // Adding new data? Don't forget to add it to deepCopy() and swap()!

  // Used for mission-selective VIMap.
  mutable std::unordered_set<vi_map::MissionId> selected_missions_;
  mutable std::default_random_engine generator_;

  mutable std::recursive_mutex resource_mutex_;
};
}  // namespace vi_map

namespace backend {

template <typename MapType>
class MapManager;

template <>
struct traits<vi_map::VIMap> : public MapTraits<vi_map::VIMap> {};

}  // namespace backend

namespace vi_map {
typedef backend::MapManager<vi_map::VIMap> VIMapManager;
}  // namespace vi_map

#include "./vi-map/vi-map-inl.h"
#endif  // VI_MAP_VI_MAP_H_
