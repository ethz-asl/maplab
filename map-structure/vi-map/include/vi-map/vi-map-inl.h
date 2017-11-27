#ifndef VI_MAP_VI_MAP_INL_H_
#define VI_MAP_VI_MAP_INL_H_

#include <algorithm>
#include <limits>
#include <map>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <aslam/common/memory.h>
#include <aslam/common/pose-types.h>
#include <eigen-checks/glog.h>
#include <map-resources/resource-common.h>
#include <map-resources/resource-map.h>

#include "vi-map/vertex.h"
#include "vi-map/vi-map.h"
#include "vi-map/vi_map.pb.h"

namespace vi_map {
size_t VIMap::numVertices() const {
  if (selected_missions_.empty()) {
    return posegraph.numVertices();
  } else {
    pose_graph::VertexIdList all_vertex_ids;
    getAllVertexIds(&all_vertex_ids);
    return all_vertex_ids.size();
  }
}

size_t VIMap::numVerticesInMission(const vi_map::MissionId& mission_id) const {
  pose_graph::VertexIdList vertices;
  getAllVertexIdsInMission(mission_id, &vertices);
  return vertices.size();
}

bool VIMap::hasVertex(const pose_graph::VertexId& id) const {
  bool vertex_exists = posegraph.vertexExists(id);
  if (vertex_exists && !selected_missions_.empty()) {
    const vi_map::MissionId& mission_id = getVertex(id).getMissionId();
    if (selected_missions_.count(mission_id) == 0) {
      VLOG(3) << "The vertex " << id << " exists, but it belongs to "
              << " a mission " << mission_id << " that is not selected.";
      return false;
    }
  }
  return vertex_exists;
}
vi_map::Vertex& VIMap::getVertex(const pose_graph::VertexId& id) {
  CHECK(id.isValid());
  vi_map::Vertex& vertex =
      posegraph.getVertexPtrMutable(id)->getAs<vi_map::Vertex>();
  CHECK(vertex.getNCameras() != nullptr);
  return vertex;
}
vi_map::Vertex* VIMap::getVertexPtr(const pose_graph::VertexId& id) {
  CHECK(id.isValid());
  vi_map::Vertex* vertex_ptr = dynamic_cast<vi_map::Vertex*>(  // NOLINT
      posegraph.getVertexPtrMutable(id));
  CHECK(vertex_ptr != nullptr);
  CHECK(vertex_ptr->getNCameras() != nullptr);
  return vertex_ptr;
}
const vi_map::Vertex& VIMap::getVertex(const pose_graph::VertexId& id) const {
  CHECK(id.isValid());
  const vi_map::Vertex& vertex =
      posegraph.getVertexPtr(id)->getAs<const vi_map::Vertex>();
  CHECK(vertex.getNCameras() != nullptr);
  return vertex;
}
const vi_map::Vertex* VIMap::getVertexPtr(
    const pose_graph::VertexId& id) const {
  CHECK(id.isValid());
  const vi_map::Vertex* vertex_ptr =
      dynamic_cast<const vi_map::Vertex*>(  // NOLINT
          posegraph.getVertexPtr(id));
  CHECK(vertex_ptr != nullptr);
  CHECK(vertex_ptr->getNCameras() != nullptr);
  return vertex_ptr;
}

size_t VIMap::numEdges() const {
  if (selected_missions_.empty()) {
    return posegraph.numEdges();
  } else {
    pose_graph::EdgeIdList all_edge_ids;
    getAllEdgeIds(&all_edge_ids);
    return all_edge_ids.size();
  }
}
bool VIMap::hasEdge(const pose_graph::EdgeId& id) const {
  CHECK(id.isValid());
  bool edge_exists = posegraph.edgeExists(id);
  if (edge_exists && !selected_missions_.empty()) {
    const vi_map::MissionId& mission_id_from =
        getMissionIdForVertex(getEdgeAs<vi_map::Edge>(id).from());
    const vi_map::MissionId& mission_id_to =
        getMissionIdForVertex(getEdgeAs<vi_map::Edge>(id).to());
    if (selected_missions_.count(mission_id_from) == 0u &&
        selected_missions_.count(mission_id_to) == 0u) {
      VLOG(3) << "The edge " << id << " exists, but it belongs to a mission "
              << "that is not selected.";
      return false;
    }
  }
  return edge_exists;
}
template <typename EdgeType>
EdgeType& VIMap::getEdgeAs(const pose_graph::EdgeId& id) {
  return posegraph.getEdgePtrMutable(id)->getAs<EdgeType>();
}
template <typename EdgeType>
const EdgeType& VIMap::getEdgeAs(const pose_graph::EdgeId& id) const {
  return posegraph.getEdgePtr(id)->getAs<const EdgeType>();
}
template <typename EdgeType>
EdgeType* VIMap::getEdgePtrAs(const pose_graph::EdgeId& id) {
  EdgeType* edge_ptr =
      dynamic_cast<EdgeType*>(posegraph.getEdgePtrMutable(id));  // NOLINT
  CHECK(edge_ptr != nullptr);
  return edge_ptr;
}
template <typename EdgeType>
const EdgeType* VIMap::getEdgePtrAs(const pose_graph::EdgeId& id) const {
  const EdgeType* edge_ptr =
      dynamic_cast<const EdgeType*>(posegraph.getEdgePtr(id));  // NOLINT
  CHECK(edge_ptr != nullptr);
  return edge_ptr;
}

size_t VIMap::numMissions() const {
  if (selected_missions_.empty()) {
    return missions.size();
  } else {
    return selected_missions_.size();
  }
}
bool VIMap::hasMission(const vi_map::MissionId& id) const {
  CHECK(id.isValid());
  const bool has_mission = missions.count(id) > 0u;
  if (has_mission && !selected_missions_.empty()) {
    // Mission must belong to selected missions to return true.
    return (selected_missions_.count(id) > 0);
  }
  return has_mission;
}

vi_map::VIMission& VIMap::getMission(const vi_map::MissionId& id) {
  CHECK(id.isValid());
  vi_map::VIMission& mission =
      common::getChecked(missions, id)->getAs<vi_map::VIMission>();
  return mission;
}
const vi_map::VIMission& VIMap::getMission(const vi_map::MissionId& id) const {
  CHECK(id.isValid());
  return common::getChecked(missions, id)->getAs<vi_map::VIMission>();
}

vi_map::MissionId VIMap::getIdOfFirstMission() const {
  CHECK(!missions.empty());
  if (selected_missions_.empty()) {
    return missions.begin()->first;
  } else {
    CHECK_GT(missions.count(*selected_missions_.begin()), 0u);
    return *selected_missions_.begin();
  }
}

size_t VIMap::numMissionBaseFrames() const {
  return mission_base_frames.size();
}
bool VIMap::hasMissionBaseFrame(const vi_map::MissionBaseFrameId& id) const {
  CHECK(id.isValid());
  return mission_base_frames.count(id) > 0u;
}
vi_map::MissionBaseFrame& VIMap::getMissionBaseFrame(
    const vi_map::MissionBaseFrameId& id) {
  CHECK(id.isValid());
  return common::getChecked(mission_base_frames, id);
}
const vi_map::MissionBaseFrame& VIMap::getMissionBaseFrame(
    const vi_map::MissionBaseFrameId& id) const {
  CHECK(id.isValid());
  return common::getChecked(mission_base_frames, id);
}

vi_map::MissionBaseFrame& VIMap::getMissionBaseFrameForMission(
    const vi_map::MissionId& id) {
  CHECK(id.isValid());
  const vi_map::VIMission& mission = const_this->getMission(id);
  return getMissionBaseFrame(mission.getBaseFrameId());
}
const vi_map::MissionBaseFrame& VIMap::getMissionBaseFrameForMission(
    const vi_map::MissionId& id) const {
  const vi_map::VIMission& mission = getMission(id);
  return getMissionBaseFrame(mission.getBaseFrameId());
}

void VIMap::getAllLandmarkIds(LandmarkIdSet* landmark_ids) const {
  CHECK_NOTNULL(landmark_ids)->clear();
  if (selected_missions_.empty()) {
    landmark_index.getAllLandmarkIds(landmark_ids);
  } else {
    pose_graph::VertexIdList vertex_ids;
    getAllVertexIds(&vertex_ids);
    for (const pose_graph::VertexId& vertex_id : vertex_ids) {
      for (const vi_map::Landmark& landmark :
           getVertex(vertex_id).getLandmarks()) {
        landmark_ids->insert(landmark.id());
      }
    }
  }
}

void VIMap::getAllLandmarkIds(LandmarkIdList* landmark_ids) const {
  CHECK_NOTNULL(landmark_ids)->clear();
  LandmarkIdSet landmark_set;
  getAllLandmarkIds(&landmark_set);
  landmark_ids->insert(
      landmark_ids->end(), landmark_set.begin(), landmark_set.end());
}

size_t VIMap::numLandmarksInIndex() const {
  return landmark_index.numLandmarks();
}

bool VIMap::hasLandmark(const vi_map::LandmarkId& id) const {
  CHECK(id.isValid());
  return landmark_index.hasLandmark(id);
}

void VIMap::updateLandmarkIndexReference(
    const vi_map::LandmarkId& landmark_id,
    const pose_graph::VertexId& vertex_id) {
  landmark_index.updateVertexOfLandmark(landmark_id, vertex_id);
}

vi_map::Landmark& VIMap::getLandmark(const vi_map::LandmarkId& id) {
  CHECK(id.isValid());

  vi_map::Vertex& vertex = getVertex(landmark_index.getStoringVertexId(id));
  return vertex.getLandmarks().getLandmark(id);
}

const vi_map::Landmark& VIMap::getLandmark(const vi_map::LandmarkId& id) const {
  CHECK(id.isValid());

  const vi_map::Vertex& vertex =
      getVertex(landmark_index.getStoringVertexId(id));
  return vertex.getLandmarks().getLandmark(id);
}

void VIMap::getLandmarkObserverMissions(
    const vi_map::LandmarkId& landmark_id,
    std::unordered_set<vi_map::MissionId>* missions) const {
  CHECK_NOTNULL(missions);
  CHECK(landmark_id.isValid());
  CHECK(hasLandmark(landmark_id))
      << "Landmark " << landmark_id << " not found!";
  missions->clear();

  const vi_map::Landmark& landmark = getLandmark(landmark_id);
  landmark.forEachObservation([&](const KeypointIdentifier& observer_backlink) {
    const vi_map::Vertex& vertex =
        getVertex(observer_backlink.frame_id.vertex_id);
    const vi_map::MissionId& mission_id = vertex.getMissionId();
    if (selected_missions_.empty() ||
        selected_missions_.count(mission_id) > 0) {
      missions->insert(mission_id);
    }
  });
}

void VIMap::getLandmarkObserverVertices(
    const vi_map::LandmarkId& landmark_id,
    pose_graph::VertexIdSet* observer_vertices) const {
  CHECK_NOTNULL(observer_vertices);
  CHECK(landmark_id.isValid());
  CHECK(hasLandmark(landmark_id));
  observer_vertices->clear();

  const vi_map::Landmark& landmark = getLandmark(landmark_id);
  landmark.forEachObservation([&](const KeypointIdentifier& observer_backlink) {
    if (hasVertex(observer_backlink.frame_id.vertex_id)) {
      observer_vertices->insert(observer_backlink.frame_id.vertex_id);
    }
  });
}

unsigned int VIMap::numLandmarkObserverMissions(
    const vi_map::LandmarkId& landmark_id) const {
  CHECK(landmark_id.isValid());
  CHECK(hasLandmark(landmark_id));

  std::unordered_set<vi_map::MissionId> missions;
  const vi_map::Landmark& landmark = getLandmark(landmark_id);
  landmark.forEachObservation([&](const KeypointIdentifier& observer_backlink) {
    if (hasVertex(observer_backlink.frame_id.vertex_id)) {
      const vi_map::Vertex& vertex =
          getVertex(observer_backlink.frame_id.vertex_id);
      missions.insert(vertex.getMissionId());
    }
  });
  return missions.size();
}

vi_map::Vertex& VIMap::getLandmarkStoreVertex(const vi_map::LandmarkId& id) {
  CHECK(id.isValid());
  const pose_graph::VertexId& vertex_id = landmark_index.getStoringVertexId(id);
  return getVertex(vertex_id);
}

const vi_map::Vertex& VIMap::getLandmarkStoreVertex(
    const vi_map::LandmarkId& id) const {
  CHECK(id.isValid());
  const pose_graph::VertexId& vertex_id = landmark_index.getStoringVertexId(id);
  return getVertex(vertex_id);
}

const vi_map::MissionId& VIMap::getMissionIdForVertex(
    const pose_graph::VertexId& id) const {
  CHECK(id.isValid());
  const vi_map::Vertex& vertex = getVertex(id);
  return vertex.getMissionId();
}

const vi_map::VIMission& VIMap::getMissionForVertex(
    const pose_graph::VertexId& id) const {
  CHECK(id.isValid());
  const vi_map::MissionId mission_id = getMissionIdForVertex(id);
  return getMission(mission_id);
}
vi_map::VIMission& VIMap::getMissionForVertex(const pose_graph::VertexId& id) {
  CHECK(id.isValid());
  const vi_map::MissionId mission_id = getMissionIdForVertex(id);
  return getMission(mission_id);
}

const vi_map::MissionId& VIMap::getMissionIdForLandmark(
    const vi_map::LandmarkId& id) const {
  CHECK(id.isValid());
  const vi_map::Vertex& vertex = getLandmarkStoreVertex(id);
  return vertex.getMissionId();
}
const vi_map::VIMission& VIMap::getMissionForLandmark(
    const vi_map::LandmarkId& id) const {
  CHECK(id.isValid());
  const vi_map::MissionId& mission_id = getMissionIdForLandmark(id);
  return getMission(mission_id);
}
vi_map::VIMission& VIMap::getMissionForLandmark(const vi_map::LandmarkId& id) {
  CHECK(id.isValid());
  const vi_map::MissionId& mission_id = getMissionIdForLandmark(id);
  return getMission(mission_id);
}

vi_map::MissionBaseFrame& VIMap::getMissionBaseFrameForVertex(
    const pose_graph::VertexId& id) {
  const vi_map::VIMission& mission = getMissionForVertex(id);
  return getMissionBaseFrame(mission.getBaseFrameId());
}

const vi_map::MissionBaseFrame& VIMap::getMissionBaseFrameForVertex(
    const pose_graph::VertexId& id) const {
  const vi_map::VIMission& mission = getMissionForVertex(id);
  return getMissionBaseFrame(mission.getBaseFrameId());
}

Eigen::Vector3d VIMap::getLandmark_LM_p_fi(const vi_map::LandmarkId& id) const {
  CHECK(id.isValid());
  const vi_map::Vertex& vertex = getLandmarkStoreVertex(id);
  return vertex.getLandmark_p_LM_fi(id);
}

void VIMap::setLandmark_LM_p_fi(
    const vi_map::LandmarkId& landmark_id, const Eigen::Vector3d& LM_p_fi) {
  CHECK(landmark_id.isValid());
  vi_map::Vertex& vertex = getLandmarkStoreVertex(landmark_id);
  vertex.setLandmark_LM_p_fi(landmark_id, LM_p_fi);
}

Eigen::Vector3d VIMap::getLandmark_G_p_fi(const vi_map::LandmarkId& id) const {
  CHECK(id.isValid());

  CHECK(landmark_index.hasLandmark(id));
  const vi_map::Vertex& vertex = getLandmarkStoreVertex(id);
  pose::Position3D LM_p_fi = vertex.getLandmark_p_LM_fi(id);
  const vi_map::VIMission& mission = getMission(vertex.getMissionId());
  const vi_map::MissionBaseFrame& mission_baseframe =
      VIMap::getMissionBaseFrame(mission.getBaseFrameId());

  return mission_baseframe.transformPointInMissionFrameToGlobalFrame(LM_p_fi);
}

Eigen::Vector3d VIMap::getLandmark_p_I_fi(
    const vi_map::LandmarkId& landmark_id,
    const vi_map::Vertex& observer_vertex) const {
  CHECK(hasLandmark(landmark_id));
  CHECK(hasVertex(observer_vertex.id()));

  const Eigen::Vector3d& M_p_I = observer_vertex.get_p_M_I();
  const Eigen::Quaterniond& M_q_I = observer_vertex.get_q_M_I();
  Eigen::Vector3d G_p_fi = getLandmark_G_p_fi(landmark_id);

  pose::Transformation M_T_I(M_q_I, M_p_I);

  const vi_map::MissionBaseFrame& mission_baseframe =
      getMissionBaseFrameForVertex(observer_vertex.id());
  const Eigen::Vector3d M_p_fi =
      mission_baseframe.transformPointInGlobalFrameToMissionFrame(G_p_fi);
  const Eigen::Vector3d I_p_fi = M_T_I.inverse() * M_p_fi;

  return I_p_fi;
}

Eigen::Vector3d VIMap::getLandmark_p_C_fi(
    const LandmarkId& landmark_id, const Vertex& observer_vertex,
    unsigned int frame_idx) const {
  CHECK(hasLandmark(landmark_id));
  CHECK(hasVertex(observer_vertex.id()));

  const aslam::NCamera& ncamera =
      sensor_manager_.getNCameraForMission(observer_vertex.getMissionId());
  const pose::Transformation& C_T_I = ncamera.get_T_C_B(frame_idx);
  const Eigen::Vector3d& M_p_I = observer_vertex.get_p_M_I();
  const Eigen::Quaterniond& M_q_I = observer_vertex.get_q_M_I();
  pose::Transformation M_T_I(M_q_I, M_p_I);

  Eigen::Vector3d G_p_fi = getLandmark_G_p_fi(landmark_id);

  const vi_map::MissionBaseFrame& mission_baseframe =
      getMissionBaseFrameForVertex(observer_vertex.id());
  const Eigen::Vector3d M_p_fi =
      mission_baseframe.transformPointInGlobalFrameToMissionFrame(G_p_fi);

  return C_T_I * M_T_I.inverse() * M_p_fi;
}

Eigen::Vector3d VIMap::getVertex_G_p_I(
    const pose_graph::VertexId& vertex_id) const {
  CHECK(hasVertex(vertex_id));
  const vi_map::Vertex& vertex = getVertex(vertex_id);
  const vi_map::MissionBaseFrame& mission_baseframe =
      getMissionBaseFrameForVertex(vertex_id);
  return mission_baseframe.transformPointInMissionFrameToGlobalFrame(
      vertex.get_p_M_I());
}

inline Eigen::Quaterniond VIMap::getVertex_G_q_I(
    const pose_graph::VertexId& vertex_id) const {
  CHECK(hasVertex(vertex_id));

  const vi_map::Vertex& vertex = getVertex(vertex_id);
  const vi_map::MissionBaseFrame& mission_baseframe =
      getMissionBaseFrameForVertex(vertex_id);
  return mission_baseframe.transformRotationInMissionFrameToGlobalFrame(
      vertex.get_q_M_I());
}

inline pose::Transformation VIMap::getVertex_T_G_I(
    const pose_graph::VertexId& vertex_id) const {
  CHECK(hasVertex(vertex_id));
  const vi_map::Vertex& vertex = getVertex(vertex_id);
  const vi_map::MissionBaseFrame& mission_baseframe =
      getMissionBaseFrameForVertex(vertex_id);
  return mission_baseframe.get_T_G_M() * vertex.get_T_M_I();
}

inline void VIMap::getLandmarkDescriptors(
    const LandmarkId& id, DescriptorsType* result) const {
  CHECK_NOTNULL(result);
  const Landmark& landmark = getLandmark(id);

  int index = 0;
  landmark.forEachObservation([&](const KeypointIdentifier& observer_backlink) {
    const DescriptorType descriptor =
        getVertex(observer_backlink.frame_id.vertex_id)
            .getVisualFrame(observer_backlink.frame_id.frame_index)
            .getDescriptors()
            .col(observer_backlink.keypoint_index);
    if (index == 0) {
      result->resize(descriptor.rows(), landmark.numberOfObservations());
    } else {
      CHECK_EQ(result->rows(), descriptor.rows());
    }
    result->col(index) = descriptor;
    ++index;
  });
}

void VIMap::getMissionLandmarkCounts(
    MissionToLandmarkCountMap* mission_to_landmark_count) const {
  CHECK_NOTNULL(mission_to_landmark_count);
  mission_to_landmark_count->clear();

  // Put missions with zero landmark counts on the map.
  vi_map::MissionIdList mission_ids;
  getAllMissionIds(&mission_ids);
  for (const vi_map::MissionId& mission_id : mission_ids) {
    mission_to_landmark_count->insert(std::make_pair(mission_id, 0));
  }

  // Count landmarks by iterating through vertices/landmark stores.
  pose_graph::VertexIdList vertex_ids;
  posegraph.getAllVertexIds(&vertex_ids);
  for (const pose_graph::VertexId& vertex_id : vertex_ids) {
    const vi_map::Vertex& vertex = getVertex(vertex_id);
    const vi_map::LandmarkStore& landmark_store = vertex.getLandmarks();

    if (hasMission(vertex.getMissionId())) {
      (*mission_to_landmark_count)[vertex.getMissionId()] +=
          landmark_store.size();
    }
  }
}

void VIMap::getAllVertexIds(pose_graph::VertexIdList* vertices) const {
  CHECK_NOTNULL(vertices)->clear();
  if (selected_missions_.empty()) {
    posegraph.getAllVertexIds(vertices);
  } else {
    vi_map::MissionIdList all_missions;
    getAllMissionIds(&all_missions);

    for (const vi_map::MissionId& mission_id : all_missions) {
      pose_graph::VertexIdList mission_vertices;
      getAllVertexIdsInMission(mission_id, &mission_vertices);
      vertices->insert(
          vertices->end(), mission_vertices.begin(), mission_vertices.end());
    }
  }
}

void VIMap::forEachVisualFrame(
    const std::function<void(
        const aslam::VisualFrame& visual_frame, Vertex&, size_t,
        const MissionBaseFrame& mission)>& action) {
  pose_graph::VertexIdList all_vertex_ids;
  getAllVertexIds(&all_vertex_ids);
  for (const pose_graph::VertexId& vertex_id : all_vertex_ids) {
    const MissionBaseFrame& mission_frame =
        getMissionBaseFrameForVertex(vertex_id);
    Vertex& vertex = getVertex(vertex_id);
    size_t num_frames = vertex.numFrames();
    for (size_t i = 0; i < num_frames; ++i) {
      const aslam::VisualFrame& frame = vertex.getVisualFrame(i);
      action(frame, vertex, i, mission_frame);
    }
  }
}

void VIMap::forEachVisualFrame(const std::function<void(
                                   const aslam::VisualFrame& visual_frame,
                                   const Vertex&, size_t)>& action) const {
  pose_graph::VertexIdList all_vertex_ids;
  getAllVertexIds(&all_vertex_ids);
  for (const pose_graph::VertexId& vertex_id : all_vertex_ids) {
    const Vertex& vertex = getVertex(vertex_id);
    size_t num_frames = vertex.numFrames();
    for (size_t i = 0; i < num_frames; ++i) {
      const aslam::VisualFrame& frame = vertex.getVisualFrame(i);
      action(frame, vertex, i);
    }
  }
}

inline void VIMap::forEachVisualFrame(
    const std::function<void(const VisualFrameIdentifier&)>& action) const {
  pose_graph::VertexIdList all_vertex_ids;
  getAllVertexIds(&all_vertex_ids);
  for (const pose_graph::VertexId& vertex_id : all_vertex_ids) {
    const Vertex& vertex = getVertex(vertex_id);
    size_t num_frames = vertex.numFrames();
    for (size_t i = 0; i < num_frames; ++i) {
      action(VisualFrameIdentifier(vertex_id, i));
    }
  }
}

inline void VIMap::forEachListedVisualFrame(
    const VisualFrameIdentifierList& list,
    const std::function<void(
        const aslam::VisualFrame&, const Vertex&, const size_t,
        const MissionBaseFrame&)>& action) const {
  for (const VisualFrameIdentifier& frame_id : list) {
    const MissionBaseFrame& mission_frame =
        getMissionBaseFrameForVertex(frame_id.vertex_id);
    const Vertex& vertex = getVertex(frame_id.vertex_id);
    const aslam::VisualFrame& frame =
        vertex.getVisualFrame(frame_id.frame_index);
    action(frame, vertex, frame_id.frame_index, mission_frame);
  }
}

void VIMap::getAllEdgeIds(pose_graph::EdgeIdList* edges) const {
  CHECK_NOTNULL(edges)->clear();
  if (selected_missions_.empty()) {
    posegraph.getAllEdgeIds(edges);
  } else {
    vi_map::MissionIdList all_missions;
    getAllMissionIds(&all_missions);

    for (const vi_map::MissionId& mission_id : all_missions) {
      pose_graph::EdgeIdList mission_edges;
      getAllEdgeIdsInMissionAlongGraph(mission_id, &mission_edges);
      edges->insert(edges->end(), mission_edges.begin(), mission_edges.end());
    }
  }
}

void VIMap::getAllMissionIds(vi_map::MissionIdList* mission_ids) const {
  CHECK_NOTNULL(mission_ids)->clear();
  if (selected_missions_.empty()) {
    for (const vi_map::VIMissionMap::value_type& pair : missions) {
      mission_ids->emplace_back(pair.first);
    }
  } else {
    mission_ids->insert(
        mission_ids->begin(), selected_missions_.begin(),
        selected_missions_.end());
  }
}

void VIMap::getAllMissionIds(vi_map::MissionIdSet* mission_ids) const {
  CHECK_NOTNULL(mission_ids)->clear();
  vi_map::MissionIdList mission_ids_list;
  getAllMissionIds(&mission_ids_list);
  mission_ids->insert(mission_ids_list.begin(), mission_ids_list.end());
}

void VIMap::getAllMissionIdsSortedByTimestamp(
    vi_map::MissionIdList* sorted_mission_ids) const {
  CHECK_NOTNULL(sorted_mission_ids);
  sorted_mission_ids->clear();

  vi_map::MissionIdList all_mission_ids;
  getAllMissionIds(&all_mission_ids);

  typedef std::pair<int64_t, vi_map::MissionId> OrderingPair;
  typedef std::vector<OrderingPair> OrderingToMissionIds;
  OrderingToMissionIds ordering_to_mission_id;

  int num_empty_missions = 0;
  for (const vi_map::MissionId& mission_id : all_mission_ids) {
    const vi_map::VIMission& mission = getMission(mission_id);
    const pose_graph::VertexId& root_vertex_id = mission.getRootVertexId();
    if (mission.getOrdering() != -1) {
      ordering_to_mission_id.emplace_back(mission.getOrdering(), mission_id);
    } else if (root_vertex_id.isValid()) {
      const vi_map::Vertex& vertex = getVertex(root_vertex_id);
      // Sort missions according to the 0th frame. We want here just
      // approximation of mission ordering.
      // TODO(dymczykm) Using system timestamp is not safe when using multiple
      // devices or rebooting the device.
      static constexpr unsigned int frame_index = 0;
      ordering_to_mission_id.emplace_back(
          vertex.getVisualFrame(frame_index).getTimestampNanoseconds(),
          mission_id);
    } else {
      ordering_to_mission_id.emplace_back(
          std::numeric_limits<int>::max() - num_empty_missions, mission_id);
      ++num_empty_missions;
    }
  }

  std::sort(
      ordering_to_mission_id.begin(), ordering_to_mission_id.end(),
      [](const OrderingPair& lhs, const OrderingPair& rhs) {
        return lhs.first < rhs.first;
      });

  for (const OrderingPair& mission_pair : ordering_to_mission_id) {
    sorted_mission_ids->push_back(mission_pair.second);
  }
}

void VIMap::getAllMissionBaseFrameIds(
    vi_map::MissionBaseFrameIdList* frames) const {
  CHECK_NOTNULL(frames);
  for (const vi_map::MissionBaseFrameMap::value_type& pair :
       mission_base_frames) {
    frames->emplace_back(pair.first);
  }
}

void VIMap::forEachLandmark(
    const std::function<void(const Landmark&)>& action) const {
  pose_graph::VertexIdList vertex_ids;
  getAllVertexIds(&vertex_ids);
  for (const pose_graph::VertexId& vertex_id : vertex_ids) {
    const LandmarkStore& vertex_stored_landmarks =
        getVertex(vertex_id).getLandmarks();
    for (const Landmark& landmark : vertex_stored_landmarks) {
      action(landmark);
    }
  }
}

void VIMap::forEachLandmark(
    const std::function<void(Landmark*)>& action) {  // NOLINT
  pose_graph::VertexIdList vertex_ids;
  getAllVertexIds(&vertex_ids);
  for (const pose_graph::VertexId& vertex_id : vertex_ids) {
    LandmarkStore& vertex_stored_landmarks =
        getVertex(vertex_id).getLandmarks();
    for (Landmark& landmark : vertex_stored_landmarks) {
      action(&landmark);
    }
  }
}

void VIMap::forEachVertex(
    const std::function<void(const Vertex&)>& action) const {
  pose_graph::VertexIdList vertex_ids;
  getAllVertexIds(&vertex_ids);
  for (const pose_graph::VertexId& vertex_id : vertex_ids) {
    action(getVertex(vertex_id));
  }
}

void VIMap::forEachVertex(
    const std::function<void(Vertex*)>& action) {  // NOLINT
  pose_graph::VertexIdList vertex_ids;
  getAllVertexIds(&vertex_ids);
  for (const pose_graph::VertexId& vertex_id : vertex_ids) {
    action(&getVertex(vertex_id));
  }
}

void VIMap::forEachLandmark(
    const std::function<void(
        const LandmarkId&, const Landmark&, const Vertex&,
        const MissionBaseFrame&, size_t landmark_counter)>& action) const {
  pose_graph::VertexIdList vertex_ids;
  getAllVertexIds(&vertex_ids);
  size_t landmark_counter = 0u;
  for (const pose_graph::VertexId& vertex_id : vertex_ids) {
    const Vertex& vertex = getVertex(vertex_id);
    const MissionBaseFrame& mission_frame =
        getMissionBaseFrameForVertex(vertex_id);
    const LandmarkStore& vertex_stored_landmarks = vertex.getLandmarks();
    for (const Landmark& landmark : vertex_stored_landmarks) {
      CHECK(landmark.id().isValid());
      action(landmark.id(), landmark, vertex, mission_frame, landmark_counter);
      ++landmark_counter;
    }
  }
}

void VIMap::removeLandmark(const LandmarkId landmark_id) {
  CHECK(hasLandmark(landmark_id));

  vi_map::Landmark& landmark = getLandmark(landmark_id);
  const pose_graph::VertexId& store_vertex_id =
      landmark_index.getStoringVertexId(landmark_id);
  vi_map::Vertex& store_vertex = getVertex(store_vertex_id);
  CHECK(store_vertex.getLandmarks().hasLandmark(landmark_id));

  // Invalidate all the keypoints seeing the landmark we will delete.
  LandmarkId invalid_global_landmark_id;
  invalid_global_landmark_id.setInvalid();

  // Delete all the observations referencing THIS global landmark ID also.
  // This is important for merged landmarks. We are just deleting one global
  // ID, we should remove all backlinks from the store landmark that point to
  // this specific one.
  KeypointIdentifierList observations = landmark.getObservations();
  for (const KeypointIdentifier& observation : observations) {
    vi_map::Vertex& observer_vertex = getVertex(observation.frame_id.vertex_id);
    observer_vertex.setObservedLandmarkId(
        observation.frame_id.frame_index, observation.keypoint_index,
        invalid_global_landmark_id);
    landmark.removeObservation(observation);
  }
  landmark_index.removeLandmark(landmark_id);
  store_vertex.getLandmarks().removeLandmark(landmark_id);
}

void VIMap::addVertex(vi_map::Vertex::UniquePtr vertex_ptr) {
  CHECK(hasMission(vertex_ptr->getMissionId()));
  CHECK(!hasVertex(vertex_ptr->id())) << "A vertex with id " << vertex_ptr->id()
                                      << " already exists.";

  posegraph.addVertex(std::move(vertex_ptr));
}

void VIMap::addEdge(vi_map::Edge::UniquePtr edge_ptr) {
  CHECK(edge_ptr);
  CHECK(hasMission(getMissionIdForVertex(edge_ptr->to())));
  CHECK(hasMission(getMissionIdForVertex(edge_ptr->from())));
  CHECK(!hasEdge(edge_ptr->id()));

  posegraph.addEdge(std::move(edge_ptr));
}

pose_graph::Edge::EdgeType VIMap::getEdgeType(
    pose_graph::EdgeId edge_id) const {
  return posegraph.getEdgePtr(edge_id)->getType();
}

inline bool VIMap::getNextVertex(
    const pose_graph::VertexId& current_vertex_id,
    pose_graph::Edge::EdgeType edge_type,
    pose_graph::VertexId* next_vertex_id) const {
  CHECK_NOTNULL(next_vertex_id);
  CHECK(hasVertex(current_vertex_id))
      << "No vertex with id " << current_vertex_id.hexString() << ".";

  std::unordered_set<pose_graph::EdgeId> outgoing_edges;
  getVertex(current_vertex_id).getOutgoingEdges(&outgoing_edges);

  for (const pose_graph::EdgeId& edge_id : outgoing_edges) {
    const pose_graph::Edge* edge = posegraph.getEdgePtr(edge_id);
    if (edge->getType() == edge_type) {
      *next_vertex_id = edge->to();
      return true;
    }
  }
  return false;
}

bool VIMap::getPreviousVertex(
    const pose_graph::VertexId& current_vertex_id,
    pose_graph::Edge::EdgeType edge_type,
    pose_graph::VertexId* previous_vertex_id) const {
  CHECK_NOTNULL(previous_vertex_id);

  std::unordered_set<pose_graph::EdgeId> incoming_edges;
  getVertex(current_vertex_id).getIncomingEdges(&incoming_edges);

  for (const pose_graph::EdgeId& edge_id : incoming_edges) {
    const pose_graph::Edge* edge = posegraph.getEdgePtr(edge_id);
    if (edge->getType() == edge_type) {
      *previous_vertex_id = edge->from();
      return true;
    }
  }
  return false;
}

void VIMap::removeVertex(pose_graph::VertexId vertex_id) {
  CHECK(hasVertex(vertex_id));
  const vi_map::Vertex& vertex = getVertex(vertex_id);

  // Verify if the vertex contains no landmarks.
  CHECK_EQ(vertex.getLandmarks().size(), 0u)
      << "Cannot remove vertex " << vertex_id.hexString()
      << " as it still contains landmarks. Please "
      << " dereference and remove the landmarks first.";

  // Verify if the vertex contains no valid global landmark ids.
  vi_map::LandmarkIdList global_landmark_ids;
  vertex.getAllObservedLandmarkIds(&global_landmark_ids);
  for (const vi_map::LandmarkId& global_landmark_id : global_landmark_ids) {
    CHECK(!global_landmark_id.isValid())
        << "Cannot remove vertex " << vertex_id.hexString()
        << " as it still contains valid references "
        << "to global landmark ids. Please dereference the observed landmarks "
        << "first.";
  }

  // Verify if the vertex does not contain any resources.
  for (size_t enum_index = 0u; enum_index < backend::kNumResourceTypes;
       ++enum_index) {
    const backend::ResourceType resource_type =
        static_cast<backend::ResourceType>(enum_index);

    for (unsigned int frame_idx = 0; frame_idx < vertex.numFrames();
         ++frame_idx) {
      CHECK(!vertex.hasFrameResourceOfType(frame_idx, resource_type))
          << "Cannot remove vertex " << vertex_id.hexString()
          << " as it still contains a resource of type " << enum_index
          << " stored in a frame " << frame_idx
          << ". Remove the resource first.";
    }
  }

  posegraph.removeVertex(vertex_id);
}

void VIMap::removeEdge(pose_graph::EdgeId edge_id) {
  CHECK(hasEdge(edge_id));
  posegraph.removeEdge(edge_id);
}

size_t VIMap::removeLoopClosureEdges() {
  return posegraph
      .removeEdgesOfType<pose_graph::Edge::EdgeType::kLoopClosure>();
}

void VIMap::getAllLandmarkIdsObservedAtVertices(
    const pose_graph::VertexIdSet& vertex_ids,
    vi_map::LandmarkIdSet* observed_landmarks) const {
  CHECK_NOTNULL(observed_landmarks)->clear();

  for (const pose_graph::VertexId& vertex_id : vertex_ids) {
    const vi_map::Vertex& vertex = getVertex(vertex_id);
    vi_map::LandmarkIdList global_landmark_ids;
    vertex.getAllObservedLandmarkIds(&global_landmark_ids);

    for (const vi_map::LandmarkId& global_landmark_id : global_landmark_ids) {
      if (global_landmark_id.isValid()) {
        observed_landmarks->insert(global_landmark_id);
      }
    }
  }
}

template <typename DataType>
bool VIMap::getMissionResource(
    const backend::ResourceType& type,
    const MissionIdList& involved_mission_ids, DataType* resource) const {
  CHECK_NOTNULL(resource);
  std::lock_guard<std::recursive_mutex> lock(resource_mutex_);
  backend::ResourceId resource_id;
  if (getResourceIdForMissions(type, involved_mission_ids, &resource_id)) {
    return getResource<DataType>(resource_id, type, resource);
  } else {
    return false;
  }
}

template <typename DataType>
void VIMap::storeMissionResource(
    const backend::ResourceType& type, const DataType& resource,
    const MissionIdList& involved_mission_ids) {
  CHECK(!involved_mission_ids.empty());
  std::lock_guard<std::recursive_mutex> lock(resource_mutex_);
  backend::ResourceId resource_id;
  addResource(type, resource, &resource_id);
  addResourceIdForMissions(type, involved_mission_ids, resource_id);
}

template <typename DataType>
void VIMap::deleteMissionResource(
    const backend::ResourceType& resource_type,
    const MissionIdList& involved_mission_ids) {
  CHECK(!involved_mission_ids.empty());
  std::lock_guard<std::recursive_mutex> lock(resource_mutex_);
  backend::ResourceId resource_id;
  if (getResourceIdForMissions(
          resource_type, involved_mission_ids, &resource_id)) {
    CHECK(resource_id.isValid());
    deleteResource<DataType>(resource_id, resource_type);
    deleteResourceIdForMissions(
        resource_type, involved_mission_ids, resource_id);
  }
}

template <typename DataType>
void VIMap::replaceMissionResource(
    const backend::ResourceType& resource_type, const DataType& resource,
    const MissionIdList& involved_mission_ids) {
  CHECK(!involved_mission_ids.empty());
  std::lock_guard<std::recursive_mutex> lock(resource_mutex_);
  backend::ResourceId resource_id;
  if (getResourceIdForMissions(
          resource_type, involved_mission_ids, &resource_id)) {
    CHECK(resource_id.isValid());
    deleteResource<DataType>(resource_id, resource_type);
    deleteResourceIdForMissions(
        resource_type, involved_mission_ids, resource_id);
  }
  storeMissionResource(resource_type, resource, involved_mission_ids);
}

template <typename DataType>
void VIMap::storeFrameResourceToFolder(
    const DataType& resource, const std::string& resource_folder,
    const unsigned int frame_idx, const backend::ResourceType& type,
    Vertex* vertex_ptr) {
  CHECK_NOTNULL(vertex_ptr);
  CHECK_LT(static_cast<unsigned int>(frame_idx), vertex_ptr->numFrames());
  std::lock_guard<std::recursive_mutex> lock(resource_mutex_);

  backend::ResourceId resource_id;
  if (resource_folder.empty()) {
    addResource(type, resource, resource_folder, &resource_id);
  } else {
    addResource(type, resource, &resource_id);
  }

  vertex_ptr->addFrameResourceIdOfType(frame_idx, type, resource_id);
}

template <typename DataType>
void VIMap::storeFrameResource(
    const DataType& resource, const unsigned int frame_idx,
    const backend::ResourceType& type, Vertex* vertex_ptr) {
  CHECK_NOTNULL(vertex_ptr);
  CHECK_LT(static_cast<unsigned int>(frame_idx), vertex_ptr->numFrames());
  const std::string kUseDefaultResourceFolder = "";
  storeFrameResourceToFolder(
      resource, kUseDefaultResourceFolder, frame_idx, type, vertex_ptr);
}

template <typename DataType>
bool VIMap::getFrameResource(
    const Vertex& vertex, const unsigned int frame_idx,
    const backend::ResourceType& resource_type, DataType* resource) const {
  std::lock_guard<std::recursive_mutex> lock(resource_mutex_);
  backend::ResourceIdSet resource_ids;
  vertex.getFrameResourceIdsOfType(frame_idx, resource_type, &resource_ids);
  if (resource_ids.size() == 1u) {
    return getResource(*(resource_ids.begin()), resource_type, resource);
  } else if (resource_ids.size() > 1u) {
    size_t type = static_cast<size_t>(resource_type);
    LOG(FATAL) << "VisualFrame " << frame_idx << " of Vertex " << vertex.id()
               << " has an invalid number (" << resource_ids.size()
               << ") of resources of type " << backend::ResourceTypeNames[type]
               << ".";
  }
  return false;
}

template <typename DataType>
bool VIMap::hasFrameResource(
    const Vertex& vertex, const unsigned int frame_idx,
    const backend::ResourceType& resource_type) const {
  std::lock_guard<std::recursive_mutex> lock(resource_mutex_);
  return vertex.hasFrameResourceOfType(frame_idx, resource_type);
}

template <typename DataType>
void VIMap::replaceFrameResource(
    const DataType& resource, const unsigned int frame_idx,
    const backend::ResourceType& type, Vertex* vertex_ptr) {
  CHECK_NOTNULL(vertex_ptr);
  CHECK_LT(static_cast<unsigned int>(frame_idx), vertex_ptr->numFrames());

  CHECK(vertex_ptr->hasFrameResourceOfType(frame_idx, type));
  deleteFrameResourcesOfType<DataType>(frame_idx, type, vertex_ptr);
  storeFrameResource(resource, frame_idx, type, vertex_ptr);
}

template <typename DataType>
void VIMap::deleteFrameResourcesOfType(
    const unsigned int frame_idx, const backend::ResourceType& type,
    Vertex* vertex_ptr) {
  CHECK_NOTNULL(vertex_ptr);
  CHECK_LT(static_cast<unsigned int>(frame_idx), vertex_ptr->numFrames());
  std::lock_guard<std::recursive_mutex> lock(resource_mutex_);

  CHECK(vertex_ptr->hasFrameResourceOfType(frame_idx, type));
  backend::ResourceIdSet resource_ids;
  vertex_ptr->getFrameResourceIdsOfType(frame_idx, type, &resource_ids);
  for (const backend::ResourceId& resource_id : resource_ids) {
    deleteResource<DataType>(resource_id, type);
  }
  vertex_ptr->deleteFrameResourceIdsOfType(frame_idx, type);
}

void VIMap::clear() {
  posegraph.clear();
  missions.clear();
  mission_base_frames.clear();
  landmark_index.clear();
  selected_missions_.clear();
}

template <typename DataType>
bool VIMap::getOptionalCameraResource(
    const VIMission& mission, const backend::ResourceType& type,
    const aslam::CameraId& camera_id, const int64_t timestamp_ns,
    DataType* resource) const {
  CHECK_NOTNULL(resource);

  backend::ResourceId resource_id;
  if (!mission.getOptionalCameraResourceId(
          type, camera_id, timestamp_ns, &resource_id)) {
    return false;
  }

  return getResource(resource_id, type, resource);
}

template <typename DataType>
bool VIMap::getClosestOptionalCameraResource(
    const VIMission& mission, const backend::ResourceType& type,
    const aslam::CameraId& camera_id, const int64_t timestamp_ns,
    const int64_t tolerance_ns, DataType* resource,
    int64_t* closest_timestamp_ns) const {
  CHECK_NOTNULL(resource);
  CHECK_NOTNULL(closest_timestamp_ns);

  backend::StampedResourceId stamped_resource_id;
  if (!mission.getClosestOptionalCameraResourceId(
          type, camera_id, timestamp_ns, tolerance_ns, &stamped_resource_id)) {
    return false;
  }
  *closest_timestamp_ns = stamped_resource_id.first;

  return getResource(stamped_resource_id.second, type, resource);
}

template <typename DataType>
void VIMap::addOptionalCameraResource(
    const backend::ResourceType& type, const aslam::CameraId& camera_id,
    const int64_t timestamp_ns, const DataType& resource, VIMission* mission) {
  CHECK_NOTNULL(mission);

  CHECK(!mission->hasOptionalCameraResourceId(type, camera_id, timestamp_ns));

  backend::ResourceId resource_id;
  addResource(type, resource, &resource_id);

  mission->addOptionalCameraResourceId(
      type, camera_id, resource_id, timestamp_ns);
}

template <typename DataType>
bool VIMap::deleteOptionalCameraResource(
    const backend::ResourceType& type, const aslam::CameraId& camera_id,
    const int64_t timestamp_ns, VIMission* mission) {
  constexpr bool kKeepResourceFile = false;
  return deleteOptionalCameraResource<DataType>(
      type, camera_id, timestamp_ns, kKeepResourceFile, mission);
}

template <typename DataType>
bool VIMap::deleteOptionalCameraResource(
    const backend::ResourceType& type, const aslam::CameraId& camera_id,
    const int64_t timestamp_ns, const bool keep_resource_file,
    VIMission* mission) {
  CHECK_NOTNULL(mission);

  backend::ResourceId resource_id;
  if (!mission->getOptionalCameraResourceId(
          type, camera_id, timestamp_ns, &resource_id)) {
    return false;
  }

  if (!mission->deleteOptionalCameraResourceId(type, camera_id, timestamp_ns)) {
    return false;
  }

  return deleteResource<DataType>(resource_id, type, keep_resource_file);
}

template <typename DataType>
bool VIMap::getAllCloseOptionalCameraResources(
    const VIMission& mission, const backend::ResourceType& type,
    const int64_t timestamp_ns, const int64_t tolerance_ns,
    aslam::CameraIdList* camera_ids,
    std::vector<int64_t>* closest_timestamps_ns,
    std::vector<DataType>* resources) const {
  CHECK_NOTNULL(camera_ids);
  CHECK_NOTNULL(closest_timestamps_ns);
  CHECK_NOTNULL(resources);

  if (!findAllCloseOptionalCameraResources(
          mission, type, timestamp_ns, tolerance_ns, camera_ids,
          closest_timestamps_ns)) {
    return false;
  }
  const size_t num_resources = camera_ids->size();
  CHECK_EQ(num_resources, closest_timestamps_ns->size());
  resources->resize(num_resources);

  for (size_t idx = 0u; idx < num_resources; ++idx) {
    const aslam::CameraId& camera_id = (*camera_ids)[idx];
    const int64_t timestamp_ns = (*closest_timestamps_ns)[idx];
    DataType& resource = (*resources)[idx];
    CHECK(
        getOptionalCameraResource(
            mission.id(), type, timestamp_ns, tolerance_ns, camera_id,
            timestamp_ns, &resource));
  }
  return true;
}

const SensorManager& VIMap::getSensorManager() const {
  return sensor_manager_;
}

SensorManager& VIMap::getSensorManager() {
  return sensor_manager_;
}

template<class MeasurementType>
const MeasurementBuffer<MeasurementType>& VIMap::getOptionalSensorMeasurements(
    const SensorId& sensor_id, const MissionId& mission_id) const {
  CHECK(sensor_manager_.hasSensor(sensor_id));
  CHECK(mission_id.isValid());
  return common::getChecked(optional_sensor_data_map_, mission_id)
      .getMeasurements<MeasurementType>(sensor_id);
}

template <class MeasurementType>
inline void VIMap::addOptionalSensorMeasurement(
    const MeasurementType& measurement, const MissionId& mission_id) {
  CHECK(hasMission(mission_id));
  const SensorId& sensor_id = measurement.getSensorId();
  CHECK(sensor_manager_.hasSensor(sensor_id));
  OptionalSensorDataMap::iterator optional_sensor_data_iterator =
      optional_sensor_data_map_.find(mission_id);
  if (optional_sensor_data_iterator == optional_sensor_data_map_.end()) {
    OptionalSensorData optional_sensor_data;
    optional_sensor_data.addMeasurement(measurement);
    optional_sensor_data_map_.emplace(mission_id, optional_sensor_data);
  } else {
    optional_sensor_data_iterator->second.addMeasurement(measurement);
  }
}
}  // namespace vi_map

#endif  // VI_MAP_VI_MAP_INL_H_
