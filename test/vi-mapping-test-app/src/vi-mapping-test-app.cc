#include "vi-mapping-test-app/vi-mapping-test-app.h"

#include <random>

#include <gtest/gtest.h>
#include <map-manager/map-manager.h>
#include <maplab-common/test/testing-predicates.h>
#include <vi-map/check-map-consistency.h>
#include <vi-map/vi-map.h>

namespace visual_inertial_mapping {

VIMappingTestApp::VIMappingTestApp() : random_seed_(5) {}

VIMappingTestApp::~VIMappingTestApp() {
  vi_map::VIMapManager map_manager;
  if (map_manager.hasMap(map_key_)) {
    map_manager.deleteMap(map_key_);
  }
}

void VIMappingTestApp::loadDataset(const std::string& folder_name) {
  // Store the reference keyframe poses and landmark positions.
  vi_map::VIMapManager map_manager;
  map_manager.loadMapFromFolder(folder_name, &map_key_);
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(map_key_);

  vi_map::LandmarkIdSet landmark_ids;
  map->getAllLandmarkIds(&landmark_ids);
  for (const vi_map::LandmarkId& landmark_id : landmark_ids) {
    const Eigen::Vector3d& p_B_fi = map->getLandmark(landmark_id).get_p_B();
    landmark_reference_positions_.insert(std::make_pair(landmark_id, p_B_fi));
  }

  pose_graph::VertexIdList vertex_ids;
  map->getAllVertexIds(&vertex_ids);
  for (const pose_graph::VertexId& vertex_id : vertex_ids) {
    const pose::Transformation& T_M_I = map->getVertex(vertex_id).get_T_M_I();
    vertex_reference_poses_.insert(std::make_pair(vertex_id, T_M_I));
  }
}

vi_map::VIMap* VIMappingTestApp::getMapMutable() {
  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(map_key_);
  return map.get();
}

bool VIMappingTestApp::isMapConsistent() {
  const vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapReadAccess map =
      map_manager.getMapReadAccess(map_key_);
  return vi_map::checkMapConsistency(*map);
}

void VIMappingTestApp::sparsifyMission() {
  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(map_key_);
  vi_map::MissionIdList mission_ids;
  map->getAllMissionIds(&mission_ids);
  CHECK(!mission_ids.empty());
  const vi_map::MissionId& first_mission_id = *mission_ids.begin();

  constexpr int kEveryNthVertexToKeep = 5;
  map->sparsifyMission(first_mission_id, kEveryNthVertexToKeep);
}

size_t VIMappingTestApp::numVerticesOnMap() const {
  const vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapReadAccess map =
      map_manager.getMapReadAccess(map_key_);
  return map->numVertices();
}

void VIMappingTestApp::corruptLandmarkPositions(
    double std_dev_m, int every_nth) {
  CHECK_GE(std_dev_m, 0);
  std::mt19937 gen(random_seed_);
  std::normal_distribution<> dis(0, std_dev_m);

  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(map_key_);

  vi_map::LandmarkIdSet landmark_ids;
  map->getAllLandmarkIds(&landmark_ids);
  unsigned int index = 0;
  for (const vi_map::LandmarkId& landmark_id : landmark_ids) {
    if (index % every_nth == 0) {
      vi_map::Landmark& landmark = map->getLandmark(landmark_id);
      const Eigen::Vector3d& p_B_fi = landmark.get_p_B();
      const Eigen::Vector3d noise(dis(gen), dis(gen), dis(gen));
      landmark.set_p_B(p_B_fi + noise);
    }

    ++index;
  }
}

void VIMappingTestApp::corruptKeyframePoses(
    double position_std_dev_m, double orientation_std_dev_quat, int every_nth) {
  CHECK_GE(position_std_dev_m, 0);
  CHECK_GE(orientation_std_dev_quat, 0);
  std::mt19937 gen(random_seed_);
  std::normal_distribution<> p_dis(0, position_std_dev_m);
  std::normal_distribution<> q_dis(0, orientation_std_dev_quat);

  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(map_key_);

  // Get set of first two vertices that shouldn't be corrupted.
  vi_map::MissionIdList mission_ids;
  map->getAllMissionIds(&mission_ids);
  pose_graph::VertexIdSet not_to_corrupt_vertices;
  for (const vi_map::MissionId& mission_id : mission_ids) {
    const vi_map::Mission& mission = map->getMission(mission_id);
    pose_graph::VertexId current_vertex_id = mission.getRootVertexId();
    not_to_corrupt_vertices.insert(current_vertex_id);

    if (map->getNextVertex(
            current_vertex_id, pose_graph::Edge::EdgeType::kViwls,
            &current_vertex_id)) {
      not_to_corrupt_vertices.insert(current_vertex_id);
    }
  }

  pose_graph::VertexIdList vertex_ids;
  unsigned int index = 0;
  map->getAllVertexIds(&vertex_ids);
  for (const pose_graph::VertexId& vertex_id : vertex_ids) {
    // This vertex will be fixed by the optimizer, do not perturb it.
    if (not_to_corrupt_vertices.count(vertex_id) > 0) {
      continue;
    }

    if (index % every_nth == 0) {
      vi_map::Vertex& vertex = map->getVertex(vertex_id);
      pose::Transformation T_M_I = vertex.get_T_M_I();

      const Eigen::Vector3d p_noise(p_dis(gen), p_dis(gen), p_dis(gen));
      T_M_I.getPosition() += p_noise;

      Eigen::Quaterniond q_noise(1, q_dis(gen), q_dis(gen), q_dis(gen));
      q_noise.normalize();
      T_M_I.getRotation().toImplementation() *= q_noise;

      vertex.set_T_M_I(T_M_I);
    }

    ++index;
  }
}

pose_graph::EdgeId VIMappingTestApp::addWrongLoopClosureEdge() {
  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(map_key_);

  const double kSwitchVariable = 1.0;
  const double kSwitchVariableVariance = 1e-3;
  Eigen::Matrix<double, 6, 6> T_Ia_Ib_covariance;
  T_Ia_Ib_covariance.setIdentity();
  T_Ia_Ib_covariance *= 1e-2 * 1e-2;
  pose::Transformation T_Ia_Ib;

  vi_map::MissionIdList mission_ids;
  map->getAllMissionIds(&mission_ids);

  const vi_map::MissionId& first_mission_id = *mission_ids.begin();

  pose_graph::VertexId current_vertex_id =
      map->getMission(first_mission_id).getRootVertexId();
  pose_graph::VertexIdList sorted_vertex_ids;
  do {
    sorted_vertex_ids.push_back(current_vertex_id);
  } while (map->getNextVertex(
      current_vertex_id, pose_graph::Edge::EdgeType::kViwls,
      &current_vertex_id));

  const unsigned int index_offset_vertex_from =
      static_cast<unsigned int>(0.2 * sorted_vertex_ids.size());
  const unsigned int index_offset_vertex_to =
      static_cast<unsigned int>(0.9 * sorted_vertex_ids.size());

  CHECK_LT(index_offset_vertex_from, sorted_vertex_ids.size());
  CHECK_LT(index_offset_vertex_to, sorted_vertex_ids.size());

  const pose_graph::VertexId vertex_id_from =
      sorted_vertex_ids[index_offset_vertex_from];
  const pose_graph::VertexId vertex_id_to =
      sorted_vertex_ids[index_offset_vertex_to];

  CHECK(vertex_id_from.isValid());
  CHECK(vertex_id_to.isValid());

  pose_graph::EdgeId edge_id;
  common::generateId(&edge_id);
  vi_map::LoopClosureEdge::UniquePtr loop_closure_edge(
      new vi_map::LoopClosureEdge(
          edge_id, vertex_id_from, vertex_id_to, kSwitchVariable,
          kSwitchVariableVariance, T_Ia_Ib, T_Ia_Ib_covariance));

  map->addEdge(std::move(loop_closure_edge));

  return edge_id;
}

const Eigen::Vector3d& VIMappingTestApp::getLandmarkReferencePosition(
    const vi_map::LandmarkId& landmark_id) const {
  CHECK(landmark_id.isValid());
  LandmarkIdPositionPairs::const_iterator it;
  it = landmark_reference_positions_.find(landmark_id);
  CHECK(it != landmark_reference_positions_.end());

  return it->second;
}

const pose::Transformation& VIMappingTestApp::getVertexReferencePose(
    const pose_graph::VertexId& vertex_id) const {
  CHECK(vertex_id.isValid());

  VertexIdPosePairs::const_iterator it;
  it = vertex_reference_poses_.find(vertex_id);
  CHECK(it != vertex_reference_poses_.end());

  return it->second;
}

void VIMappingTestApp::testIfKeyframesMatchReference(double precision_m) const {
  // Store the reference keyframe poses and landmark positions.
  const vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapReadAccess map =
      map_manager.getMapReadAccess(map_key_);

  pose_graph::VertexIdList vertex_ids;
  map->getAllVertexIds(&vertex_ids);
  for (const pose_graph::VertexId& vertex_id : vertex_ids) {
    const pose::Transformation& T_M_I = map->getVertex(vertex_id).get_T_M_I();
    EXPECT_NEAR_ASLAM_TRANSFORMATION(
        getVertexReferencePose(vertex_id), T_M_I, precision_m);
  }
}

void VIMappingTestApp::testIfLandmarksMatchReference(
    double max_distance, double min_required_fraction) const {
  CHECK_GT(min_required_fraction, 0.0 - std::numeric_limits<double>::epsilon());
  CHECK_LE(min_required_fraction, 1.0 + std::numeric_limits<double>::epsilon());
  // Store the reference keyframe poses and landmark positions.
  const vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapReadAccess map =
      map_manager.getMapReadAccess(map_key_);

  vi_map::LandmarkIdSet landmark_ids;
  map->getAllLandmarkIds(&landmark_ids);
  unsigned int num_failing = 0;
  for (const vi_map::LandmarkId& landmark_id : landmark_ids) {
    const Eigen::Vector3d& p_B_fi = map->getLandmark(landmark_id).get_p_B();
    // Scale precision (see documentation of isApprox in Eigen) to check
    // absolute distance in meters between the expected and actual positions.
    const double scaled_precision =
        max_distance /
        std::min(
            p_B_fi.norm(), getLandmarkReferencePosition(landmark_id).norm());
    if (fabs(min_required_fraction - 1.0) <
        std::numeric_limits<double>::epsilon()) {
      EXPECT_NEAR_EIGEN(
          getLandmarkReferencePosition(landmark_id), p_B_fi, scaled_precision);
    } else {
      if (!getLandmarkReferencePosition(landmark_id)
               .isApprox(p_B_fi, scaled_precision)) {
        ++num_failing;
      }
    }
  }

  if (!landmark_ids.empty()) {
    CHECK_LE(
        (static_cast<double>(num_failing) / landmark_ids.size()),
        min_required_fraction);
  }
}

void VIMappingTestApp::testIfSwitchVariablesLargerThan(
    double min_value, double min_required_fraction) const {
  CHECK_GT(min_required_fraction, 0.0 - std::numeric_limits<double>::epsilon());
  CHECK_LE(min_required_fraction, 1.0 + std::numeric_limits<double>::epsilon());
  CHECK_GT(min_value, 0.0 - std::numeric_limits<double>::epsilon());
  CHECK_LE(min_value, 1.0 + std::numeric_limits<double>::epsilon());

  const vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapReadAccess map =
      map_manager.getMapReadAccess(map_key_);
  pose_graph::EdgeIdList all_edge_ids;
  map->getAllEdgeIds(&all_edge_ids);
  unsigned int num_lc_edges = 0;
  unsigned int num_lc_edges_over_threshold = 0;
  for (const pose_graph::EdgeId& edge_id : all_edge_ids) {
    if (map->getEdgeType(edge_id) == pose_graph::Edge::EdgeType::kLoopClosure) {
      ++num_lc_edges;
      const vi_map::LoopClosureEdge& lc_edge =
          map->getEdgeAs<vi_map::LoopClosureEdge>(edge_id);
      if (lc_edge.getSwitchVariable() > min_value) {
        ++num_lc_edges_over_threshold;
      }
    }
  }

  if (num_lc_edges > 0) {
    CHECK_GE(
        (static_cast<double>(num_lc_edges_over_threshold) / num_lc_edges),
        min_required_fraction);
  }
}

double VIMappingTestApp::getSpecificSwitchVariable(
    const pose_graph::EdgeId& edge_id) const {
  const vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapReadAccess map =
      map_manager.getMapReadAccess(map_key_);

  CHECK_EQ(
      static_cast<int>(map->getEdgeType(edge_id)),
      static_cast<int>(pose_graph::Edge::EdgeType::kLoopClosure));

  const vi_map::LoopClosureEdge& lc_edge =
      map->getEdgeAs<vi_map::LoopClosureEdge>(edge_id);
  return lc_edge.getSwitchVariable();
}

}  // namespace visual_inertial_mapping
