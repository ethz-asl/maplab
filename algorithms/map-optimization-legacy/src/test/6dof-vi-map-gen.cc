#include "map-optimization-legacy/test/6dof-vi-map-gen.h"

#include <cmath>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <ceres/ceres.h>
#include <glog/logging.h>
#include <posegraph/unique-id.h>
#include <sensors/imu.h>
#include <vi-map/pose-graph.h>
#include <vi-map/vertex.h>
#include <vi-map/vi-map.h>

#include "map-optimization-legacy/test/6dof-pose-graph-gen.h"
#include "map-optimization-legacy/test/6dof-test-trajectory-gen.h"
#include "map-optimization-legacy/test/vi-optimization-test-helpers.h"

namespace map_optimization_legacy {

void SixDofVIMapGenerator::generateVIMap() {
  graph_gen_.constructCamera();

  graph_gen_.settings_.num_of_landmarks = 500;
  setImuSigmasConstant(1e-10, &graph_gen_.settings_.imu_sigmas);

  graph_gen_.generatePathAndLandmarks();

  const size_t kNumOfVertices = 20;
  const int kNumOfFixedVertices = 1;
  const Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
  const Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero();
  const double kInitialPositionSigma = 0;
  const double kInitialRotationSigma = 0;

  graph_gen_.fillPosegraph(
      kNumOfVertices, kInitialPositionSigma, kInitialRotationSigma, gyro_bias,
      accel_bias, kNumOfFixedVertices);

  pose_graph::VertexIdList all_vertex_ids;
  pose_graph::EdgeIdList all_edge_ids;
  graph_gen_.posegraph_.getAllVertexIds(&all_vertex_ids);
  graph_gen_.posegraph_.getAllEdgeIds(&all_edge_ids);
  CHECK_EQ(kNumOfVertices, all_vertex_ids.size());
  CHECK_EQ(kNumOfVertices - 1, all_edge_ids.size());

  vi_map::MissionId mission_id;
  common::generateId(&mission_id);

  const vi_map::Vertex* vertex_ptr = static_cast<const vi_map::Vertex*>(
      graph_gen_.posegraph_.getVertexPtr(all_vertex_ids[0]));

  pose::Transformation T_G_M;
  T_G_M = vertex_ptr->get_T_M_I();
  Eigen::Matrix<double, 6, 6> T_G_M_covariance;
  T_G_M_covariance.setIdentity();

  vi_map_.addNewMissionWithBaseframe(
      mission_id, T_G_M, T_G_M_covariance, graph_gen_.cameras_,
      vi_map::Mission::BackBone::kViwls);

  constexpr char kImuHardwareId[] = "imu0";
  vi_map::SensorId imu_sensor_id;
  common::generateId(&imu_sensor_id);
  vi_map::Imu::UniquePtr imu_sensor = aligned_unique<vi_map::Imu>(
      imu_sensor_id, static_cast<std::string>(kImuHardwareId));
  imu_sensor->setImuSigmas(graph_gen_.settings_.imu_sigmas);
  vi_map_.getSensorManager().addSensor(std::move(imu_sensor), mission_id);

  for (pose_graph::VertexId& vertex_id : all_vertex_ids) {
    vi_map::Vertex* vertex_ptr = static_cast<vi_map::Vertex*>(
        graph_gen_.posegraph_.getVertexPtrMutable(vertex_id));
    CHECK_NOTNULL(vertex_ptr);

    vertex_ptr->setMissionId(mission_id);
    pose_graph::EdgeIdSet incoming_edges;
    vertex_ptr->getIncomingEdges(&incoming_edges);

    vi_map::Vertex::UniquePtr vertex_copy_ptr(new vi_map::Vertex(*vertex_ptr));
    vi_map_.addVertex(std::move(vertex_copy_ptr));

    if (incoming_edges.empty()) {
      vi_map::VIMission& mission = vi_map_.getMission(mission_id);
      mission.setRootVertexId(vertex_id);
    }
  }
  CHECK_EQ(vi_map_.numVertices(), kNumOfVertices);

  for (const pose_graph::EdgeId& edge_id : all_edge_ids) {
    vi_map::ViwlsEdge* edge_ptr = static_cast<vi_map::ViwlsEdge*>(
        graph_gen_.posegraph_.getEdgePtrMutable(edge_id));
    CHECK_NOTNULL(edge_ptr);

    vi_map::ViwlsEdge::UniquePtr edge_copy_ptr(
        new vi_map::ViwlsEdge(*edge_ptr));
    vi_map_.addEdge(std::move(edge_copy_ptr));
  }
  CHECK_EQ(vi_map_.numEdges(), kNumOfVertices - 1);

  // Add landmarks to the VIMap.
  for (const std::pair<vi_map::LandmarkId, vi_map::Landmark::Ptr>&
           landmark_pair : graph_gen_.landmarks_) {
    bool store_is_set = false;
    pose_graph::VertexId vertex_id_to_store;

    vi_map::Landmark& landmark = *CHECK_NOTNULL(landmark_pair.second.get());

    if (!landmark_pair.first.isValid()) {
      continue;
    }
    CHECK_EQ(landmark_pair.first, landmark.id());

    if (landmark.numberOfObservations() == 0) {
      continue;
    }
    // Get the first observing vertex for this landmark.
    for (const vi_map::KeypointIdentifier& backlink :
         landmark.getObservations()) {
      if (!store_is_set) {
        vertex_id_to_store = backlink.frame_id.vertex_id;
        store_is_set = true;
      }
      // Check that this landmark is in the global IDs for the vertex.
      if (vi_map_.getVertex(backlink.frame_id.vertex_id)
              .getObservedLandmarkId(
                  backlink.frame_id.frame_index, backlink.keypoint_index) !=
          landmark_pair.first) {
        LOG(ERROR) << "No existing reference to this keypoint.";
      }
    }
    Eigen::Vector3d G_p_fi = landmark.get_p_B();
    pose::Transformation G_T_I =
        vi_map_.getVertex(vertex_id_to_store).get_T_M_I();

    Eigen::Matrix<double, 3, 3> I_R_G;
    // I_q_G_passive is equal to G_q_I_active.
    Eigen::Quaterniond I_q_G_passive = G_T_I.getRotation().toImplementation();
    common::toRotationMatrixJPL(I_q_G_passive.coeffs(), &I_R_G);

    Eigen::Vector3d I_p_fi = G_T_I.inverse() * G_p_fi;
    landmark.set_p_B(I_p_fi);

    vi_map_.getVertex(vertex_id_to_store).getLandmarks().addLandmark(landmark);
    vi_map_.landmark_index.addLandmarkAndVertexReference(
        landmark_pair.first, vertex_id_to_store);
  }
}
Eigen::Vector3d SixDofVIMapGenerator::getTrueVertexPosition(
    const pose_graph::VertexId& vertex_id) const {
  return graph_gen_.true_vertex_positions_.at(vertex_id);
}
Eigen::Vector4d SixDofVIMapGenerator::getTrueVertexRotation(
    const pose_graph::VertexId& vertex_id) const {
  return graph_gen_.true_vertex_rotations_.at(vertex_id).coeffs();
}
}  // namespace map_optimization_legacy
