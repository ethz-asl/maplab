#include "vi-map-helpers/vi-map-geometry.h"

#include <limits>

#include <aslam/frames/visual-frame.h>
#include <maplab-common/geometry.h>
#include <maplab-common/parallel-process.h>
#include <vi-map/vi-map.h>

namespace vi_map_helpers {

VIMapGeometry::VIMapGeometry(const vi_map::VIMap& map) : map_(map) {}

pose::Transformation VIMapGeometry::getVisualFrame_T_G_C(
    const vi_map::VisualFrameIdentifier& frame_id) const {
  return map_.getVertex_T_G_I(frame_id.vertex_id) *
         map_.getVertex(frame_id.vertex_id)
             .getNCameras()
             ->get_T_C_B(frame_id.frame_index)
             .inverse();
}

double VIMapGeometry::getMedianSceneDepth(
    const vi_map::VisualFrameIdentifier& frame_id) const {
  const vi_map::Vertex& vertex = map_.getVertex(frame_id.vertex_id);
  const aslam::VisualFrame& frame = vertex.getVisualFrame(frame_id.frame_index);
  const Eigen::Vector3d p_G_C = getVisualFrame_T_G_C(frame_id).getPosition();

  const size_t num_keypoints = frame.getNumKeypointMeasurements();
  std::vector<double> square_depths;
  square_depths.reserve(num_keypoints);

  vi_map::LandmarkIdList landmark_ids;
  vertex.getFrameObservedLandmarkIds(frame_id.frame_index, &landmark_ids);

  for (const vi_map::LandmarkId& landmark_id : landmark_ids) {
    if (landmark_id.isValid()) {
      const vi_map::Landmark::Quality quality =
          map_.getLandmark(landmark_id).getQuality();
      if (quality != vi_map::Landmark::Quality::kGood) {
        continue;
      }
      square_depths.push_back(
          (map_.getLandmark_G_p_fi(landmark_id) - p_G_C).squaredNorm());
    }
  }
  if (square_depths.empty()) {
    LOG(WARNING)
        << "No landmarks found to compute median scene depth, returning "
           "infinity!";
    return std::numeric_limits<double>::infinity();
  }

  std::nth_element(
      square_depths.begin(), square_depths.begin() + square_depths.size() / 2,
      square_depths.end());
  return sqrt(square_depths[square_depths.size() / 2]);
}

int VIMapGeometry::getNeighboursWithinRange(
    const pose_graph::VertexId& vertex_id, double range_m,
    pose_graph::VertexIdSet* neighbours) const {
  CHECK(map_.hasVertex(vertex_id));
  CHECK_NOTNULL(neighbours);

  neighbours->clear();
  pose_graph::VertexIdList vertex_ids;
  map_.getAllVertexIds(&vertex_ids);
  const Eigen::Vector3d& p_M_I = map_.getVertex(vertex_id).get_p_M_I();
  for (const pose_graph::VertexId& id : vertex_ids) {
    double distance_m = (map_.getVertex(id).get_p_M_I() - p_M_I).norm();
    if (distance_m < range_m) {
      neighbours->insert(id);
    }
  }
  return neighbours->size();
}

void VIMapGeometry::get_p_G_I_CovarianceEigenValuesAndVectorsAscending(
    const vi_map::MissionId& mission_id, Eigen::Vector3d* eigenvalues,
    Eigen::Matrix3d* eigenvectors) const {
  CHECK_NOTNULL(eigenvalues);
  CHECK_NOTNULL(eigenvectors);
  CHECK(map_.hasMission(mission_id));
  Eigen::Matrix3Xd p_G_I;
  map_.getAllVertex_p_G_I(mission_id, &p_G_I);
  common::geometry::computeCovarianceEigenValuesAndVectors(
      p_G_I, eigenvalues, eigenvectors);
}

Eigen::Vector3d VIMapGeometry::get_bv_G_root_average(
    const vi_map::MissionId& mission_id) const {
  CHECK(map_.hasMission(mission_id));
  pose_graph::VertexId root_vertex_id =
      map_.getMission(mission_id).getRootVertexId();
  Eigen::Matrix3Xd p_G_I;
  map_.getAllVertex_p_G_I(mission_id, &p_G_I);
  return p_G_I.rowwise().mean() - map_.getVertex_G_p_I(root_vertex_id);
}

}  // namespace vi_map_helpers
