#include <Eigen/Core>
#include <maplab-common/pose_types.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>
#include <maplab-common/unique-id.h>
#include <vi-map/mission.h>
#include <vi-map/unique-id.h>
#include <vi-map/vertex.h>

#include "map-optimization-legacy/graph-ba-optimizer.h"

namespace map_optimization_legacy {

class ViwlsGraph : public testing::Test {
 protected:
  ViwlsGraph() : map_() {}

  virtual void SetUp() {
    cameras_ = aslam::NCamera::createTestNCamera(1);
  }

  void createMission();
  void createVertex();

  void corruptTransformations();
  void copyToOptimizer(GraphBaOptimizer* optimizer);
  void copyFromOptimizer(GraphBaOptimizer* optimizer);

  vi_map::VIMission* mission_;
  vi_map::Vertex* vertex_;
  vi_map::MissionBaseFrame* mission_baseframe_;
  aslam::NCamera::Ptr cameras_;

  // Reference transformations.
  Eigen::Vector3d mission_baseframe_position_;
  Eigen::Quaterniond mission_baseframe_orientation_;
  Eigen::Vector3d vertex_position_;
  Eigen::Quaterniond vertex_orientation_;

  vi_map::VIMap map_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

void ViwlsGraph::createMission() {
  vi_map::MissionBaseFrame mission_baseframe;
  vi_map::MissionBaseFrameId baseframe_id;
  common::generateId(&baseframe_id);
  mission_baseframe.setId(baseframe_id);

  mission_baseframe.set_p_G_M(Eigen::Matrix<double, 3, 1>::Zero());
  mission_baseframe.set_q_G_M(Eigen::Quaterniond::Identity());

  vi_map::MissionId mission_id;
  vi_map::VIMission::UniquePtr mission(new vi_map::VIMission);
  common::generateId(&mission_id);
  mission->setId(mission_id);
  mission->setBaseFrameId(baseframe_id);

  mission_baseframe_position_ = mission_baseframe.get_p_G_M();
  mission_baseframe_orientation_ = mission_baseframe.get_q_G_M();

  map_.missions.emplace(mission->id(), std::move(mission));
  map_.mission_base_frames.emplace(mission_baseframe.id(), mission_baseframe);

  mission_ = &(map_.getMission(mission_id));
  mission_baseframe_ = &(map_.getMissionBaseFrame(baseframe_id));
}

void ViwlsGraph::createVertex() {
  CHECK_NOTNULL(mission_);
  CHECK(cameras_);

  Eigen::Quaterniond orientation(0, sqrt(2) / 2, 0, -sqrt(2) / 2);
  Eigen::Vector3d position(3, 4, 5);

  pose_graph::VertexId vertex_id;
  vi_map::Vertex::UniquePtr vertex(new vi_map::Vertex(cameras_));

  common::generateId(&vertex_id);
  vertex->setId(vertex_id);
  vertex->set_p_M_I(position);
  vertex->set_q_M_I(orientation);
  vertex->setMissionId(mission_->id());

  vertex_position_ = vertex->get_p_M_I();
  vertex_orientation_ = vertex->get_q_M_I();

  map_.posegraph.addVertex(std::move(vertex));
  vertex_ = map_.getVertexPtr(vertex_id);
}

void ViwlsGraph::corruptTransformations() {
  CHECK_NOTNULL(vertex_);
  CHECK_NOTNULL(mission_baseframe_);

  Eigen::Vector3d random_position;
  random_position.setRandom();
  Eigen::Quaterniond random_orientation;
  random_orientation.coeffs().setRandom();
  random_orientation.normalize();
  if (random_orientation.w() < 0.0) {
    random_orientation.coeffs() = -random_orientation.coeffs();
  }

  vertex_->set_p_M_I(random_position);
  vertex_->set_q_M_I(random_orientation);
  mission_baseframe_->set_p_G_M(random_position);
  mission_baseframe_->set_q_G_M(random_orientation);
}

void ViwlsGraph::copyToOptimizer(GraphBaOptimizer* optimizer) {
  CHECK_NOTNULL(optimizer);
  optimizer->copyDataFromMap();
}

void ViwlsGraph::copyFromOptimizer(GraphBaOptimizer* optimizer) {
  CHECK_NOTNULL(optimizer);
  optimizer->copyDataToMap();
}

TEST_F(ViwlsGraph, OptimizerCopyFromToMapTest) {
  createMission();
  createVertex();
  GraphBaOptimizer optimizer(&map_);
  copyToOptimizer(&optimizer);
  corruptTransformations();
  copyFromOptimizer(&optimizer);

  EXPECT_NEAR_EIGEN(vertex_->get_p_M_I(), vertex_position_, 1e-15);
  EXPECT_NEAR_EIGEN_QUATERNION(
      vertex_->get_q_M_I(), vertex_orientation_, 1e-15);

  EXPECT_NEAR_EIGEN(
      mission_baseframe_->get_p_G_M(), mission_baseframe_position_, 1e-15);
  EXPECT_NEAR_EIGEN_QUATERNION(
      mission_baseframe_->get_q_G_M(), mission_baseframe_orientation_, 1e-15);
}

}  // namespace map_optimization_legacy

MAPLAB_UNITTEST_ENTRYPOINT
