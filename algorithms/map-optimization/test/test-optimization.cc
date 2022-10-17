#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/distortion-fisheye.h>
#include <aslam/cameras/ncamera.h>
#include <aslam/cameras/random-camera-generator.h>
#include <ceres/ceres.h>
#include <map-manager/map-manager.h>
#include <map-optimization/solver-options.h>
#include <map-optimization/solver.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>
#include <memory>
#include <vi-map/test/vi-map-generator.h>
#include <vi-map/vi-map.h>
#include <vi-map/vi-mission.h>
#include <vi-mapping-test-app/vi-mapping-test-app.h>
#include <visualization/rviz-visualization-sink.h>
#include <visualization/viwls-graph-plotter.h>

#include "map-optimization/vi-map-optimizer.h"

namespace visual_inertial_mapping {

class ViMappingTest : public ::testing::Test {
 protected:
  ViMappingTest() : ::testing::Test() {
    constexpr bool kVisualizeMaps = false;
    if (kVisualizeMaps) {
      visualization::RVizVisualizationSink::init();
      plotter_.reset(new visualization::ViwlsGraphRvizPlotter);
    }
  }

  virtual void SetUp() {
    test_app_.loadDataset("./test_maps/common_test_map");
  }

  virtual void corruptVertices();
  virtual void corruptLandmarks();
  void testWheelCalibrationTwoVertices();
  void testWheelCalibrationThreeVertices();

  virtual void corruptAbs6DoFSensorExtrinsics();
  virtual void corruptCameraExtrinsics();
  virtual void addLoopClosureEdges();
  virtual void addWrongLoopClosureEdge();
  virtual void addAbsolute6DoFConstraints();
  virtual void visualizeMap();

  bool optimize(
      const bool vision_only, const bool absolute_6dof, const bool lc_edges,
      const bool fix_camera_extrinsics, const bool fix_abs_6dof_extrinsics);

  VIMappingTestApp test_app_;

  std::unique_ptr<visualization::ViwlsGraphRvizPlotter> plotter_;
};

void ViMappingTest::visualizeMap() {
  if (plotter_) {
    plotter_->visualizeMap(*test_app_.getMapMutable());
  }
}

void ViMappingTest::corruptVertices() {
  constexpr double kKeyframePositionStdDevM = 0.2;
  constexpr double kOrientationStdDevQuat = 0.05;
  constexpr int kEveryNthToCorrupt = 1;
  test_app_.corruptKeyframePoses(
      kKeyframePositionStdDevM, kOrientationStdDevQuat, kEveryNthToCorrupt);
}

void ViMappingTest::corruptLandmarks() {
  constexpr double kLandmarkPositionStdDevM = 0.2;
  constexpr int kEveryNthToCorrupt = 1;
  test_app_.corruptLandmarkPositions(
      kLandmarkPositionStdDevM, kEveryNthToCorrupt);
}

void ViMappingTest::corruptAbs6DoFSensorExtrinsics() {
  constexpr double kPositionStdDevM = 0.5;
  constexpr double kOrientationStdDevQuat = 0.05;
  test_app_.corruptAbs6DoFSensorExtrinsics(
      kPositionStdDevM, kOrientationStdDevQuat);
}

void ViMappingTest::corruptCameraExtrinsics() {
  constexpr double kPositionStdDevM = 0.03;
  constexpr double kOrientationStdDevQuat = 0.01;
  test_app_.corruptCameraExtrinsics(kPositionStdDevM, kOrientationStdDevQuat);
}

void ViMappingTest::addLoopClosureEdges() {
  constexpr int kAddBetweenEveryNthVertex = 10;
  test_app_.addLoopClosureEdges(kAddBetweenEveryNthVertex);
}

void ViMappingTest::addWrongLoopClosureEdge() {
  test_app_.addWrongLoopClosureEdge();
}

void ViMappingTest::addAbsolute6DoFConstraints() {
  constexpr int kAddAtEveryNthVertex = 10;
  test_app_.addAbsolute6DoFConstraints(kAddAtEveryNthVertex);
}

bool ViMappingTest::optimize(
    const bool vision_only, const bool absolute_6dof, const bool lc_edges,
    const bool fix_camera_extrinsics, const bool fix_abs_6dof_extrinsics) {
  map_optimization::ViProblemOptions options =
      map_optimization::ViProblemOptions::initFromGFlags();
  options.fix_landmark_positions = vision_only;
  options.add_inertial_constraints = !vision_only;
  options.add_absolute_pose_constraints = absolute_6dof;
  options.add_loop_closure_edges = lc_edges;
  options.fix_extrinsics_rotation = fix_camera_extrinsics;
  options.fix_extrinsics_translation = fix_camera_extrinsics;
  options.fix_absolute_pose_sensor_extrinsics = fix_abs_6dof_extrinsics;

  visualization::ViwlsGraphRvizPlotter* plotter = nullptr;
  constexpr bool kSignalHandlerEnabled = false;
  map_optimization::VIMapOptimizer optimizer(plotter, kSignalHandlerEnabled);

  vi_map::VIMapManager map_manager;
  vi_map::VIMap* map = CHECK_NOTNULL(test_app_.getMapMutable());

  vi_map::MissionIdSet mission_ids;
  map->getAllMissionIds(&mission_ids);
  return optimizer.optimize(options, mission_ids, map);
}

void ViMappingTest::testWheelCalibrationThreeVertices() {
  // G: global frame
  // A: point A frame
  // B: point B frame
  // M: mission

  std::string bold = "\e[1m";
  std::string reset = "\e[0m";

  aslam::Quaternion q_M_IA(1, 0, 0, 0);
  aslam::Position3D p_M_IA(0, 0, 0);
  aslam::Transformation T_M_I_A(q_M_IA, p_M_IA);

  aslam::Quaternion q_M_I_B(1, 0, 0, 0);
  aslam::Position3D p_M_I_B(2, 0, 0);
  aslam::Transformation T_M_I_B(q_M_I_B, p_M_I_B);

  aslam::Quaternion q_M_I_C(1, 0, 0, 0);
  aslam::Position3D p_M_I_C(2, 2, 0);
  aslam::Transformation T_M_I_C(q_M_I_C, p_M_I_C);

  Eigen::Matrix4d T_A_B_eig;
  T_A_B_eig << 1, 0, 0, 2, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
  aslam::Transformation T_A_B(T_A_B_eig);

  Eigen::Matrix4d T_B_C_eig;
  T_B_C_eig << 1, 0, 0, 0, 0, 1, 0, 2, 0, 0, 1, 0, 0, 0, 0, 1;
  aslam::Transformation T_B_C(T_B_C_eig);

  const Eigen::Matrix<double, 6, 6> T_A_B_covariance_p_q =
      Eigen::Matrix<double, 6, 6>::Identity() * 0.01;

  aslam::Quaternion q_S_B(0.9961947, 0.0871557, 0, 0);  // 10 deg X
  aslam::Position3D p_S_B;
  p_S_B << 0, 0, 0;
  aslam::Transformation T_S_B(q_S_B, p_S_B);
  map_optimization::ViProblemOptions options =
      map_optimization::ViProblemOptions::initFromGFlags();
  options.fix_wheel_extrinsics = false;
  options.fix_vertices = true;

  // Generate a map.
  vi_map::VIMap::Ptr map(new vi_map::VIMap);
  vi_map::MissionId mission_id = aslam::createRandomId<vi_map::MissionId>();
  // CAMERA (not used but required to run code)
  aslam::NCamera::Ptr cameras_;
  cameras_ = aslam::createTestNCamera(1);
  map->addNewMissionWithBaseframe(
      mission_id, aslam::Transformation(),
      Eigen::Matrix<double, 6, 6>::Identity(),
      vi_map::Mission::BackBone::kWheelOdometry);

  // SENSORS
  aslam::SensorId wheel_odometry_sensor_id;
  aslam::generateId(&wheel_odometry_sensor_id);
  vi_map::WheelOdometry::UniquePtr wheel_odometry_sensor(
      new vi_map::WheelOdometry(wheel_odometry_sensor_id, "wheel_topic"));
  CHECK(wheel_odometry_sensor);
  CHECK(wheel_odometry_sensor_id.isValid());
  // T_S_B corresponds to T_B_S where base frame B = imu frame I
  map->getSensorManager().addSensor<vi_map::WheelOdometry>(
      std::move(wheel_odometry_sensor), wheel_odometry_sensor_id, T_S_B);
  aslam::SensorIdSet sensor_ids;
  map->getSensorManager().getAllSensorIds(&sensor_ids);
  map->associateMissionSensors(sensor_ids, mission_id);

  vi_map::Vertex::UniquePtr vA(new vi_map::Vertex(cameras_));
  pose_graph::VertexId id_A = aslam::createRandomId<pose_graph::VertexId>();
  vA->setId(id_A);
  vA->setMissionId(mission_id);
  vA->set_T_M_I(T_M_I_A);
  vi_map::Vertex::UniquePtr vB(new vi_map::Vertex(cameras_));
  pose_graph::VertexId id_B = aslam::createRandomId<pose_graph::VertexId>();
  vB->setId(id_B);
  vB->setMissionId(mission_id);
  vB->set_T_M_I(T_M_I_B);

  vi_map::Vertex::UniquePtr vC(new vi_map::Vertex(cameras_));
  pose_graph::VertexId id_C = aslam::createRandomId<pose_graph::VertexId>();
  vC->setId(id_C);
  vC->setMissionId(mission_id);
  vC->set_T_M_I(T_M_I_C);

  map->addVertex(std::move(vA));
  map->addVertex(std::move(vB));
  map->addVertex(std::move(vC));
  map->getMission(mission_id).setRootVertexId(id_A);

  pose_graph::EdgeId edge_id_AB;
  aslam::generateId(&edge_id_AB);
  vi_map::Edge::UniquePtr edge_A_B(new vi_map::TransformationEdge(
      vi_map::Edge::EdgeType::kWheelOdometry, edge_id_AB /* id */,
      id_A /* from */, id_B /* to */, T_A_B, T_A_B_covariance_p_q,
      wheel_odometry_sensor_id));
  pose_graph::EdgeId edge_id_BC;
  aslam::generateId(&edge_id_BC);
  vi_map::Edge::UniquePtr edge_B_C(new vi_map::TransformationEdge(
      vi_map::Edge::EdgeType::kWheelOdometry, edge_id_BC /* id */,
      id_B /* from */, id_C /* to */, T_B_C, T_A_B_covariance_p_q,
      wheel_odometry_sensor_id));
  map->addEdge(std::move(edge_A_B));
  map->addEdge(std::move(edge_B_C));

  // OPTIMIZE
  options.solver_options.max_num_iterations = 60;
  options.add_inertial_constraints = false;
  options.add_visual_constraints = false;
  options.add_wheel_odometry_constraints = true;

  visualization::ViwlsGraphRvizPlotter* plotter = nullptr;
  constexpr bool kSignalHandlerEnabled = false;
  map_optimization::VIMapOptimizer optimizer(plotter, kSignalHandlerEnabled);

  vi_map::MissionIdSet mission_ids;
  map->getAllMissionIds(&mission_ids);

  map_optimization::OptimizationProblem::UniquePtr optimization_problem(
      map_optimization::constructOptimizationProblem(
          mission_ids, options, map.get()));

  map_optimization::solve(options.solver_options, optimization_problem.get());

  // RESULTS
  aslam::Transformation T_S_B_opt =
      map->getSensorManager().getSensor_T_B_S(wheel_odometry_sensor_id);
  VLOG(2) << "========================================";
  VLOG(2) << bold << "T_S_B:\n" << T_S_B;
  VLOG(2) << "T_S_B opt:\n" << T_S_B_opt;
  VLOG(2) << "T_A_B\n" << T_A_B;
  VLOG(2)
      << "T_A_B opt\n"
      << map->getEdgePtrAs<vi_map::TransformationEdge>(edge_id_AB)->get_T_A_B();

  VLOG(2) << "T_M_I_A:\n" << T_M_I_A;
  VLOG(2) << "T_M_I_A opt:\n" << map->getVertex(id_A).get_T_M_I();
  VLOG(2) << "T_M_I_B:\n" << T_M_I_B;
  VLOG(2) << "T_M_I_B opt:\n" << map->getVertex(id_B).get_T_M_I();
  VLOG(2) << "T_M_I_C:\n" << T_M_I_C;

  // Check if optimization recovers unit quaternion for extrinsics.
  EXPECT_NEAR_KINDR_QUATERNION(
      aslam::Quaternion(), T_S_B_opt.getRotation(), 0.0001);

  // Check if vertex positions remained the same
  EXPECT_NEAR_EIGEN(
      T_M_I_A.getPosition(), map->getVertex(id_A).get_T_M_I().getPosition(),
      1e-12);
  EXPECT_NEAR_EIGEN(
      T_M_I_B.getPosition(), map->getVertex(id_B).get_T_M_I().getPosition(),
      1e-12);
  EXPECT_NEAR_EIGEN(
      T_M_I_C.getPosition(), map->getVertex(id_C).get_T_M_I().getPosition(),
      1e-12);
  EXPECT_NEAR_KINDR_QUATERNION(
      T_M_I_A.getRotation(), map->getVertex(id_A).get_T_M_I().getRotation(),
      1e-12);
  EXPECT_NEAR_KINDR_QUATERNION(
      T_M_I_B.getRotation(), map->getVertex(id_B).get_T_M_I().getRotation(),
      1e-12);
  EXPECT_NEAR_KINDR_QUATERNION(
      T_M_I_C.getRotation(), map->getVertex(id_C).get_T_M_I().getRotation(),
      1e-12);
}

void ViMappingTest::testWheelCalibrationTwoVertices() {
  // G: global frame
  // A: point A frame
  // B: point B frame
  // M: mission

  std::string bold = "\e[1m";
  std::string reset = "\e[0m";
  Eigen::Matrix4d T_M_I_A_eig;
  T_M_I_A_eig << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
  aslam::Transformation T_M_I_A(T_M_I_A_eig);

  Eigen::Matrix4d T_M_I_B_eig;
  T_M_I_B_eig << 1, 0, 0, 0, 0, 1, 0, 3, 0, 0, 1, 0, 0, 0, 0, 1;
  aslam::Transformation T_M_I_B(T_M_I_B_eig);

  Eigen::Matrix4d T_A_B_eig;
  T_A_B_eig << 1, 0, 0, 0, 0, 1, 0, 3, 0, 0, 1, 0, 0, 0, 0, 1;
  aslam::Transformation T_A_B(T_A_B_eig);

  const Eigen::Matrix<double, 6, 6> T_A_B_covariance_p_q =
      Eigen::Matrix<double, 6, 6>::Identity() * 0.01;

  //   aslam::Quaternion q_S_B(0.9998477, 0.0174524, 0, 0);  // 2 deg
  //   aslam::Quaternion q_S_B(0.9990482 , 0.0436194, 0, 0);  // 5 deg
  aslam::Quaternion q_S_B(0.9848078, 0.1736482, 0, 0);  // 20 deg
  aslam::Position3D p_S_B;
  p_S_B << 0, 0, 1;
  aslam::Transformation T_S_B(q_S_B, p_S_B);

  map_optimization::ViProblemOptions options =
      map_optimization::ViProblemOptions::initFromGFlags();
  options.fix_wheel_extrinsics = false;
  options.fix_vertices = true;

  // Generate a map.
  vi_map::VIMap::Ptr map(new vi_map::VIMap);
  vi_map::MissionId mission_id = aslam::createRandomId<vi_map::MissionId>();
  // CAMERA (not used but required to run code)
  aslam::NCamera::Ptr cameras_;
  cameras_ = aslam::createTestNCamera(1);
  map->addNewMissionWithBaseframe(
      mission_id, aslam::Transformation(),
      Eigen::Matrix<double, 6, 6>::Identity(),
      vi_map::Mission::BackBone::kWheelOdometry);

  // SENSORS
  aslam::SensorId wheel_odometry_sensor_id;
  aslam::generateId(&wheel_odometry_sensor_id);
  vi_map::WheelOdometry::UniquePtr wheel_odometry_sensor(
      new vi_map::WheelOdometry(wheel_odometry_sensor_id, "wheel_topic"));
  CHECK(wheel_odometry_sensor);
  CHECK(wheel_odometry_sensor_id.isValid());
  // T_S_B corresponds to T_B_S where base frame B = imu frame I
  map->getSensorManager().addSensor<vi_map::WheelOdometry>(
      std::move(wheel_odometry_sensor), wheel_odometry_sensor_id, T_S_B);
  aslam::SensorIdSet sensor_ids;
  map->getSensorManager().getAllSensorIds(&sensor_ids);
  map->associateMissionSensors(sensor_ids, mission_id);

  vi_map::Vertex::UniquePtr vA(new vi_map::Vertex(cameras_));
  pose_graph::VertexId id_A = aslam::createRandomId<pose_graph::VertexId>();
  vA->setId(id_A);
  vA->setMissionId(mission_id);
  vA->set_T_M_I(T_M_I_A);
  vi_map::Vertex::UniquePtr vB(new vi_map::Vertex(cameras_));
  pose_graph::VertexId id_B = aslam::createRandomId<pose_graph::VertexId>();
  vB->setId(id_B);
  vB->setMissionId(mission_id);
  vB->set_T_M_I(T_M_I_B);

  map->addVertex(std::move(vA));
  map->addVertex(std::move(vB));
  map->getMission(mission_id).setRootVertexId(id_A);

  pose_graph::EdgeId edge_id_AB;
  aslam::generateId(&edge_id_AB);
  vi_map::Edge::UniquePtr edge_A_B(new vi_map::TransformationEdge(
      vi_map::Edge::EdgeType::kWheelOdometry, edge_id_AB /* id */,
      id_A /* from */, id_B /* to */, T_A_B, T_A_B_covariance_p_q,
      wheel_odometry_sensor_id));
  map->addEdge(std::move(edge_A_B));

  // OPTIMIZE
  ceres::Solver::Options solver_options =
      map_optimization::initSolverOptionsFromFlags();
  solver_options.max_num_iterations = 60;

  options.add_inertial_constraints = false;
  options.add_visual_constraints = false;
  options.add_wheel_odometry_constraints = true;

  visualization::ViwlsGraphRvizPlotter* plotter = nullptr;
  constexpr bool kSignalHandlerEnabled = false;
  map_optimization::VIMapOptimizer optimizer(plotter, kSignalHandlerEnabled);

  vi_map::MissionIdSet mission_ids;
  map->getAllMissionIds(&mission_ids);

  map_optimization::OptimizationProblem::UniquePtr optimization_problem(
      map_optimization::constructOptimizationProblem(
          mission_ids, options, map.get()));

  map_optimization::solve(options.solver_options, optimization_problem.get());

  // RESULTS
  aslam::Transformation T_S_B_opt =
      map->getSensorManager().getSensor_T_B_S(wheel_odometry_sensor_id);
  VLOG(2) << "========================================";
  VLOG(2) << bold << "T_S_B:\n" << T_S_B;
  VLOG(2) << "T_S_B opt:\n" << T_S_B_opt;
  VLOG(2) << "T_A_B\n" << T_A_B;
  VLOG(2)
      << "T_A_B opt\n"
      << map->getEdgePtrAs<vi_map::TransformationEdge>(edge_id_AB)->get_T_A_B();

  VLOG(2) << "T_M_I_A:\n" << T_M_I_A;
  VLOG(2) << "T_M_I_A opt:\n" << map->getVertex(id_A).get_T_M_I();
  VLOG(2) << "T_M_I_B:\n" << T_M_I_B;
  VLOG(2) << "T_M_I_B opt:\n" << map->getVertex(id_B).get_T_M_I();
  VLOG(2) << reset << "Done with wheel test!";

  // Check if optimization recovers unit quaternion for extrinsics.
  EXPECT_NEAR_KINDR_QUATERNION(
      aslam::Quaternion(), T_S_B_opt.getRotation(), 0.0001);

  // Check if vertex positions remained the same
  EXPECT_NEAR_EIGEN(
      T_M_I_A.getPosition(), map->getVertex(id_A).get_T_M_I().getPosition(),
      1e-12);
  EXPECT_NEAR_EIGEN(
      T_M_I_B.getPosition(), map->getVertex(id_B).get_T_M_I().getPosition(),
      1e-12);
  EXPECT_NEAR_KINDR_QUATERNION(
      T_M_I_A.getRotation(), map->getVertex(id_A).get_T_M_I().getRotation(),
      1e-12);
  EXPECT_NEAR_KINDR_QUATERNION(
      T_M_I_B.getRotation(), map->getVertex(id_B).get_T_M_I().getRotation(),
      1e-12);
}

TEST_F(ViMappingTest, TestCorruptedVIOpt) {
  corruptVertices();
  corruptLandmarks();

  visualizeMap();

  ASSERT_TRUE(test_app_.isMapConsistent());

  constexpr bool kVisionOnly = false;
  constexpr bool kAbsolute6DoF = false;
  constexpr bool kLcEdges = false;
  constexpr bool kFixCamExtrinsics = true;
  constexpr bool kFixAbsolute6DoFExtrinsics = true;
  EXPECT_TRUE(optimize(
      kVisionOnly, kAbsolute6DoF, kLcEdges, kFixCamExtrinsics,
      kFixAbsolute6DoFExtrinsics));

  constexpr double kPrecisionM = 0.01;
  test_app_.testIfKeyframesMatchReference(kPrecisionM);
  constexpr double kMinPassingLandmarkFraction = 0.99;
  test_app_.testIfLandmarksMatchReference(
      kPrecisionM, kMinPassingLandmarkFraction);

  visualizeMap();
}

TEST_F(ViMappingTest, TestCorruptedVOpt) {
  corruptVertices();

  visualizeMap();

  ASSERT_TRUE(test_app_.isMapConsistent());

  constexpr bool kVisionOnly = true;
  constexpr bool kAbsolute6DoF = false;
  constexpr bool kLcEdges = false;
  constexpr bool kFixCamExtrinsics = true;
  constexpr bool kFixAbsolute6DoFExtrinsics = true;
  EXPECT_TRUE(optimize(
      kVisionOnly, kAbsolute6DoF, kLcEdges, kFixCamExtrinsics,
      kFixAbsolute6DoFExtrinsics));

  // We expect less accuracy when using vision only
  constexpr double kPrecisionKeyframesM = 0.03;
  test_app_.testIfKeyframesMatchReference(kPrecisionKeyframesM);
  // Landmarks are fixed so they should not move at all
  constexpr double kPrecisionLandmarksM = 0.0;
  constexpr double kMinPassingLandmarkFraction = 1.0;
  test_app_.testIfLandmarksMatchReference(
      kPrecisionLandmarksM, kMinPassingLandmarkFraction);

  visualizeMap();
}

TEST_F(ViMappingTest, TestCorruptedVIOptWithAbs6DoFEnabledButHasNone) {
  corruptVertices();
  corruptLandmarks();

  visualizeMap();

  ASSERT_TRUE(test_app_.isMapConsistent());

  constexpr bool kVisionOnly = false;
  constexpr bool kAbsolute6DoF = true;
  constexpr bool kLcEdges = false;
  constexpr bool kFixCamExtrinsics = true;
  constexpr bool kFixAbsolute6DoFExtrinsics = true;
  EXPECT_TRUE(optimize(
      kVisionOnly, kAbsolute6DoF, kLcEdges, kFixCamExtrinsics,
      kFixAbsolute6DoFExtrinsics));

  constexpr double kPrecisionM = 0.01;
  test_app_.testIfKeyframesMatchReference(kPrecisionM);
  constexpr double kMinPassingLandmarkFraction = 0.99;
  test_app_.testIfLandmarksMatchReference(
      kPrecisionM, kMinPassingLandmarkFraction);

  visualizeMap();
}

TEST_F(ViMappingTest, TestCorruptedVIOptWithAbs6DoFEnabled) {
  addAbsolute6DoFConstraints();
  corruptVertices();
  corruptLandmarks();

  visualizeMap();

  ASSERT_TRUE(test_app_.isMapConsistent());

  constexpr bool kVisionOnly = false;
  constexpr bool kAbsolute6DoF = true;
  constexpr bool kLcEdges = false;
  constexpr bool kFixCamExtrinsics = true;
  constexpr bool kFixAbsolute6DoFExtrinsics = true;
  EXPECT_TRUE(optimize(
      kVisionOnly, kAbsolute6DoF, kLcEdges, kFixCamExtrinsics,
      kFixAbsolute6DoFExtrinsics));

  constexpr double kPrecisionKeyframesM = 0.001;
  test_app_.testIfKeyframesMatchReference(kPrecisionKeyframesM);
  constexpr double kPrecisionLandmarksM = 0.01;
  constexpr double kMinPassingLandmarkFraction = 0.99;
  test_app_.testIfLandmarksMatchReference(
      kPrecisionLandmarksM, kMinPassingLandmarkFraction);

  visualizeMap();
}

TEST_F(ViMappingTest, TestCorruptedVIOptWithAbs6DoFEnabledCalib) {
  addAbsolute6DoFConstraints();
  corruptVertices();
  corruptLandmarks();
  corruptAbs6DoFSensorExtrinsics();

  visualizeMap();

  ASSERT_TRUE(test_app_.isMapConsistent());

  constexpr bool kVisionOnly = false;
  constexpr bool kAbsolute6DoF = true;
  constexpr bool kLcEdges = false;
  constexpr bool kFixCamExtrinsics = true;
  constexpr bool kFixAbsolute6DoFExtrinsics = false;
  EXPECT_TRUE(optimize(
      kVisionOnly, kAbsolute6DoF, kLcEdges, kFixCamExtrinsics,
      kFixAbsolute6DoFExtrinsics));

  constexpr double kPrecisionKeyframesM = 0.001;
  test_app_.testIfKeyframesMatchReference(kPrecisionKeyframesM);
  constexpr double kPrecisionLandmarksM = 0.01;
  constexpr double kMinPassingLandmarkFraction = 0.99;
  test_app_.testIfLandmarksMatchReference(
      kPrecisionLandmarksM, kMinPassingLandmarkFraction);

  visualizeMap();
}

TEST_F(ViMappingTest, TestCorruptedVIOptWithLcEdgesEnabledButHasNone) {
  corruptVertices();
  corruptLandmarks();

  visualizeMap();

  ASSERT_TRUE(test_app_.isMapConsistent());

  constexpr bool kVisionOnly = false;
  constexpr bool kAbsolute6DoF = false;
  constexpr bool kLcEdges = true;
  constexpr bool kFixCamExtrinsics = true;
  constexpr bool kFixAbsolute6DoFExtrinsics = true;
  EXPECT_TRUE(optimize(
      kVisionOnly, kAbsolute6DoF, kLcEdges, kFixCamExtrinsics,
      kFixAbsolute6DoFExtrinsics));

  constexpr double kPrecisionM = 0.01;
  test_app_.testIfKeyframesMatchReference(kPrecisionM);
  constexpr double kMinPassingLandmarkFraction = 0.99;
  test_app_.testIfLandmarksMatchReference(
      kPrecisionM, kMinPassingLandmarkFraction);

  visualizeMap();
}

TEST_F(ViMappingTest, TestCorruptedVIOptWithLcEdgesEnabled) {
  // Add lc edges that constrain the relative position of a set of vertex pairs
  // to exactly what it is now.
  addLoopClosureEdges();
  corruptVertices();
  corruptLandmarks();

  visualizeMap();

  ASSERT_TRUE(test_app_.isMapConsistent());

  constexpr bool kVisionOnly = false;
  constexpr bool kAbsolute6DoF = false;
  constexpr bool kLcEdges = true;
  constexpr bool kFixCamExtrinsics = true;
  constexpr bool kFixAbsolute6DoFExtrinsics = true;
  EXPECT_TRUE(optimize(
      kVisionOnly, kAbsolute6DoF, kLcEdges, kFixCamExtrinsics,
      kFixAbsolute6DoFExtrinsics));

  constexpr double kPrecisionKeyframesM = 0.001;
  test_app_.testIfKeyframesMatchReference(kPrecisionKeyframesM);
  constexpr double kPrecisionLandmarksM = 0.01;
  constexpr double kMinPassingLandmarkFraction = 0.99;
  test_app_.testIfLandmarksMatchReference(
      kPrecisionLandmarksM, kMinPassingLandmarkFraction);

  visualizeMap();
}

TEST_F(ViMappingTest, TestCorruptedVIOptWithLcEdgesEnabledWithWrongLcEdge) {
  // Add lc edges that constrain the relative position of a set of vertex pairs
  // to exactly what it is now.
  addLoopClosureEdges();
  // Add one wrong loop closure edge. This shouldn't do anything if the
  // switchable constraints are working.
  addWrongLoopClosureEdge();
  corruptVertices();
  corruptLandmarks();

  visualizeMap();

  ASSERT_TRUE(test_app_.isMapConsistent());

  constexpr bool kVisionOnly = false;
  constexpr bool kAbsolute6DoF = false;
  constexpr bool kLcEdges = true;
  constexpr bool kFixCamExtrinsics = true;
  constexpr bool kFixAbsolute6DoFExtrinsics = true;
  EXPECT_TRUE(optimize(
      kVisionOnly, kAbsolute6DoF, kLcEdges, kFixCamExtrinsics,
      kFixAbsolute6DoFExtrinsics));

  constexpr double kPrecisionKeyframesM = 0.001;
  test_app_.testIfKeyframesMatchReference(kPrecisionKeyframesM);
  constexpr double kPrecisionLandmarksM = 0.01;
  constexpr double kMinPassingLandmarkFraction = 0.99;
  test_app_.testIfLandmarksMatchReference(
      kPrecisionLandmarksM, kMinPassingLandmarkFraction);

  visualizeMap();
}

// Checks whether clean extrinsics are recovered from noisy extrinsics.
TEST_F(ViMappingTest, TestWheelOdometryExtrinsics) {
  VLOG(2) << "Running wheel odom test with TWO nodes!";
  testWheelCalibrationTwoVertices();
  VLOG(2) << "Running wheel odom test with THREE nodes!";
  testWheelCalibrationThreeVertices();
}

}  // namespace visual_inertial_mapping

MAPLAB_UNITTEST_ENTRYPOINT
