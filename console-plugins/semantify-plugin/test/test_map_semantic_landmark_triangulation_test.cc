#include <Eigen/Core>
#include <gtest/gtest.h>
#include <glog/logging.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <aslam/triangulation/triangulation.h>
#include <vi-map/landmark-quality-metrics.h>
#include <vi-map/semantic-landmark-quality-metrics.h>

#include <vi-map/test/vi-map-generator.h>

#include <iostream>

namespace vi_map {

class SemanticLandmakrTriangulationTest : public ::testing::Test {
 protected:
  SemanticLandmakrTriangulationTest() : map_(), generator_(map_, 42) {}

  virtual void SetUp() {
    pose::Transformation T_G_V;
    pose::Transformation T_G_M;

    missions_ = generator_.createMission(T_G_M);
    T_G_V.getPosition() << 1, 1, -1;
    vertices_[0] = generator_.createVertex(missions_, T_G_V);
    T_G_V.getPosition() << 0.5, 0.5, -1;
    vertices_[1] = generator_.createVertex(missions_, T_G_V);
    T_G_V.getPosition() << 1.5, 1.5, -1;
    vertices_[2] = generator_.createVertex(missions_, T_G_V);
    T_G_V.getPosition() << 0, 1, -1;
    vertices_[3] = generator_.createVertex(missions_, T_G_V);
    T_G_V.getPosition() << 2, 0, -1;
    vertices_[4] = generator_.createVertex(missions_, T_G_V);

    landmark_id_ = generator_.createLandmark(
        Eigen::Vector3d(1, 1, 1),
        vertices_[0],
        {vertices_[1], vertices_[2], vertices_[3], vertices_[4]});
    semantic_landmark_id_ = generator_.createSemanticLandmark(
        Eigen::Vector3d(1, 1, 1), kClassId,
        vertices_[0],
        {vertices_[1], vertices_[2], vertices_[3], vertices_[4]});

    generator_.generateMap();
  }

  vi_map::MissionId missions_;
  pose_graph::VertexId vertices_[5];
  vi_map::LandmarkId landmark_id_;
  vi_map::SemanticLandmarkId semantic_landmark_id_;

  VIMap map_;
  VIMapGenerator generator_;
  int kClassId = 5;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

TEST_F(SemanticLandmakrTriangulationTest, TestTriangulateLandmark) {
  pose_graph::VertexIdList vertex_ids;
  map_.getAllVertexIdsInMission(missions_, &vertex_ids);

  // check mission id is correct for all vertex
  for (const pose_graph::VertexId& vertex_id : vertex_ids) {
    EXPECT_EQ(map_.getVertex(vertex_id).getMissionId(), missions_);
  }
  // check the vertices are created
  for (const pose_graph::VertexId& vertex_id : vertices_) {
    EXPECT_TRUE(map_.hasVertex(vertex_id));
  }
  // check the landmark is stored correctly for storing vertex and observed vertices
  for (const pose_graph::VertexId& vertex_id : vertices_) {
    LandmarkIdList observed_landmarks;
    map_.getVertex(vertex_id).getAllObservedLandmarkIds(&observed_landmarks);
    CHECK_EQ(observed_landmarks.size(),1u);
    CHECK_EQ(landmark_id_,observed_landmarks[0]);
    LandmarkStore stored_landmarks = map_.getVertex(vertex_id).getLandmarks();
    if (vertex_id == vertices_[0]) {
      CHECK_EQ(stored_landmarks.size(),1u);
    } else {
      CHECK_EQ(stored_landmarks.size(),0u);
    }
  }
  // triangulate the landmark and compare
  pose_graph::VertexId storing_vertex_id = vertices_[0];
  vi_map::Vertex& storing_vertex = map_.getVertex(storing_vertex_id);
  vi_map::LandmarkStore& landmark_store = storing_vertex.getLandmarks();
  const aslam::Transformation& T_M_I_storing = storing_vertex.get_T_M_I();
  const aslam::Transformation& T_G_M_storing = 
          map_.getMissionBaseFrameForVertex(storing_vertex_id).get_T_G_M();
  const aslam::Transformation T_G_I_storing = T_G_M_storing * T_M_I_storing;

  for (vi_map::Landmark& landmark : landmark_store) {
    // The following have one entry per measurement:
    Eigen::Matrix3Xd G_bearing_vectors;
    Eigen::Matrix3Xd p_G_C_vector;
    const vi_map::KeypointIdentifierList& observations =
        landmark.getObservations();
    CHECK(observations.size() > 2u) <<  
        "Landmark triangulation failed because too few observations.";

    G_bearing_vectors.resize(Eigen::NoChange, observations.size());
    p_G_C_vector.resize(Eigen::NoChange, observations.size());

    int num_measurements = 0;
    for (const vi_map::KeypointIdentifier& observation : observations) {
      const pose_graph::VertexId& observer_id = observation.frame_id.vertex_id;
      CHECK(map_.hasVertex(observer_id))
          << "Observer " << observer_id << " of store landmark "
          << landmark.id() << " not in currently loaded map!";
      const vi_map::Vertex& observer = map_.getVertex(observer_id);
      const aslam::VisualFrame& visual_frame =
          observer.getVisualFrame(observation.frame_id.frame_index);
      const aslam::Transformation& T_G_M_observer =
          map_.getMissionBaseFrameForVertex(observer_id).get_T_G_M();
      
      aslam::Transformation T_G_I_observer;
      const aslam::Transformation& T_M_I_observer = observer.get_T_M_I();
      T_G_I_observer = T_G_M_observer * T_M_I_observer;

      Eigen::Vector2d measurement =
          visual_frame.getKeypointMeasurement(observation.keypoint_index);
      // LOG(INFO)<<"Default measurement: "<<measurement;
      
      Eigen::Vector3d C_bearing_vector;
      CHECK(observer.getCamera(observation.frame_id.frame_index)
              ->backProject3(measurement, &C_bearing_vector)) <<
              "Landmark triangulation failed proj failed.";

      const aslam::CameraId& cam_id =
          observer.getCamera(observation.frame_id.frame_index)->getId();
      aslam::Transformation T_G_C =
          (T_G_I_observer *
           observer.getNCameras()->get_T_C_B(cam_id).inverse());
      G_bearing_vectors.col(num_measurements) =
          T_G_C.getRotationMatrix() * C_bearing_vector;
      p_G_C_vector.col(num_measurements) = T_G_C.getPosition();
      ++num_measurements;
    }
    G_bearing_vectors.conservativeResize(Eigen::NoChange, num_measurements);
    p_G_C_vector.conservativeResize(Eigen::NoChange, num_measurements);

    CHECK(num_measurements > 2) <<"Landmark triangulation too few meas.";

    // point in global frame
    Eigen::Vector3d p_G_fi;
    aslam::TriangulationResult triangulation_result =
        aslam::linearTriangulateFromNViews(
            G_bearing_vectors, p_G_C_vector, &p_G_fi);

    EXPECT_TRUE(triangulation_result.wasTriangulationSuccessful());
    // landmark p_B is in the storing vertex frame
    Eigen::Vector3d ground_truth = 
        (storing_vertex.get_T_M_I()*landmark.get_p_B());
    // can't compare eigen matrix directly
    EXPECT_NEAR(p_G_fi(0), ground_truth(0),1e-6);
    EXPECT_NEAR(p_G_fi(1), ground_truth(1),1e-6);
    EXPECT_NEAR(p_G_fi(2), ground_truth(2),1e-6);    
  }
}

TEST_F(SemanticLandmakrTriangulationTest, TestTriangulateSemanticLandmark) {
  pose_graph::VertexIdList vertex_ids;
  map_.getAllVertexIdsInMission(missions_, &vertex_ids);

  // check mission id is correct for all vertex
  for (const pose_graph::VertexId& vertex_id : vertex_ids) {
    EXPECT_EQ(map_.getVertex(vertex_id).getMissionId(), missions_);
  }
  // check the vertices are created
  for (const pose_graph::VertexId& vertex_id : vertices_) {
    EXPECT_TRUE(map_.hasVertex(vertex_id));
  }
  // check the landmark is stored correctly for storing vertex and observed vertices
  for (const pose_graph::VertexId& vertex_id : vertices_) {
    SemanticLandmarkIdList observed_landmarks;
    map_.getVertex(vertex_id).getAllObservedSemanticLandmarkIds(&observed_landmarks);
    CHECK_EQ(observed_landmarks.size(),1u);
    CHECK_EQ(semantic_landmark_id_,observed_landmarks[0]);
    SemanticLandmarkStore stored_landmarks = map_.getVertex(vertex_id).getSemanticLandmarks();
    if (vertex_id == vertices_[0]) {
      CHECK_EQ(stored_landmarks.size(),1u);
    } else {
      CHECK_EQ(stored_landmarks.size(),0u);
    }
  }
  // triangulate the landmark and compare
  pose_graph::VertexId storing_vertex_id = vertices_[0];
  vi_map::Vertex& storing_vertex = map_.getVertex(storing_vertex_id);
  vi_map::SemanticLandmarkStore& landmark_store = storing_vertex.getSemanticLandmarks();
  const aslam::Transformation& T_M_I_storing = storing_vertex.get_T_M_I();
  const aslam::Transformation& T_G_M_storing = 
          map_.getMissionBaseFrameForVertex(storing_vertex_id).get_T_G_M();
  const aslam::Transformation T_G_I_storing = T_G_M_storing * T_M_I_storing;

  for (vi_map::SemanticLandmark& landmark : landmark_store) {
    // The following have one entry per measurement:
    Eigen::Matrix3Xd G_bearing_vectors;
    Eigen::Matrix3Xd p_G_C_vector;
    const vi_map::SemanticObjectIdentifierList& observations =
        landmark.getObservations();
    CHECK(observations.size() > 2u) <<  
        "Landmark triangulation failed because too few observations.";

    G_bearing_vectors.resize(Eigen::NoChange, observations.size());
    p_G_C_vector.resize(Eigen::NoChange, observations.size());

    int num_measurements = 0;
    for (const vi_map::SemanticObjectIdentifier& observation : observations) {
      const pose_graph::VertexId& observer_id = observation.frame_id.vertex_id;
      CHECK(map_.hasVertex(observer_id))
          << "Observer " << observer_id << " of store landmark "
          << landmark.id() << " not in currently loaded map!";
      const vi_map::Vertex& observer = map_.getVertex(observer_id);
      const aslam::VisualFrame& visual_frame =
          observer.getVisualFrame(observation.frame_id.frame_index);
      const aslam::Transformation& T_G_M_observer =
          map_.getMissionBaseFrameForVertex(observer_id).get_T_G_M();
      
      aslam::Transformation T_G_I_observer;
      const aslam::Transformation& T_M_I_observer = observer.get_T_M_I();
      T_G_I_observer = T_G_M_observer * T_M_I_observer;

      Eigen::Vector4d semantic_measurement =
          visual_frame.getSemanticObjectMeasurement(observation.measurement_index);
      // LOG(INFO)<<"Before type cast: "<<semantic_measurement.block<2,1>(0,0);
      Eigen::Vector2d measurement = semantic_measurement.block<2,1>(0,0).array().cast<double>().matrix();
      // LOG(INFO)<<"After type cast: "<<measurement;

      Eigen::Vector3d C_bearing_vector;
      CHECK(observer.getCamera(observation.frame_id.frame_index)
              ->backProject3(measurement, &C_bearing_vector)) <<
              "Landmark triangulation failed proj failed.";

      const aslam::CameraId& cam_id =
          observer.getCamera(observation.frame_id.frame_index)->getId();
      aslam::Transformation T_G_C =
          (T_G_I_observer *
           observer.getNCameras()->get_T_C_B(cam_id).inverse());
      G_bearing_vectors.col(num_measurements) =
          T_G_C.getRotationMatrix() * C_bearing_vector;
      p_G_C_vector.col(num_measurements) = T_G_C.getPosition();
      ++num_measurements;
    }
    G_bearing_vectors.conservativeResize(Eigen::NoChange, num_measurements);
    p_G_C_vector.conservativeResize(Eigen::NoChange, num_measurements);

    CHECK(num_measurements > 2) <<"Landmark triangulation too few meas.";

    // point in global frame
    Eigen::Vector3d p_G_fi;
    aslam::TriangulationResult triangulation_result =
        aslam::linearTriangulateFromNViews(
            G_bearing_vectors, p_G_C_vector, &p_G_fi);

    EXPECT_TRUE(triangulation_result.wasTriangulationSuccessful());
    // landmark p_B is in the storing vertex frame
    Eigen::Vector3d ground_truth = 
        (storing_vertex.get_T_M_I()*landmark.get_p_B());
    // can't compare eigen matrix directly
    EXPECT_NEAR(p_G_fi(0), ground_truth(0),1e-6);
    EXPECT_NEAR(p_G_fi(1), ground_truth(1),1e-6);
    EXPECT_NEAR(p_G_fi(2), ground_truth(2),1e-6);
  }
}

}  // namespace vi_map

MAPLAB_UNITTEST_ENTRYPOINT
