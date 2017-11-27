#include "vi-map/test/vi-map-test-helpers.h"

#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/distortion-radtan.h>
#include <map-resources/resource-common.h>
#include <map-resources/test/resource-template.h>
#include <sensors/sensor-factory.h>

#include "vi-map/test/vi-map-generator.h"
#include "vi-map/vi-map.h"

namespace vi_map {
namespace test {

void generateMap(vi_map::VIMap* map) {
  constexpr size_t kNumberOfAdditionalVertices = 0u;
  generateMap(kNumberOfAdditionalVertices, map);
}

void generateMap(const size_t number_of_vertices, vi_map::VIMap* map) {
  CHECK_NOTNULL(map);

  // Generate a map.
  constexpr int kMapGeneratorSeed = 10;
  vi_map::VIMapGenerator map_generator(*map, kMapGeneratorSeed);
  const vi_map::MissionId mission_id = map_generator.createMission();

  pose_graph::VertexIdList empty_vertex_id_list;
  pose::Transformation T_G_I;
  const pose_graph::VertexId vertex_id_1 =
      map_generator.createVertex(mission_id, T_G_I);
  T_G_I.getPosition() << 1, 0, 0;
  const pose_graph::VertexId vertex_id_2 =
      map_generator.createVertex(mission_id, T_G_I);
  T_G_I.getPosition() << 2, 0, 0;
  const pose_graph::VertexId vertex_id_3 =
      map_generator.createVertex(mission_id, T_G_I);

  // Create additional vertices.
  for (size_t i = 3u; i < number_of_vertices; ++i) {
    T_G_I.getPosition() << i, 0, 0;
    const pose_graph::VertexId vertex_id =
        map_generator.createVertex(mission_id, T_G_I);

    // Create a landmark for each vertex.
    Eigen::Vector3d map_landmark_p_G_fi;
    map_landmark_p_G_fi << i, 2, 3;
    map_generator.createLandmark(
        map_landmark_p_G_fi, vertex_id, empty_vertex_id_list);
  }

  Eigen::Vector3d map_landmark_p_G_fi;
  map_landmark_p_G_fi << 1, 2, 3;
  map_generator.createLandmark(
      map_landmark_p_G_fi, vertex_id_1, empty_vertex_id_list);
  map_generator.createLandmark(
      map_landmark_p_G_fi, vertex_id_2, empty_vertex_id_list);
  map_generator.createLandmark(
      map_landmark_p_G_fi, vertex_id_3, empty_vertex_id_list);
  map_landmark_p_G_fi << 4, 2, 3;
  map_generator.createLandmark(
      map_landmark_p_G_fi, vertex_id_2, empty_vertex_id_list);
  map_generator.createLandmark(
      map_landmark_p_G_fi, vertex_id_3, empty_vertex_id_list);

  map_generator.generateMap();
}

void generateMapWithOptionalCameraResources(
    const size_t number_of_vertices, const std::string& map_folder,
    vi_map::VIMap* map) {
  CHECK_NOTNULL(map);

  generateMap(number_of_vertices, map);

  cv::Mat color_image_dummy =
      cv::Mat(720, 480, CV_8UC3, cv::Scalar(20u, 30u, 40u));

  // This is needed to make sure the new resources are stored to some folder.
  map->setMapFolder(map_folder);

  // Add some optional camera resources to each mission.
  vi_map::MissionIdList mission_ids;
  map->getAllMissionIds(&mission_ids);
  for (const vi_map::MissionId& mission_id : mission_ids) {
    VLOG(1) << "Add resources to mission " << mission_id;

    vi_map::VIMission& mission = map->getMission(mission_id);

    // Add cameras.
    aslam::Camera::ConstPtr camera_1 =
        aslam::PinholeCamera::createTestCamera<aslam::RadTanDistortion>();
    aslam::Camera::ConstPtr camera_2 =
        aslam::PinholeCamera::createTestCamera<aslam::RadTanDistortion>();
    aslam::Transformation T_C1_B;
    T_C1_B.setRandom();
    aslam::Transformation T_C2_B;
    T_C2_B.setRandom();
    mission.addOptionalCameraWithExtrinsics(*camera_1, T_C1_B);
    mission.addOptionalCameraWithExtrinsics(*camera_2, T_C2_B);

    const aslam::CameraId camera_id_1 = camera_1->getId();
    backend::ResourceId resource_id_1_1;
    backend::ResourceId resource_id_1_2;
    backend::ResourceId resource_id_1_3;
    std::unique_ptr<cv::Mat> resource_1_1;
    std::unique_ptr<cv::Mat> resource_1_2;
    std::unique_ptr<cv::Mat> resource_1_3;

    const aslam::CameraId camera_id_2 = camera_2->getId();
    backend::ResourceId resource_id_2_1;
    backend::ResourceId resource_id_2_2;
    backend::ResourceId resource_id_2_3;
    std::unique_ptr<cv::Mat> resource_2_1;
    std::unique_ptr<cv::Mat> resource_2_2;
    std::unique_ptr<cv::Mat> resource_2_3;

    // Add some resources for camera 1.
    common::generateId(&resource_id_1_1);
    backend::ResourceTemplate<cv::Mat>::createUniqueResource(
        color_image_dummy, resource_id_1_1, &resource_1_1);
    map->addOptionalCameraResource(
        backend::ResourceType::kRawColorImage, camera_id_1, 10, *resource_1_1,
        &mission);
    common::generateId(&resource_id_1_2);
    backend::ResourceTemplate<cv::Mat>::createUniqueResource(
        color_image_dummy, resource_id_1_2, &resource_1_2);
    map->addOptionalCameraResource(
        backend::ResourceType::kUndistortedColorImage, camera_id_1, 12,
        *resource_1_2, &mission);
    common::generateId(&resource_id_1_3);
    backend::ResourceTemplate<cv::Mat>::createUniqueResource(
        color_image_dummy, resource_id_1_3, &resource_1_3);
    map->addOptionalCameraResource(
        backend::ResourceType::kRectifiedColorImage, camera_id_1, 11,
        *resource_1_3, &mission);

    // Add some resources for camera 2.
    common::generateId(&resource_id_2_1);
    backend::ResourceTemplate<cv::Mat>::createUniqueResource(
        color_image_dummy, resource_id_2_1, &resource_2_1);
    map->addOptionalCameraResource(
        backend::ResourceType::kColorImageForDepthMap, camera_id_2, 10,
        *resource_2_1, &mission);
    common::generateId(&resource_id_2_2);
    backend::ResourceTemplate<cv::Mat>::createUniqueResource(
        color_image_dummy, resource_id_2_2, &resource_2_2);
    map->addOptionalCameraResource(
        backend::ResourceType::kRawColorImage, camera_id_2, 11, *resource_2_2,
        &mission);
    common::generateId(&resource_id_2_3);
    backend::ResourceTemplate<cv::Mat>::createUniqueResource(
        color_image_dummy, resource_id_2_3, &resource_2_3);
    map->addOptionalCameraResource(
        backend::ResourceType::kUndistortedColorImage, camera_id_2, 12,
        *resource_2_3, &mission);
  }
}

bool hasAllOptionalCameraResourcesOfOtherMission(
    const vi_map::VIMission& mission_a, const vi_map::VIMission& mission_b) {
  const backend::ResourceTypeToOptionalCameraResourcesMap& opt_res_a =
      mission_a.getAllOptionalCameraResourceIds();

  bool result = true;

  for (const backend::ResourceTypeToOptionalCameraResourcesMap::value_type&
           type_to_res : opt_res_a) {
    const backend::ResourceType type = type_to_res.first;
    for (const backend::OptionalCameraResourcesMap::value_type& cam_to_res :
         type_to_res.second) {
      const aslam::CameraId& camera_id = cam_to_res.first;
      const backend::StampedResourceIds& stamped_resources =
          cam_to_res.second.resource_id_map();
      for (const backend::StampedResourceId& stamped_resource :
           stamped_resources) {
        const int64_t timestamp_ns = stamped_resource.first;
        const backend::ResourceId& resource_id = stamped_resource.second;

        backend::ResourceId retrieved_resource_id;
        result ==
            result&& mission_b.getOptionalCameraResourceId(
                type, camera_id, timestamp_ns, &retrieved_resource_id);
        result == result && (resource_id == retrieved_resource_id);
      }
    }
  }
  return result;
}

bool hasAllOptionalCamerasOfOtherMission(
    const vi_map::VIMission& mission_a, const vi_map::VIMission& mission_b) {
  const backend::OptionalCameraMap& opt_cams_a =
      mission_a.getAllOptionalCamerasWithExtrinsics();
  const backend::OptionalCameraMap& opt_cams_b =
      mission_b.getAllOptionalCamerasWithExtrinsics();

  bool result = true;

  for (const backend::OptionalCameraMap::value_type& id_camera_pair :
       opt_cams_a) {
    const aslam::CameraId& camera_id = id_camera_pair.first;
    backend::OptionalCameraMap::const_iterator it = opt_cams_b.find(camera_id);
    if (it != opt_cams_b.end()) {
      return false;
    }

    const backend::CameraWithExtrinsics& cam_w_extrinsics_a =
        id_camera_pair.second;
    const backend::CameraWithExtrinsics& cam_w_extrinsics_b = it->second;

    const aslam::Transformation& T_C_B_a = cam_w_extrinsics_a.first;
    const aslam::Transformation& T_C_B_b = cam_w_extrinsics_b.first;
    result == result && (T_C_B_a == T_C_B_b);

    if (cam_w_extrinsics_a.second) {
      const aslam::Camera& camera_a = *cam_w_extrinsics_a.second;
      if (cam_w_extrinsics_b.second) {
        const aslam::Camera& camera_b = *cam_w_extrinsics_b.second;
        result == result && (camera_a == camera_b);
      } else {
        return false;
      }
    }
  }
  return result;
}

bool compareVIMap(const vi_map::VIMap& map_a, const vi_map::VIMap& map_b) {
  bool result = true;
  {
    // Check missions.
    result = result && (map_a.numMissions() == map_b.numMissions());
    vi_map::MissionIdList missions_a, missions_b;
    map_a.getAllMissionIds(&missions_a);
    map_b.getAllMissionIds(&missions_b);
    CHECK_EQ(missions_a.size(), missions_b.size());
    std::sort(missions_a.begin(), missions_a.end());
    std::sort(missions_b.begin(), missions_b.end());
    for (size_t i = 0u; i < missions_a.size(); ++i) {
      result = result && (missions_a[i] == missions_b[i]);
    }
  }
  {
    // Check vertices.
    result = result && (map_a.numVertices() == map_b.numVertices());
    pose_graph::VertexIdList vertex_ids_a, vertex_ids_b;
    map_a.getAllVertexIds(&vertex_ids_a);
    map_b.getAllVertexIds(&vertex_ids_b);
    std::sort(vertex_ids_a.begin(), vertex_ids_a.end());
    std::sort(vertex_ids_b.begin(), vertex_ids_b.end());
    CHECK_EQ(vertex_ids_a.size(), vertex_ids_b.size());
    for (size_t i = 0u; i < vertex_ids_a.size(); ++i) {
      result = result && (vertex_ids_a[i] == vertex_ids_b[i]);
      const vi_map::Vertex& vertex_a = map_a.getVertex(vertex_ids_a[i]);
      const vi_map::Vertex& vertex_b = map_b.getVertex(vertex_ids_b[i]);
      result = result && (vertex_a == vertex_b);
      // Check NCamera.
      result = result && (*vertex_a.getNCameras() == *vertex_b.getNCameras());
    }
  }
  {
    // Check edges.
    result = result && (map_a.numEdges() == map_b.numEdges());
    pose_graph::EdgeIdList edge_ids_a, edge_ids_b;
    map_a.getAllEdgeIds(&edge_ids_a);
    map_b.getAllEdgeIds(&edge_ids_b);
    std::sort(edge_ids_a.begin(), edge_ids_a.end());
    std::sort(edge_ids_b.begin(), edge_ids_b.end());
    CHECK_EQ(edge_ids_a.size(), edge_ids_b.size());
    for (size_t i = 0u; i < edge_ids_a.size(); ++i) {
      result = result && (edge_ids_a[i] == edge_ids_b[i]);
      result = result && (map_a.getEdgeType(edge_ids_a[i]) ==
                          map_b.getEdgeType(edge_ids_b[i]));
    }
  }
  {
    // Check landmarks.
    result = result && (map_a.numLandmarks() == map_b.numLandmarks());
    result =
        result && (map_a.numLandmarksInIndex() == map_b.numLandmarksInIndex());

    vi_map::LandmarkIdList store_landmarks_a, store_landmarks_b;
    map_a.getAllLandmarkIds(&store_landmarks_a);
    map_b.getAllLandmarkIds(&store_landmarks_b);
    std::sort(store_landmarks_a.begin(), store_landmarks_a.end());
    std::sort(store_landmarks_b.begin(), store_landmarks_b.end());
    CHECK_EQ(store_landmarks_a.size(), store_landmarks_b.size());
    for (size_t i = 0u; i < store_landmarks_a.size(); ++i) {
      result = result && (store_landmarks_a[i] == store_landmarks_b[i]);
    }
    for (size_t i = 0u; i < store_landmarks_a.size(); ++i) {
      result = result && (map_a.getLandmark(store_landmarks_a[i]) ==
                          map_b.getLandmark(store_landmarks_b[i]));
    }
  }
  {
    // Check the optional camera resources.
    vi_map::MissionIdList missions_a, missions_b;
    map_a.getAllMissionIds(&missions_a);
    map_b.getAllMissionIds(&missions_b);
    CHECK_EQ(missions_a.size(), missions_b.size());
    std::sort(missions_a.begin(), missions_a.end());
    std::sort(missions_b.begin(), missions_b.end());
    for (size_t i = 0u; i < missions_a.size(); ++i) {
      const vi_map::VIMission& mission_a = map_a.getMission(missions_a[i]);
      const vi_map::VIMission& mission_b = map_b.getMission(missions_b[i]);

      result = result && hasAllOptionalCameraResourcesOfOtherMission(
                             mission_a, mission_b);
      result = result && hasAllOptionalCameraResourcesOfOtherMission(
                             mission_b, mission_a);

      result =
          result && hasAllOptionalCamerasOfOtherMission(mission_a, mission_b);
      result =
          result && hasAllOptionalCamerasOfOtherMission(mission_b, mission_a);
    }
  }

  return result;
}

void generateOptionalSensorDataAndAddToMap(vi_map::VIMap* map) {
  CHECK_NOTNULL(map);
  vi_map::MissionIdList mission_ids;
  map->getAllMissionIds(&mission_ids);

  constexpr int kNumGpsMeasurements = 10;
  for (const vi_map::MissionId& mission_id : mission_ids) {
    GpsUtm::UniquePtr gps_utm_sensor = createTestSensor<GpsUtm>();
    CHECK(gps_utm_sensor);
    const SensorId gps_utm_sensor_id = gps_utm_sensor->getId();
    CHECK(gps_utm_sensor_id.isValid());
    map->getSensorManager().addSensor(std::move(gps_utm_sensor), mission_id);
    GpsWgs::UniquePtr gps_wgs_sensor = createTestSensor<GpsWgs>();
    CHECK(gps_wgs_sensor);
    const SensorId gps_wgs_sensor_id = gps_wgs_sensor->getId();
    CHECK(gps_wgs_sensor_id.isValid());
    map->getSensorManager().addSensor(std::move(gps_wgs_sensor), mission_id);
    for (int measurement_idx = 0; measurement_idx < kNumGpsMeasurements;
         measurement_idx++) {
      vi_map::GpsWgsMeasurement gps_wgs_measurement(gps_wgs_sensor_id);
      gps_wgs_measurement.setRandom();
      vi_map::GpsUtmMeasurement gps_utm_measurement(gps_utm_sensor_id);
      gps_utm_measurement.setRandom();
      map->addOptionalSensorMeasurement(gps_wgs_measurement, mission_id);
      map->addOptionalSensorMeasurement(gps_utm_measurement, mission_id);
    }
  }
}

}  // namespace test
}  // namespace vi_map
