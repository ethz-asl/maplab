#include "vi-map/test/vi-map-test-helpers.h"

#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/distortion-radtan.h>
#include <map-resources/resource-common.h>
#include <map-resources/test/resource-template.h>
#include <sensors/lidar.h>
#include <sensors/sensor-factory.h>

#include "vi-map/test/vi-map-generator.h"
#include "vi-map/vi-map.h"

namespace vi_map {
namespace test {

void generateOptionalSensorResourceAndAddToMap(vi_map::VIMap* map) {
  CHECK_NOTNULL(map);

  cv::Mat color_image_dummy =
      cv::Mat(720, 480, CV_8UC3, cv::Scalar(20u, 30u, 40u));

  // Add some optional camera resources to each mission.
  vi_map::MissionIdList mission_ids;
  map->getAllMissionIds(&mission_ids);
  for (const vi_map::MissionId& mission_id : mission_ids) {
    VLOG(1) << "Adding camera resources to mission " << mission_id;

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
    map->getSensorManager().addOptionalCameraWithExtrinsics(
        *camera_1, T_C1_B, mission_id);
    map->getSensorManager().addOptionalCameraWithExtrinsics(
        *camera_2, T_C2_B, mission_id);

    const aslam::CameraId camera_id_1 = camera_1->getId();
    const aslam::CameraId camera_id_2 = camera_2->getId();

    backend::ResourceId camera_resource_id_1_1 =
        common::createRandomId<backend::ResourceId>();
    backend::ResourceId camera_resource_id_1_2 =
        common::createRandomId<backend::ResourceId>();
    backend::ResourceId camera_resource_id_1_3 =
        common::createRandomId<backend::ResourceId>();
    backend::ResourceId camera_resource_id_2_1 =
        common::createRandomId<backend::ResourceId>();
    backend::ResourceId camera_resource_id_2_2 =
        common::createRandomId<backend::ResourceId>();
    backend::ResourceId camera_resource_id_2_3 =
        common::createRandomId<backend::ResourceId>();

    std::unique_ptr<cv::Mat> camera_resource_1_1;
    std::unique_ptr<cv::Mat> camera_resource_1_2;
    std::unique_ptr<cv::Mat> camera_resource_1_3;
    backend::ResourceTemplate<cv::Mat>::createUniqueResource(
        color_image_dummy, camera_resource_id_1_1, &camera_resource_1_1);
    backend::ResourceTemplate<cv::Mat>::createUniqueResource(
        color_image_dummy, camera_resource_id_1_2, &camera_resource_1_2);
    backend::ResourceTemplate<cv::Mat>::createUniqueResource(
        color_image_dummy, camera_resource_id_1_3, &camera_resource_1_3);
    map->addOptionalSensorResource<aslam::CameraId, cv::Mat>(
        backend::ResourceType::kRawColorImage, camera_id_1, 10,
        *camera_resource_1_1, &mission);
    map->addOptionalSensorResource<aslam::CameraId, cv::Mat>(
        backend::ResourceType::kUndistortedColorImage, camera_id_1, 12,
        *camera_resource_1_2, &mission);
    map->addOptionalSensorResource<aslam::CameraId, cv::Mat>(
        backend::ResourceType::kRectifiedColorImage, camera_id_1, 11,
        *camera_resource_1_3, &mission);

    std::unique_ptr<cv::Mat> camera_resource_2_1;
    std::unique_ptr<cv::Mat> camera_resource_2_2;
    std::unique_ptr<cv::Mat> camera_resource_2_3;
    backend::ResourceTemplate<cv::Mat>::createUniqueResource(
        color_image_dummy, camera_resource_id_2_1, &camera_resource_2_1);
    backend::ResourceTemplate<cv::Mat>::createUniqueResource(
        color_image_dummy, camera_resource_id_2_2, &camera_resource_2_2);
    backend::ResourceTemplate<cv::Mat>::createUniqueResource(
        color_image_dummy, camera_resource_id_2_3, &camera_resource_2_3);
    map->addOptionalSensorResource<aslam::CameraId, cv::Mat>(
        backend::ResourceType::kColorImageForDepthMap, camera_id_2, 10,
        *camera_resource_2_1, &mission);
    map->addOptionalSensorResource<aslam::CameraId, cv::Mat>(
        backend::ResourceType::kRawColorImage, camera_id_2, 12,
        *camera_resource_2_2, &mission);
    map->addOptionalSensorResource<aslam::CameraId, cv::Mat>(
        backend::ResourceType::kUndistortedColorImage, camera_id_2, 11,
        *camera_resource_2_3, &mission);

    Sensor::UniquePtr lidar_sensor = createTestSensor(SensorType::kLidar);
    const SensorId lidar_sensor_id = lidar_sensor->getId();
    CHECK(lidar_sensor_id.isValid());
    map->getSensorManager().addSensor(std::move(lidar_sensor), mission_id);

    const int64_t kDefaultTimestampNanoseconds = 0;
    vi_map::LidarMeasurement lidar_measurement_dummy(
        lidar_sensor_id, kDefaultTimestampNanoseconds);
    lidar_measurement_dummy.setRandom();

    backend::ResourceId sensor_resource_id_1 =
        common::createRandomId<backend::ResourceId>();
    backend::ResourceId sensor_resource_id_2 =
        common::createRandomId<backend::ResourceId>();
    backend::ResourceId sensor_resource_id_3 =
        common::createRandomId<backend::ResourceId>();

    std::unique_ptr<resources::PointCloud> sensor_resource_1;
    std::unique_ptr<resources::PointCloud> sensor_resource_2;
    std::unique_ptr<resources::PointCloud> sensor_resource_3;
    backend::ResourceTemplate<resources::PointCloud>::createUniqueResource(
        lidar_measurement_dummy.getPointCloud(), sensor_resource_id_1,
        &sensor_resource_1);
    backend::ResourceTemplate<resources::PointCloud>::createUniqueResource(
        lidar_measurement_dummy.getPointCloud(), sensor_resource_id_2,
        &sensor_resource_2);
    backend::ResourceTemplate<resources::PointCloud>::createUniqueResource(
        lidar_measurement_dummy.getPointCloud(), sensor_resource_id_3,
        &sensor_resource_3);
    map->addOptionalSensorResource<SensorId, resources::PointCloud>(
        backend::ResourceType::kPointCloudXYZI, lidar_sensor_id, 10,
        *sensor_resource_1, &mission);
    map->addOptionalSensorResource<SensorId, resources::PointCloud>(
        backend::ResourceType::kPointCloudXYZI, lidar_sensor_id, 12,
        *sensor_resource_2, &mission);
    map->addOptionalSensorResource<SensorId, resources::PointCloud>(
        backend::ResourceType::kPointCloudXYZI, lidar_sensor_id, 11,
        *sensor_resource_3, &mission);
  }
}

void generateOptionalSensorResourceIdsAndAddToAllMissions(vi_map::VIMap* map) {
  CHECK_NOTNULL(map);

  cv::Mat color_image_dummy =
      cv::Mat(720, 480, CV_8UC3, cv::Scalar(20u, 30u, 40u));

  // Add some optional camera resources to each mission.
  vi_map::MissionIdList mission_ids;
  map->getAllMissionIds(&mission_ids);
  for (const vi_map::MissionId& mission_id : mission_ids) {
    VLOG(1) << "Adding camera resources to mission " << mission_id;

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
    map->getSensorManager().addOptionalCameraWithExtrinsics(
        *camera_1, T_C1_B, mission_id);
    map->getSensorManager().addOptionalCameraWithExtrinsics(
        *camera_2, T_C2_B, mission_id);

    const aslam::CameraId camera_id_1 = camera_1->getId();
    const aslam::CameraId camera_id_2 = camera_2->getId();

    backend::ResourceId camera_resource_id_1_1 =
        common::createRandomId<backend::ResourceId>();
    backend::ResourceId camera_resource_id_1_2 =
        common::createRandomId<backend::ResourceId>();
    backend::ResourceId camera_resource_id_1_3 =
        common::createRandomId<backend::ResourceId>();
    backend::ResourceId camera_resource_id_2_1 =
        common::createRandomId<backend::ResourceId>();
    backend::ResourceId camera_resource_id_2_2 =
        common::createRandomId<backend::ResourceId>();
    backend::ResourceId camera_resource_id_2_3 =
        common::createRandomId<backend::ResourceId>();

    mission.addOptionalSensorResourceId(
        backend::ResourceType::kRawColorImage, camera_id_1,
        camera_resource_id_1_1, 10);
    mission.addOptionalSensorResourceId(
        backend::ResourceType::kUndistortedColorImage, camera_id_1,
        camera_resource_id_1_2, 12);
    mission.addOptionalSensorResourceId(
        backend::ResourceType::kRectifiedColorImage, camera_id_1,
        camera_resource_id_1_3, 11);
    mission.addOptionalSensorResourceId(
        backend::ResourceType::kRectifiedColorImage, camera_id_2,
        camera_resource_id_2_1, 10);
    mission.addOptionalSensorResourceId(
        backend::ResourceType::kRectifiedColorImage, camera_id_2,
        camera_resource_id_2_2, 12);
    mission.addOptionalSensorResourceId(
        backend::ResourceType::kRectifiedColorImage, camera_id_2,
        camera_resource_id_2_3, 11);

    Sensor::UniquePtr lidar_sensor = createTestSensor(SensorType::kLidar);
    const SensorId lidar_sensor_id = lidar_sensor->getId();
    CHECK(lidar_sensor_id.isValid());
    map->getSensorManager().addSensor(std::move(lidar_sensor), mission_id);

    const int64_t kDefaultTimestampNanoseconds = 0;
    vi_map::LidarMeasurement lidar_measurement_dummy(
        lidar_sensor_id, kDefaultTimestampNanoseconds);
    lidar_measurement_dummy.setRandom();

    backend::ResourceId sensor_resource_id_1 =
        common::createRandomId<backend::ResourceId>();
    backend::ResourceId sensor_resource_id_2 =
        common::createRandomId<backend::ResourceId>();
    backend::ResourceId sensor_resource_id_3 =
        common::createRandomId<backend::ResourceId>();
    mission.addOptionalSensorResourceId(
        backend::ResourceType::kPointCloudXYZI, lidar_sensor_id,
        sensor_resource_id_1, 10);
    mission.addOptionalSensorResourceId(
        backend::ResourceType::kPointCloudXYZI, lidar_sensor_id,
        sensor_resource_id_2, 12);
    mission.addOptionalSensorResourceId(
        backend::ResourceType::kPointCloudXYZI, lidar_sensor_id,
        sensor_resource_id_3, 11);
  }
}

bool hasAllOptionalCameraResourcesOfOtherMission(
    const vi_map::VIMission& mission_a, const vi_map::VIMission& mission_b) {
  const backend::ResourceTypeToCameraIdToResourcesMap& opt_res_a =
      mission_a.getAllOptionalSensorResourceIds<aslam::CameraId>();

  bool result = true;

  for (const backend::ResourceTypeToCameraIdToResourcesMap::value_type&
           type_to_res : opt_res_a) {
    const backend::ResourceType type = type_to_res.first;
    for (const backend::CameraIdToResourcesMap::value_type& cam_to_res :
         type_to_res.second) {
      const aslam::CameraId& camera_id = cam_to_res.first;
      const backend::StampedResourceIds& stamped_resources =
          cam_to_res.second.resource_id_map();
      for (const backend::StampedResourceId& stamped_resource :
           stamped_resources) {
        const int64_t timestamp_ns = stamped_resource.first;
        const backend::ResourceId& resource_id = stamped_resource.second;

        backend::ResourceId retrieved_resource_id;
        result &= mission_b.getOptionalSensorResourceId(
            type, camera_id, timestamp_ns, &retrieved_resource_id);
        result &= resource_id == retrieved_resource_id;
      }
    }
  }
  return result;
}

bool hasAllOptionalCamerasOfOtherMission(
    const SensorManager& sensor_manager_a,
    const SensorManager& sensor_manager_b, const vi_map::VIMission& mission_a,
    const vi_map::VIMission& mission_b) {
  std::vector<backend::CameraWithExtrinsics> cameras_with_extrinsics_a;
  sensor_manager_a.getOptionalCamerasWithExtrinsicsForMissionId(
      mission_a.id(), &cameras_with_extrinsics_a);
  std::vector<backend::CameraWithExtrinsics> cameras_with_extrinsics_b;
  sensor_manager_b.getOptionalCamerasWithExtrinsicsForMissionId(
      mission_b.id(), &cameras_with_extrinsics_b);

  bool result = true;

  for (const backend::CameraWithExtrinsics& camera_with_extrinsics_a :
       cameras_with_extrinsics_a) {
    const aslam::CameraId& camera_id_a =
        camera_with_extrinsics_a.second->getId();

    bool found_same_camera_in_b = false;
    const backend::CameraWithExtrinsics* camera_with_extrinsics_b_ptr = nullptr;
    for (const backend::CameraWithExtrinsics& camera_with_extrinsics_b :
         cameras_with_extrinsics_b) {
      const aslam::CameraId& camera_id_b =
          camera_with_extrinsics_b.second->getId();
      if (camera_id_a == camera_id_b) {
        CHECK(!found_same_camera_in_b)
            << "The same camera is twice in mission b!";
        found_same_camera_in_b = true;
        camera_with_extrinsics_b_ptr = &camera_with_extrinsics_b;
      }
    }

    if (!found_same_camera_in_b) {
      return false;
    }
    CHECK_NOTNULL(camera_with_extrinsics_b_ptr);

    const backend::CameraWithExtrinsics& cam_w_extrinsics_a =
        camera_with_extrinsics_a;
    const backend::CameraWithExtrinsics& cam_w_extrinsics_b =
        *camera_with_extrinsics_b_ptr;

    const aslam::Transformation& T_C_B_a = cam_w_extrinsics_a.first;
    const aslam::Transformation& T_C_B_b = cam_w_extrinsics_b.first;
    result = result && (T_C_B_a == T_C_B_b);

    if (cam_w_extrinsics_a.second) {
      const aslam::Camera& camera_a = *cam_w_extrinsics_a.second;
      if (cam_w_extrinsics_b.second) {
        const aslam::Camera& camera_b = *cam_w_extrinsics_b.second;
        result &= camera_a == camera_b;
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
      result = result && hasAllOptionalCamerasOfOtherMission(
                             map_a.getSensorManager(), map_b.getSensorManager(),
                             mission_a, mission_b);
      result = result && hasAllOptionalCamerasOfOtherMission(
                             map_a.getSensorManager(), map_b.getSensorManager(),
                             mission_b, mission_a);
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
