#include "vi-map/test/vi-map-test-helpers.h"

#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/distortion-radtan.h>
#include <map-resources/resource-common.h>
#include <map-resources/temporal-resource-id-buffer.h>
#include <map-resources/test/resource-template.h>
#include <sensors/lidar.h>

#include "vi-map/test/vi-map-generator.h"
#include "vi-map/vi-map.h"

namespace vi_map {
namespace test {

void generateSensorResourceAndAddToMap(vi_map::VIMap* map) {
  CHECK_NOTNULL(map);

  cv::Mat color_image_dummy =
      cv::Mat(720, 480, CV_8UC3, cv::Scalar(20u, 30u, 40u));

  // Add some sensor resources to each mission.
  vi_map::MissionIdList mission_ids;
  map->getAllMissionIds(&mission_ids);
  for (const vi_map::MissionId& mission_id : mission_ids) {
    VLOG(1) << "Adding camera resources to mission " << mission_id;

    vi_map::VIMission& mission = map->getMission(mission_id);

    const aslam::SensorId& base_sensor_id =
        map->getSensorManager().getBaseSensorId(mission.getImuId());

    // Add cameras.
    aslam::Camera::Ptr camera_1 =
        aslam::PinholeCamera::createTestCamera<aslam::RadTanDistortion>();
    std::vector<aslam::Camera::Ptr> camera_1_vec;
    camera_1_vec.push_back(camera_1);
    aslam::Camera::Ptr camera_2 =
        aslam::PinholeCamera::createTestCamera<aslam::RadTanDistortion>();
    std::vector<aslam::Camera::Ptr> camera_2_vec;
    camera_2_vec.push_back(camera_2);

    aslam::Transformation T_C1_B;
    T_C1_B.setRandom();
    aslam::Transformation T_C2_B;
    T_C2_B.setRandom();

    aslam::SensorId ncamera_1_id;
    aslam::generateId(&ncamera_1_id);
    aslam::Transformation T_C_Cn1;
    T_C_Cn1.setRandom();
    aslam::TransformationVector T_C_Cn1_vec;
    T_C_Cn1_vec.push_back(T_C_Cn1);
    aslam::NCamera::UniquePtr ncamera_1 = aligned_unique<aslam::NCamera>(
        ncamera_1_id, T_C_Cn1_vec, camera_1_vec, "test ncamera 1");

    aslam::SensorId ncamera_2_id;
    aslam::generateId(&ncamera_2_id);
    aslam::Transformation T_C_Cn2;
    T_C_Cn2.setRandom();
    aslam::TransformationVector T_C_Cn2_vec;
    T_C_Cn2_vec.push_back(T_C_Cn2);
    aslam::NCamera::UniquePtr ncamera_2 = aligned_unique<aslam::NCamera>(
        ncamera_2_id, T_C_Cn2_vec, camera_2_vec, "test ncamera 2");

    aslam::Transformation T_Cn_B;
    T_Cn_B.setRandom();
    map->getSensorManager().addSensor<aslam::NCamera>(
        std::move(ncamera_1), base_sensor_id, T_Cn_B);
    map->getSensorManager().addSensor<aslam::NCamera>(
        std::move(ncamera_2), base_sensor_id, T_Cn_B);

    backend::ResourceId camera_resource_id_1_1 =
        aslam::createRandomId<backend::ResourceId>();
    backend::ResourceId camera_resource_id_1_2 =
        aslam::createRandomId<backend::ResourceId>();
    backend::ResourceId camera_resource_id_1_3 =
        aslam::createRandomId<backend::ResourceId>();
    backend::ResourceId camera_resource_id_2_1 =
        aslam::createRandomId<backend::ResourceId>();
    backend::ResourceId camera_resource_id_2_2 =
        aslam::createRandomId<backend::ResourceId>();
    backend::ResourceId camera_resource_id_2_3 =
        aslam::createRandomId<backend::ResourceId>();

    std::unique_ptr<cv::Mat> camera_resource_1_1;
    std::unique_ptr<cv::Mat> camera_resource_1_2;
    std::unique_ptr<cv::Mat> camera_resource_1_3;
    backend::ResourceTemplate<cv::Mat>::createUniqueResource(
        color_image_dummy, camera_resource_id_1_1, &camera_resource_1_1);
    backend::ResourceTemplate<cv::Mat>::createUniqueResource(
        color_image_dummy, camera_resource_id_1_2, &camera_resource_1_2);
    backend::ResourceTemplate<cv::Mat>::createUniqueResource(
        color_image_dummy, camera_resource_id_1_3, &camera_resource_1_3);
    map->addSensorResource<cv::Mat>(
        backend::ResourceType::kRawColorImage, ncamera_1_id, 10,
        *camera_resource_1_1, &mission);
    map->addSensorResource<cv::Mat>(
        backend::ResourceType::kUndistortedColorImage, ncamera_1_id, 12,
        *camera_resource_1_2, &mission);
    map->addSensorResource<cv::Mat>(
        backend::ResourceType::kRectifiedColorImage, ncamera_1_id, 11,
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
    map->addSensorResource<cv::Mat>(
        backend::ResourceType::kColorImageForDepthMap, ncamera_2_id, 10,
        *camera_resource_2_1, &mission);
    map->addSensorResource<cv::Mat>(
        backend::ResourceType::kRawColorImage, ncamera_2_id, 12,
        *camera_resource_2_2, &mission);
    map->addSensorResource<cv::Mat>(
        backend::ResourceType::kUndistortedColorImage, ncamera_2_id, 11,
        *camera_resource_2_3, &mission);

    vi_map::Lidar::UniquePtr lidar_sensor(new vi_map::Lidar());
    const aslam::SensorId& lidar_sensor_id = lidar_sensor->getId();
    CHECK(lidar_sensor_id.isValid());

    aslam::Transformation T_L_B;
    T_L_B.setRandom();

    map->getSensorManager().addSensor<vi_map::Lidar>(
        std::move(lidar_sensor), base_sensor_id, T_L_B);

    const int64_t kDefaultTimestampNanoseconds = 0;
    vi_map::MaplabLidarMeasurement lidar_measurement_dummy(
        lidar_sensor_id, kDefaultTimestampNanoseconds);
    lidar_measurement_dummy.setRandom();

    backend::ResourceId sensor_resource_id_1 =
        aslam::createRandomId<backend::ResourceId>();
    backend::ResourceId sensor_resource_id_2 =
        aslam::createRandomId<backend::ResourceId>();
    backend::ResourceId sensor_resource_id_3 =
        aslam::createRandomId<backend::ResourceId>();

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
    map->addSensorResource<resources::PointCloud>(
        backend::ResourceType::kPointCloudXYZI, lidar_sensor_id, 10,
        *sensor_resource_1, &mission);
    map->addSensorResource<resources::PointCloud>(
        backend::ResourceType::kPointCloudXYZI, lidar_sensor_id, 12,
        *sensor_resource_2, &mission);
    map->addSensorResource<resources::PointCloud>(
        backend::ResourceType::kPointCloudXYZI, lidar_sensor_id, 11,
        *sensor_resource_3, &mission);
  }
}

void generateSensorResourceIdsAndAddToAllMissions(vi_map::VIMap* map) {
  CHECK_NOTNULL(map);

  cv::Mat color_image_dummy =
      cv::Mat(720, 480, CV_8UC3, cv::Scalar(20u, 30u, 40u));

  // Add some sensor camera resources to each mission.
  vi_map::MissionIdList mission_ids;
  map->getAllMissionIds(&mission_ids);
  for (const vi_map::MissionId& mission_id : mission_ids) {
    VLOG(1) << "Adding camera resources to mission " << mission_id;

    vi_map::VIMission& mission = map->getMission(mission_id);

    const aslam::SensorId& base_sensor_id =
        map->getSensorManager().getBaseSensorId(mission.getNCameraId());
    CHECK_NE(base_sensor_id, mission.getNCameraId());

    // Add cameras.
    aslam::Camera::Ptr camera_1 =
        aslam::PinholeCamera::createTestCamera<aslam::RadTanDistortion>();
    std::vector<aslam::Camera::Ptr> camera_1_vec;
    camera_1_vec.push_back(camera_1);
    aslam::Camera::Ptr camera_2 =
        aslam::PinholeCamera::createTestCamera<aslam::RadTanDistortion>();
    std::vector<aslam::Camera::Ptr> camera_2_vec;
    camera_2_vec.push_back(camera_2);

    aslam::Transformation T_C1_B;
    T_C1_B.setRandom();
    aslam::Transformation T_C2_B;
    T_C2_B.setRandom();

    aslam::SensorId ncamera_1_id;
    aslam::generateId(&ncamera_1_id);
    aslam::Transformation T_C_Cn1;
    T_C_Cn1.setRandom();
    aslam::TransformationVector T_C_Cn1_vec;
    T_C_Cn1_vec.push_back(T_C_Cn1);
    aslam::NCamera::UniquePtr ncamera_1 = aligned_unique<aslam::NCamera>(
        ncamera_1_id, T_C_Cn1_vec, camera_1_vec, "test ncamera 1");

    aslam::SensorId ncamera_2_id;
    aslam::generateId(&ncamera_2_id);
    aslam::Transformation T_C_Cn2;
    T_C_Cn2.setRandom();
    aslam::TransformationVector T_C_Cn2_vec;
    T_C_Cn2_vec.push_back(T_C_Cn2);
    aslam::NCamera::UniquePtr ncamera_2 = aligned_unique<aslam::NCamera>(
        ncamera_2_id, T_C_Cn2_vec, camera_2_vec, "test ncamera 2");

    aslam::Transformation T_Cn_B;
    T_Cn_B.setRandom();
    map->getSensorManager().addSensor<aslam::NCamera>(
        std::move(ncamera_1), base_sensor_id, T_Cn_B);
    map->getSensorManager().addSensor<aslam::NCamera>(
        std::move(ncamera_2), base_sensor_id, T_Cn_B);

    backend::ResourceId camera_resource_id_1_1 =
        aslam::createRandomId<backend::ResourceId>();
    backend::ResourceId camera_resource_id_1_2 =
        aslam::createRandomId<backend::ResourceId>();
    backend::ResourceId camera_resource_id_1_3 =
        aslam::createRandomId<backend::ResourceId>();
    backend::ResourceId camera_resource_id_2_1 =
        aslam::createRandomId<backend::ResourceId>();
    backend::ResourceId camera_resource_id_2_2 =
        aslam::createRandomId<backend::ResourceId>();
    backend::ResourceId camera_resource_id_2_3 =
        aslam::createRandomId<backend::ResourceId>();

    mission.addSensorResourceId(
        backend::ResourceType::kRawColorImage, ncamera_1_id,
        camera_resource_id_1_1, 10);
    mission.addSensorResourceId(
        backend::ResourceType::kUndistortedColorImage, ncamera_1_id,
        camera_resource_id_1_2, 12);
    mission.addSensorResourceId(
        backend::ResourceType::kRectifiedColorImage, ncamera_1_id,
        camera_resource_id_1_3, 11);
    mission.addSensorResourceId(
        backend::ResourceType::kRectifiedColorImage, ncamera_2_id,
        camera_resource_id_2_1, 10);
    mission.addSensorResourceId(
        backend::ResourceType::kRectifiedColorImage, ncamera_2_id,
        camera_resource_id_2_2, 12);
    mission.addSensorResourceId(
        backend::ResourceType::kRectifiedColorImage, ncamera_2_id,
        camera_resource_id_2_3, 11);

    vi_map::Lidar::UniquePtr lidar_sensor(new vi_map::Lidar());
    const aslam::SensorId lidar_sensor_id = lidar_sensor->getId();
    CHECK(lidar_sensor_id.isValid());

    aslam::Transformation T_L_B;
    T_L_B.setRandom();

    map->getSensorManager().addSensor<vi_map::Lidar>(
        std::move(lidar_sensor), base_sensor_id, T_L_B);

    const int64_t kDefaultTimestampNanoseconds = 0;
    vi_map::MaplabLidarMeasurement lidar_measurement_dummy(
        lidar_sensor_id, kDefaultTimestampNanoseconds);
    lidar_measurement_dummy.setRandom();

    backend::ResourceId sensor_resource_id_1 =
        aslam::createRandomId<backend::ResourceId>();
    backend::ResourceId sensor_resource_id_2 =
        aslam::createRandomId<backend::ResourceId>();
    backend::ResourceId sensor_resource_id_3 =
        aslam::createRandomId<backend::ResourceId>();
    mission.addSensorResourceId(
        backend::ResourceType::kPointCloudXYZI, lidar_sensor_id,
        sensor_resource_id_1, 10);
    mission.addSensorResourceId(
        backend::ResourceType::kPointCloudXYZI, lidar_sensor_id,
        sensor_resource_id_2, 12);
    mission.addSensorResourceId(
        backend::ResourceType::kPointCloudXYZI, lidar_sensor_id,
        sensor_resource_id_3, 11);
  }
}

bool hasAllSensorResourcesOfOtherMission(
    const vi_map::VIMap& map_a, const vi_map::VIMission& mission_a,
    const vi_map::VIMap& map_b, const vi_map::VIMission& mission_b) {
  const backend::ResourceTypeToSensorIdToResourcesMap& opt_res_a =
      mission_a.getAllSensorResourceIds();

  bool result = true;

  for (const backend::ResourceTypeToSensorIdToResourcesMap::value_type&
           type_to_res : opt_res_a) {
    const backend::ResourceType type = type_to_res.first;
    for (const backend::SensorIdToResourcesMap::value_type&
             sensor_id_to_ressource : type_to_res.second) {
      const aslam::SensorId& sensor_id = sensor_id_to_ressource.first;

      CHECK(map_a.getSensorManager().hasSensor(sensor_id));

      const aslam::Transformation& T_B_S_a =
          map_a.getSensorManager().getSensor_T_B_S(sensor_id);

      const backend::StampedResourceIds& stamped_resources =
          sensor_id_to_ressource.second.resource_id_map();
      for (const backend::StampedResourceId& stamped_resource :
           stamped_resources) {
        const int64_t timestamp_ns = stamped_resource.first;
        const backend::ResourceId& resource_id = stamped_resource.second;

        const aslam::Transformation& T_B_S_b =
            map_b.getSensorManager().getSensor_T_B_S(sensor_id);

        backend::ResourceId retrieved_resource_id;
        if (!mission_b.getSensorResourceId(
                type, sensor_id, timestamp_ns, &retrieved_resource_id)) {
          LOG(WARNING) << "Mission B does not have a sensor resource of type "
                       << backend::ResourceTypeNames[static_cast<int>(type)]
                       << " for sensor " << sensor_id
                       << ", timestamp_ns: " << timestamp_ns << "!";
          result = false;
          continue;
        }

        const bool result_1 = resource_id == retrieved_resource_id;
        LOG_IF(WARNING, !result_1)
            << "The resource in Mission B has a different id!";

        const bool result_2 = map_b.getSensorManager().hasSensor(sensor_id);
        LOG_IF(WARNING, !result_2) << "The resource in mission B does not have "
                                      "a sensor in the sensor manager!";

        const bool result_3 = (T_B_S_a.getTransformationMatrix() -
                               T_B_S_b.getTransformationMatrix())
                                  .cwiseAbs()
                                  .maxCoeff() < aslam::common::macros::kEpsilon;
        LOG_IF(WARNING, !result_3)
            << "The resource in Mission B has a different T_B_S!";

        result &= result_1 && result_2 && result_3;
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
    // Check the sensor camera resources.
    vi_map::MissionIdList missions_a, missions_b;
    map_a.getAllMissionIds(&missions_a);
    map_b.getAllMissionIds(&missions_b);
    CHECK_EQ(missions_a.size(), missions_b.size());
    std::sort(missions_a.begin(), missions_a.end());
    std::sort(missions_b.begin(), missions_b.end());
    for (size_t i = 0u; i < missions_a.size(); ++i) {
      const vi_map::VIMission& mission_a = map_a.getMission(missions_a[i]);
      const vi_map::VIMission& mission_b = map_b.getMission(missions_b[i]);
      result = result && hasAllSensorResourcesOfOtherMission(
                             map_a, mission_a, map_b, mission_b);
    }
    for (size_t i = 0u; i < missions_b.size(); ++i) {
      const vi_map::VIMission& mission_b = map_a.getMission(missions_b[i]);
      const vi_map::VIMission& mission_a = map_b.getMission(missions_a[i]);
      result = result && hasAllSensorResourcesOfOtherMission(
                             map_b, mission_b, map_a, mission_a);
    }
  }
  return result;
}

}  // namespace test
}  // namespace vi_map
