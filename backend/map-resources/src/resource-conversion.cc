#include "map-resources/resource-conversion.h"

#include <aslam/cameras/camera-factory.h>
#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/camera-unified-projection.h>
#include <aslam/cameras/camera.h>
#include <aslam/cameras/distortion.h>
#include <glog/logging.h>
#include <maplab-common/pose_types.h>
#include <opencv2/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <voxblox/core/common.h>

namespace backend {

bool convertDepthMapToPointCloud(
    const cv::Mat& depth_map, const aslam::Camera& camera,
    resources::PointCloud* point_cloud) {
  CHECK_NOTNULL(point_cloud);

  cv::Mat image(1, 1, CV_8UC1);
  return convertDepthMapToPointCloud(depth_map, image, camera, point_cloud);
}

bool convertDepthMapToPointCloud(
    const cv::Mat& depth_map, const aslam::Camera& camera,
    voxblox::Pointcloud* point_cloud) {
  CHECK_NOTNULL(point_cloud)->clear();

  cv::Mat image(1, 1, CV_8UC1);
  return convertDepthMapToPointCloud(depth_map, image, camera, point_cloud);
}

bool convertDepthMapWithImageToPointCloud(
    const cv::Mat& depth_map, const cv::Mat& image, const aslam::Camera& camera,
    resources::PointCloud* point_cloud) {
  CHECK_NOTNULL(point_cloud);

  return convertDepthMapToPointCloud(depth_map, image, camera, point_cloud);
}

bool convertDepthMapWithImageToPointCloud(
    const cv::Mat& depth_map, const cv::Mat& image, const aslam::Camera& camera,
    voxblox::Pointcloud* points_C, voxblox::Colors* colors) {
  CHECK_NOTNULL(points_C)->clear();
  CHECK_NOTNULL(colors);
  resources::VoxbloxColorPointCloud voxblox_point_cloud;
  voxblox_point_cloud.points_C = points_C;
  voxblox_point_cloud.colors = colors;
  return convertDepthMapToPointCloud(
      depth_map, image, camera, &voxblox_point_cloud);
}

template <>
void addPointToPointCloud(
    const Eigen::Vector3d& point_C, const size_t index,
    voxblox::Pointcloud* point_cloud) {
  CHECK_NOTNULL(point_cloud);
  CHECK_GT(point_cloud->size(), index);

  voxblox::Point& point = (*point_cloud)[index];
  point = point_C.cast<voxblox::FloatingPoint>();
}

template <>
void addPointToPointCloud(
    const Eigen::Vector3d& point_C, const size_t index,
    resources::VoxbloxColorPointCloud* point_cloud) {
  CHECK_NOTNULL(point_cloud);
  CHECK_NOTNULL(point_cloud->points_C);
  CHECK_GT(point_cloud->points_C->size(), index);

  voxblox::Point& point = (*point_cloud->points_C)[index];
  point = point_C.cast<voxblox::FloatingPoint>();
}

template <>
void addPointToPointCloud(
    const Eigen::Vector3d& point_C, const size_t index,
    resources::PointCloud* point_cloud) {
  CHECK_NOTNULL(point_cloud);
  const size_t start_index = 3u * index;
  CHECK_LT(start_index + 2u, point_cloud->xyz.size());

  point_cloud->xyz[start_index] = static_cast<float>(point_C.x());
  point_cloud->xyz[start_index + 1u] = static_cast<float>(point_C.y());
  point_cloud->xyz[start_index + 2u] = static_cast<float>(point_C.z());
}

template <>
void addPointToPointCloud(
    const Eigen::Vector3d& point_C, const size_t index,
    sensor_msgs::PointCloud2* point_cloud) {
  CHECK_NOTNULL(point_cloud);
  constexpr size_t kNumFields = 4u;
  constexpr size_t kFloat32SizeBytes = 4u;
  size_t byte_index = index * kNumFields * kFloat32SizeBytes;
  CHECK_LT(
      byte_index + (3u * kFloat32SizeBytes) - 1u, point_cloud->data.size());
  std::memcpy(
      &point_cloud->data[byte_index], point_C.data(), 2u * kFloat32SizeBytes);
}

template <>
void addPointToPointCloud(
    const Eigen::Vector3d& point_C, const size_t index,
    pcl::PointCloud<pcl::PointXYZI>* point_cloud) {
  CHECK_NOTNULL(point_cloud);
  CHECK_LT(index, point_cloud->points.size());
  point_cloud->points[index].x = static_cast<float>(point_C(0));
  point_cloud->points[index].y = static_cast<float>(point_C(1));
  point_cloud->points[index].z = static_cast<float>(point_C(2));
}

template <>
void getPointFromPointCloud(
    const voxblox::Pointcloud& point_cloud, const size_t index,
    Eigen::Vector3d* point_C) {
  CHECK_NOTNULL(point_C);
  Eigen::Vector3d& point_C_out = *point_C;
  DCHECK_GT(point_cloud.size(), index);
  const voxblox::Point& point_C_in = point_cloud[index];
  point_C_out = point_C_in.cast<double>();
}

template <>
void getPointFromPointCloud(
    const resources::VoxbloxColorPointCloud& point_cloud, const size_t index,
    Eigen::Vector3d* point_C) {
  CHECK_NOTNULL(point_C);
  CHECK_NOTNULL(point_cloud.points_C);

  DCHECK_GT(point_cloud.points_C->size(), index);
  *point_C = (*point_cloud.points_C)[index].cast<double>();
}

template <>
void getPointFromPointCloud(
    const resources::PointCloud& point_cloud, const size_t index,
    Eigen::Vector3d* point_C) {
  CHECK_NOTNULL(point_C);

  Eigen::Vector3d& point_C_out = *point_C;

  const size_t real_index = index * 3u;
  DCHECK_GT(point_cloud.xyz.size(), real_index + 2u);
  point_C_out[0] = static_cast<double>(point_cloud.xyz[real_index]);
  point_C_out[1] = static_cast<double>(point_cloud.xyz[real_index + 1u]);
  point_C_out[2] = static_cast<double>(point_cloud.xyz[real_index + 2u]);
}

template <>
void getPointFromPointCloud(
    const sensor_msgs::PointCloud2& point_cloud, const size_t index,
    Eigen::Vector3d* point_C) {
  CHECK_NOTNULL(point_C);

  const size_t point_step = point_cloud.point_step;
  size_t byte_index = index * point_step;
  CHECK_LT(byte_index + point_step - 1u, point_cloud.data.size());

  // Expecting the first three fields to be of type float32 and corresponding
  // to x, y, z.
  Eigen::Vector3d& point_C_out = *point_C;
  const float* const x = CHECK_NOTNULL(
      reinterpret_cast<const float*>(&point_cloud.data[byte_index]));
  point_C_out(0) = static_cast<double>(*x);
  byte_index += sizeof(float);
  const float* const y = CHECK_NOTNULL(
      reinterpret_cast<const float*>(&point_cloud.data[byte_index]));
  point_C_out(1) = static_cast<double>(*y);
  byte_index += sizeof(float);
  const float* const z = CHECK_NOTNULL(
      reinterpret_cast<const float*>(&point_cloud.data[byte_index]));
  point_C_out(2) = static_cast<double>(*z);
}

template <>
void addColorToPointCloud(
    const resources::RgbaColor& color, const size_t index,
    resources::PointCloud* point_cloud) {
  CHECK_NOTNULL(point_cloud);
  const size_t start_index = 3u * index;

  CHECK_LT(start_index + 2, point_cloud->colors.size());
  point_cloud->colors[start_index] = color[0];
  point_cloud->colors[start_index + 1] = color[1];
  point_cloud->colors[start_index + 2] = color[2];
}

template <>
void addColorToPointCloud(
    const resources::RgbaColor& /*color*/, const size_t /*index*/,
    voxblox::Pointcloud* /*point_cloud*/) {
  LOG(FATAL) << "This is a vector of 3D points only. Colors can't be added.";
}

template <>
void addColorToPointCloud(
    const resources::RgbaColor& color, const size_t index,
    resources::VoxbloxColorPointCloud* point_cloud) {
  CHECK_NOTNULL(point_cloud);
  CHECK_NOTNULL(point_cloud->colors);
  CHECK_GT(point_cloud->colors->size(), index);

  voxblox::Color& voxblox_color = (*point_cloud->colors)[index];
  voxblox_color.r = color[0];
  voxblox_color.g = color[1];
  voxblox_color.b = color[2];
  voxblox_color.a = 255u;
}

template <>
void addColorToPointCloud(
    const resources::RgbaColor& /*color*/, const size_t /*index*/,
    pcl::PointCloud<pcl::PointXYZI>* /*point_cloud*/) {
  LOG(FATAL) << "This is a point-cloud with x, y, z and intensity. "
             << "Colors can't be added.";
}

template <>
void getColorFromPointCloud(
    const resources::VoxbloxColorPointCloud& point_cloud, const size_t index,
    resources::RgbaColor* color) {
  CHECK_NOTNULL(color);
  CHECK_NOTNULL(point_cloud.points_C);

  DCHECK_GT(point_cloud.colors->size(), index);
  const voxblox::Color& color_in = (*point_cloud.colors)[index];
  resources::RgbaColor& color_out = *color;

  color_out[0] = color_in.r;
  color_out[1] = color_in.g;
  color_out[2] = color_in.b;
  color_out[3] = color_in.a;
}

template <>
void getColorFromPointCloud(
    const resources::PointCloud& point_cloud, const size_t index,
    resources::RgbaColor* color) {
  CHECK_NOTNULL(color);

  const size_t real_index = index * 3u;

  DCHECK_GT(point_cloud.colors.size(), real_index + 2u);
  resources::RgbaColor& color_out = *color;

  color_out[0] = point_cloud.colors[real_index];
  color_out[1] = point_cloud.colors[real_index + 1u];
  color_out[2] = point_cloud.colors[real_index + 2u];
  color_out[3] = 255;
}

template <>
void getColorFromPointCloud(
    const sensor_msgs::PointCloud2& point_cloud, const size_t index,
    resources::RgbaColor* color) {
  CHECK_NOTNULL(color);
  // Expecting the following memory layout for a point
  // [float x, float x, float x, uint_8 r, uint_8 g, uint_8 b, uint_8 a]
  constexpr size_t kPointSizeBytes = 16u;
  size_t byte_index = index * kPointSizeBytes + 12u;
  CHECK_LT(byte_index + 3u, point_cloud.data.size());
  (*color)[0] = point_cloud.data[byte_index];
  ++byte_index;
  (*color)[1] = point_cloud.data[byte_index];
  ++byte_index;
  (*color)[2] = point_cloud.data[byte_index];
  ++byte_index;
  (*color)[3] = point_cloud.data[byte_index];
}

template <>
void addScalarToPointCloud(
    const float scalar, const size_t index,
    resources::PointCloud* point_cloud) {
  CHECK_NOTNULL(point_cloud);
  CHECK_LT(index, point_cloud->scalars.size());
  point_cloud->scalars[index] = scalar;
}

template <>
void addScalarToPointCloud(
    const float /*scalar*/, const size_t /*index*/,
    resources::VoxbloxColorPointCloud* /*point_cloud*/) {
  LOG(FATAL) << "Scalars can't be added to Voxblox color point-clouds.";
}

template <>
void addScalarToPointCloud(
    const float scalar, const size_t index,
    sensor_msgs::PointCloud2* point_cloud) {
  CHECK_NOTNULL(point_cloud);
  constexpr size_t kNumFields = 4u;
  constexpr size_t kFloat32SizeBytes = 4u;
  const size_t byte_index =
      index * kNumFields * kFloat32SizeBytes + 3u * kFloat32SizeBytes;
  CHECK_LT(byte_index + kFloat32SizeBytes - 1u, point_cloud->data.size());
  std::memcpy(&point_cloud->data[byte_index], &scalar, sizeof(float));
}

template <>
void addScalarToPointCloud(
    const float scalar, const size_t index,
    pcl::PointCloud<pcl::PointXYZI>* point_cloud) {
  CHECK_NOTNULL(point_cloud);
  CHECK_LT(index, point_cloud->points.size());
  point_cloud->points[index].intensity = scalar;
}

template <>
void getScalarFromPointCloud(
    const resources::PointCloud& point_cloud, const size_t index,
    float* scalar) {
  CHECK_NOTNULL(scalar);

  DCHECK_GT(point_cloud.scalars.size(), index);
  *scalar = point_cloud.scalars[index];
}

template <>
void getScalarFromPointCloud(
    const sensor_msgs::PointCloud2& point_cloud, const size_t index,
    float* scalar) {
  CHECK_NOTNULL(scalar);
  constexpr size_t kNumFields = 4u;
  CHECK_EQ(point_cloud.fields.size(), kNumFields);
  const size_t point_step = point_cloud.point_step;

  // Expecting four fields to be of type float32 and corresponding
  // to x, y, z, scalar.
  const size_t byte_index_point_start = index * point_step + 3u * sizeof(float);
  CHECK_LT(
      byte_index_point_start + sizeof(float) - 1u, point_cloud.data.size());
  std::memcpy(scalar, &point_cloud.data[byte_index_point_start], sizeof(float));
}

template <>
void resizePointCloud(
    const size_t size, const bool /*has_color*/, const bool /*has_normals*/,
    const bool /*has_scalar*/, voxblox::Pointcloud* point_cloud) {
  CHECK_NOTNULL(point_cloud);
  point_cloud->resize(size);
}

template <>
void resizePointCloud(
    const size_t size, const bool has_color, const bool /*has_normals*/,
    const bool /*has_scalar*/, resources::VoxbloxColorPointCloud* point_cloud) {
  CHECK_NOTNULL(point_cloud);
  CHECK_NOTNULL(point_cloud->colors)->clear();
  CHECK_NOTNULL(point_cloud->points_C)->clear();

  point_cloud->points_C->resize(size);
  if (has_color) {
    point_cloud->colors->resize(size);
  }
}

template <>
void resizePointCloud(
    const size_t size, const bool has_color, const bool has_normals,
    const bool has_scalar, resources::PointCloud* point_cloud) {
  CHECK_NOTNULL(point_cloud);
  point_cloud->resize(size, has_normals, has_color, has_scalar);
}

template <>
void resizePointCloud(
    const size_t num_points, const bool has_color, const bool /*has_normals*/,
    const bool has_scalar, sensor_msgs::PointCloud2* point_cloud) {
  CHECK_NOTNULL(point_cloud);
  assert(sizeof(float) == 4u);
  CHECK_GT(num_points, 0u);

  point_cloud->height = 1u;
  point_cloud->width = num_points;

  point_cloud->is_bigendian = false;

  sensor_msgs::PointField point_field_x;
  point_field_x.name = "x";
  point_field_x.offset = 0u;
  point_field_x.datatype = sensor_msgs::PointField::FLOAT32;
  point_field_x.count = 1u;
  point_cloud->fields.emplace_back(point_field_x);

  sensor_msgs::PointField point_field_y;
  point_field_y.name = "y";
  point_field_y.offset = sizeof(float);
  point_field_y.datatype = sensor_msgs::PointField::FLOAT32;
  point_field_y.count = 1u;
  point_cloud->fields.emplace_back(point_field_y);

  sensor_msgs::PointField point_field_z;
  point_field_z.name = "z";
  point_field_z.offset = 2u * sizeof(float);
  point_field_z.datatype = sensor_msgs::PointField::FLOAT32;
  point_field_z.count = 1u;
  point_cloud->fields.emplace_back(point_field_z);

  CHECK(!(has_color && has_scalar)) << "The PointCloud2 can support either "
                                    << "color or a scalar, but not both.";
  constexpr size_t kFloat32SizeBytes = 4u;
  assert(kFloat32SizeBytes == sizeof(float));
  point_cloud->point_step = 3u * kFloat32SizeBytes;
  if (has_color) {
    sensor_msgs::PointField point_field_color_r;
    point_field_color_r.name = "r";
    point_field_color_r.offset = 3u * sizeof(float);
    point_field_color_r.datatype = sensor_msgs::PointField::UINT8;
    point_field_color_r.count = 1u;
    point_cloud->fields.emplace_back(point_field_color_r);
    sensor_msgs::PointField point_field_color_g;
    point_field_color_g.name = "g";
    point_field_color_g.offset = 3u * sizeof(float) + 1u;
    point_field_color_g.datatype = sensor_msgs::PointField::UINT8;
    point_field_color_g.count = 1u;
    point_cloud->fields.emplace_back(point_field_color_r);
    sensor_msgs::PointField point_field_color_b;
    point_field_color_b.name = "b";
    point_field_color_b.offset = 3u * sizeof(float) + 2u;
    point_field_color_b.datatype = sensor_msgs::PointField::UINT8;
    point_field_color_b.count = 1u;
    point_cloud->fields.emplace_back(point_field_color_r);
    sensor_msgs::PointField point_field_color_a;
    point_field_color_a.name = "a";
    point_field_color_a.offset = 3u * sizeof(float) + 3u;
    point_field_color_a.datatype = sensor_msgs::PointField::UINT8;
    point_field_color_a.count = 1u;
    point_cloud->fields.emplace_back(point_field_color_r);
    constexpr size_t kUint8SizeBytes = 1u;
    point_cloud->point_step += 4u * kUint8SizeBytes;
  } else if (has_scalar) {
    sensor_msgs::PointField point_field_intensity;
    point_field_intensity.name = "i";
    point_field_intensity.offset = 3u * sizeof(float);
    point_field_intensity.datatype = sensor_msgs::PointField::FLOAT32;
    point_field_intensity.count = 1u;
    point_cloud->fields.emplace_back(point_field_intensity);
    point_cloud->point_step += kFloat32SizeBytes;
  }
  point_cloud->row_step = num_points * point_cloud->point_step;
  point_cloud->is_dense = false;
  point_cloud->data.resize(point_cloud->row_step);
}

template <>
void resizePointCloud(
    const size_t num_points, const bool /*has_color*/,
    const bool /*has_normals*/, const bool has_scalar,
    pcl::PointCloud<pcl::PointXYZI>* point_cloud) {
  CHECK_NOTNULL(point_cloud);
  CHECK_GT(num_points, 0u);
  CHECK(has_scalar) << "A scalar is required for the point-cloud with x, y, z, "
                    << "and intensity.";
  point_cloud->points.resize(num_points);
}

void createCameraWithoutDistortion(
    const aslam::Camera& camera,
    aslam::Camera::Ptr* camera_without_distortion) {
  CHECK_NOTNULL(camera_without_distortion);

  switch (camera.getType()) {
    case aslam::Camera::Type::kPinhole:
      *camera_without_distortion = aslam::createCamera<aslam::PinholeCamera>(
          camera.getParameters(), camera.imageWidth(), camera.imageHeight());
      break;
    case aslam::Camera::Type::kUnifiedProjection:
      *camera_without_distortion =
          aslam::createCamera<aslam::UnifiedProjectionCamera>(
              camera.getParameters(), camera.imageWidth(),
              camera.imageHeight());
      break;
    default:
      LOG(FATAL) << "Unknown camera type: " << camera.getType();
  }
}

template <>
size_t getPointCloudSize(const voxblox::Pointcloud& point_cloud) {
  return point_cloud.size();
}

template <>
size_t getPointCloudSize(const resources::VoxbloxColorPointCloud& point_cloud) {
  const size_t num_points = point_cloud.points_C->size();
  if (point_cloud.colors != nullptr) {
    const size_t num_colors = point_cloud.colors->size();
    if (num_colors > 0u) {
      CHECK_EQ(num_points, num_colors);
    }
  }
  return num_points;
}

template <>
size_t getPointCloudSize(const resources::PointCloud& point_cloud) {
  return point_cloud.size();
}

template <>
size_t getPointCloudSize(const sensor_msgs::PointCloud2& point_cloud) {
  constexpr size_t kFloat32NumBytes = 4u;
  assert(sizeof(float) == kFloat32NumBytes);

  constexpr size_t kNumFields = 4u;
  CHECK_EQ(point_cloud.fields.size(), kNumFields);
  for (const sensor_msgs::PointField& point_field : point_cloud.fields) {
    CHECK_EQ(point_field.datatype, sensor_msgs::PointField::FLOAT32);
  }

  constexpr size_t kPointNumBytes = kNumFields * kFloat32NumBytes;
  CHECK_EQ(point_cloud.point_step, kPointNumBytes);

  const size_t num_bytes = point_cloud.data.size();
  CHECK_GT(num_bytes, 0u);
  CHECK_EQ(num_bytes % kPointNumBytes, 0u);
  const size_t num_points = num_bytes / kPointNumBytes;
  return num_points;
}

template <>
bool hasColorInformation(const voxblox::Pointcloud& /*point_cloud*/) {
  return false;
}

template <>
bool hasColorInformation(const resources::VoxbloxColorPointCloud& point_cloud) {
  return point_cloud.colors->size() == point_cloud.points_C->size();
}

template <>
bool hasColorInformation(const resources::PointCloud& point_cloud) {
  return point_cloud.xyz.size() == point_cloud.colors.size();
}

template <>
bool hasColorInformation(const sensor_msgs::PointCloud2& point_cloud) {
  if (point_cloud.fields.size() < 4u) {
    return false;
  }
  if (point_cloud.fields[3u].name == "r") {
    CHECK_EQ(point_cloud.fields.size(), 7u);
    CHECK_EQ(point_cloud.fields[4u].name, "g");
    CHECK_EQ(point_cloud.fields[5u].name, "b");
    CHECK_EQ(point_cloud.fields[6u].name, "a");
    return true;
  }
  return false;
}

template <>
bool hasScalarInformation(const sensor_msgs::PointCloud2& point_cloud) {
  if (point_cloud.fields.size() < 4u) {
    return false;
  }
  if (point_cloud.fields[3u].name == "i") {
    CHECK_EQ(point_cloud.fields.size(), 4u);
    return true;
  }
  return false;
}

template <>
bool hasScalarInformation(const resources::PointCloud& point_cloud) {
  return !point_cloud.scalars.empty();
}

}  // namespace backend
