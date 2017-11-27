#ifndef MAP_RESOURCES_TEST_RESOURCE_TEMPLATE_H_
#define MAP_RESOURCES_TEST_RESOURCE_TEMPLATE_H_

#include <string>
#include <unordered_map>
#include <vector>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <maplab-common/file-system-tools.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "map-resources/resource-common.h"

namespace backend {

// NOTE: [ADD_RESOURCE_DATA_TYPE] Add enum for datatype.
enum class DataTypes {
  kCvMat,
  kText,
  kPointCloud,
  kVoxbloxTsdfMap,
  kVoxbloxEsdfMap,
  kVoxbloxOccupancyMap
};

struct ResourceTemplateBase {
  template <typename Derived>
  Derived& getAs() {
    CHECK_NOTNULL(this);
    Derived* result = static_cast<Derived*>(this);
    CHECK_NOTNULL(result);
    return *result;
  }
  template <typename Derived>
  const Derived& getAs() const {
    CHECK_NOTNULL(this);
    const Derived* result = static_cast<const Derived*>(this);
    CHECK_NOTNULL(result);
    return *result;
  }

  ResourceId id;

  ResourceType type;
  std::string folder;
  bool is_external_folder;
  DataTypes data_type;

  typedef std::unique_ptr<ResourceTemplateBase> Ptr;

 protected:
  ResourceTemplateBase(
      const ResourceType& _type, const std::string& _folder,
      bool _is_external_folder)
      : type(_type), folder(_folder), is_external_folder(_is_external_folder) {
    common::generateId(&id);
  }
};

typedef std::vector<ResourceTemplateBase::Ptr> ResourceTemplateBaseVector;

std::ostream& operator<<(
    std::ostream& stream, const ResourceTemplateBase& template_base) {
  stream << "Resource id: " << template_base.id.hexString()
         << " type: " << static_cast<int>(template_base.type)
         << " data_type: " << static_cast<int>(template_base.data_type)
         << " folder: " << template_base.folder
         << " is_external_folder: " << template_base.is_external_folder;
  return stream;
}

template <typename DataType>
struct ResourceTemplate : public ResourceTemplateBase {
  ResourceTemplate(
      const ResourceType& _type, const std::string& _folder,
      const bool _is_external_folder, const DataType& _resource)
      : ResourceTemplateBase(_type, _folder, _is_external_folder) {
    data_type = getDataType();
    createUniqueResource(_resource, id, &resource_ptr);
  }

  // NOTE: [ADD_RESOURCE_DATA_TYPE] Implement below.
  DataTypes getDataType();

  // NOTE: [ADD_RESOURCE_DATA_TYPE] Implement below.
  static void createUniqueResource(
      const DataType& default_resource, const ResourceId& id,
      typename std::unique_ptr<DataType>* unique_resource);

  const DataType& resource() const {
    CHECK(resource_ptr);
    return *resource_ptr;
  }

  DataType& resource() {
    CHECK(resource_ptr);
    return *resource_ptr;
  }

  std::unique_ptr<DataType> resource_ptr;

  typedef std::unique_ptr<ResourceTemplate> Ptr;
};

template <>
DataTypes ResourceTemplate<cv::Mat>::getDataType() {
  return DataTypes::kCvMat;
}

template <>
DataTypes ResourceTemplate<std::string>::getDataType() {
  return DataTypes::kText;
}

template <>
DataTypes ResourceTemplate<resources::PointCloud>::getDataType() {
  return DataTypes::kPointCloud;
}

template <>
DataTypes ResourceTemplate<voxblox::TsdfMap>::getDataType() {
  return DataTypes::kVoxbloxTsdfMap;
}

template <>
DataTypes ResourceTemplate<voxblox::EsdfMap>::getDataType() {
  return DataTypes::kVoxbloxEsdfMap;
}

template <>
DataTypes ResourceTemplate<voxblox::OccupancyMap>::getDataType() {
  return DataTypes::kVoxbloxOccupancyMap;
}

template <>
void ResourceTemplate<cv::Mat>::createUniqueResource(
    const cv::Mat& default_resource, const ResourceId& id,
    std::unique_ptr<cv::Mat>* unique_resource) {
  CHECK_NOTNULL(unique_resource)->reset(new cv::Mat(default_resource.clone()));
  const int font = CV_FONT_HERSHEY_SIMPLEX;
  cv::putText(
      *(*unique_resource), id.hexString(),
      cv::Point(0, (*unique_resource)->rows - 90), font, 2,
      cv::Scalar::all(255), 3);
}

template <>
void ResourceTemplate<std::string>::createUniqueResource(
    const std::string& default_resource, const ResourceId& id,
    std::unique_ptr<std::string>* unique_resource) {
  CHECK_NOTNULL(unique_resource)
      ->reset(new std::string(default_resource + id.hexString()));
}

template <>
void ResourceTemplate<resources::PointCloud>::createUniqueResource(
    const resources::PointCloud& default_resource, const ResourceId& id,
    std::unique_ptr<resources::PointCloud>* unique_resource) {
  CHECK_NOTNULL(unique_resource)->reset(new resources::PointCloud());

  aslam::HashId hash_id;
  id.toHashId(&hash_id);
  uint64_t id_uint[2];
  hash_id.toUint64(id_uint);

  if (!default_resource.colors.empty()) {
    CHECK_EQ(default_resource.xyz.size(), default_resource.colors.size());
  }
  if (!default_resource.normals.empty()) {
    CHECK_EQ(default_resource.xyz.size(), default_resource.normals.size());
  }

  for (size_t i = 0u; i < default_resource.xyz.size(); i += 3) {
    static constexpr float kMultiplicationFactor = 1.0e-8f;
    float x = default_resource.xyz[i];
    float y = default_resource.xyz[i + 1];
    float z = default_resource.xyz[i + 2];
    x += static_cast<float>(id_uint[0]) * kMultiplicationFactor;
    y += static_cast<float>(id_uint[1]) * kMultiplicationFactor;
    z += static_cast<float>(id_uint[0]) * kMultiplicationFactor;
    (*unique_resource)->xyz.push_back(x);
    (*unique_resource)->xyz.push_back(y);
    (*unique_resource)->xyz.push_back(z);
  }
  (*unique_resource)->normals = default_resource.normals;
  (*unique_resource)->colors = default_resource.colors;
}

// Convert hash into a block index by splitting the id into two uint64 and the
// uint64 into two uint32. We then take the first three uint32 as block index
// coordinates (x,y,z). Since we don't use the last part of our 128bit id, this
// should still be a unique mapping.
voxblox::BlockIndex hashToBlockIndex(const ResourceId& id) {
  CHECK(id.isValid());
  aslam::HashId hash_id;
  id.toHashId(&hash_id);
  uint64_t id_uint_array[2u];
  hash_id.toUint64(id_uint_array);

  const uint32_t x =
      (uint32_t)((id_uint_array[0] & 0xFFFFFFFF00000000LL) >> 32);
  const uint32_t y = (uint32_t)(id_uint_array[0] & 0xFFFFFFFFLL);
  const uint32_t z =
      (uint32_t)((id_uint_array[1] & 0xFFFFFFFF00000000LL) >> 32);

  return voxblox::BlockIndex(x, y, z);
}

template <>
void ResourceTemplate<voxblox::TsdfMap>::createUniqueResource(
    const voxblox::TsdfMap& default_resource, const ResourceId& id,
    std::unique_ptr<voxblox::TsdfMap>* unique_resource) {
  CHECK_NOTNULL(unique_resource)
      ->reset(new voxblox::TsdfMap(default_resource.getTsdfLayer()));

  voxblox::BlockIndex unique_block_idx = hashToBlockIndex(id);
  CHECK(!(*unique_resource)->getTsdfLayer().hasBlock(unique_block_idx))
      << "The block we intended to allocate to make the map unique is already "
         "allocated!";
  (*unique_resource)
      ->getTsdfLayerPtr()
      ->allocateBlockPtrByIndex(unique_block_idx);
}

template <>
void ResourceTemplate<voxblox::EsdfMap>::createUniqueResource(
    const voxblox::EsdfMap& default_resource, const ResourceId& id,
    std::unique_ptr<voxblox::EsdfMap>* unique_resource) {
  CHECK_NOTNULL(unique_resource)
      ->reset(new voxblox::EsdfMap(default_resource.getEsdfLayer()));

  voxblox::BlockIndex unique_block_idx = hashToBlockIndex(id);
  CHECK(!(*unique_resource)->getEsdfLayer().hasBlock(unique_block_idx))
      << "The block we intended to allocate to make the map unique is already "
         "allocated!";
  (*unique_resource)
      ->getEsdfLayerPtr()
      ->allocateBlockPtrByIndex(unique_block_idx);
}

template <>
void ResourceTemplate<voxblox::OccupancyMap>::createUniqueResource(
    const voxblox::OccupancyMap& default_resource, const ResourceId& id,
    std::unique_ptr<voxblox::OccupancyMap>* unique_resource) {
  CHECK_NOTNULL(unique_resource)
      ->reset(new voxblox::OccupancyMap(default_resource.getOccupancyLayer()));

  const voxblox::BlockIndex unique_block_idx = hashToBlockIndex(id);
  CHECK(!(*unique_resource)->getOccupancyLayer().hasBlock(unique_block_idx))
      << "The block we intended to allocate to make the map unique is "
         "already "
         "allocated!";
  (*unique_resource)
      ->getOccupancyLayerPtr()
      ->allocateBlockPtrByIndex(unique_block_idx);
}

void showTwoImagesSideBySide(
    const cv::Mat& resource_A, const cv::Mat& resource_B) {
  CHECK_EQ(resource_A.type(), resource_B.type());
  const cv::Size size_left = resource_A.size();
  const cv::Size size_right = resource_B.size();
  cv::Mat combined_cvmat(
      size_left.height, size_left.width + size_right.width, resource_B.type());
  cv::Mat left(
      combined_cvmat, cv::Rect(0, 0, size_left.width, size_left.height));
  resource_A.copyTo(left);
  cv::Mat right(
      combined_cvmat,
      cv::Rect(size_left.width, 0, size_right.width, size_right.height));
  resource_B.copyTo(right);
  static const std::string kWindowTitle = "Image Comparison";
  cv::imshow(kWindowTitle, combined_cvmat);
  cv::waitKey(0);
}

}  // namespace backend

#endif  // MAP_RESOURCES_TEST_RESOURCE_TEMPLATE_H_
