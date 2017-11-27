#include "map-resources/resource-loader.h"

#include <cstdio>
#include <fstream>  // NOLINT

#include <maplab-common/file-system-tools.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <voxblox/io/layer_io.h>

#include "map-resources/tinyply/tinyply.h"

namespace backend {

void ResourceLoader::migrateResource(
    const ResourceId& id, const ResourceType& type,
    const std::string& old_folder, const std::string& new_folder,
    const bool move_resource) {
  CHECK(!old_folder.empty());
  CHECK(!new_folder.empty());
  std::string old_file_path;
  getResourceFilePath(id, type, old_folder, &old_file_path);
  CHECK(common::fileExists(old_file_path))
      << "path: \'" << old_file_path << "\'";

  std::string new_file_path;
  getResourceFilePath(id, type, new_folder, &new_file_path);
  CHECK(!common::fileExists(new_file_path));
  CHECK(common::createPathToFile(new_file_path));

  std::ifstream source_file(old_file_path, std::ios::binary);
  std::ofstream destination_file(new_file_path, std::ios::binary);
  CHECK(source_file.is_open());
  CHECK(destination_file.is_open());

  // Copy the file.
  destination_file << source_file.rdbuf();

  if (move_resource) {
    common::deleteFile(old_file_path);
  }
}

void ResourceLoader::deleteResourceFile(
    const ResourceId& id, const ResourceType& type, const std::string& folder) {
  CHECK(!folder.empty());
  std::string file_path;
  getResourceFilePath(id, type, folder, &file_path);
  CHECK_EQ(std::remove(file_path.c_str()), 0);
}

void ResourceLoader::getResourceFilePath(
    const ResourceId& id, const ResourceType& type, const std::string& folder,
    std::string* file_path) const {
  CHECK(!folder.empty());
  CHECK_NOTNULL(file_path)->clear();

  common::concatenateFolderAndFileName(
      folder, ResourceTypeNames[static_cast<size_t>(type)], file_path);

  const std::string filename =
      id.hexString() + ResourceTypeFileSuffix[static_cast<size_t>(type)];
  common::concatenateFolderAndFileName(*file_path, filename, file_path);
}

bool ResourceLoader::resourceFileExists(
    const ResourceId& id, const ResourceType& type,
    const std::string& folder) const {
  CHECK(!folder.empty());
  std::string file_path;
  getResourceFilePath(id, type, folder, &file_path);
  return common::fileExists(file_path);
}

template <>
void ResourceLoader::saveResourceToFile<cv::Mat>(
    const std::string& file_path, const ResourceType& /*type*/,
    const cv::Mat& resource) const {
  CHECK(!file_path.empty());
  CHECK(!common::fileExists(file_path));
  CHECK(common::createPathToFile(file_path));
  CHECK(cv::imwrite(file_path, resource))
      << "Failed to store cv::Mat to " << file_path << ".";
}

// NOTE: [ADD_RESOURCE_TYPE] Add case if you add a new cv::Mat resource type.
template <>
bool ResourceLoader::loadResourceFromFile<cv::Mat>(
    const std::string& file_path, const ResourceType& type,
    cv::Mat* resource) const {
  CHECK(!file_path.empty());
  CHECK_NOTNULL(resource);
  if (!common::fileExists(file_path)) {
    VLOG(1) << "Resource file does not exist! Path: " << file_path;
    return false;
  }

  bool wrong_type = false;
  switch (type) {
    case ResourceType::kRawDepthMap:
    case ResourceType::kOptimizedDepthMap:
      *resource = cv::imread(file_path, CV_LOAD_IMAGE_UNCHANGED);
      wrong_type = CV_MAT_TYPE(resource->type()) != CV_16U;
      break;
    case ResourceType::kUndistortedImage:
    case ResourceType::kRectifiedImage:
    case ResourceType::kImageForDepthMap:
    case ResourceType::kRawImage:
      *resource = cv::imread(file_path, CV_LOAD_IMAGE_GRAYSCALE);
      wrong_type = CV_MAT_TYPE(resource->type()) != CV_8U;
      break;
    case ResourceType::kUndistortedColorImage:
    case ResourceType::kRectifiedColorImage:
    case ResourceType::kColorImageForDepthMap:
    case ResourceType::kRawColorImage:
      *resource = cv::imread(file_path, CV_LOAD_IMAGE_COLOR);
      wrong_type = CV_MAT_TYPE(resource->type()) != CV_8UC3;
      break;
    case ResourceType::kDisparityMap:
      *resource = cv::imread(file_path, CV_LOAD_IMAGE_UNCHANGED);
      wrong_type = CV_MAT_TYPE(resource->type()) != CV_16U;
      break;
    default:
      LOG(FATAL) << "Unknown cv::Mat resource type: "
                 << ResourceTypeNames[static_cast<size_t>(type)];
  }
  if (wrong_type) {
    VLOG(1) << "cv::Mat Resource at: " << file_path << " has wrong image type!";
    return false;
  }

  bool empty_resource = !resource->data || resource->empty();
  if (empty_resource) {
    VLOG(1) << "Loading resource of type "
            << ResourceTypeNames[static_cast<size_t>(type)] << " from "
            << file_path << " failed!";
    return false;
  }
  return true;
}

template <>
void ResourceLoader::saveResourceToFile<std::string>(
    const std::string& file_path, const ResourceType& /*type*/,
    const std::string& resource) const {
  CHECK(!file_path.empty());
  CHECK(!common::fileExists(file_path));
  CHECK(common::createPathToFile(file_path));

  std::ofstream text_file(file_path);
  CHECK(text_file.is_open())
      << "Could not write text resource to file_path: " << file_path;
  text_file << resource;
}

// NOTE: [ADD_RESOURCE_TYPE] Add case if you add a new string resource type.
template <>
bool ResourceLoader::loadResourceFromFile<std::string>(
    const std::string& file_path, const ResourceType& type,
    std::string* resource) const {
  CHECK(!file_path.empty());
  CHECK_NOTNULL(resource);
  if (!common::fileExists(file_path)) {
    VLOG(1) << "Resource file does not exist! path: " << file_path;
    return false;
  }

  std::ifstream infile(file_path);
  if (!infile.is_open()) {
    VLOG(1) << "Could not open text resource file " << file_path;
    return false;
  }
  std::stringstream buffer;
  switch (type) {
    case ResourceType::kPmvsReconstructionPath:
    case ResourceType::kTsdfGridPath:
    case ResourceType::kEsdfGridPath:
    case ResourceType::kOccupancyGridPath:
    // TODO(mfehr): don't read and write path resources to file but store it
    // in the maps meta data.

    //  Fall through intended.
    case ResourceType::kText:
      buffer << infile.rdbuf();
      *resource = buffer.str();
      break;
    default:
      LOG(FATAL) << "Unknown text resource type: "
                 << ResourceTypeNames[static_cast<size_t>(type)];
  }
  if (resource->empty()) {
    VLOG(1) << "The std::string resource of type "
            << ResourceTypeNames[static_cast<size_t>(type)]
            << " is empty! file_path: " << file_path;
    return false;
  }
  return true;
}

template <>
void ResourceLoader::saveResourceToFile<voxblox::TsdfMap>(
    const std::string& file_path, const ResourceType& /*type*/,
    const voxblox::TsdfMap& resource) const {
  CHECK(!file_path.empty());
  CHECK(!common::fileExists(file_path));
  CHECK(common::createPathToFile(file_path));

  CHECK(voxblox::io::SaveLayer(resource.getTsdfLayer(), file_path));
}

template <>
bool ResourceLoader::loadResourceFromFile<voxblox::TsdfMap>(
    const std::string& file_path, const ResourceType& /*type*/,
    voxblox::TsdfMap* resource) const {
  CHECK(!file_path.empty());
  CHECK_NOTNULL(resource);
  if (!common::fileExists(file_path)) {
    VLOG(1) << "Resource file does not exist! Path: " << file_path;
    return false;
  }

  voxblox::Layer<voxblox::TsdfVoxel>::Ptr layer_ptr;
  const bool success =
      voxblox::io::LoadLayer<voxblox::TsdfVoxel>(file_path, &layer_ptr);
  if (success) {
    *resource = voxblox::TsdfMap(layer_ptr);
  }
  return success;
}

template <>
void ResourceLoader::saveResourceToFile<voxblox::EsdfMap>(
    const std::string& file_path, const ResourceType& /*type*/,
    const voxblox::EsdfMap& resource) const {
  CHECK(!file_path.empty());
  CHECK(!common::fileExists(file_path));
  CHECK(common::createPathToFile(file_path));

  CHECK(voxblox::io::SaveLayer(resource.getEsdfLayer(), file_path));
}

template <>
bool ResourceLoader::loadResourceFromFile<voxblox::EsdfMap>(
    const std::string& file_path, const ResourceType& /*type*/,
    voxblox::EsdfMap* resource) const {
  CHECK(!file_path.empty());
  CHECK_NOTNULL(resource);
  if (!common::fileExists(file_path)) {
    VLOG(1) << "Resource file does not exist! Path: " << file_path;
    return false;
  }

  voxblox::Layer<voxblox::EsdfVoxel>::Ptr layer_ptr;
  const bool success =
      voxblox::io::LoadLayer<voxblox::EsdfVoxel>(file_path, &layer_ptr);

  if (success) {
    *resource = voxblox::EsdfMap(layer_ptr);
  }

  return success;
}

template <>
void ResourceLoader::saveResourceToFile<voxblox::OccupancyMap>(
    const std::string& file_path, const ResourceType& /*type*/,
    const voxblox::OccupancyMap& resource) const {
  CHECK(!file_path.empty());
  CHECK(!common::fileExists(file_path));
  CHECK(common::createPathToFile(file_path));

  CHECK(voxblox::io::SaveLayer(resource.getOccupancyLayer(), file_path));
}

template <>
bool ResourceLoader::loadResourceFromFile<voxblox::OccupancyMap>(
    const std::string& file_path, const ResourceType& /*type*/,
    voxblox::OccupancyMap* resource) const {
  CHECK(!file_path.empty());
  CHECK_NOTNULL(resource);
  if (!common::fileExists(file_path)) {
    VLOG(1) << "Resource file does not exist! Path: " << file_path;
    return false;
  }

  voxblox::Layer<voxblox::OccupancyVoxel>::Ptr layer_ptr;
  const bool success =
      voxblox::io::LoadLayer<voxblox::OccupancyVoxel>(file_path, &layer_ptr);

  if (success) {
    *resource = voxblox::OccupancyMap(layer_ptr);
  }
  return success;
}

template <>
void ResourceLoader::saveResourceToFile(
    const std::string& file_path, const ResourceType& /*type*/,
    const resources::PointCloud& resource) const {
  CHECK(!common::fileExists(file_path)) << "path: " << file_path;
  CHECK(common::createPathToFile(file_path));

  std::filebuf filebuf;
  filebuf.open(file_path, std::ios::out | std::ios::binary);
  CHECK(filebuf.is_open());

  std::ostream output_stream(&filebuf);
  tinyply::PlyFile ply_file;

  // Const-casting is necessary as tinyply requires non-const access to the
  // vectors for reading.
  ply_file.add_properties_to_element(
      "vertex", {"x", "y", "z"}, const_cast<std::vector<float>&>(resource.xyz));
  if (!resource.normals.empty()) {
    ply_file.add_properties_to_element(
        "vertex", {"nx", "ny", "nz"},
        const_cast<std::vector<float>&>(resource.normals));
  }
  if (!resource.colors.empty()) {
    ply_file.add_properties_to_element(
        "vertex", {"red", "green", "blue"},
        const_cast<std::vector<unsigned char>&>(resource.colors));
  }

  ply_file.comments.push_back("generated by tinyply from maplab");
  ply_file.write(output_stream, true);
  filebuf.close();
}

template <>
bool ResourceLoader::loadResourceFromFile(
    const std::string& file_path, const ResourceType& /*type*/,
    resources::PointCloud* resource) const {
  CHECK_NOTNULL(resource);
  if (!common::fileExists(file_path)) {
    VLOG(1) << "Resource file does not exist! Path: " << file_path;
    return false;
  }

  std::ifstream stream_ply(file_path);
  if (stream_ply.is_open()) {
    tinyply::PlyFile ply_file(stream_ply);
    const int xyz_point_count = ply_file.request_properties_from_element(
        "vertex", {"x", "y", "z"}, resource->xyz);
    const int colors_count = ply_file.request_properties_from_element(
        "vertex", {"nx", "ny", "nz"}, resource->normals);
    const int normals_count = ply_file.request_properties_from_element(
        "vertex", {"red", "green", "blue"}, resource->colors);
    if (xyz_point_count > 0) {
      if (colors_count > 0) {
        // If colors are present, their count should match the point count.
        CHECK_EQ(xyz_point_count, colors_count);
      }
      if (normals_count > 0) {
        // If normals are present, their count should match the point count.
        CHECK_EQ(xyz_point_count, normals_count);
      }

      ply_file.read(stream_ply);
    }
    stream_ply.close();
    return true;
  }
  return false;
}

const CacheStatistic& ResourceLoader::getCacheStatistic() const {
  return cache_.getStatistic();
}

const ResourceCache::Config& ResourceLoader::getCacheConfig() const {
  return cache_.getConfig();
}

}  // namespace backend
