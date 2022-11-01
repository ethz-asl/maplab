#ifndef MAP_RESOURCES_TEST_RESOURCES_TEST_H_
#define MAP_RESOURCES_TEST_RESOURCES_TEST_H_

#include <string>
#include <unordered_map>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <maplab-common/file-system-tools.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <resources-common/tinyply/tinyply.h>

#include "map-resources/resource-common.h"
#include "map-resources/test/resource-template.h"

namespace backend {

const std::string kTestDataBaseFolder = "./map_resources_test_data/";  // NOLINT
const std::string kTestMapFolderA = "/map_A/";                         // NOLINT
const std::string kTestMapFolderB = "/map_B/";                         // NOLINT
const std::string kTestMapFolderC = "/map_C/";                         // NOLINT

const std::string kTestExternalFolderX =  // NOLINT
    "/foo/bar/external_folder_X/";
const std::string kTestExternalFolderY =  // NOLINT
    "/foobar/external_folder_Y/";
const std::string kTestExternalFolderZ =  // NOLINT
    "/foobar/external_folder_Z/";

const std::string kRestrictedFolder = "/foobar";  // NOLINT

class ResourceTest : public ::testing::Test {
 public:
  ResourceTest()
      : voxblox_tsdf_map_config_(),
        voxblox_tsdf_map_(voxblox_tsdf_map_config_),
        voxblox_esdf_map_config_(),
        voxblox_esdf_map_(voxblox_esdf_map_config_),
        voxblox_occupancy_map_config_(),
        voxblox_occupancy_map_(voxblox_occupancy_map_config_) {}

  virtual void SetUp() {}

  void loadPointcloud(
      const std::string& filename, resources::PointCloud* pointcloud) {
    CHECK_NOTNULL(pointcloud);
    pointcloud->loadFromFile(filename);
  }

  // NOTE: [ADD_RESOURCE_DATA_TYPE] Add and load test resource of new type.
  void loadTestResources() {
    // Read all the test resources.
    depth_cvmat_resource_ =
        cv::imread(kTestDataBaseFolder + "/16bit.pgm", CV_LOAD_IMAGE_UNCHANGED);
    CHECK_EQ(CV_MAT_TYPE(depth_cvmat_resource_.type()), CV_16U);

    grayscale_cvmat_resource_ =
        cv::imread(kTestDataBaseFolder + "/8bit.pgm", CV_LOAD_IMAGE_GRAYSCALE);
    CHECK_EQ(CV_MAT_TYPE(grayscale_cvmat_resource_.type()), CV_8U);

    color_cv_mat_resource_ =
        cv::imread(kTestDataBaseFolder + "/color.png", CV_LOAD_IMAGE_COLOR);
    CHECK_EQ(CV_MAT_TYPE(color_cv_mat_resource_.type()), CV_8UC3);

    loadPointcloud(
        kTestDataBaseFolder + "pcl_resource_xyz.ply",
        &pointcloud_xyz_resource_);

    // We generate a cloud with intensities based on the xyz coordinates.
    pointcloud_xyz_intensity_resource_ = pointcloud_xyz_resource_;
    pointcloud_xyz_intensity_resource_.scalars.resize(
        pointcloud_xyz_resource_.size());
    for (size_t idx = 0u; idx < pointcloud_xyz_resource_.size(); ++idx) {
      const size_t xyz_index = idx * 3;
      pointcloud_xyz_intensity_resource_.scalars[idx] =
          pointcloud_xyz_intensity_resource_.xyz[xyz_index] +
          pointcloud_xyz_intensity_resource_.xyz[xyz_index + 1u] +
          pointcloud_xyz_intensity_resource_.xyz[xyz_index + 2u];
    }

    loadPointcloud(
        kTestDataBaseFolder + "pcl_resource_xyz_rgb_normal.ply",
        &pointcloud_xyzrgbn_resource_);

    test_object_bounding_boxes_.resize(2);
    resources::ObjectInstanceBoundingBox& bbox_A =
        test_object_bounding_boxes_[0];
    bbox_A.bounding_box.x = 1;
    bbox_A.bounding_box.y = 2;
    bbox_A.bounding_box.width = 3;
    bbox_A.bounding_box.height = 4;
    bbox_A.class_number = 5;
    bbox_A.instance_number = 6;
    bbox_A.confidence = 0.97f;
    bbox_A.class_name = "class_A";
    resources::ObjectInstanceBoundingBox& bbox_B =
        test_object_bounding_boxes_[1];
    bbox_B.bounding_box.x = 7;
    bbox_B.bounding_box.y = 8;
    bbox_B.bounding_box.width = 9;
    bbox_B.bounding_box.height = 10;
    bbox_B.class_number = 11;
    bbox_B.instance_number = 12;
    bbox_B.confidence = 0.80f;
    bbox_B.class_name = "class_B";
  }

  // Create the resource templates for a predefined resource folder (map or
  // external).
  void createResourceTemplates(
      const std::string& test_name, const std::string& resource_sub_folder,
      const bool is_map_folder, ResourceTemplateBaseVector* templates) {
    CHECK_NOTNULL(templates);
    CHECK(!resource_sub_folder.empty());
    CHECK(!test_name.empty());

    loadTestResources();

    const std::string test_result_folder =
        kTestDataBaseFolder + "/" + test_name;
    test_result_folder_ = test_result_folder;

    std::string resource_folder;
    if (is_map_folder) {
      resource_folder =
          test_result_folder + "/" + resource_sub_folder + "/resources/";
    } else {
      resource_folder = test_result_folder + "/" + resource_sub_folder;
    }

    VLOG(2) << "Creating resource templates for the "
            << (is_map_folder ? "map" : "external")
            << " resource folder: " << resource_folder;

    // NOTE: [ADD_RESOURCE_TYPE] Add templates for new resource types.

    // NOTE: Every resource added below will create an additional death test in
    // most of the unit test. As death tests can be quite slow on OSX (about 7s
    // compared to 100-200ms on Ubuntu), we only test certain resource types on
    // Ubuntu.

    const bool is_external_folder = !is_map_folder;

    // Add depth map resources.
    templates->emplace_back(new ResourceTemplate<cv::Mat>(
        ResourceType::kRawDepthMap, resource_folder, is_external_folder,
        depth_cvmat_resource_));

#ifndef __APPLE__
    templates->emplace_back(new ResourceTemplate<cv::Mat>(
        ResourceType::kOptimizedDepthMap, resource_folder, is_external_folder,
        depth_cvmat_resource_));
#endif

#ifndef __APPLE__
    // Add grayscale image resources.
    templates->emplace_back(new ResourceTemplate<cv::Mat>(
        ResourceType::kRawImage, resource_folder, is_external_folder,
        grayscale_cvmat_resource_));
    templates->emplace_back(new ResourceTemplate<cv::Mat>(
        ResourceType::kUndistortedImage, resource_folder, is_external_folder,
        grayscale_cvmat_resource_));
    templates->emplace_back(new ResourceTemplate<cv::Mat>(
        ResourceType::kRectifiedImage, resource_folder, is_external_folder,
        grayscale_cvmat_resource_));
    templates->emplace_back(new ResourceTemplate<cv::Mat>(
        ResourceType::kImageForDepthMap, resource_folder, is_external_folder,
        grayscale_cvmat_resource_));
#endif

#ifndef __APPLE__
    // Add color image resources.
    templates->emplace_back(new ResourceTemplate<cv::Mat>(
        ResourceType::kRawColorImage, resource_folder, is_external_folder,
        color_cv_mat_resource_));
    templates->emplace_back(new ResourceTemplate<cv::Mat>(
        ResourceType::kUndistortedColorImage, resource_folder,
        is_external_folder, color_cv_mat_resource_));
    templates->emplace_back(new ResourceTemplate<cv::Mat>(
        ResourceType::kRectifiedColorImage, resource_folder, is_external_folder,
        color_cv_mat_resource_));
    templates->emplace_back(new ResourceTemplate<cv::Mat>(
        ResourceType::kColorImageForDepthMap, resource_folder,
        is_external_folder, color_cv_mat_resource_));
#endif

    // Point cloud resources.
    templates->emplace_back(new ResourceTemplate<resources::PointCloud>(
        ResourceType::kPointCloudXYZ, resource_folder, is_external_folder,
        pointcloud_xyz_resource_));
#ifndef __APPLE__
    templates->emplace_back(new ResourceTemplate<resources::PointCloud>(
        ResourceType::kPointCloudXYZRGBN, resource_folder, is_external_folder,
        pointcloud_xyzrgbn_resource_));
    templates->emplace_back(new ResourceTemplate<resources::PointCloud>(
        ResourceType::kPointCloudXYZI, resource_folder, is_external_folder,
        pointcloud_xyz_intensity_resource_));
#endif

#ifndef __APPLE__
    // Add text resources.
    templates->emplace_back(new ResourceTemplate<std::string>(
        ResourceType::kText, resource_folder, is_external_folder,
        "text_resource_kText"));
#endif

#ifndef __APPLE__
    // TODO(mfehr): path resources should be treated differently from text and
    // not stored in text
    // files but rather serialized as part of the map/meta_data.
    templates->emplace_back(new ResourceTemplate<std::string>(
        ResourceType::kPmvsReconstructionPath, resource_folder,
        is_external_folder, "text_resource_kPmvsReconstructionPath"));
    templates->emplace_back(new ResourceTemplate<std::string>(
        ResourceType::kTsdfGridPath, resource_folder, is_external_folder,
        "text_resource_kTsdfGridPath"));
    templates->emplace_back(new ResourceTemplate<std::string>(
        ResourceType::kEsdfGridPath, resource_folder, is_external_folder,
        "text_resource_kEsdfGridPath"));
    templates->emplace_back(new ResourceTemplate<std::string>(
        ResourceType::kOccupancyGridPath, resource_folder, is_external_folder,
        "text_resource_kOccupancyGridPath"));
#endif

#ifndef __APPLE__
    templates->emplace_back(new ResourceTemplate<voxblox::TsdfMap>(
        ResourceType::kVoxbloxTsdfMap, resource_folder, is_external_folder,
        voxblox_tsdf_map_));
    templates->emplace_back(new ResourceTemplate<voxblox::EsdfMap>(
        ResourceType::kVoxbloxEsdfMap, resource_folder, is_external_folder,
        voxblox_esdf_map_));
    templates->emplace_back(new ResourceTemplate<voxblox::OccupancyMap>(
        ResourceType::kVoxbloxOccupancyMap, resource_folder, is_external_folder,
        voxblox_occupancy_map_));
#endif

#ifndef __APPLE__
    // Add object bounding box resources.
    templates->emplace_back(
        new ResourceTemplate<resources::ObjectInstanceBoundingBoxes>(
            ResourceType::kObjectInstanceBoundingBoxes, resource_folder,
            is_external_folder, test_object_bounding_boxes_));
#endif
  }

  cv::Mat depth_cvmat_resource_;
  cv::Mat grayscale_cvmat_resource_;
  cv::Mat color_cv_mat_resource_;
  resources::PointCloud pointcloud_xyz_resource_;
  resources::PointCloud pointcloud_xyzrgbn_resource_;
  resources::PointCloud pointcloud_xyz_intensity_resource_;

  voxblox::TsdfMap::Config voxblox_tsdf_map_config_;
  voxblox::TsdfMap voxblox_tsdf_map_;

  voxblox::EsdfMap::Config voxblox_esdf_map_config_;
  voxblox::EsdfMap voxblox_esdf_map_;

  voxblox::OccupancyMap::Config voxblox_occupancy_map_config_;
  voxblox::OccupancyMap voxblox_occupancy_map_;

  std::string test_result_folder_;

  resources::ObjectInstanceBoundingBoxes test_object_bounding_boxes_;
};

}  // namespace backend

#endif  // MAP_RESOURCES_TEST_RESOURCES_TEST_H_
