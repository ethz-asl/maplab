#include <gflags/gflags.h>
#include <glog/logging.h>
#include <maplab-common/file-system-tools.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <opencv2/core.hpp>

#include "map-resources/resource-common.h"
#include "map-resources/resource-map.h"
#include "map-resources/resource_info_map.pb.h"
#include "map-resources/resource_metadata.pb.h"
#include "map-resources/test/resources-test.h"

namespace backend {

class ResourceMapTest : public ResourceTest {
 public:
  virtual void SetUp() {}

  template <typename DataType>
  void addResourceTemplateToMap(
      ResourceTemplateBase* template_base, ResourceMap* map) {
    CHECK_NOTNULL(template_base);
    CHECK_NOTNULL(map);
    ResourceTemplate<DataType>& resource_template =
        template_base->getAs<ResourceTemplate<DataType>>();
    map->addResource<DataType>(
        resource_template.type, resource_template.resource(),
        resource_template.folder, &(resource_template.id));
  }

  // Populate the resource map based on a set of templates.
  void addResourceTemplatesToMap(
      ResourceMap* map, ResourceTemplateBaseVector* templates) {
    CHECK_NOTNULL(map);
    CHECK_NOTNULL(templates);

    for (ResourceTemplateBase::Ptr& template_base : *templates) {
      // NOTE: [ADD_RESOURCE_DATA_TYPE] Add a case.
      switch (template_base->data_type) {
        case DataTypes::kCvMat: {
          addResourceTemplateToMap<cv::Mat>(template_base.get(), map);
          break;
        }
        case DataTypes::kText: {
          addResourceTemplateToMap<std::string>(template_base.get(), map);
          break;
        }
        case DataTypes::kPointCloud: {
          addResourceTemplateToMap<resources::PointCloud>(
              template_base.get(), map);
          break;
        }
        case DataTypes::kVoxbloxTsdfMap: {
          addResourceTemplateToMap<voxblox::TsdfMap>(template_base.get(), map);
          break;
        }
        case DataTypes::kVoxbloxEsdfMap: {
          addResourceTemplateToMap<voxblox::EsdfMap>(template_base.get(), map);
          break;
        }
        case DataTypes::kVoxbloxOccupancyMap: {
          addResourceTemplateToMap<voxblox::OccupancyMap>(
              template_base.get(), map);
          break;
        }
        default:
          LOG(FATAL) << "Unknown DataType: "
                     << static_cast<int>(template_base->data_type);
      }
    }
  }

  template <typename DataType>
  bool getResourceFromMap(
      ResourceTemplateBase* template_base, ResourceMap* map,
      DataType* resource) {
    CHECK_NOTNULL(template_base);
    CHECK_NOTNULL(map);
    CHECK_NOTNULL(resource);

    ResourceTemplate<DataType>& resource_template =
        template_base->getAs<ResourceTemplate<DataType>>();
    return map->getResource<DataType>(
        resource_template.id, resource_template.type, resource);
  }

  // Check if all templates can be loaded from the map and compare the returned
  // resource with the
  // one stored in the template.
  void getResourcesFromMapAndCheck(
      ResourceMap* map, const ResourceTemplateBaseVector& templates) {
    CHECK_NOTNULL(map);
    CHECK(!templates.empty());

    for (const ResourceTemplateBase::Ptr& template_base : templates) {
      // NOTE: [ADD_RESOURCE_DATA_TYPE] Add a case.
      switch (template_base->data_type) {
        case DataTypes::kCvMat: {
          cv::Mat resource;
          EXPECT_TRUE(getResourceFromMap(template_base.get(), map, &resource))
              << *template_base;

          ResourceTemplate<cv::Mat>& resource_template =
              template_base->getAs<ResourceTemplate<cv::Mat>>();
          EXPECT_TRUE(isSameResource(resource_template.resource(), resource))
              << *template_base;
          break;
        }
        case DataTypes::kText: {
          std::string resource;
          EXPECT_TRUE(getResourceFromMap(template_base.get(), map, &resource))
              << *template_base;

          ResourceTemplate<std::string>& resource_template =
              template_base->getAs<ResourceTemplate<std::string>>();
          EXPECT_TRUE(isSameResource(resource_template.resource(), resource))
              << *template_base;
          break;
        }
        case DataTypes::kPointCloud: {
          resources::PointCloud resource;
          EXPECT_TRUE(getResourceFromMap(template_base.get(), map, &resource))
              << *template_base;

          ResourceTemplate<resources::PointCloud>& resource_template =
              template_base->getAs<ResourceTemplate<resources::PointCloud>>();
          EXPECT_TRUE(isSameResource(resource_template.resource(), resource))
              << *template_base;
          break;
        }
        case DataTypes::kVoxbloxTsdfMap: {
          voxblox::TsdfMap::Config config;
          voxblox::TsdfMap resource(config);
          EXPECT_TRUE(getResourceFromMap(template_base.get(), map, &resource))
              << *template_base;

          ResourceTemplate<voxblox::TsdfMap>& resource_template =
              template_base->getAs<ResourceTemplate<voxblox::TsdfMap>>();
          EXPECT_TRUE(isSameResource(resource_template.resource(), resource))
              << *template_base;
          break;
        }
        case DataTypes::kVoxbloxEsdfMap: {
          voxblox::EsdfMap::Config config;
          voxblox::EsdfMap resource(config);
          EXPECT_TRUE(getResourceFromMap(template_base.get(), map, &resource))
              << *template_base;

          ResourceTemplate<voxblox::EsdfMap>& resource_template =
              template_base->getAs<ResourceTemplate<voxblox::EsdfMap>>();
          EXPECT_TRUE(isSameResource(resource_template.resource(), resource))
              << *template_base;
          break;
        }
        case DataTypes::kVoxbloxOccupancyMap: {
          voxblox::OccupancyMap::Config config;
          voxblox::OccupancyMap resource(config);
          EXPECT_TRUE(getResourceFromMap(template_base.get(), map, &resource))
              << *template_base;

          ResourceTemplate<voxblox::OccupancyMap>& resource_template =
              template_base->getAs<ResourceTemplate<voxblox::OccupancyMap>>();
          EXPECT_TRUE(isSameResource(resource_template.resource(), resource))
              << *template_base;
          break;
        }
        default:
          LOG(FATAL) << "Unknown DataType: "
                     << static_cast<int>(template_base->data_type);
      }
    }
  }

  // Delete resources corresponding to a set of templates and check if they are
  // not available
  // anymore.
  void deleteResourcesFromMapAndCheck(
      ResourceMap* map, const ResourceTemplateBaseVector& templates) {
    CHECK_NOTNULL(map);
    CHECK(!templates.empty());

    for (const ResourceTemplateBase::Ptr& template_base : templates) {
      // NOTE: [ADD_RESOURCE_DATA_TYPE] Add a case.
      switch (template_base->data_type) {
        case DataTypes::kCvMat: {
          const ResourceTemplate<cv::Mat>& resource_template =
              template_base->getAs<ResourceTemplate<cv::Mat>>();
          map->deleteResource<cv::Mat>(
              resource_template.id, resource_template.type);
          cv::Mat resource;
          EXPECT_FALSE(getResourceFromMap(template_base.get(), map, &resource))
              << *template_base;
          EXPECT_TRUE(resource.empty()) << *template_base;
          break;
        }
        case DataTypes::kText: {
          const ResourceTemplate<std::string>& resource_template =
              template_base->getAs<ResourceTemplate<std::string>>();
          map->deleteResource<std::string>(
              resource_template.id, resource_template.type);
          std::string resource;
          EXPECT_FALSE(getResourceFromMap(template_base.get(), map, &resource))
              << *template_base;
          EXPECT_TRUE(resource.empty()) << *template_base;
          break;
        }
        case DataTypes::kPointCloud: {
          const ResourceTemplate<resources::PointCloud>& resource_template =
              template_base->getAs<ResourceTemplate<resources::PointCloud>>();
          map->deleteResource<std::string>(
              resource_template.id, resource_template.type);
          resources::PointCloud resource;
          EXPECT_FALSE(getResourceFromMap(template_base.get(), map, &resource))
              << *template_base;
          EXPECT_TRUE(resource.empty()) << *template_base;
          break;
        }
        case DataTypes::kVoxbloxTsdfMap: {
          const ResourceTemplate<voxblox::TsdfMap>& resource_template =
              template_base->getAs<ResourceTemplate<voxblox::TsdfMap>>();
          map->deleteResource<std::string>(
              resource_template.id, resource_template.type);
          voxblox::TsdfMap::Config config;
          voxblox::TsdfMap resource(config);
          EXPECT_FALSE(getResourceFromMap(template_base.get(), map, &resource))
              << *template_base;
          break;
        }
        case DataTypes::kVoxbloxEsdfMap: {
          const ResourceTemplate<voxblox::EsdfMap>& resource_template =
              template_base->getAs<ResourceTemplate<voxblox::EsdfMap>>();
          map->deleteResource<std::string>(
              resource_template.id, resource_template.type);
          voxblox::EsdfMap::Config config;
          voxblox::EsdfMap resource(config);
          EXPECT_FALSE(getResourceFromMap(template_base.get(), map, &resource))
              << *template_base;
          break;
        }
        case DataTypes::kVoxbloxOccupancyMap: {
          const ResourceTemplate<voxblox::OccupancyMap>& resource_template =
              template_base->getAs<ResourceTemplate<voxblox::OccupancyMap>>();
          map->deleteResource<std::string>(
              resource_template.id, resource_template.type);
          voxblox::OccupancyMap::Config config;
          voxblox::OccupancyMap resource(config);
          EXPECT_FALSE(getResourceFromMap(template_base.get(), map, &resource))
              << *template_base;
          break;
        }
        default:
          LOG(FATAL) << "Unknown DataType: "
                     << static_cast<int>(template_base->data_type);
      }
    }
  }

  // Replace the resources of one template list with the resources of the other
  // template list.
  // Requires both template lists to be identlical with respect to number, order
  // and type of
  // resources.
  void replaceResourcesAndCheck(
      ResourceMap* map, const ResourceTemplateBaseVector& old_templates,
      const ResourceTemplateBaseVector& new_templates) {
    CHECK_EQ(old_templates.size(), new_templates.size());

    const size_t num_templates = new_templates.size();
    for (size_t template_idx = 0u; template_idx < num_templates;
         ++template_idx) {
      const ResourceTemplateBase::Ptr& old_template_base =
          old_templates[template_idx];
      const ResourceTemplateBase::Ptr& new_template_base =
          new_templates[template_idx];

      CHECK_EQ(
          static_cast<int>(old_template_base->data_type),
          static_cast<int>(new_template_base->data_type));
      // NOTE: [ADD_RESOURCE_DATA_TYPE] Add a case.
      switch (old_template_base->data_type) {
        case DataTypes::kCvMat: {
          const ResourceTemplate<cv::Mat>& old_template =
              old_template_base->getAs<ResourceTemplate<cv::Mat>>();
          const ResourceTemplate<cv::Mat>& new_template =
              new_template_base->getAs<ResourceTemplate<cv::Mat>>();

          map->replaceResource<cv::Mat>(
              old_template.id, old_template.type, new_template.resource());

          cv::Mat resource;
          EXPECT_TRUE(
              getResourceFromMap(old_template_base.get(), map, &resource))
              << old_template;
          EXPECT_TRUE(isSameResource(new_template.resource(), resource))
              << old_template;
          break;
        }
        case DataTypes::kText: {
          const ResourceTemplate<std::string>& old_template =
              old_template_base->getAs<ResourceTemplate<std::string>>();
          const ResourceTemplate<std::string>& new_template =
              new_template_base->getAs<ResourceTemplate<std::string>>();

          map->replaceResource<std::string>(
              old_template.id, old_template.type, new_template.resource());

          std::string resource;
          EXPECT_TRUE(
              getResourceFromMap(old_template_base.get(), map, &resource))
              << old_template;
          EXPECT_TRUE(isSameResource(new_template.resource(), resource))
              << old_template;
          break;
        }
        case DataTypes::kPointCloud: {
          const ResourceTemplate<resources::PointCloud>& old_template =
              old_template_base
                  ->getAs<ResourceTemplate<resources::PointCloud>>();
          const ResourceTemplate<resources::PointCloud>& new_template =
              new_template_base
                  ->getAs<ResourceTemplate<resources::PointCloud>>();

          map->replaceResource<resources::PointCloud>(
              old_template.id, old_template.type, new_template.resource());

          resources::PointCloud resource;
          EXPECT_TRUE(
              getResourceFromMap(old_template_base.get(), map, &resource))
              << old_template;
          EXPECT_TRUE(isSameResource(new_template.resource(), resource))
              << old_template;
          break;
        }
        case DataTypes::kVoxbloxTsdfMap: {
          const ResourceTemplate<voxblox::TsdfMap>& old_template =
              old_template_base->getAs<ResourceTemplate<voxblox::TsdfMap>>();
          const ResourceTemplate<voxblox::TsdfMap>& new_template =
              new_template_base->getAs<ResourceTemplate<voxblox::TsdfMap>>();

          map->replaceResource<voxblox::TsdfMap>(
              old_template.id, old_template.type, new_template.resource());

          voxblox::TsdfMap::Config config;
          voxblox::TsdfMap resource(config);
          EXPECT_TRUE(
              getResourceFromMap(old_template_base.get(), map, &resource))
              << old_template;
          EXPECT_TRUE(isSameResource(new_template.resource(), resource))
              << old_template;
          break;
        }
        case DataTypes::kVoxbloxEsdfMap: {
          const ResourceTemplate<voxblox::EsdfMap>& old_template =
              old_template_base->getAs<ResourceTemplate<voxblox::EsdfMap>>();
          const ResourceTemplate<voxblox::EsdfMap>& new_template =
              new_template_base->getAs<ResourceTemplate<voxblox::EsdfMap>>();

          map->replaceResource<voxblox::EsdfMap>(
              old_template.id, old_template.type, new_template.resource());

          voxblox::EsdfMap::Config config;
          voxblox::EsdfMap resource(config);
          EXPECT_TRUE(
              getResourceFromMap(old_template_base.get(), map, &resource))
              << old_template;
          EXPECT_TRUE(isSameResource(new_template.resource(), resource))
              << old_template;
          break;
        }
        case DataTypes::kVoxbloxOccupancyMap: {
          const ResourceTemplate<voxblox::OccupancyMap>& old_template =
              old_template_base
                  ->getAs<ResourceTemplate<voxblox::OccupancyMap>>();
          const ResourceTemplate<voxblox::OccupancyMap>& new_template =
              new_template_base
                  ->getAs<ResourceTemplate<voxblox::OccupancyMap>>();

          map->replaceResource<voxblox::OccupancyMap>(
              old_template.id, old_template.type, new_template.resource());

          voxblox::OccupancyMap::Config config;
          voxblox::OccupancyMap resource(config);
          EXPECT_TRUE(
              getResourceFromMap(old_template_base.get(), map, &resource))
              << old_template;
          EXPECT_TRUE(isSameResource(new_template.resource(), resource))
              << old_template;
          break;
        }
        default:
          LOG(FATAL) << "Unknown DataType: "
                     << static_cast<int>(old_template_base->data_type);
      }
    }
  }

  // Check if the cache statistic corresponds to a certain number of hits and
  // misses for every
  // template.
  void checkResourceCacheStatisticForTemplates(
      const ResourceMap& map, const ResourceTemplateBaseVector& templates,
      const size_t expected_hit, const size_t expected_miss) const {
    std::unordered_map<size_t, size_t> type_to_num_resources_per_type_map;
    const size_t num_templates = templates.size();
    for (size_t template_idx = 0u; template_idx < num_templates;
         ++template_idx) {
      const ResourceTemplateBase::Ptr& template_base = templates[template_idx];
      ++(type_to_num_resources_per_type_map[static_cast<size_t>(
          template_base->type)]);
    }

    for (size_t type_idx = 0u; type_idx < kNumResourceTypes; ++type_idx) {
      const size_t multiplier = type_to_num_resources_per_type_map[type_idx];

      EXPECT_EQ(
          map.getNumResourceCacheHits(static_cast<ResourceType>(type_idx)),
          expected_hit * multiplier);
      EXPECT_EQ(
          map.getNumResourceCacheMiss(static_cast<ResourceType>(type_idx)),
          expected_miss * multiplier);
    }

    map.printCacheStatisticsToLog(1);
    map.printResourceStatisticsToLog(1);
  }

  static constexpr bool kIsExternalFolder = false;
  static constexpr bool kIsMapFolder = true;

  ResourceTemplateBaseVector templates_map_A_;
  ResourceTemplateBaseVector templates_map_B_;

  ResourceTemplateBaseVector templates_external_folder_X_;
  ResourceTemplateBaseVector templates_external_folder_X_2_;
  ResourceTemplateBaseVector templates_external_folder_Y_;
  ResourceTemplateBaseVector templates_external_folder_Z_;
};

TEST_F(ResourceMapTest, TestResourceMapAddResourceDefaultFolder) {
  createResourceTemplates(
      "TestResourceMapAddResourceDefaultFolder", kTestMapFolderA, kIsMapFolder,
      &templates_map_A_);

  ResourceMap map(test_result_folder_ + kTestMapFolderA);

  addResourceTemplatesToMap(&map, &templates_map_A_);

  getResourcesFromMapAndCheck(&map, templates_map_A_);
  checkResourceCacheStatisticForTemplates(map, templates_map_A_, 0u, 1u);
  getResourcesFromMapAndCheck(&map, templates_map_A_);
  checkResourceCacheStatisticForTemplates(map, templates_map_A_, 1u, 1u);
  getResourcesFromMapAndCheck(&map, templates_map_A_);
  checkResourceCacheStatisticForTemplates(map, templates_map_A_, 2u, 1u);
}

TEST_F(ResourceMapTest, TestResourceMapAddResourceExternalFolder) {
  createResourceTemplates(
      "TestResourceMapAddResourceExternalFolder", kTestExternalFolderX,
      kIsExternalFolder, &templates_external_folder_X_);

  ResourceMap map(test_result_folder_ + kTestMapFolderA);
  addResourceTemplatesToMap(&map, &templates_external_folder_X_);

  getResourcesFromMapAndCheck(&map, templates_external_folder_X_);
  checkResourceCacheStatisticForTemplates(
      map, templates_external_folder_X_, 0u, 1u);
  getResourcesFromMapAndCheck(&map, templates_external_folder_X_);
  checkResourceCacheStatisticForTemplates(
      map, templates_external_folder_X_, 1u, 1u);
  getResourcesFromMapAndCheck(&map, templates_external_folder_X_);
  checkResourceCacheStatisticForTemplates(
      map, templates_external_folder_X_, 2u, 1u);
}

TEST_F(ResourceMapTest, TestResourceMapAddResourceDefaultAndExternalFolder) {
  createResourceTemplates(
      "TestResourceMapAddResourceDefaultAndExternalFolder", kTestMapFolderA,
      kIsMapFolder, &templates_map_A_);
  createResourceTemplates(
      "TestResourceMapAddResourceDefaultAndExternalFolder",
      kTestExternalFolderX, kIsExternalFolder, &templates_external_folder_X_);

  ResourceMap map(test_result_folder_ + kTestMapFolderA);
  addResourceTemplatesToMap(&map, &templates_map_A_);
  addResourceTemplatesToMap(&map, &templates_external_folder_X_);

  getResourcesFromMapAndCheck(&map, templates_map_A_);
  getResourcesFromMapAndCheck(&map, templates_external_folder_X_);
}

TEST_F(ResourceMapTest, TestResourceMapSwitchingResourceFolders) {
  FLAGS_v = 2;

  createResourceTemplates(
      "TestResourceMapSwitchingResourceFolders", kTestMapFolderA, kIsMapFolder,
      &templates_map_A_);
  createResourceTemplates(
      "TestResourceMapSwitchingResourceFolders", kTestMapFolderB, kIsMapFolder,
      &templates_map_B_);
  createResourceTemplates(
      "TestResourceMapSwitchingResourceFolders", kTestExternalFolderX,
      kIsExternalFolder, &templates_external_folder_X_);

  ResourceMap map(test_result_folder_ + kTestMapFolderA);

  // The resource folder in use should be the same as the map folder resource
  // folder.
  std::string default_resource_folder;
  map.getResourceFolderInUse(&default_resource_folder);
  EXPECT_TRUE(
      common::isSameRealPath(
          default_resource_folder,
          test_result_folder_ + kTestMapFolderA + "/resources"));
  map.getMapResourceFolder(&default_resource_folder);
  EXPECT_TRUE(
      common::isSameRealPath(
          default_resource_folder,
          test_result_folder_ + kTestMapFolderA + "/resources"));

  // No external folders expected.
  std::vector<std::string> external_folders;
  map.getExternalResourceFolders(&external_folders);
  EXPECT_TRUE(external_folders.empty());

  // Add and check resources for map resource folder A.
  addResourceTemplatesToMap(&map, &templates_map_A_);
  getResourcesFromMapAndCheck(&map, templates_map_A_);

  // The resource folder in use should be the same as the map folder resource
  // folder.
  map.getResourceFolderInUse(&default_resource_folder);
  EXPECT_TRUE(
      common::isSameRealPath(
          default_resource_folder,
          test_result_folder_ + kTestMapFolderA + "/resources"));
  map.getMapResourceFolder(&default_resource_folder);
  EXPECT_TRUE(
      common::isSameRealPath(
          default_resource_folder,
          test_result_folder_ + kTestMapFolderA + "/resources"));

  // No external folders expected.
  map.getExternalResourceFolders(&external_folders);
  EXPECT_TRUE(external_folders.empty());

  // Switch map folder should switch the default map folder and move the old one
  // to external folders.
  map.setMapFolder(test_result_folder_ + kTestMapFolderB);

  // Default and the folder that is actually used should be the same.
  map.getResourceFolderInUse(&default_resource_folder);
  EXPECT_TRUE(
      common::isSameRealPath(
          default_resource_folder,
          test_result_folder_ + kTestMapFolderB + "/resources"));
  map.getMapResourceFolder(&default_resource_folder);
  EXPECT_TRUE(
      common::isSameRealPath(
          default_resource_folder,
          test_result_folder_ + kTestMapFolderB + "/resources"));

  // Old default folder should have been moved to external folders.
  map.getExternalResourceFolders(&external_folders);
  EXPECT_EQ(external_folders.size(), 1u);
  if (external_folders.size() == 1u) {
    EXPECT_TRUE(
        common::isSameRealPath(
            external_folders[0],
            test_result_folder_ + kTestMapFolderA + "/resources"));
  }

  // Check if the resources from the old map resource folder are still
  // available.
  getResourcesFromMapAndCheck(&map, templates_map_A_);

  // Add and check resources for map resource folder B.
  addResourceTemplatesToMap(&map, &templates_map_B_);
  getResourcesFromMapAndCheck(&map, templates_map_B_);

  // Default and the folder that is actually used should be the same.
  map.getResourceFolderInUse(&default_resource_folder);
  EXPECT_TRUE(
      common::isSameRealPath(
          default_resource_folder,
          test_result_folder_ + kTestMapFolderB + "/resources"));
  map.getMapResourceFolder(&default_resource_folder);
  EXPECT_TRUE(
      common::isSameRealPath(
          default_resource_folder,
          test_result_folder_ + kTestMapFolderB + "/resources"));

  // Old default folder should have been moved to external folders.
  map.getExternalResourceFolders(&external_folders);
  EXPECT_EQ(external_folders.size(), 1u);
  if (external_folders.size() == 1u) {
    EXPECT_TRUE(
        common::isSameRealPath(
            external_folders[0],
            test_result_folder_ + kTestMapFolderA + "/resources"));
  }

  // Switch to external folder.
  map.useExternalResourceFolder(
      test_result_folder_ + "/" + kTestExternalFolderX);

  // Resource folder in use should point to external folder and default resource
  // folder to the
  // previously set one.
  map.getResourceFolderInUse(&default_resource_folder);
  EXPECT_TRUE(
      common::isSameRealPath(
          default_resource_folder, test_result_folder_ + kTestExternalFolderX));
  map.getMapResourceFolder(&default_resource_folder);
  EXPECT_TRUE(
      common::isSameRealPath(
          default_resource_folder,
          test_result_folder_ + kTestMapFolderB + "/resources"));

  // We should now have 2 external resoruce folders.
  map.getExternalResourceFolders(&external_folders);
  EXPECT_EQ(external_folders.size(), 2u);
  if (external_folders.size() == 2u) {
    EXPECT_TRUE(
        common::isSameRealPath(
            external_folders[0],
            test_result_folder_ + kTestMapFolderA + "/resources"));
    EXPECT_TRUE(
        common::isSameRealPath(
            external_folders[1], test_result_folder_ + kTestExternalFolderX));
  }

  // Check if the resources from map folder A and B are still available.
  getResourcesFromMapAndCheck(&map, templates_map_A_);
  getResourcesFromMapAndCheck(&map, templates_map_B_);

  // Add and check resources for external folder X.
  addResourceTemplatesToMap(&map, &templates_external_folder_X_);
  getResourcesFromMapAndCheck(&map, templates_external_folder_X_);

  // Trying to set the same external resource folder should not result in an
  // additional resource folder being registered.
  map.useExternalResourceFolder(
      test_result_folder_ + "/" + kTestExternalFolderX);

  // Should still be the same as before registering the external resource folder
  // again.
  map.getResourceFolderInUse(&default_resource_folder);
  EXPECT_TRUE(
      common::isSameRealPath(
          default_resource_folder, test_result_folder_ + kTestExternalFolderX));
  map.getMapResourceFolder(&default_resource_folder);
  EXPECT_TRUE(
      common::isSameRealPath(
          default_resource_folder,
          test_result_folder_ + kTestMapFolderB + "/resources"));

  // We should still have the same 2 external resource folders.
  map.getExternalResourceFolders(&external_folders);
  EXPECT_EQ(external_folders.size(), 2u);
  if (external_folders.size() == 2u) {
    EXPECT_TRUE(
        common::isSameRealPath(
            external_folders[0],
            test_result_folder_ + kTestMapFolderA + "/resources"));
    EXPECT_TRUE(
        common::isSameRealPath(
            external_folders[1], test_result_folder_ + kTestExternalFolderX));
  }

  // Check if the resources from map folder A, B  and the external folder are
  // still available.
  getResourcesFromMapAndCheck(&map, templates_map_A_);
  getResourcesFromMapAndCheck(&map, templates_map_B_);
  getResourcesFromMapAndCheck(&map, templates_external_folder_X_);

  // Switch back to default resource folder.
  map.useMapResourceFolder();

  // Both the default and the folder in use should be the same again.
  map.getResourceFolderInUse(&default_resource_folder);
  EXPECT_TRUE(
      common::isSameRealPath(
          default_resource_folder,
          test_result_folder_ + kTestMapFolderB + "/resources"));
  map.getMapResourceFolder(&default_resource_folder);
  EXPECT_TRUE(
      common::isSameRealPath(
          default_resource_folder,
          test_result_folder_ + kTestMapFolderB + "/resources"));

  // Should not change the external resource folders. We should still have 2
  // external resource folders.
  map.getExternalResourceFolders(&external_folders);
  EXPECT_EQ(external_folders.size(), 2u);
  if (external_folders.size() == 2u) {
    EXPECT_TRUE(
        common::isSameRealPath(
            external_folders[0],
            test_result_folder_ + kTestMapFolderA + "/resources"));
    EXPECT_TRUE(
        common::isSameRealPath(
            external_folders[1], test_result_folder_ + kTestExternalFolderX));
  }

  // Check if the resources from map folder A, B  and the external folder are
  // still available.
  getResourcesFromMapAndCheck(&map, templates_map_A_);
  getResourcesFromMapAndCheck(&map, templates_map_B_);
  getResourcesFromMapAndCheck(&map, templates_external_folder_X_);
}

TEST_F(ResourceMapTest, TestResourceMapDeleteResource) {
  createResourceTemplates(
      "TestResourceMapAddResourceDefaultAndExternalFolder", kTestMapFolderA,
      kIsMapFolder, &templates_map_A_);
  createResourceTemplates(
      "TestResourceMapAddResourceDefaultAndExternalFolder",
      kTestExternalFolderX, kIsExternalFolder, &templates_external_folder_X_);

  ResourceMap map(test_result_folder_ + kTestMapFolderA);

  // Add and check resources.
  addResourceTemplatesToMap(&map, &templates_map_A_);
  addResourceTemplatesToMap(&map, &templates_external_folder_X_);
  getResourcesFromMapAndCheck(&map, templates_map_A_);
  getResourcesFromMapAndCheck(&map, templates_external_folder_X_);

  // Delete resources and check that they are not available anymore.
  deleteResourcesFromMapAndCheck(&map, templates_map_A_);
  deleteResourcesFromMapAndCheck(&map, templates_external_folder_X_);
}

TEST_F(ResourceMapTest, TestResourceMapReplaceResource) {
  createResourceTemplates(
      "TestResourceMapReplaceResource", kTestMapFolderA, kIsMapFolder,
      &templates_map_A_);
  createResourceTemplates(
      "TestResourceMapReplaceResource", kTestMapFolderB, kIsMapFolder,
      &templates_map_B_);
  createResourceTemplates(
      "TestResourceMapReplaceResource", kTestExternalFolderX, kIsExternalFolder,
      &templates_external_folder_X_);
  createResourceTemplates(
      "TestResourceMapReplaceResource", kTestExternalFolderY, kIsExternalFolder,
      &templates_external_folder_Y_);

  ResourceMap map(test_result_folder_ + kTestMapFolderA);

  // Add and check resources.
  addResourceTemplatesToMap(&map, &templates_map_A_);
  addResourceTemplatesToMap(&map, &templates_external_folder_X_);
  getResourcesFromMapAndCheck(&map, templates_map_A_);
  getResourcesFromMapAndCheck(&map, templates_external_folder_X_);

  replaceResourcesAndCheck(&map, templates_map_A_, templates_map_B_);
  replaceResourcesAndCheck(
      &map, templates_external_folder_X_, templates_external_folder_Y_);
}

TEST_F(ResourceMapTest, TestResourceMapMigrateResourcesToMapFolder) {
  createResourceTemplates(
      "TestResourceMapMigrateResourcesToMapFolder", kTestMapFolderA,
      kIsMapFolder, &templates_map_A_);
  createResourceTemplates(
      "TestResourceMapMigrateResourcesToMapFolder", kTestExternalFolderX,
      kIsExternalFolder, &templates_external_folder_X_);
  createResourceTemplates(
      "TestResourceMapMigrateResourcesToMapFolder", kTestExternalFolderY,
      kIsExternalFolder, &templates_external_folder_Y_);

  ResourceMap map(test_result_folder_ + kTestMapFolderA);

  // Add and check resources.
  addResourceTemplatesToMap(&map, &templates_map_A_);
  addResourceTemplatesToMap(&map, &templates_external_folder_X_);
  addResourceTemplatesToMap(&map, &templates_external_folder_Y_);
  getResourcesFromMapAndCheck(&map, templates_map_A_);
  getResourcesFromMapAndCheck(&map, templates_external_folder_X_);
  getResourcesFromMapAndCheck(&map, templates_external_folder_Y_);

  // Check if map resoure folder is correct.
  std::string default_resource_folder;
  map.getMapResourceFolder(&default_resource_folder);
  EXPECT_TRUE(
      common::isSameRealPath(
          default_resource_folder,
          test_result_folder_ + kTestMapFolderA + "/resources"));

  // Check if we have the correct external resource folders.
  std::vector<std::string> external_folders;
  map.getExternalResourceFolders(&external_folders);
  EXPECT_EQ(external_folders.size(), 2u);
  if (external_folders.size() == 2u) {
    EXPECT_TRUE(
        common::isSameRealPath(
            external_folders[0], test_result_folder_ + kTestExternalFolderX));
    EXPECT_TRUE(
        common::isSameRealPath(
            external_folders[1], test_result_folder_ + kTestExternalFolderY));
  }

  constexpr bool kMoveResources = true;
  map.migrateAllResourcesToMapResourceFolder(kMoveResources);

  // Check if all external resource folders have been removed.
  map.getExternalResourceFolders(&external_folders);
  EXPECT_EQ(external_folders.size(), 0u);

  // Check if all resources are still available.
  getResourcesFromMapAndCheck(&map, templates_map_A_);
  getResourcesFromMapAndCheck(&map, templates_external_folder_X_);
  getResourcesFromMapAndCheck(&map, templates_external_folder_Y_);

  // Check if the map resource folder is set correctly and also in use.
  map.getMapResourceFolder(&default_resource_folder);
  EXPECT_TRUE(
      common::isSameRealPath(
          default_resource_folder,
          test_result_folder_ + kTestMapFolderA + "/resources"));
  map.getResourceFolderInUse(&default_resource_folder);
  EXPECT_TRUE(
      common::isSameRealPath(
          default_resource_folder,
          test_result_folder_ + kTestMapFolderA + "/resources"));
}

TEST_F(ResourceMapTest, TestResourceMapMergeMapsAndCleanUp) {
  FLAGS_v = 1;

  createResourceTemplates(
      "TestResourceMapMergeMapsAndCleanUp", kTestMapFolderA, kIsMapFolder,
      &templates_map_A_);
  createResourceTemplates(
      "TestResourceMapMergeMapsAndCleanUp", kTestExternalFolderX,
      kIsExternalFolder, &templates_external_folder_X_);
  createResourceTemplates(
      "TestResourceMapMergeMapsAndCleanUp", kTestExternalFolderY,
      kIsExternalFolder, &templates_external_folder_Y_);

  // Add and check resources to map A.
  ResourceMap map_A(test_result_folder_ + kTestMapFolderA);
  addResourceTemplatesToMap(&map_A, &templates_map_A_);
  addResourceTemplatesToMap(&map_A, &templates_external_folder_X_);
  addResourceTemplatesToMap(&map_A, &templates_external_folder_Y_);
  getResourcesFromMapAndCheck(&map_A, templates_map_A_);
  getResourcesFromMapAndCheck(&map_A, templates_external_folder_X_);
  getResourcesFromMapAndCheck(&map_A, templates_external_folder_Y_);

  createResourceTemplates(
      "TestResourceMapMergeMapsAndCleanUp", kTestMapFolderB, kIsMapFolder,
      &templates_map_B_);
  // NOTE(mfehr): we generate more resources for external folder X to provoke a
  // collision of the external resource folders. The templates are stored in a
  // separate X_2 member variable.
  createResourceTemplates(
      "TestResourceMapMergeMapsAndCleanUp", kTestExternalFolderX,
      kIsExternalFolder, &templates_external_folder_X_2_);
  createResourceTemplates(
      "TestResourceMapMergeMapsAndCleanUp", kTestExternalFolderZ,
      kIsExternalFolder, &templates_external_folder_Z_);

  // Add and check resources to map B.
  ResourceMap map_B(test_result_folder_ + kTestMapFolderB);
  addResourceTemplatesToMap(&map_B, &templates_map_B_);
  addResourceTemplatesToMap(&map_B, &templates_external_folder_X_2_);
  addResourceTemplatesToMap(&map_B, &templates_external_folder_Z_);
  getResourcesFromMapAndCheck(&map_B, templates_map_B_);
  getResourcesFromMapAndCheck(&map_B, templates_external_folder_X_2_);
  getResourcesFromMapAndCheck(&map_B, templates_external_folder_Z_);

  // Check state of map resource folder for both maps.
  std::string default_resource_folder;
  map_A.getMapResourceFolder(&default_resource_folder);
  EXPECT_TRUE(
      common::isSameRealPath(
          default_resource_folder,
          test_result_folder_ + kTestMapFolderA + "/resources"));
  map_A.getResourceFolderInUse(&default_resource_folder);
  EXPECT_TRUE(
      common::isSameRealPath(
          default_resource_folder,
          test_result_folder_ + kTestMapFolderA + "/resources"));
  map_B.getMapResourceFolder(&default_resource_folder);
  EXPECT_TRUE(
      common::isSameRealPath(
          default_resource_folder,
          test_result_folder_ + kTestMapFolderB + "/resources"));
  map_B.getResourceFolderInUse(&default_resource_folder);
  EXPECT_TRUE(
      common::isSameRealPath(
          default_resource_folder,
          test_result_folder_ + kTestMapFolderB + "/resources"));

  // Check state of external resource folder for both maps.
  std::vector<std::string> external_folders;
  map_A.getExternalResourceFolders(&external_folders);
  EXPECT_EQ(external_folders.size(), 2u);
  if (external_folders.size() == 2u) {
    EXPECT_TRUE(
        common::isSameRealPath(
            external_folders[0], test_result_folder_ + kTestExternalFolderX));
    EXPECT_TRUE(
        common::isSameRealPath(
            external_folders[1], test_result_folder_ + kTestExternalFolderY));
  }
  map_B.getExternalResourceFolders(&external_folders);
  EXPECT_EQ(external_folders.size(), 2u);
  if (external_folders.size() == 2u) {
    EXPECT_TRUE(
        common::isSameRealPath(
            external_folders[0], test_result_folder_ + kTestExternalFolderX));
    EXPECT_TRUE(
        common::isSameRealPath(
            external_folders[1], test_result_folder_ + kTestExternalFolderZ));
  }

  // Merge maps.
  map_A.mergeFromMap(map_B);

  // Default folder should not have changed.
  map_A.getMapResourceFolder(&default_resource_folder);
  EXPECT_TRUE(
      common::isSameRealPath(
          default_resource_folder,
          test_result_folder_ + kTestMapFolderA + "/resources"));
  map_A.getResourceFolderInUse(&default_resource_folder);
  EXPECT_TRUE(
      common::isSameRealPath(
          default_resource_folder,
          test_result_folder_ + kTestMapFolderA + "/resources"));

  // External folder should contain all folders of map B.
  map_A.getExternalResourceFolders(&external_folders);
  EXPECT_EQ(external_folders.size(), 5u);
  if (external_folders.size() == 5u) {
    EXPECT_TRUE(
        common::isSameRealPath(
            external_folders[0], test_result_folder_ + kTestExternalFolderX));
    EXPECT_TRUE(
        common::isSameRealPath(
            external_folders[1], test_result_folder_ + kTestExternalFolderY));
    EXPECT_TRUE(
        common::isSameRealPath(
            external_folders[2],
            test_result_folder_ + kTestMapFolderB + "/resources/"));
    EXPECT_TRUE(
        common::isSameRealPath(
            external_folders[3], test_result_folder_ + kTestExternalFolderX));
    EXPECT_TRUE(
        common::isSameRealPath(
            external_folders[4], test_result_folder_ + kTestExternalFolderZ));
  }

  // Check if resources are still accessible.
  getResourcesFromMapAndCheck(&map_A, templates_map_A_);
  getResourcesFromMapAndCheck(&map_A, templates_external_folder_X_);
  getResourcesFromMapAndCheck(&map_A, templates_external_folder_Y_);
  getResourcesFromMapAndCheck(&map_A, templates_map_B_);
  getResourcesFromMapAndCheck(&map_A, templates_external_folder_X_2_);
  getResourcesFromMapAndCheck(&map_A, templates_external_folder_Z_);

  // Cleanup map.
  map_A.cleanupResourceFolders();

  // Default folder should not have changed.
  map_A.getMapResourceFolder(&default_resource_folder);
  EXPECT_TRUE(
      common::isSameRealPath(
          default_resource_folder,
          test_result_folder_ + kTestMapFolderA + "/resources"));
  map_A.getResourceFolderInUse(&default_resource_folder);
  EXPECT_TRUE(
      common::isSameRealPath(
          default_resource_folder,
          test_result_folder_ + kTestMapFolderA + "/resources"));

  // External folder X should have been merged.
  map_A.getExternalResourceFolders(&external_folders);
  EXPECT_EQ(external_folders.size(), 4u);
  if (external_folders.size() == 4u) {
    EXPECT_TRUE(
        common::isSameRealPath(
            external_folders[0], test_result_folder_ + kTestExternalFolderX));
    EXPECT_TRUE(
        common::isSameRealPath(
            external_folders[1], test_result_folder_ + kTestExternalFolderY));
    EXPECT_TRUE(
        common::isSameRealPath(
            external_folders[2],
            test_result_folder_ + kTestMapFolderB + "/resources/"));
    EXPECT_TRUE(
        common::isSameRealPath(
            external_folders[3], test_result_folder_ + kTestExternalFolderZ));
  }

  // Check if resources are still accessible.
  getResourcesFromMapAndCheck(&map_A, templates_map_A_);
  getResourcesFromMapAndCheck(&map_A, templates_external_folder_X_);
  getResourcesFromMapAndCheck(&map_A, templates_external_folder_Y_);
  getResourcesFromMapAndCheck(&map_A, templates_map_B_);
  getResourcesFromMapAndCheck(&map_A, templates_external_folder_X_2_);
  getResourcesFromMapAndCheck(&map_A, templates_external_folder_Z_);

  // Migrate and move all resources to the map resource folder.
  constexpr bool kMoveResources = true;
  map_A.migrateAllResourcesToMapResourceFolder(kMoveResources);

  // Default folder should not have changed.
  map_A.getMapResourceFolder(&default_resource_folder);
  EXPECT_TRUE(
      common::isSameRealPath(
          default_resource_folder,
          test_result_folder_ + kTestMapFolderA + "/resources"));
  map_A.getResourceFolderInUse(&default_resource_folder);
  EXPECT_TRUE(
      common::isSameRealPath(
          default_resource_folder,
          test_result_folder_ + kTestMapFolderA + "/resources"));

  // External folders should now be empty.
  map_A.getExternalResourceFolders(&external_folders);
  EXPECT_TRUE(external_folders.empty());

  // Check if resources are still accessible.
  getResourcesFromMapAndCheck(&map_A, templates_map_A_);
  getResourcesFromMapAndCheck(&map_A, templates_external_folder_X_);
  getResourcesFromMapAndCheck(&map_A, templates_external_folder_Y_);
  getResourcesFromMapAndCheck(&map_A, templates_map_B_);
  getResourcesFromMapAndCheck(&map_A, templates_external_folder_X_2_);
  getResourcesFromMapAndCheck(&map_A, templates_external_folder_Z_);

  // Migrate and move all resources to the a external folder.
  map_A.migrateAllResourcesToFolder(
      test_result_folder_ + kTestExternalFolderX, kMoveResources);

  // Default folder should point to the external folder now, but the map
  // resource folder should remain the same.
  map_A.getMapResourceFolder(&default_resource_folder);
  EXPECT_TRUE(
      common::isSameRealPath(
          default_resource_folder,
          test_result_folder_ + kTestMapFolderA + "/resources"));
  map_A.getResourceFolderInUse(&default_resource_folder);
  EXPECT_TRUE(
      common::isSameRealPath(
          default_resource_folder, test_result_folder_ + kTestExternalFolderX));

  // External folders should now be empty.
  map_A.getExternalResourceFolders(&external_folders);
  EXPECT_EQ(external_folders.size(), 1u);
  if (external_folders.size() == 1u) {
    EXPECT_TRUE(
        common::isSameRealPath(
            external_folders[0], test_result_folder_ + kTestExternalFolderX));
  }

  // Check if resources are still accessible.
  getResourcesFromMapAndCheck(&map_A, templates_map_A_);
  getResourcesFromMapAndCheck(&map_A, templates_external_folder_X_);
  getResourcesFromMapAndCheck(&map_A, templates_external_folder_Y_);
  getResourcesFromMapAndCheck(&map_A, templates_map_B_);
  getResourcesFromMapAndCheck(&map_A, templates_external_folder_X_2_);
  getResourcesFromMapAndCheck(&map_A, templates_external_folder_Z_);
}

TEST_F(ResourceMapTest, TestResourceMapFullVsRelativePath) {
  // Make sure that if we use different relative / full paths they will be
  // recognized as the same folder and treated accordingly.
  createResourceTemplates(
      "TestResourceMapFullVsRelativePath", "awesome_maps/map_A", kIsMapFolder,
      &templates_map_A_);
  createResourceTemplates(
      "TestResourceMapFullVsRelativePath", "awesome_maps/../awesome_maps/map_A",
      kIsMapFolder, &templates_map_B_);

  const std::string full_path =
      common::getCurrentWorkingDirectory() +
      "/map_resources_test_data/TestResourceMapFullVsRelativePath/awesome_maps/"
      "map_A";

  ResourceMap map(full_path);

  addResourceTemplatesToMap(&map, &templates_map_A_);
  addResourceTemplatesToMap(&map, &templates_map_B_);
  getResourcesFromMapAndCheck(&map, templates_map_A_);
  getResourcesFromMapAndCheck(&map, templates_map_B_);

  // Check default folder and map resource folder.
  std::string default_resource_folder;
  map.getMapResourceFolder(&default_resource_folder);
  EXPECT_TRUE(
      common::isSameRealPath(
          default_resource_folder,
          test_result_folder_ + "/awesome_maps/map_A/resources"));
  map.getResourceFolderInUse(&default_resource_folder);
  EXPECT_TRUE(
      common::isSameRealPath(
          default_resource_folder,
          test_result_folder_ + "/awesome_maps/map_A/resources"));

  // External folders should now be empty.
  std::vector<std::string> external_folders;
  map.getExternalResourceFolders(&external_folders);
  EXPECT_TRUE(external_folders.empty());
}

TEST_F(ResourceMapTest, TestResourceMapMigrateResourcesToExternalFolder) {
  createResourceTemplates(
      "TestResourceMapMigrateResourcesToExternalFolder", kTestMapFolderA,
      kIsMapFolder, &templates_map_A_);
  createResourceTemplates(
      "TestResourceMapMigrateResourcesToExternalFolder", kTestExternalFolderX,
      kIsExternalFolder, &templates_external_folder_X_);
  createResourceTemplates(
      "TestResourceMapMigrateResourcesToExternalFolder", kTestExternalFolderY,
      kIsExternalFolder, &templates_external_folder_Y_);

  ResourceMap map(test_result_folder_ + kTestMapFolderA);

  // Add and check resources.
  addResourceTemplatesToMap(&map, &templates_map_A_);
  addResourceTemplatesToMap(&map, &templates_external_folder_X_);
  addResourceTemplatesToMap(&map, &templates_external_folder_Y_);
  getResourcesFromMapAndCheck(&map, templates_map_A_);
  getResourcesFromMapAndCheck(&map, templates_external_folder_X_);
  getResourcesFromMapAndCheck(&map, templates_external_folder_Y_);

  // Check if map resoure folder is correct.
  std::string default_resource_folder;
  map.getMapResourceFolder(&default_resource_folder);
  EXPECT_TRUE(
      common::isSameRealPath(
          default_resource_folder,
          test_result_folder_ + kTestMapFolderA + "/resources"));

  // Check if we have the correct external resource folders.
  std::vector<std::string> external_folders;
  map.getExternalResourceFolders(&external_folders);
  EXPECT_EQ(external_folders.size(), 2u);
  if (external_folders.size() == 2u) {
    EXPECT_TRUE(
        common::isSameRealPath(
            external_folders[0], test_result_folder_ + kTestExternalFolderX));
    EXPECT_TRUE(
        common::isSameRealPath(
            external_folders[1], test_result_folder_ + kTestExternalFolderY));
  }

  constexpr bool kMoveResources = true;
  map.migrateAllResourcesToFolder(
      test_result_folder_ + kTestExternalFolderZ, kMoveResources);

  // Check if all external resource folders have been removed.
  map.getExternalResourceFolders(&external_folders);
  EXPECT_EQ(external_folders.size(), 1u);
  if (external_folders.size() == 1u) {
    EXPECT_TRUE(
        common::isSameRealPath(
            external_folders[0], test_result_folder_ + kTestExternalFolderZ));
  }

  // Check if all resources are still available.
  getResourcesFromMapAndCheck(&map, templates_map_A_);
  getResourcesFromMapAndCheck(&map, templates_external_folder_X_);
  getResourcesFromMapAndCheck(&map, templates_external_folder_Y_);

  // Check if the map resource folder still points to the previous folder, but
  // the external folder is in use.
  map.getMapResourceFolder(&default_resource_folder);
  EXPECT_TRUE(
      common::isSameRealPath(
          default_resource_folder,
          test_result_folder_ + kTestMapFolderA + "/resources"));
  map.getResourceFolderInUse(&default_resource_folder);
  EXPECT_TRUE(
      common::isSameRealPath(
          default_resource_folder, test_result_folder_ + kTestExternalFolderZ));
}

TEST_F(ResourceMapTest, TestMetaDataSerialization) {
  static const std::string kDummyMapDescription =
      "This is the most awesome map ever created!";

  ResourceMap::MetaData metadata_before(test_result_folder_ + kTestMapFolderA);
  metadata_before.map_description = kDummyMapDescription;

  metadata_before.map_resource_folder = test_result_folder_ + kTestMapFolderA +
                                        metadata_before.kResourceFolderName;
  metadata_before.resource_folder_in_use = ResourceMap::kMapResourceFolder;
  metadata_before.map_description = kDummyMapDescription;
  metadata_before.external_resource_folders.push_back(
      test_result_folder_ + kTestExternalFolderX);
  metadata_before.external_resource_folders.push_back(
      test_result_folder_ + kTestExternalFolderY);
  metadata_before.external_resource_folders.push_back(
      test_result_folder_ + kTestExternalFolderZ);

  metadata::proto::MetaData metadata_proto;
  metadata_before.serialize(&metadata_proto);

  ResourceMap::MetaData metadata_after(metadata_proto);

  EXPECT_EQ(metadata_before.map_folder, metadata_after.map_folder);
  EXPECT_EQ(
      metadata_before.map_resource_folder, metadata_after.map_resource_folder);
  EXPECT_EQ(metadata_before.map_description, metadata_after.map_description);
  EXPECT_EQ(
      metadata_before.resource_folder_in_use,
      metadata_after.resource_folder_in_use);
  EXPECT_EQ(
      metadata_before.external_resource_folders.size(),
      metadata_after.external_resource_folders.size());

  for (size_t i = 0u; i < metadata_before.external_resource_folders.size();
       ++i) {
    EXPECT_EQ(
        metadata_before.external_resource_folders[i],
        metadata_after.external_resource_folders[i]);
  }
}

TEST_F(ResourceMapTest, TestResourceMapSerialization) {
  createResourceTemplates(
      "TestResourceMapSerialization", kTestMapFolderA, kIsMapFolder,
      &templates_map_A_);
  createResourceTemplates(
      "TestResourceMapSerialization", kTestExternalFolderX, kIsExternalFolder,
      &templates_external_folder_X_);

  ResourceMap map_before(test_result_folder_ + kTestMapFolderA);
  addResourceTemplatesToMap(&map_before, &templates_map_A_);
  addResourceTemplatesToMap(&map_before, &templates_external_folder_X_);
  getResourcesFromMapAndCheck(&map_before, templates_map_A_);
  getResourcesFromMapAndCheck(&map_before, templates_external_folder_X_);

  resource_info::proto::ResourceInfoMap map_proto;
  map_before.serializeResourceInfo(&map_proto);
  metadata::proto::MetaData metadata;
  map_before.serializeMetaData(&metadata);

  ResourceMap map_after(metadata);
  map_after.deserializeResourceInfo(map_proto);

  getResourcesFromMapAndCheck(&map_after, templates_map_A_);
  getResourcesFromMapAndCheck(&map_after, templates_external_folder_X_);

  ResourceMap::MetaData metadata_before = map_before.getMetaDataCopy();
  ResourceMap::MetaData metadata_after = map_after.getMetaDataCopy();

  EXPECT_EQ(metadata_before.map_folder, metadata_after.map_folder);
  EXPECT_EQ(
      metadata_before.map_resource_folder, metadata_after.map_resource_folder);
  EXPECT_EQ(metadata_before.map_description, metadata_after.map_description);
  EXPECT_EQ(
      metadata_before.resource_folder_in_use,
      metadata_after.resource_folder_in_use);
  EXPECT_EQ(
      metadata_before.external_resource_folders.size(),
      metadata_after.external_resource_folders.size());

  for (size_t i = 0u; i < metadata_before.external_resource_folders.size();
       ++i) {
    EXPECT_EQ(
        metadata_before.external_resource_folders[i],
        metadata_after.external_resource_folders[i]);
  }
}

TEST_F(ResourceMapTest, TestResourceInfoSerializationSplit) {
  createResourceTemplates(
      "TestResourceInfoSerializationSplit", kTestMapFolderA, kIsMapFolder,
      &templates_map_A_);
  createResourceTemplates(
      "TestResourceInfoSerializationSplit", kTestExternalFolderX,
      kIsExternalFolder, &templates_external_folder_X_);

  ResourceMap map_before(test_result_folder_ + kTestMapFolderA);
  addResourceTemplatesToMap(&map_before, &templates_map_A_);
  addResourceTemplatesToMap(&map_before, &templates_external_folder_X_);
  getResourcesFromMapAndCheck(&map_before, templates_map_A_);
  getResourcesFromMapAndCheck(&map_before, templates_external_folder_X_);

  const size_t num_resources = map_before.numResources();

  const size_t num_resources_per_proto_test_cases[] = {1u,
                                                       2u,
                                                       5u,
                                                       num_resources - 1u,
                                                       num_resources,
                                                       num_resources + 1u,
                                                       2u * num_resources};
  for (const size_t num_resource_per_proto :
       num_resources_per_proto_test_cases) {
    CHECK_GT(num_resource_per_proto, 0u);
    const size_t num_resource_protos =
        (num_resources + num_resource_per_proto - 1u) / num_resource_per_proto;

    std::vector<resource_info::proto::ResourceInfoMap> resource_info_protos(
        num_resource_protos);
    for (size_t proto_idx = 0u; proto_idx < num_resource_protos; ++proto_idx) {
      resource_info::proto::ResourceInfoMap& map_proto =
          resource_info_protos[proto_idx];
      map_before.serializeResourceInfo(
          &map_proto, proto_idx * num_resource_per_proto,
          num_resource_per_proto);
    }

    metadata::proto::MetaData metadata;
    map_before.serializeMetaData(&metadata);

    ResourceMap map_after(metadata);
    for (size_t proto_idx = 0u; proto_idx < num_resource_protos; ++proto_idx) {
      const resource_info::proto::ResourceInfoMap& map_proto =
          resource_info_protos[proto_idx];
      map_after.deserializeResourceInfo(map_proto);
    }

    EXPECT_EQ(num_resources, map_after.numResources());

    getResourcesFromMapAndCheck(&map_after, templates_map_A_);
    getResourcesFromMapAndCheck(&map_after, templates_external_folder_X_);
  }
}

TEST_F(ResourceMapTest, TestResourceInfoSerializationEmpty) {
  ResourceMap map_before(test_result_folder_ + kTestMapFolderA);

  const size_t num_resources = map_before.numResources();

  resource_info::proto::ResourceInfoMap map_proto;
  map_before.serializeResourceInfo(&map_proto);

  metadata::proto::MetaData metadata;
  map_before.serializeMetaData(&metadata);

  ResourceMap map_after(metadata);
  map_after.deserializeResourceInfo(map_proto);

  EXPECT_EQ(num_resources, map_after.numResources());
}

}  // namespace backend

MAPLAB_UNITTEST_ENTRYPOINT
