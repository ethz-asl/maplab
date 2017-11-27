#include <glog/logging.h>
#include <maplab-common/file-system-tools.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <opencv2/core.hpp>

#include "map-resources/resource-common.h"
#include "map-resources/resource-loader.h"
#include "map-resources/test/resources-test.h"

namespace backend {

class ResourceLoaderTest : public ResourceTest {
 public:
  virtual void SetUp() {}

  template <typename DataType>
  void addTemplateToResourceLoader(
      ResourceTemplateBase* template_base, ResourceLoader* loader) {
    CHECK_NOTNULL(template_base);
    CHECK_NOTNULL(loader);
    ResourceTemplate<DataType>& resource_template =
        template_base->getAs<ResourceTemplate<DataType>>();
    loader->addResource<DataType>(
        resource_template.id, resource_template.type, resource_template.folder,
        resource_template.resource());
  }

  // Populate the resource loader with a prepared set of resource templates.
  void addTemplatesToResourceLoader(
      ResourceLoader* loader, ResourceTemplateBaseVector* templates) {
    CHECK_NOTNULL(loader);
    CHECK_NOTNULL(templates);

    for (ResourceTemplateBase::Ptr& template_base : templates_) {
      // NOTE: [ADD_RESOURCE_DATA_TYPE] Add a case.
      switch (template_base->data_type) {
        case DataTypes::kCvMat: {
          addTemplateToResourceLoader<cv::Mat>(template_base.get(), loader);
          break;
        }
        case DataTypes::kText: {
          addTemplateToResourceLoader<std::string>(template_base.get(), loader);
          break;
        }
        case DataTypes::kPointCloud: {
          addTemplateToResourceLoader<resources::PointCloud>(
              template_base.get(), loader);
          break;
        }
        case DataTypes::kVoxbloxTsdfMap: {
          addTemplateToResourceLoader<voxblox::TsdfMap>(
              template_base.get(), loader);
          break;
        }
        case DataTypes::kVoxbloxEsdfMap: {
          addTemplateToResourceLoader<voxblox::EsdfMap>(
              template_base.get(), loader);
          break;
        }
        case DataTypes::kVoxbloxOccupancyMap: {
          addTemplateToResourceLoader<voxblox::OccupancyMap>(
              template_base.get(), loader);
          break;
        }
        default:
          LOG(FATAL) << "Unknown DataType: "
                     << static_cast<int>(template_base->data_type);
      }
    }
  }

  template <typename DataType>
  bool getAndCheckTemplateFromResourceLoader(
      ResourceTemplateBase* template_base, DataType* resource,
      ResourceLoader* loader) {
    CHECK_NOTNULL(template_base);
    CHECK_NOTNULL(resource);
    CHECK_NOTNULL(loader);

    ResourceTemplate<DataType>& resource_template =
        template_base->getAs<ResourceTemplate<DataType>>();

    loader->getResource<DataType>(
        resource_template.id, resource_template.type, resource_template.folder,
        resource);

    return isSameResource(resource_template.resource(), *resource);
  }

  // Retrieve resources based on the templates from resource loader and compare
  // them to each other.
  void getAndCheckTemplatesFromResourceLoader(
      ResourceLoader* loader, ResourceTemplateBaseVector* resource_templates) {
    CHECK_NOTNULL(loader);
    CHECK_NOTNULL(resource_templates);
    for (ResourceTemplateBase::Ptr& template_base : *resource_templates) {
      // NOTE: [ADD_RESOURCE_DATA_TYPE] Add a case.
      switch (template_base->data_type) {
        case DataTypes::kCvMat: {
          cv::Mat resource;
          getAndCheckTemplateFromResourceLoader(
              template_base.get(), &resource, loader);
          break;
        }
        case DataTypes::kText: {
          std::string resource;
          getAndCheckTemplateFromResourceLoader(
              template_base.get(), &resource, loader);
          break;
        }
        case DataTypes::kPointCloud: {
          resources::PointCloud resource;
          getAndCheckTemplateFromResourceLoader(
              template_base.get(), &resource, loader);
          break;
        }
        case DataTypes::kVoxbloxTsdfMap: {
          voxblox::TsdfMap::Config config;
          voxblox::TsdfMap resource(config);
          getAndCheckTemplateFromResourceLoader(
              template_base.get(), &resource, loader);
          break;
        }
        case DataTypes::kVoxbloxEsdfMap: {
          voxblox::EsdfMap::Config config;
          voxblox::EsdfMap resource(config);
          getAndCheckTemplateFromResourceLoader(
              template_base.get(), &resource, loader);
          break;
        }
        case DataTypes::kVoxbloxOccupancyMap: {
          voxblox::OccupancyMap::Config config;
          voxblox::OccupancyMap resource(config);
          getAndCheckTemplateFromResourceLoader(
              template_base.get(), &resource, loader);
          break;
        }
        default:
          LOG(FATAL) << "Unknown data_type: "
                     << static_cast<int>(template_base->data_type);
      }
    }
  }

  ResourceTemplateBaseVector templates_;
};

TEST_F(ResourceLoaderTest, TestAddResource) {
  constexpr bool kIsMapFolder = true;
  createResourceTemplates(
      "TestAddResource", kTestMapFolderA, kIsMapFolder, &templates_);

  ResourceLoader loader;
  addTemplatesToResourceLoader(&loader, &templates_);
}

TEST_F(ResourceLoaderTest, TestAddResourceCannotCreateFile) {
  constexpr bool kIsMapFolder = true;
  createResourceTemplates(
      "TestAddResourceCannotCreateFile", kTestMapFolderA, kIsMapFolder,
      &templates_);

  static const std::string kErrorMsg = "createPathToFile";

  ResourceLoader loader;
  for (ResourceTemplateBase::Ptr& template_base : templates_) {
    template_base->folder = kRestrictedFolder;
    // NOTE: [ADD_RESOURCE_DATA_TYPE] Add a case.
    switch (template_base->data_type) {
      case DataTypes::kCvMat: {
        EXPECT_DEATH(
            addTemplateToResourceLoader<cv::Mat>(template_base.get(), &loader),
            kErrorMsg);
        break;
      }
      case DataTypes::kText: {
        EXPECT_DEATH(
            addTemplateToResourceLoader<std::string>(
                template_base.get(), &loader),
            kErrorMsg);
        break;
      }
      case DataTypes::kPointCloud: {
        EXPECT_DEATH(
            addTemplateToResourceLoader<resources::PointCloud>(
                template_base.get(), &loader),
            kErrorMsg);
        break;
      }
      case DataTypes::kVoxbloxTsdfMap: {
        EXPECT_DEATH(
            addTemplateToResourceLoader<voxblox::TsdfMap>(
                template_base.get(), &loader),
            kErrorMsg);
        break;
      }
      case DataTypes::kVoxbloxEsdfMap: {
        EXPECT_DEATH(
            addTemplateToResourceLoader<voxblox::EsdfMap>(
                template_base.get(), &loader),
            kErrorMsg);
        break;
      }
      case DataTypes::kVoxbloxOccupancyMap: {
        EXPECT_DEATH(
            addTemplateToResourceLoader<voxblox::OccupancyMap>(
                template_base.get(), &loader),
            kErrorMsg);
        break;
      }
      default:
        LOG(FATAL) << "Unknown DataType: "
                   << static_cast<int>(template_base->data_type);
    }
  }
}

TEST_F(ResourceLoaderTest, TestAddResourceEmptyFolder) {
  constexpr bool kIsMapFolder = true;
  createResourceTemplates(
      "TestAddResourceEmptyFolder", kTestMapFolderA, kIsMapFolder, &templates_);

  static const std::string kErrorMsg = "empty()";

  ResourceLoader loader;
  for (ResourceTemplateBase::Ptr& template_base : templates_) {
    template_base->folder = "";
    // NOTE: [ADD_RESOURCE_DATA_TYPE] Add a case.
    switch (template_base->data_type) {
      case DataTypes::kCvMat: {
        EXPECT_DEATH(
            addTemplateToResourceLoader<cv::Mat>(template_base.get(), &loader),
            kErrorMsg);
        break;
      }
      case DataTypes::kText: {
        EXPECT_DEATH(
            addTemplateToResourceLoader<std::string>(
                template_base.get(), &loader),
            kErrorMsg);
        break;
      }
      case DataTypes::kPointCloud: {
        EXPECT_DEATH(
            addTemplateToResourceLoader<resources::PointCloud>(
                template_base.get(), &loader),
            kErrorMsg);
        break;
      }
      case DataTypes::kVoxbloxTsdfMap: {
        EXPECT_DEATH(
            addTemplateToResourceLoader<voxblox::TsdfMap>(
                template_base.get(), &loader),
            kErrorMsg);
        break;
      }
      case DataTypes::kVoxbloxEsdfMap: {
        EXPECT_DEATH(
            addTemplateToResourceLoader<voxblox::EsdfMap>(
                template_base.get(), &loader),
            kErrorMsg);
        break;
      }
      case DataTypes::kVoxbloxOccupancyMap: {
        EXPECT_DEATH(
            addTemplateToResourceLoader<voxblox::OccupancyMap>(
                template_base.get(), &loader),
            kErrorMsg);
        break;
      }
      default:
        LOG(FATAL) << "Unknown DataType: "
                   << static_cast<int>(template_base->data_type);
    }
  }
}

TEST_F(ResourceLoaderTest, TestAddResourceDuplicateFiles) {
  constexpr bool kIsMapFolder = true;
  createResourceTemplates(
      "TestAddResourceDuplicateFiles", kTestMapFolderA, kIsMapFolder,
      &templates_);

  ResourceLoader loader;
  addTemplatesToResourceLoader(&loader, &templates_);

  static const std::string kErrorMsg = "fileExists";

  for (ResourceTemplateBase::Ptr& template_base : templates_) {
    // NOTE: [ADD_RESOURCE_DATA_TYPE] Add a case.
    switch (template_base->data_type) {
      case DataTypes::kCvMat: {
        EXPECT_DEATH(
            addTemplateToResourceLoader<cv::Mat>(template_base.get(), &loader),
            kErrorMsg);
        break;
      }
      case DataTypes::kText: {
        EXPECT_DEATH(
            addTemplateToResourceLoader<std::string>(
                template_base.get(), &loader),
            kErrorMsg);
        break;
      }
      case DataTypes::kPointCloud: {
        EXPECT_DEATH(
            addTemplateToResourceLoader<resources::PointCloud>(
                template_base.get(), &loader),
            kErrorMsg);
        break;
      }
      case DataTypes::kVoxbloxTsdfMap: {
        EXPECT_DEATH(
            addTemplateToResourceLoader<voxblox::TsdfMap>(
                template_base.get(), &loader),
            kErrorMsg);
        break;
      }
      case DataTypes::kVoxbloxEsdfMap: {
        EXPECT_DEATH(
            addTemplateToResourceLoader<voxblox::EsdfMap>(
                template_base.get(), &loader),
            kErrorMsg);
        break;
      }
      case DataTypes::kVoxbloxOccupancyMap: {
        EXPECT_DEATH(
            addTemplateToResourceLoader<voxblox::OccupancyMap>(
                template_base.get(), &loader),
            kErrorMsg);
        break;
      }
      default:
        LOG(FATAL) << "Unknown data_type: "
                   << static_cast<int>(template_base->data_type);
    }
  }
}

TEST_F(ResourceLoaderTest, TestGetResource) {
  constexpr bool kIsMapFolder = true;
  createResourceTemplates(
      "TestGetResource", kTestMapFolderA, kIsMapFolder, &templates_);

  ResourceLoader loader;
  addTemplatesToResourceLoader(&loader, &templates_);

  getAndCheckTemplatesFromResourceLoader(&loader, &templates_);
}

TEST_F(ResourceLoaderTest, TestGetInexistentResource) {
  constexpr bool kIsMapFolder = true;
  createResourceTemplates(
      "TestGetInexistentResource", kTestMapFolderA, kIsMapFolder, &templates_);

  static const std::string kErrorMsg = "Failed to load";

  ResourceLoader loader;
  for (ResourceTemplateBase::Ptr& template_base : templates_) {
    // NOTE: [ADD_RESOURCE_DATA_TYPE] Add a case.
    switch (template_base->data_type) {
      case DataTypes::kCvMat: {
        cv::Mat resource;
        EXPECT_DEATH(
            getAndCheckTemplateFromResourceLoader(
                template_base.get(), &resource, &loader),
            kErrorMsg);
        break;
      }
      case DataTypes::kText: {
        std::string resource;
        EXPECT_DEATH(
            getAndCheckTemplateFromResourceLoader(
                template_base.get(), &resource, &loader),
            kErrorMsg);
        break;
      }
      case DataTypes::kPointCloud: {
        resources::PointCloud resource;
        EXPECT_DEATH(
            getAndCheckTemplateFromResourceLoader(
                template_base.get(), &resource, &loader),
            kErrorMsg);
        break;
      }
      case DataTypes::kVoxbloxTsdfMap: {
        voxblox::TsdfMap::Config config;
        voxblox::TsdfMap resource(config);
        EXPECT_DEATH(
            getAndCheckTemplateFromResourceLoader(
                template_base.get(), &resource, &loader),
            kErrorMsg);
        break;
      }
      case DataTypes::kVoxbloxEsdfMap: {
        voxblox::EsdfMap::Config config;
        voxblox::EsdfMap resource(config);
        EXPECT_DEATH(
            getAndCheckTemplateFromResourceLoader(
                template_base.get(), &resource, &loader),
            kErrorMsg);
        break;
      }
      case DataTypes::kVoxbloxOccupancyMap: {
        voxblox::OccupancyMap::Config config;
        voxblox::OccupancyMap resource(config);
        EXPECT_DEATH(
            getAndCheckTemplateFromResourceLoader(
                template_base.get(), &resource, &loader),
            kErrorMsg);
        break;
      }
      default:
        LOG(FATAL) << "Unknown data_type: "
                   << static_cast<int>(template_base->data_type);
    }
  }
}

TEST_F(ResourceLoaderTest, TestGetResourceWrongType) {
  constexpr bool kIsMapFolder = true;
  createResourceTemplates(
      "TestGetResourceWrongType", kTestMapFolderA, kIsMapFolder, &templates_);

  static const std::string kErrorMsg = "Failed to load";

  ResourceLoader loader;
  addTemplatesToResourceLoader(&loader, &templates_);

  for (ResourceTemplateBase::Ptr& template_base : templates_) {
    // NOTE: [ADD_RESOURCE_DATA_TYPE] Add a case.
    switch (template_base->data_type) {
      case DataTypes::kCvMat: {
        // Assign wrong resource type.
        template_base->type = ResourceType::kText;
        cv::Mat resource;
        EXPECT_DEATH(
            getAndCheckTemplateFromResourceLoader(
                template_base.get(), &resource, &loader),
            kErrorMsg);
        break;
      }
      case DataTypes::kText: {
        // Assign wrong resource type.
        template_base->type = ResourceType::kOptimizedDepthMap;
        std::string resource;
        EXPECT_DEATH(
            getAndCheckTemplateFromResourceLoader(
                template_base.get(), &resource, &loader),
            kErrorMsg);
        break;
      }
      case DataTypes::kPointCloud: {
        // Assign wrong resource type.
        template_base->type = ResourceType::kOptimizedDepthMap;
        resources::PointCloud resource;
        EXPECT_DEATH(
            getAndCheckTemplateFromResourceLoader(
                template_base.get(), &resource, &loader),
            kErrorMsg);
        break;
      }
      case DataTypes::kVoxbloxTsdfMap: {
        // Assign wrong resource type.
        template_base->type = ResourceType::kOptimizedDepthMap;
        voxblox::TsdfMap::Config config;
        voxblox::TsdfMap resource(config);
        EXPECT_DEATH(
            getAndCheckTemplateFromResourceLoader(
                template_base.get(), &resource, &loader),
            kErrorMsg);
        break;
      }
      case DataTypes::kVoxbloxEsdfMap: {
        // Assign wrong resource type.
        template_base->type = ResourceType::kOptimizedDepthMap;
        voxblox::EsdfMap::Config config;
        voxblox::EsdfMap resource(config);
        EXPECT_DEATH(
            getAndCheckTemplateFromResourceLoader(
                template_base.get(), &resource, &loader),
            kErrorMsg);
        break;
      }
      case DataTypes::kVoxbloxOccupancyMap: {
        // Assign wrong resource type.
        template_base->type = ResourceType::kOptimizedDepthMap;
        voxblox::OccupancyMap::Config config;
        voxblox::OccupancyMap resource(config);
        EXPECT_DEATH(
            getAndCheckTemplateFromResourceLoader(
                template_base.get(), &resource, &loader),
            kErrorMsg);
        break;
      }
      default:
        LOG(FATAL) << "Unknown DataType: "
                   << static_cast<int>(template_base->data_type);
    }
  }
}

TEST_F(ResourceLoaderTest, TestGetResourceWrongId) {
  constexpr bool kIsMapFolder = true;
  createResourceTemplates(
      "TestGetResourceWrongId", kTestMapFolderA, kIsMapFolder, &templates_);

  static const std::string kErrorMsg = "Failed to load";

  ResourceLoader loader;
  addTemplatesToResourceLoader(&loader, &templates_);

  for (ResourceTemplateBase::Ptr& template_base : templates_) {
    // Generate different id such that it won't match anymore.
    common::generateId(&(template_base->id));

    // NOTE: [ADD_RESOURCE_DATA_TYPE] Add a case.
    switch (template_base->data_type) {
      case DataTypes::kCvMat: {
        cv::Mat resource;
        EXPECT_DEATH(
            getAndCheckTemplateFromResourceLoader(
                template_base.get(), &resource, &loader),
            kErrorMsg);
        break;
      }
      case DataTypes::kText: {
        std::string resource;
        EXPECT_DEATH(
            getAndCheckTemplateFromResourceLoader(
                template_base.get(), &resource, &loader),
            kErrorMsg);
        break;
      }
      case DataTypes::kPointCloud: {
        resources::PointCloud resource;
        EXPECT_DEATH(
            getAndCheckTemplateFromResourceLoader(
                template_base.get(), &resource, &loader),
            kErrorMsg);
        break;
      }
      case DataTypes::kVoxbloxTsdfMap: {
        voxblox::TsdfMap::Config config;
        voxblox::TsdfMap resource(config);
        EXPECT_DEATH(
            getAndCheckTemplateFromResourceLoader(
                template_base.get(), &resource, &loader),
            kErrorMsg);
        break;
      }
      case DataTypes::kVoxbloxEsdfMap: {
        voxblox::EsdfMap::Config config;
        voxblox::EsdfMap resource(config);
        EXPECT_DEATH(
            getAndCheckTemplateFromResourceLoader(
                template_base.get(), &resource, &loader),
            kErrorMsg);
        break;
      }
      case DataTypes::kVoxbloxOccupancyMap: {
        voxblox::OccupancyMap::Config config;
        voxblox::OccupancyMap resource(config);
        EXPECT_DEATH(
            getAndCheckTemplateFromResourceLoader(
                template_base.get(), &resource, &loader),
            kErrorMsg);
        break;
      }
      default:
        LOG(FATAL) << "Unknown DataType: "
                   << static_cast<int>(template_base->data_type);
    }
  }
}

TEST_F(ResourceLoaderTest, TestGetResourceWrongFolder) {
  constexpr bool kIsMapFolder = true;
  createResourceTemplates(
      "TestGetResourceWrongFolder", kTestMapFolderA, kIsMapFolder, &templates_);

  static const std::string kErrorMsg = "Failed to load";

  ResourceLoader loader;
  addTemplatesToResourceLoader(&loader, &templates_);

  for (ResourceTemplateBase::Ptr& template_base : templates_) {
    // Assign the wrong folder.
    template_base->folder = "/foobar_barfoo";

    // NOTE: [ADD_RESOURCE_DATA_TYPE] Add a case.
    switch (template_base->data_type) {
      case DataTypes::kCvMat: {
        cv::Mat resource;
        EXPECT_DEATH(
            getAndCheckTemplateFromResourceLoader(
                template_base.get(), &resource, &loader),
            kErrorMsg);
        break;
      }
      case DataTypes::kText: {
        std::string resource;
        EXPECT_DEATH(
            getAndCheckTemplateFromResourceLoader(
                template_base.get(), &resource, &loader),
            kErrorMsg);
        break;
      }
      case DataTypes::kPointCloud: {
        resources::PointCloud resource;
        EXPECT_DEATH(
            getAndCheckTemplateFromResourceLoader(
                template_base.get(), &resource, &loader),
            kErrorMsg);
        break;
      }

      case DataTypes::kVoxbloxTsdfMap: {
        voxblox::TsdfMap::Config config;
        voxblox::TsdfMap resource(config);
        EXPECT_DEATH(
            getAndCheckTemplateFromResourceLoader(
                template_base.get(), &resource, &loader),
            kErrorMsg);
        break;
      }
      case DataTypes::kVoxbloxEsdfMap: {
        voxblox::EsdfMap::Config config;
        voxblox::EsdfMap resource(config);
        EXPECT_DEATH(
            getAndCheckTemplateFromResourceLoader(
                template_base.get(), &resource, &loader),
            kErrorMsg);
        break;
      }
      case DataTypes::kVoxbloxOccupancyMap: {
        voxblox::OccupancyMap::Config config;
        voxblox::OccupancyMap resource(config);
        EXPECT_DEATH(
            getAndCheckTemplateFromResourceLoader(
                template_base.get(), &resource, &loader),
            kErrorMsg);
        break;
      }
      default:
        LOG(FATAL) << "Unknown DataType: "
                   << static_cast<int>(template_base->data_type);
    }
  }
}

TEST_F(ResourceLoaderTest, TestDeleteResource) {
  constexpr bool kIsMapFolder = true;
  createResourceTemplates(
      "TestDeleteResource", kTestMapFolderA, kIsMapFolder, &templates_);

  ResourceLoader loader;
  addTemplatesToResourceLoader(&loader, &templates_);

  for (ResourceTemplateBase::Ptr& template_base : templates_) {
    // NOTE: [ADD_RESOURCE_DATA_TYPE] Add a case.
    switch (template_base->data_type) {
      case DataTypes::kCvMat: {
        ResourceTemplate<cv::Mat>& resource_template =
            template_base->getAs<ResourceTemplate<cv::Mat>>();
        loader.deleteResource<cv::Mat>(
            resource_template.id, resource_template.type,
            resource_template.folder);
        break;
      }
      case DataTypes::kText: {
        ResourceTemplate<std::string>& resource_template =
            template_base->getAs<ResourceTemplate<std::string>>();
        loader.deleteResource<std::string>(
            resource_template.id, resource_template.type,
            resource_template.folder);
        break;
      }
      case DataTypes::kPointCloud: {
        ResourceTemplate<resources::PointCloud>& resource_template =
            template_base->getAs<ResourceTemplate<resources::PointCloud>>();
        loader.deleteResource<resources::PointCloud>(
            resource_template.id, resource_template.type,
            resource_template.folder);
        break;
      }
      case DataTypes::kVoxbloxTsdfMap: {
        ResourceTemplate<voxblox::TsdfMap>& resource_template =
            template_base->getAs<ResourceTemplate<voxblox::TsdfMap>>();
        loader.deleteResource<voxblox::TsdfMap>(
            resource_template.id, resource_template.type,
            resource_template.folder);
        break;
      }
      case DataTypes::kVoxbloxEsdfMap: {
        ResourceTemplate<voxblox::EsdfMap>& resource_template =
            template_base->getAs<ResourceTemplate<voxblox::EsdfMap>>();
        loader.deleteResource<voxblox::EsdfMap>(
            resource_template.id, resource_template.type,
            resource_template.folder);
        break;
      }
      case DataTypes::kVoxbloxOccupancyMap: {
        ResourceTemplate<voxblox::OccupancyMap>& resource_template =
            template_base->getAs<ResourceTemplate<voxblox::OccupancyMap>>();
        loader.deleteResource<voxblox::OccupancyMap>(
            resource_template.id, resource_template.type,
            resource_template.folder);
        break;
      }
      default:
        LOG(FATAL) << "Unknown DataType: "
                   << static_cast<int>(template_base->data_type);
    }
  }
}

TEST_F(ResourceLoaderTest, TestDeleteInexistentResource) {
  constexpr bool kIsMapFolder = true;
  createResourceTemplates(
      "TestDeleteInexistentResource", kTestMapFolderA, kIsMapFolder,
      &templates_);

  static const std::string kErrorMsg = "remove";

  ResourceLoader loader;

  for (ResourceTemplateBase::Ptr& template_base : templates_) {
    // NOTE: [ADD_RESOURCE_DATA_TYPE] Add a case.
    switch (template_base->data_type) {
      case DataTypes::kCvMat: {
        ResourceTemplate<cv::Mat>& resource_template =
            template_base->getAs<ResourceTemplate<cv::Mat>>();
        EXPECT_DEATH(
            loader.deleteResource<cv::Mat>(
                resource_template.id, resource_template.type,
                resource_template.folder),
            kErrorMsg);
        break;
      }
      case DataTypes::kText: {
        ResourceTemplate<std::string>& resource_template =
            template_base->getAs<ResourceTemplate<std::string>>();
        EXPECT_DEATH(
            loader.deleteResource<std::string>(
                resource_template.id, resource_template.type,
                resource_template.folder),
            kErrorMsg);
        break;
      }
      case DataTypes::kPointCloud: {
        ResourceTemplate<resources::PointCloud>& resource_template =
            template_base->getAs<ResourceTemplate<resources::PointCloud>>();
        EXPECT_DEATH(
            loader.deleteResource<resources::PointCloud>(
                resource_template.id, resource_template.type,
                resource_template.folder),
            kErrorMsg);
        break;
      }
      case DataTypes::kVoxbloxTsdfMap: {
        ResourceTemplate<voxblox::TsdfMap>& resource_template =
            template_base->getAs<ResourceTemplate<voxblox::TsdfMap>>();
        EXPECT_DEATH(
            loader.deleteResource<voxblox::TsdfMap>(
                resource_template.id, resource_template.type,
                resource_template.folder),
            kErrorMsg);
        break;
      }
      case DataTypes::kVoxbloxEsdfMap: {
        ResourceTemplate<voxblox::EsdfMap>& resource_template =
            template_base->getAs<ResourceTemplate<voxblox::EsdfMap>>();
        EXPECT_DEATH(
            loader.deleteResource<voxblox::EsdfMap>(
                resource_template.id, resource_template.type,
                resource_template.folder),
            kErrorMsg);
        break;
      }
      case DataTypes::kVoxbloxOccupancyMap: {
        ResourceTemplate<voxblox::OccupancyMap>& resource_template =
            template_base->getAs<ResourceTemplate<voxblox::OccupancyMap>>();
        EXPECT_DEATH(
            loader.deleteResource<voxblox::OccupancyMap>(
                resource_template.id, resource_template.type,
                resource_template.folder),
            kErrorMsg);
        break;
      }
      default:
        LOG(FATAL) << "Unknown DataType: "
                   << static_cast<int>(template_base->data_type);
    }
  }
}

TEST_F(ResourceLoaderTest, TestMigrateResourceMove) {
  constexpr bool kIsMapFolder = true;
  createResourceTemplates(
      "TestMigrateResourceMove", kTestMapFolderA, kIsMapFolder, &templates_);

  static const std::string kErrorMsg = "Failed to load";

  ResourceLoader loader;
  addTemplatesToResourceLoader(&loader, &templates_);

  constexpr bool kMoveResource = true;

  const std::string new_folder =
      kTestDataBaseFolder + "/TestMigrateResourceMove/new_map/resources/";

  // Move the resources.
  for (ResourceTemplateBase::Ptr& template_base : templates_) {
    loader.migrateResource(
        template_base->id, template_base->type, template_base->folder,
        new_folder, kMoveResource);
  }

  // Try to access previous resource folder files.
  for (ResourceTemplateBase::Ptr& template_base : templates_) {
    // NOTE: [ADD_RESOURCE_DATA_TYPE] Add a case.
    switch (template_base->data_type) {
      case DataTypes::kCvMat: {
        cv::Mat resource;
        EXPECT_DEATH(
            getAndCheckTemplateFromResourceLoader(
                template_base.get(), &resource, &loader),
            kErrorMsg);
        break;
      }
      case DataTypes::kText: {
        std::string resource;
        EXPECT_DEATH(
            getAndCheckTemplateFromResourceLoader(
                template_base.get(), &resource, &loader),
            kErrorMsg);
        break;
      }
      case DataTypes::kPointCloud: {
        resources::PointCloud resource;
        EXPECT_DEATH(
            getAndCheckTemplateFromResourceLoader(
                template_base.get(), &resource, &loader),
            kErrorMsg);
        break;
      }
      case DataTypes::kVoxbloxTsdfMap: {
        voxblox::TsdfMap::Config config;
        voxblox::TsdfMap resource(config);
        EXPECT_DEATH(
            getAndCheckTemplateFromResourceLoader(
                template_base.get(), &resource, &loader),
            kErrorMsg);
        break;
      }
      case DataTypes::kVoxbloxEsdfMap: {
        voxblox::EsdfMap::Config config;
        voxblox::EsdfMap resource(config);
        EXPECT_DEATH(
            getAndCheckTemplateFromResourceLoader(
                template_base.get(), &resource, &loader),
            kErrorMsg);
        break;
      }
      case DataTypes::kVoxbloxOccupancyMap: {
        voxblox::OccupancyMap::Config config;
        voxblox::OccupancyMap resource(config);
        EXPECT_DEATH(
            getAndCheckTemplateFromResourceLoader(
                template_base.get(), &resource, &loader),
            kErrorMsg);
        break;
      }
      default:
        LOG(FATAL) << "Unknown data_type: "
                   << static_cast<int>(template_base->data_type);
    }
  }

  // Try to access resource files in new folder.
  for (ResourceTemplateBase::Ptr& template_base : templates_) {
    template_base->folder = new_folder;
  }
  getAndCheckTemplatesFromResourceLoader(&loader, &templates_);
}

TEST_F(ResourceLoaderTest, TestMigrateResourceCopy) {
  constexpr bool kIsMapFolder = true;
  createResourceTemplates(
      "TestMigrateResourceCopy", kTestMapFolderA, kIsMapFolder, &templates_);

  ResourceLoader loader;
  addTemplatesToResourceLoader(&loader, &templates_);

  constexpr bool kMoveResource = false;

  const std::string new_folder =
      kTestDataBaseFolder + "/TestMigrateResourceCopy/new_map/resources/";

  // Move the resources.
  for (ResourceTemplateBase::Ptr& template_base : templates_) {
    loader.migrateResource(
        template_base->id, template_base->type, template_base->folder,
        new_folder, kMoveResource);
  }

  // Try to access resource files in the old folder.
  getAndCheckTemplatesFromResourceLoader(&loader, &templates_);

  // Try to access resource files in the new folder.
  for (ResourceTemplateBase::Ptr& template_base : templates_) {
    template_base->folder = new_folder;
  }
  getAndCheckTemplatesFromResourceLoader(&loader, &templates_);
}

TEST_F(ResourceLoaderTest, TestReplaceResource) {
  constexpr bool kIsMapFolder = true;
  createResourceTemplates(
      "TestReplaceResource", kTestMapFolderA, kIsMapFolder, &templates_);

  ResourceLoader loader;
  addTemplatesToResourceLoader(&loader, &templates_);

  // Replace all resources.
  for (ResourceTemplateBase::Ptr& template_base : templates_) {
    // NOTE: [ADD_RESOURCE_DATA_TYPE] Add a case.
    switch (template_base->data_type) {
      case DataTypes::kCvMat: {
        ResourceTemplate<cv::Mat>& resource_template =
            template_base->getAs<ResourceTemplate<cv::Mat>>();
        resource_template.resource().setTo(cv::Scalar(0));
        loader.replaceResource<cv::Mat>(
            resource_template.id, resource_template.type,
            resource_template.folder, resource_template.resource());
        break;
      }
      case DataTypes::kText: {
        ResourceTemplate<std::string>& resource_template =
            template_base->getAs<ResourceTemplate<std::string>>();
        static const std::string kReplacementText = "foobar";
        resource_template.resource() = kReplacementText;
        loader.replaceResource<std::string>(
            resource_template.id, resource_template.type,
            resource_template.folder, resource_template.resource());
        break;
      }
      case DataTypes::kPointCloud: {
        ResourceTemplate<resources::PointCloud>& resource_template =
            template_base->getAs<ResourceTemplate<resources::PointCloud>>();
        loader.replaceResource<resources::PointCloud>(
            resource_template.id, resource_template.type,
            resource_template.folder, resource_template.resource());
        break;
      }
      case DataTypes::kVoxbloxTsdfMap: {
        ResourceTemplate<voxblox::TsdfMap>& resource_template =
            template_base->getAs<ResourceTemplate<voxblox::TsdfMap>>();
        loader.replaceResource<voxblox::TsdfMap>(
            resource_template.id, resource_template.type,
            resource_template.folder, resource_template.resource());
        break;
      }
      case DataTypes::kVoxbloxEsdfMap: {
        ResourceTemplate<voxblox::EsdfMap>& resource_template =
            template_base->getAs<ResourceTemplate<voxblox::EsdfMap>>();
        loader.replaceResource<voxblox::EsdfMap>(
            resource_template.id, resource_template.type,
            resource_template.folder, resource_template.resource());
        break;
      }
      case DataTypes::kVoxbloxOccupancyMap: {
        ResourceTemplate<voxblox::OccupancyMap>& resource_template =
            template_base->getAs<ResourceTemplate<voxblox::OccupancyMap>>();
        loader.replaceResource<voxblox::OccupancyMap>(
            resource_template.id, resource_template.type,
            resource_template.folder, resource_template.resource());
        break;
      }
      default:
        LOG(FATAL) << "Unknown DataType: "
                   << static_cast<int>(template_base->data_type);
    }
  }

  // Check if all resources have been replaced.
  getAndCheckTemplatesFromResourceLoader(&loader, &templates_);
}

TEST_F(ResourceLoaderTest, TestResourceCache) {
  const std::string resource_folder =
      kTestDataBaseFolder + "/TestResourceCache/" + kTestExternalFolderX;

  ResourceLoader loader;

  ResourceId resource_id_A, resource_id_B;
  common::generateId(&resource_id_A);
  common::generateId(&resource_id_B);
  static const std::string kTextA = "fobAAr";
  static const std::string kTextB = "foBaar";
  loader.addResource<std::string>(
      resource_id_A, ResourceType::kText, resource_folder, kTextA);
  loader.addResource<std::string>(
      resource_id_B, ResourceType::kText, resource_folder, kTextB);

  EXPECT_EQ(loader.getCacheStatistic().getNumHits(ResourceType::kText), 0u);
  EXPECT_EQ(loader.getCacheStatistic().getNumMiss(ResourceType::kText), 0u);

  // Get cache miss for resource A and B.
  std::string resource_A;
  loader.getResource<std::string>(
      resource_id_A, ResourceType::kText, resource_folder, &resource_A);

  EXPECT_EQ(loader.getCacheStatistic().getNumHits(ResourceType::kText), 0u);
  EXPECT_EQ(loader.getCacheStatistic().getNumMiss(ResourceType::kText), 1u);

  std::string resource_B;
  loader.getResource<std::string>(
      resource_id_B, ResourceType::kText, resource_folder, &resource_B);

  EXPECT_EQ(loader.getCacheStatistic().getNumHits(ResourceType::kText), 0u);
  EXPECT_EQ(loader.getCacheStatistic().getNumMiss(ResourceType::kText), 2u);

  // Get cache hit for resource A and B.
  loader.getResource<std::string>(
      resource_id_A, ResourceType::kText, resource_folder, &resource_A);

  EXPECT_EQ(loader.getCacheStatistic().getNumHits(ResourceType::kText), 1u);
  EXPECT_EQ(loader.getCacheStatistic().getNumMiss(ResourceType::kText), 2u);

  loader.getResource<std::string>(
      resource_id_B, ResourceType::kText, resource_folder, &resource_B);

  EXPECT_EQ(loader.getCacheStatistic().getNumHits(ResourceType::kText), 2u);
  EXPECT_EQ(loader.getCacheStatistic().getNumMiss(ResourceType::kText), 2u);

  // Add a lot of different resources to fill up cache, make sure maximum cache
  // size is reached and
  // both resource A and B are kicked out of the cache.
  const size_t max_num_cache_entries_per_type =
      loader.getCacheConfig().max_cache_size;

  for (size_t resource_counter = 0u;
       resource_counter < max_num_cache_entries_per_type; ++resource_counter) {
    ResourceId resource_id;
    common::generateId(&resource_id);

    const std::string resource_input = "resource_" + resource_id.hexString();

    loader.addResource<std::string>(
        resource_id, ResourceType::kText, resource_folder, resource_input);

    std::string resource_output;
    loader.getResource<std::string>(
        resource_id, ResourceType::kText, resource_folder, &resource_output);

    EXPECT_EQ(loader.getCacheStatistic().getNumHits(ResourceType::kText), 2u);
    EXPECT_EQ(
        loader.getCacheStatistic().getNumMiss(ResourceType::kText),
        2u + resource_counter + 1u);
    EXPECT_EQ(resource_input, resource_output);
  }

  // Now both resources A and B should produce a cache miss again.
  loader.getResource<std::string>(
      resource_id_A, ResourceType::kText, resource_folder, &resource_A);

  EXPECT_EQ(loader.getCacheStatistic().getNumHits(ResourceType::kText), 2u);
  EXPECT_EQ(
      loader.getCacheStatistic().getNumMiss(ResourceType::kText),
      2u + max_num_cache_entries_per_type + 1u);

  loader.getResource<std::string>(
      resource_id_B, ResourceType::kText, resource_folder, &resource_B);

  EXPECT_EQ(loader.getCacheStatistic().getNumHits(ResourceType::kText), 2u);
  EXPECT_EQ(
      loader.getCacheStatistic().getNumMiss(ResourceType::kText),
      2u + max_num_cache_entries_per_type + 2u);
}

}  // namespace backend

MAPLAB_UNITTEST_ENTRYPOINT
