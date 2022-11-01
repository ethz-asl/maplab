#ifndef MAP_RESOURCES_RESOURCE_TYPEDEFS_H_
#define MAP_RESOURCES_RESOURCE_TYPEDEFS_H_

#include <string>
#include <vector>

#include <Eigen/Core>
#include <maplab-common/pose_types.h>
#include <opencv2/core.hpp>
#include <voxblox/core/common.h>

namespace resources {

struct VoxbloxColorPointCloud {
  voxblox::Pointcloud* points_C;
  voxblox::Colors* colors;
};

struct ObjectInstanceBoundingBox {
  ObjectInstanceBoundingBox()
      : bounding_box(0, 0, 0, 0),
        class_number(0),
        instance_number(0),
        confidence(0.f),
        class_name("") {}
  cv::Rect bounding_box;
  // Number that describes the object class this object belongs to.
  int class_number;
  // Number that describes which instance of a specific class the
  // objects belong to.
  int instance_number;
  // Stores the confidence the object detector/classifier has
  // assigned to this classification.
  float confidence;
  // Stores the name of the object class, if available.
  std::string class_name;

  bool operator==(const ObjectInstanceBoundingBox& other_bbox) const {
    return bounding_box == other_bbox.bounding_box &&
           class_number == other_bbox.class_number &&
           instance_number == other_bbox.instance_number &&
           (std::fabs(confidence - other_bbox.confidence) < 1e-6f) &&
           (std::fabs(confidence - other_bbox.confidence) < 1e-6f) &&
           class_name == other_bbox.class_name;
  }

  bool operator!=(const ObjectInstanceBoundingBox& other_bbox) const {
    return !operator==(other_bbox);
  }
};

typedef std::vector<ObjectInstanceBoundingBox> ObjectInstanceBoundingBoxes;

}  // namespace resources

#endif  // MAP_RESOURCES_RESOURCE_TYPEDEFS_H_
