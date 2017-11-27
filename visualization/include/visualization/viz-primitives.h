#ifndef VISUALIZATION_VIZ_PRIMITIVES_H_
#define VISUALIZATION_VIZ_PRIMITIVES_H_

#include <vector>

#include <Eigen/Core>

#include <aslam/common/memory.h>

#include "visualization/color.h"


namespace visualization {

struct FilledBox {
  FilledBox()
      : from(Eigen::Vector3d::Zero()),
        to(Eigen::Vector3d::Zero()),
        wireframe_alpha(0.8),
        fill_alpha(0.3),
        wireframe_width(0.1) {}
  Eigen::Vector3d from;
  Eigen::Vector3d to;
  visualization::Color wireframe_color;
  visualization::Color fill_color;
  double wireframe_alpha;
  double fill_alpha;
  double wireframe_width;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct LineSegment {
  LineSegment()
      : from(Eigen::Vector3d::Zero()),
        to(Eigen::Vector3d::Zero()),
        scale(0),
        alpha(0.8) {}
  Eigen::Vector3d from;
  Eigen::Vector3d to;
  double scale;
  visualization::Color color;
  double alpha;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct Sphere {
  Sphere() : position(Eigen::Vector3d::Zero()), radius(1.0), alpha(0.8) {}
  Eigen::Vector3d position;
  double radius;
  visualization::Color color;
  double alpha;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct Pose {
  Pose()
      : G_p_B(Eigen::Vector3d::Zero()),
        G_q_B(Eigen::Quaterniond::Identity()),
        id(0u),
        action(0u),
        scale(1),
        line_width(1),
        alpha(1) {}

  Eigen::Vector3d G_p_B;
  Eigen::Quaterniond G_q_B;
  uint32_t id;
  uint32_t action;
  double scale;
  double line_width;
  double alpha;
  visualization::Color color;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef Aligned<std::vector, LineSegment> LineSegmentVector;
typedef Aligned<std::vector, Sphere> SphereVector;
typedef Aligned<std::vector, Pose> PoseVector;
typedef Aligned<std::vector, FilledBox> FilledBoxVector;

}  // namespace visualization

#endif  // VISUALIZATION_VIZ_PRIMITIVES_H_
