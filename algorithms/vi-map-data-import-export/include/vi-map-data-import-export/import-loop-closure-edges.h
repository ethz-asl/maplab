#ifndef VI_MAP_DATA_IMPORT_EXPORT_IMPORT_LOOP_CLOSURE_EDGES_H_
#define VI_MAP_DATA_IMPORT_EXPORT_IMPORT_LOOP_CLOSURE_EDGES_H_

#include <string>
#include <vector>

#include <maplab-common/pose_types.h>
#include <yaml-cpp/yaml.h>

namespace data_import_export {
struct LoopClosureEdge;

struct CameraPose {
  uint64_t timestamp_ns;
  pose::Transformation pose;
  static CameraPose fromYaml(const YAML::Node& yaml);
};

struct LoopClosureEdge {
  CameraPose from;
  CameraPose to;
  pose::Transformation T_from_to;
  double switch_variable;
  double switch_variable_variance;
  static constexpr int kCovarianceOrder = 6;
  Eigen::Matrix<double, kCovarianceOrder, kCovarianceOrder> covariance;

  static LoopClosureEdge fromYaml(const YAML::Node& yaml);
};

typedef std::vector<LoopClosureEdge> LoopClosureEdges;

LoopClosureEdges loopClosureEdgesFromYamlFile(const std::string& filename);

pose::Transformation transformationFromYaml(const YAML::Node& yaml);
}  // namespace data_import_export
#endif  // VI_MAP_DATA_IMPORT_EXPORT_IMPORT_LOOP_CLOSURE_EDGES_H_
