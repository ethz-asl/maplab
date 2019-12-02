#include "vi-map-data-import-export/import-loop-closure-edges.h"

#include <string>
#include <vector>

namespace data_import_export {

CameraPose CameraPose::fromYaml(const YAML::Node& yaml) {
  CameraPose cam;
  cam.timestamp_ns = yaml["timestamp_ns"].as<uint64_t>();
  cam.pose = transformationFromYaml(yaml["pose"]);
  return cam;
}

LoopClosureEdge LoopClosureEdge::fromYaml(const YAML::Node& yaml) {
  LoopClosureEdge edge;
  edge.T_from_to = transformationFromYaml(yaml["T_from_to"]);
  edge.from = CameraPose::fromYaml(yaml["camera_from"]);
  edge.to = CameraPose::fromYaml(yaml["camera_to"]);
  edge.switch_variable = yaml["switch_variable"].as<float>();
  edge.switch_variable_variance = yaml["switch_variable_variance"].as<double>();

  const std::vector<double> cov = yaml["covariance"].as<std::vector<double>>();
  CHECK_EQ(kCovarianceOrder * kCovarianceOrder, cov.size());
  size_t j = 0u;
  for (size_t i = 0u; i < cov.size(); ++i) {
    edge.covariance(j, i % LoopClosureEdge::kCovarianceOrder) = cov[i];
    if (i % LoopClosureEdge::kCovarianceOrder == 0 && i != 0) {
      ++j;
    }
  }
  return edge;
}

pose::Transformation transformationFromYaml(const YAML::Node& yaml) {
  const std::vector<float> translation =
      yaml["translation_m"].as<std::vector<float>>();
  const std::vector<float> quaternions =
      yaml["rotation_xyzw"].as<std::vector<float>>();

  kindr::minimal::RotationQuaternion quat(
      quaternions[3], quaternions[0], quaternions[1], quaternions[2]);
  kindr::minimal::Position pos;
  pos << translation[0], translation[1], translation[2];

  const pose::Transformation transformation(quat, pos);
  return transformation;
}

LoopClosureEdges loopClosureEdgesFromYamlFile(const std::string& filename) {
  LoopClosureEdges edges;
  YAML::Node yaml = YAML::LoadFile(filename);
  for (auto edge : yaml) {
    edges.push_back(LoopClosureEdge::fromYaml(edge));
  }
  return edges;
}
}  // namespace data_import_export
