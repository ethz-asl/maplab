#ifndef ASLAM_CV_COMMON_YAML_FILE_SERIALIZATION_H_
#define ASLAM_CV_COMMON_YAML_FILE_SERIALIZATION_H_

#include <fstream>  // NOLINT
#include <limits>
#include <string>

#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

namespace aslam {
class YamlFileSerializable {
 public:
  virtual ~YamlFileSerializable() = default;

  virtual void serialize(YAML::Node* yaml_node) const = 0;
  virtual bool deserialize(const YAML::Node& yaml_node) = 0;

  // Existing files get overwritten.
  void serializeToFile(const std::string& file_name) const {
    YAML::Node yaml_node;
    serialize(&yaml_node);
    CHECK(yaml_node.IsDefined());

    std::ofstream output_file_stream(file_name);
    CHECK(output_file_stream.is_open())
        << "Failed to open file " << file_name << " for writing.";
    output_file_stream << yaml_node;
    output_file_stream.close();
  }

  bool deserializeFromFile(const std::string& file_name) {
    YAML::Node yaml_node;
    try {
      yaml_node = YAML::LoadFile(file_name.c_str());
    } catch (const std::exception& ex) {  // NOLINT
      LOG(ERROR) << "Failed to load YAML node from file " << file_name
                 << " with the error: " << ex.what();
      return false;
    }

    return deserialize(yaml_node);
  }
};
}  // namespace aslam

#endif  // ASLAM_CV_COMMON_YAML_FILE_SERIALIZATION_H_
