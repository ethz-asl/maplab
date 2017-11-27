#ifndef MAPLAB_COMMON_YAML_FILE_SERIALIZABLE_H_
#define MAPLAB_COMMON_YAML_FILE_SERIALIZABLE_H_

#include <fstream>  // NOLINT
#include <limits>
#include <string>

#include <glog/logging.h>
#include <maplab-common/file-system-tools.h>
#include <yaml-cpp/yaml.h>

namespace common {

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
    CHECK(output_file_stream.is_open()) << "Failed to open file " << file_name
                                        << " for writing.";
    output_file_stream << yaml_node;
    output_file_stream.close();
  }

  bool deserializeFromFile(const std::string& file_name) {
    if (!common::fileExists(file_name)) {
      LOG(ERROR) << "YAML file " << file_name << " does not exist.";
      return false;
    }

    YAML::Node sensors_with_systems_node;
    try {
      sensors_with_systems_node = YAML::LoadFile(file_name.c_str());
    } catch (const std::exception& ex) {  // NOLINT
      LOG(ERROR) << "Failed to load YAML node from file " << file_name
                 << " with the error: " << ex.what();
      return false;
    }

    deserialize(sensors_with_systems_node);
    return true;
  }
};

}  // namespace common

#endif  // MAPLAB_COMMON_YAML_FILE_SERIALIZABLE_H_
