#ifndef ASLAM_CV_COMMON_YAML_SERIALIZATION_H_
#define ASLAM_CV_COMMON_YAML_SERIALIZATION_H_

#include <fstream>  // NOLINT
#include <list>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <aslam/common/memory.h>
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

#include "yaml-serialization-eigen.h"

namespace YAML {

/// \brief A function to get a value from a YAML node with non-exception error handling.
/// \param[in] node The YAML node.
/// \param[in] key The key used to dereference the node (node[key]).
/// \param[out] value The return value.
/// \returns True if the value was filled in successfully. False otherwise.
template<typename ValueType>
bool safeGet(const YAML::Node& node, const std::string& key, ValueType* value) {
  CHECK_NOTNULL(value);
  bool success = false;
  if(!node.IsMap()) {
    LOG(ERROR) << "Unable to get Node[\"" << key << "\"] because the node is not a map";
  } else {
    const YAML::Node sub_node = node[key];
    if(sub_node) {
      try {
        *value = sub_node.as<ValueType>();
        success = true;
      } catch(const YAML::Exception& e) {
        LOG(ERROR) << "Error getting key \"" << key << "\" as type "
            << typeid(ValueType).name() << ": " << e.what();
      }
    } else {
      LOG(ERROR) << "Key \"" << key << "\" does not exist";
    }
  }
  return success;
}

inline bool hasKey(const YAML::Node& node, const std::string& key) {
  if(!node.IsMap()) {
    LOG(ERROR) << "Unable to get Node[\"" << key << "\"] because the node is not a map";
    return false;
  }
  const YAML::Node sub_node = node[key];
  return static_cast<bool>(sub_node);
}

template <typename ObjectType>
bool Save(const ObjectType& object, std::ostream* ofs) {
  CHECK_NOTNULL(ofs);
  assert(ofs->good());

  try {
    YAML::Node out;
    out = object;
    *ofs << out;
  } catch (const std::exception& e) {  // NOLINT
    LOG(ERROR)<< "Encountered exception while saving yaml " << e.what();
    return false;
  }
  return true;
}

template <typename T>
bool Save(const T& object, const std::string& filename) {
  std::ofstream ofs(filename.c_str());
  return Save(object, &ofs);
}

template <typename ObjectType>
bool Load(const std::string& filename, ObjectType* object) {
  CHECK_NOTNULL(object);
  std::ifstream ifs(filename.c_str());
  if (!ifs.good()) {
    return false;
  }

  try {
    YAML::Node doc = YAML::LoadFile(filename.c_str());
    (*object) = doc.as<ObjectType>();
    return true;
  }
  catch (const std::exception& e) {  // NOLINT
    LOG(ERROR) << "Encountered exception while reading yaml " << e.what();
    return false;
  }
}

}  // namespace YAML
#endif  // ASLAM_CV_COMMON_YAML_SERIALIZATION_H_
