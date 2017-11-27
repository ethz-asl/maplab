#ifndef MAPLAB_COMMON_YAML_SERIALIZATION_H_
#define MAPLAB_COMMON_YAML_SERIALIZATION_H_

#include <fstream>  // NOLINT
#include <list>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

#include <maplab-common/eigen-yaml-serialization.h>

namespace YAML {
template <class ValueType>
struct convert<std::queue<ValueType> > {
  static Node encode(const std::queue<ValueType>& queue) {
    Node node;
    std::vector<ValueType> tmp_v;
    std::queue<ValueType> q_cpy = queue;
    while (!q_cpy.empty()) {
      tmp_v.push_back(q_cpy.front());
      q_cpy.pop();
    }
    node = tmp_v;
    return node;
  }

  static bool decode(const Node& node, std::queue<ValueType>& queue) {
    CHECK(node.IsSequence());
    for (size_t i = 0; i < node.size(); ++i) {
      ValueType tmp = node[i].as<ValueType>();
      queue.push(tmp);
    }
    return true;
  }
};

template <class KeyType>
struct convert<std::unordered_set<KeyType> > {
  static Node encode(const std::unordered_set<KeyType>& set) {
    Node node;
    for (const KeyType& value : set) {
      node.push_back(value);
    }
    return node;
  }

  static bool decode(const Node& node, std::unordered_set<KeyType>& set) {
    set.clear();
    CHECK(node.IsSequence());
    for (YAML::const_iterator it = node.begin(); it != node.end(); ++it) {
      set.insert(it->as<KeyType>());
    }
    return true;
  }
};

template <class KeyType, class ValueType>
struct convert<std::unordered_map<KeyType, ValueType> > {
  static Node encode(const std::unordered_map<KeyType, ValueType>& map) {
    Node node;
    for (const std::pair<KeyType, ValueType>& value : map) {
      node[value.first] = value.second;
    }
    return node;
  }

  static bool decode(
      const Node& node, std::unordered_map<KeyType, ValueType>& map) {
    map.clear();
    for (YAML::const_iterator it = node.begin(); it != node.end(); ++it) {
      map[it->first.as<KeyType>()] = it->second.as<ValueType>();
    }
    return true;
  }
};

template <typename ObjectType>
void Save(const ObjectType& object, std::ostream* ofs) {
  CHECK_NOTNULL(ofs);
  assert(ofs->good());
  YAML::Node out;
  out = object;
  *ofs << out;
}

template <typename T>
void Save(const T& object, const std::string& filename) {
  std::ofstream ofs(filename.c_str());
  Save(object, &ofs);
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
  } catch (const std::exception& e) {  // NOLINT
    LOG(ERROR) << "Encountered exception while reading yaml " << e.what();
    return false;
  }
}
}  // namespace YAML
#endif  // MAPLAB_COMMON_YAML_SERIALIZATION_H_
