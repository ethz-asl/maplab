#include <map>
#include <sstream>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include <maplab-common/eigen-yaml-serialization.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/yaml-serialization.h>

namespace common {
static unsigned int seed = 0;

// Overloads to get appropriate random values.
void SetRandom(int* rhs) {
  *rhs = rand_r(&seed);
}
void SetRandom(float* rhs) {
  *rhs = static_cast<float>(rand_r(&seed)) / RAND_MAX;
}
void SetRandom(double* rhs) {
  *rhs = static_cast<double>(rand_r(&seed)) / RAND_MAX;
}
template <class K, class T>
void SetRandom(std::pair<K, T>* rhs) {
  SetRandom(&(rhs->first));
  SetRandom(&(rhs->second));
}

// Add value overload specialization for queue.
template <typename T>
void AddValue(std::queue<T>& lhs, const T& rhs) {
  lhs.push(rhs);
}
// Add value overload for vector.
template <class T>
void AddValue(std::vector<T>& lhs, const T& rhs) {
  lhs.push_back(rhs);
}
// Add value overload for list.
template <class T>
void AddValue(std::list<T>& lhs, const T& rhs) {
  lhs.push_back(rhs);
}
// Add value overload for unordered_set.
template <class T>
void AddValue(std::unordered_set<T>& lhs, const T& rhs) {
  lhs.insert(rhs);
}
// Add value overload for unordered_map.
template <class K, class T>
void AddValue(std::unordered_map<K, T>& lhs, const std::pair<K, T>& rhs) {
  lhs.insert(rhs);
}

// Emit values stored in a queue.
template <typename T>
std::string EmitValues(const std::queue<T>& lhs) {
  std::queue<T> tmp = lhs;
  std::stringstream ss;
  while (!tmp.empty()) {
    ss << tmp.front() << " ";
    tmp.pop();
  }
  return ss.str();
}
// Emit values stored in a vector.
template <typename T>
std::string EmitValues(const std::vector<T>& lhs) {
  std::stringstream ss;
  for (typename std::vector<T>::const_iterator it = lhs.begin();
       it != lhs.end(); ++it) {
    ss << *it << " ";
  }
  return ss.str();
}
// Emit values stored in a list.
template <typename T>
std::string EmitValues(const std::list<T>& lhs) {
  std::stringstream ss;
  for (typename std::list<T>::const_iterator it = lhs.begin(); it != lhs.end();
       ++it) {
    ss << *it << " ";
  }
  return ss.str();
}
// Emit values stored in an unordered_set.
template <typename T>
std::string EmitValues(const std::unordered_set<T>& lhs) {
  std::vector<T> lhs_list;
  lhs_list.insert(lhs_list.begin(), lhs.begin(), lhs.end());
  std::sort(lhs_list.begin(), lhs_list.end());
  return EmitValues(lhs_list);
}
// Emit values stored in an unordered_map.
template <typename K, typename T>
std::string EmitValues(const std::map<K, T>& lhs) {
  std::stringstream ss;
  for (typename std::map<K, T>::const_iterator it = lhs.begin();
       it != lhs.end(); ++it) {
    ss << it->first << " " << it->second << "; ";
  }
  return ss.str();
}
// Emit values stored in an unordered_map.
template <typename K, typename T>
std::string EmitValues(const std::unordered_map<K, T>& lhs) {
  std::map<K, T> lhs_list;
  lhs_list.insert(lhs.begin(), lhs.end());
  return EmitValues(lhs_list);
}

// Emit values stored in a queue.
template <typename T>
bool CheckApproxSame(const std::queue<T>& lhs, const std::queue<T>& rhs) {
  std::queue<T> tmp_lhs = lhs;
  std::queue<T> tmp_rhs = rhs;
  bool isSame = true;
  if (tmp_lhs.size() != tmp_rhs.size()) {
    return false;
  }
  while (!tmp_lhs.empty() && !tmp_rhs.empty()) {
    T a = tmp_lhs.front();
    T b = tmp_rhs.front();
    tmp_lhs.pop();
    tmp_rhs.pop();
    isSame = isSame && (std::abs(a - b) < 1e-6);
    assert(std::abs(a - b) < 1e-6);
  }
  return isSame;
}

// Check values stored in a vector.
template <typename T>
bool CheckApproxSame(const std::vector<T>& lhs, const std::vector<T>& rhs) {
  bool isSame = true;
  if (lhs.size() != rhs.size()) {
    return false;
  }
  for (typename std::vector<T>::const_iterator it_lhs = lhs.begin(),
                                               it_rhs = rhs.begin();
       it_lhs != lhs.end() && it_rhs != rhs.end(); ++it_lhs, ++it_rhs) {
    T a = *it_lhs;
    T b = *it_rhs;
    isSame = isSame && (std::abs(a - b) < 1e-6);
  }
  return isSame;
}

// Check values stored in a list.
template <typename T>
bool CheckApproxSame(const std::list<T>& lhs, const std::list<T>& rhs) {
  bool isSame = true;
  if (lhs.size() != rhs.size()) {
    return false;
  }
  for (typename std::list<T>::const_iterator it_lhs = lhs.begin(),
                                             it_rhs = rhs.begin();
       it_lhs != lhs.end() && it_rhs != rhs.end(); ++it_lhs, ++it_rhs) {
    T a = *it_lhs;
    T b = *it_rhs;
    isSame = isSame && (std::abs(a - b) < 1e-6);
  }
  return isSame;
}

// Check values stored in a map.
template <typename K, typename T>
bool CheckApproxSame(const std::map<K, T>& lhs, const std::map<K, T>& rhs) {
  bool isSame = true;
  if (lhs.size() != rhs.size()) {
    return false;
  }
  for (typename std::map<K, T>::const_iterator it_lhs = lhs.begin(),
                                               it_rhs = rhs.begin();
       it_lhs != lhs.end() && it_rhs != rhs.end(); ++it_lhs, ++it_rhs) {
    std::pair<K, T> a = *it_lhs;
    std::pair<K, T> b = *it_rhs;
    isSame = isSame && (std::abs(a.first - b.first) < 1e-6);
    isSame = isSame && (std::abs(a.second - b.second) < 1e-6);
  }
  return isSame;
}

// Check values stored in an unordered_set.
template <typename T>
bool CheckApproxSame(
    const std::unordered_set<T>& lhs, const std::unordered_set<T>& rhs) {
  bool isSame = true;
  if (lhs.size() != rhs.size()) {
    return false;
  }
  std::vector<T> lhs_list, rhs_list;
  lhs_list.insert(lhs_list.begin(), lhs.begin(), lhs.end());
  rhs_list.insert(rhs_list.begin(), rhs.begin(), rhs.end());
  std::sort(lhs_list.begin(), lhs_list.end());
  std::sort(rhs_list.begin(), rhs_list.end());
  CheckApproxSame(lhs_list, rhs_list);
  return isSame;
}

// Check values stored in an unordered_map.
template <typename K, typename T>
bool CheckApproxSame(
    const std::unordered_map<K, T>& lhs, const std::unordered_map<K, T>& rhs) {
  bool isSame = true;
  if (lhs.size() != rhs.size()) {
    return false;
  }
  std::map<K, T> lhs_list, rhs_list;
  lhs_list.insert(lhs.begin(), lhs.end());
  rhs_list.insert(rhs.begin(), rhs.end());
  CheckApproxSame(lhs_list, rhs_list);
  return isSame;
}

template <typename T>
struct RemoveConstFromPairType {
  typedef T type;
};

template <typename K, typename T>
struct RemoveConstFromPairType<std::pair<K, T> > {
  typedef std::pair<typename std::remove_const<K>::type, T> type;
};

// Checks different STL container serializations.
template <typename Type>
void RunTestCase(const std::string& testname, const std::string& filename) {
  Type lhs;
  Type rhs;
  size_t num = 20;
  for (size_t i = 0; i < num; ++i) {
    // HACK, HACK, HACK.
    typename RemoveConstFromPairType<typename Type::value_type>::type tmp;
    SetRandom(&tmp);
    AddValue(lhs, tmp);
  }
  YAML::Save(lhs, filename);
  YAML::Load(filename, &rhs);
  if (!CheckApproxSame(rhs, lhs)) {
    std::cout << "Test " << testname << " failed! Matrices "
                                        "are not the same: "
              << std::endl
              << __FILE__ << ":" << __LINE__ << std::endl
              << "save:" << std::endl
              << EmitValues(lhs) << std::endl
              << "load:" << std::endl
              << EmitValues(rhs) << std::endl;
  }
}

TEST(MaplabCommon, STLYamlSerialization_VectorDouble) {
  RunTestCase<std::vector<double> >("double_vector", "double_vector.yaml");
}
TEST(MaplabCommon, STLYamlSerialization_VectorFloat) {
  RunTestCase<std::vector<float> >("float_vector", "float_vector.yaml");
}
TEST(MaplabCommon, STLYamlSerialization_VectorInt) {
  RunTestCase<std::vector<int> >("int_vector", "int_vector.yaml");
}
TEST(MaplabCommon, STLYamlSerialization_ListDouble) {
  RunTestCase<std::list<double> >("double_list", "double_list.yaml");
}
TEST(MaplabCommon, STLYamlSerialization_ListFloat) {
  RunTestCase<std::list<float> >("float_list", "float_list.yaml");
}
TEST(MaplabCommon, STLYamlSerialization_ListInt) {
  RunTestCase<std::list<int> >("int_list", "int_list.yaml");
}
TEST(MaplabCommon, STLYamlSerialization_QueueDouble) {
  RunTestCase<std::queue<double> >("double_queue", "double_queue.yaml");
}
TEST(MaplabCommon, STLYamlSerialization_QueueFloat) {
  RunTestCase<std::queue<float> >("float_queue", "float_queue.yaml");
}
TEST(MaplabCommon, STLYamlSerialization_QueueInt) {
  RunTestCase<std::queue<int> >("int_queue", "int_queue.yaml");
}
TEST(MaplabCommon, STLYamlSerialization_UnorderedSetDouble) {
  RunTestCase<std::unordered_set<double> >(
      "double_unordered_set", "double_unordered_set.yaml");
}
TEST(MaplabCommon, STLYamlSerialization_UnorderedSetFloat) {
  RunTestCase<std::unordered_set<float> >(
      "float_unordered_set", "float_unordered_set.yaml");
}
TEST(MaplabCommon, STLYamlSerialization_UnorderedSetInt) {
  RunTestCase<std::unordered_set<int> >(
      "int_unordered_set", "int_unordered_set.yaml");
}
TEST(MaplabCommon, STLYamlSerialization_UnorderedMapDouble) {
  RunTestCase<std::unordered_map<int, double> >(
      "double_unordered_map", "double_unordered_set.yaml");
}
TEST(MaplabCommon, STLYamlSerialization_UnorderedMapFloat) {
  RunTestCase<std::unordered_map<double, float> >(
      "float_unordered_set", "float_unordered_set.yaml");
}
TEST(MaplabCommon, STLYamlSerialization_UnorderedMapInt) {
  RunTestCase<std::unordered_map<float, int> >(
      "int_unordered_set", "int_unordered_set.yaml");
}
}  // namespace common

MAPLAB_UNITTEST_ENTRYPOINT
