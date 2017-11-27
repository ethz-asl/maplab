#ifndef MAPLAB_COMMON_STRING_TOOLS_H_
#define MAPLAB_COMMON_STRING_TOOLS_H_
#include <random>
#include <string>
#include <vector>

#include <glog/logging.h>

namespace common {

inline void tokenizeString(
    const std::string& input_string, char delimiter, bool remove_empty,
    std::vector<std::string>* tokens) {
  CHECK_NOTNULL(tokens)->clear();
  std::stringstream token_stream(input_string);
  std::string token;
  while (std::getline(token_stream, token, delimiter)) {
    if (token.empty() && remove_empty) {
      continue;
    }
    tokens->push_back(token);
  }
}

constexpr char kCharacterIndex[] =
    "abcdefghijklmnaoqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ1234567890";
inline std::string createRandomString(const size_t length) {
  const std::string kCharacterIndexString =
      static_cast<std::string>(kCharacterIndex);
  const size_t string_length = length + 1u;
  std::string random_string;
  random_string.resize(string_length);

  std::random_device random_device;
  std::default_random_engine random_engine(random_device());
  std::uniform_int_distribution<int> uniform_dist(
      0, kCharacterIndexString.size());
  for (size_t index = 0u; index < length; ++index) {
    random_string[index] = kCharacterIndex[uniform_dist(random_engine)];
  }
  return random_string;
}

}  // namespace common

#endif  // MAPLAB_COMMON_STRING_TOOLS_H_
