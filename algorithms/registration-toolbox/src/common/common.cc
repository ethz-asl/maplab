#include "registration-toolbox/common/common.h"

#include <algorithm>
#include <cstdlib>
#include <cxxabi.h>
#include <memory>

#include <glog/logging.h>

namespace regbox {

std::string demangle(const char* name) {
  CHECK_NOTNULL(name);
  int status = -4;
  std::unique_ptr<char, void (*)(void*)> res{
      abi::__cxa_demangle(name, NULL, NULL, &status), std::free};
  return (status == 0) ? res.get() : name;
}

std::string toLower(std::string&& s) {
  std::transform(s.begin(), s.end(), s.begin(), ::tolower);
  return s;
}

std::string toLower(const std::string& s) {
  std::string res;
  res.resize(s.size());

  std::transform(s.begin(), s.end(), res.begin(), ::tolower);
  return res;
}

}  // namespace regbox
