#ifndef REGISTRATION_TOOLBOX_COMMON_COMMON_H_
#define REGISTRATION_TOOLBOX_COMMON_COMMON_H_

#include <cctype>
#include <string>
#include <typeinfo>

namespace regbox {

std::string demangle(const char* name);

template <typename T>
std::string nameOf(const T& t) {
  return demangle(typeid(t).name());
}
template <typename T>
std::string nameOf() {
  return demangle(typeid(T).name());
}

std::string toLower(std::string&& s);
std::string toLower(const std::string& s);

}  // namespace regbox

#endif  // REGISTRATION_TOOLBOX_COMMON_COMMON_H_
