#ifndef MAPLAB_COMMON_FILE_SYSTEM_TOOLS_INL_H_
#define MAPLAB_COMMON_FILE_SYSTEM_TOOLS_INL_H_

#include <string>

namespace common {

template <typename... FurtherNames>
std::string concatenateFolderAndFileName(
    const std::string& name_1, const std::string& name_2,
    const FurtherNames&... further_names) {
  return concatenateFolderAndFileName(
      concatenateFolderAndFileName(name_1, name_2), further_names...);
}

}  // namespace common

#endif  // MAPLAB_COMMON_FILE_SYSTEM_TOOLS_INL_H_
