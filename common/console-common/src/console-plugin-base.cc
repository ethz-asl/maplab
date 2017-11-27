#include "console-common/console-plugin-base.h"

#include <string>

#include <glog/logging.h>

#include "console-common/console.h"

namespace common {

bool ConsolePluginBase::getSelectedMapKeyIfSet(
    std::string* selected_map_key) const {
  CHECK_NOTNULL(selected_map_key)->clear();
  *selected_map_key = console_->getSelectedMapKey();
  if (selected_map_key->empty()) {
    LOG(ERROR) << "No map is selected, please use \"select_map\" command to "
                  "choose one first.";
    return false;
  }
  return true;
}

}  // namespace common
