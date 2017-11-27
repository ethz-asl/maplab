#ifndef MAPLAB_COMMON_UI_UTILITY_H_
#define MAPLAB_COMMON_UI_UTILITY_H_

#include <iostream>  // NOLINT
#include <string>

#include <glog/logging.h>

namespace common {

// Ask the user for yes/no.
inline bool askUserForYesNoOverConsole(const std::string& question) {
  LOG(INFO) << question;
  LOG(INFO) << "Please type [y]es or [n]o.";
  while (true) {
    std::cout << "> " << std::flush;
    std::string user_input;
    std::getline(std::cin, user_input);
    if (user_input.size() == 1u) {
      if (user_input[0] == 'y') {
        return true;
      } else if (user_input[0] == 'n') {
        return false;
      }
    } else if (user_input.size() > 1u) {
      if (user_input == "yes") {
        return true;
      } else if (user_input == "no") {
        return false;
      }
    }
    LOG(ERROR) << "The entered input is invalid (" << user_input
               << "). Please enter either 'y' for yes or 'n' for no.";
  }
}
}  // namespace common

#endif  // MAPLAB_COMMON_UI_UTILITY_H_
