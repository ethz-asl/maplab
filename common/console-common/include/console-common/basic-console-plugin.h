#ifndef CONSOLE_COMMON_BASIC_CONSOLE_PLUGIN_H_
#define CONSOLE_COMMON_BASIC_CONSOLE_PLUGIN_H_

#include <string>

#include "console-common/console-plugin-base.h"

namespace common {

class CommandRegisterer;
class Console;

/// Contains basic console commands like help.
class BasicConsolePlugin : public ConsolePluginBase {
 public:
  BasicConsolePlugin(Console* console, CommandRegisterer* command_registerer);

  virtual std::string getPluginId() const override {
    return "console";
  }

 private:
  void helpText();
  void showHelpForPlugin(
      const std::string& plugin_name, const std::string& filter) const;
  void colorOccurencesOfFilter(
      const std::string& filter, std::string* input_word) const;

  CommandRegisterer* command_registerer_;
};

}  // namespace common

#endif  // CONSOLE_COMMON_BASIC_CONSOLE_PLUGIN_H_
