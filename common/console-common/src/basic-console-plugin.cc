#include "console-common/basic-console-plugin.h"

#include <stdio.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <iostream>  // NOLINT
#include <sstream>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <maplab-common/sigint-breaker.h>
#include <maplab-common/text-formatting.h>

#include "console-common/console.h"

DEFINE_string(
    command, "",
    "Flag for console commands where another command is to be passed as "
    "argument.");
DEFINE_string(plugin, "", "Defines the plugin for which to show help.");
DEFINE_bool(all, false, "Set to true to display all commands in help.");
DEFINE_string(
    command_filter, "",
    "If this is set, help will only show commands that match this filter.");

namespace common {

BasicConsolePlugin::BasicConsolePlugin(
    Console* console, CommandRegisterer* command_registerer)
    : ConsolePluginBase(CHECK_NOTNULL(console)),
      command_registerer_(CHECK_NOTNULL(command_registerer)) {
  addCommand(
      {"help", "h", "?"},
      [this]() -> int {
        helpText();
        return 0;
      },
      "Shows a help text. Use help --plugin <plugin_name> to get help for the "
      "chosen plugin.",
      Processing::Sync);

  addCommand(
      {"jobs"},
      [this]() -> int {
        command_registerer_->listJobs();
        return 0;
      },
      "List all known jobs.", Processing::Sync);

  addCommand(
      {"join"},
      [this]() -> int {
        command_registerer_->waitForJobsToFinish();
        return 0;
      },
      "Wait for all known jobs to complete.", Processing::Sync);

  addCommand(
      {"watch", "w"},
      [this]() -> int {
        if (FLAGS_command.empty()) {
          std::cout << "Watch called with empty command!" << std::endl;
          return kStupidUserError;
        }
        SigintBreaker breaker;
        std::cout << "Running \"" << FLAGS_command
                  << "\" until you hit Ctrl+C..." << std::endl;
        while (!breaker.isBreakRequested()) {
          console_->RunCommand(FLAGS_command);
        }
        return kSuccess;
      },
      "Runs the command supplied with --command in a loop until Ctrl+C is hit.",
      Processing::Sync);
}

void BasicConsolePlugin::helpText() {
  std::vector<std::string> plugin_names;
  console_->getNamesOfInstalledPlugins(&plugin_names);

  if (FLAGS_all || (FLAGS_plugin.empty() && !FLAGS_command_filter.empty())) {
    // Show all commands or search through all plugins.
    for (const std::string& plugin_name : plugin_names) {
      showHelpForPlugin(plugin_name, FLAGS_command_filter);
    }
  } else if (
      std::find(plugin_names.begin(), plugin_names.end(), FLAGS_plugin) !=
      plugin_names.end()) {
    // Show commands in plugin.
    showHelpForPlugin(FLAGS_plugin, FLAGS_command_filter);
  } else {
    if (!FLAGS_plugin.empty()) {
      // Plugin doesn't exist.
      LOG(ERROR) << "No plugin \"" << FLAGS_plugin << "\" is installed!";
    }
    std::cout << "The following plugins are installed:\n";
    std::sort(plugin_names.begin(), plugin_names.end());
    for (const std::string& plugin_name : plugin_names) {
      std::cout << "- " << plugin_name << "\n";
    }
    std::cout << "\nUse " << formatText("help --all", FormatOptions::kBold)
              << " to get a list of all available commands.\n";
    std::cout << "Use "
              << formatText("help --plugin <plugin_name>", FormatOptions::kBold)
              << " to get a list of all commands defined in that plugin.\n";
    std::cout << "Use " << formatText(
                               "help --command_filter <search_term>",
                               FormatOptions::kBold)
              << " to search for commands that contain <search_term>."
              << std::endl;
  }

  // Reset flags.
  FLAGS_all = false;
  FLAGS_plugin = "";
  FLAGS_command_filter = "";
}

void BasicConsolePlugin::showHelpForPlugin(
    const std::string& plugin_name, const std::string& filter) const {
  std::vector<std::string> all_commands;
  command_registerer_->getAllCommands(&all_commands);
  std::sort(all_commands.begin(), all_commands.end());
  std::set<std::string> visited_plugins;

  // Get current shell size.
  struct winsize shell_size;
  ioctl(STDOUT_FILENO, TIOCGWINSZ, &shell_size);
  const size_t shell_width = shell_size.ws_col;

  std::cout << "Showing help for plugin " << plugin_name << ".\n";

  for (const std::string& command_name : all_commands) {
    const CommandRegisterer::Command& command =
        command_registerer_->getCommand(command_name);
    if (command.plugin_name == plugin_name) {
      if (visited_plugins.find(command_name) == visited_plugins.end()) {
        // Check if command matches filter.
        if (!filter.empty()) {
          bool filter_matched = false;
          for (const std::string& command_name_variant : command.commands) {
            if (command_name_variant.find(filter) != std::string::npos) {
              filter_matched = true;
              break;
            }
          }
          // If filter hasn't been found in command name, try help text before
          // giving up.
          if (!filter_matched &&
              command.help_text.find(filter) == std::string::npos) {
            continue;
          }
        }

        // Show the possible names for this command.
        bool first_command_variant = true;
        for (const std::string& command_name_variant : command.commands) {
          if (first_command_variant) {
            first_command_variant = false;
            std::cout << "  ";
          } else {
            std::cout << formatText(", ", FormatOptions::kBold);
          }

          std::cout << formatText(command_name_variant, FormatOptions::kBold);
          visited_plugins.emplace(command_name_variant);
        }
        std::cout << "\n";

        // Show help text.
        std::istringstream help_text_stream(command.help_text);
        std::string help_word;

        static const std::string kIndentation(3u, ' ');
        std::cout << kIndentation;
        // Need to increase by one because every word is prepended by a space.
        size_t current_line_length = kIndentation.size() + 1u;
        while (help_text_stream >> help_word) {
          const size_t word_length = help_word.size();
          if ((current_line_length + word_length + 1u) >=
              (shell_width - kIndentation.size())) {
            // Start a new line.
            std::cout << "\n" << kIndentation;
            current_line_length = kIndentation.size() + 1u;
          }
          colorOccurencesOfFilter(filter, &help_word);
          std::cout << " " << help_word;
          current_line_length += word_length + 1u;
        }
        std::cout << "\n";
      }
    }
  }
}

void BasicConsolePlugin::colorOccurencesOfFilter(
    const std::string& filter, std::string* input_word) const {
  CHECK_NOTNULL(input_word);

  if (filter.empty()) {
    return;
  }
  std::string::size_type last_find_position = 0u;
  const std::string replace_with = colorText(filter, ForegroundColors::kRed);
  while ((last_find_position = input_word->find(filter, last_find_position)) !=
         std::string::npos) {
    input_word->replace(last_find_position, filter.size(), replace_with);
    last_find_position += replace_with.size();
  }
}

}  // namespace common
