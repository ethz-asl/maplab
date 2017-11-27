#include "console-common/console.h"

#include <readline/history.h>
#include <readline/readline.h>

#include <csignal>
#include <iostream>  // NOLINT

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <maplab-common/text-formatting.h>
#include <maplab-common/yaml-serialization.h>

#include "console-common/basic-console-plugin.h"
#include "console-common/command-registerer.h"

DEFINE_string(
    command_history_filename, ".vi_command_history",
    "Where the persistent command history is stored.");
DEFINE_bool(
    keep_gflags, false,
    "If set to true, changes to gflags during the following command won't be "
    "reset.");

namespace common {

constexpr char kConsoleDefaultName[] = "console";

/**
 * Shorthand for treating a command with one common::Id (e.g. mission id)
 * as argument
 */
bool idArg(const char* input, const char* command, common::Id* id) {
  if (strstr(input, command) == input) {
    std::istringstream in(input);
    std::string temp, id_string;
    in >> temp >> id_string;
    if (!id->fromHexString(id_string)) {
      LOG(ERROR) << "Received invalid id.";
      return false;
    }
    return true;
  }
  return false;
}

Console::Console(
    const std::string& console_name, CommandRegisterer* command_registerer_ptr)
    : sheep_enabled_(false),
      command_registerer_ptr_(CHECK_NOTNULL(command_registerer_ptr)),
      console_name_(console_name) {
  persistent_history_.Load();

  if (command_registerer_ptr_) {
    std::vector<std::string> all_commands;
    command_registerer_ptr_->getAllCommands(&all_commands);
    auto_completion_.addCommandsToIndex(all_commands);
  }
  auto_completion_.enableAutoCompletion();

  // Auto-install basic plugin always because it's the only one that depends on
  // the command registerer.
  installPlugin(
      ConsolePluginPtr(
          new BasicConsolePlugin(this, command_registerer_ptr_.get()),
          [](ConsolePluginBase* plugin) { delete plugin; }));

  addCommand(
      CommandRegisterer::Command(
          {"enable_sheep", "sheep"},
          [this]() -> int {
            this->sheep_enabled_ = !this->sheep_enabled_;
            return common::kSuccess;
          },
          "Enable sheep.", common::Processing::Sync, "console"));
}

Console::Console(const std::string& console_name)
    : Console(console_name, new CommandRegisterer()) {}

Console::Console() : Console(kConsoleDefaultName) {}

Console::~Console() {}

void Console::RunCommandPrompt() {
  std::string last_input;
  while (true) {
    std::string shell_prompt = console_name_with_selected_key_ + ":/$ ";
    if (sheep_enabled_) {
      const std::string kSheep("🐑");
      constexpr bool kUseReadlineEscapeCharacters = true;
      const std::string sheep = common::formatText(
          kSheep, common::FormatOptions::kBold,
          common::ForegroundColors::kGreen, common::BackgroundColors::kDefault,
          kUseReadlineEscapeCharacters);
      shell_prompt = sheep + " " + shell_prompt;
    }
    char* input = readline(shell_prompt.c_str());

    if (!input || strcmp(input, "q") == 0 || strcmp(input, "exit") == 0) {
      break;
    }

    if (input != last_input) {
      add_history(input);
      last_input = input;
      persistent_history_.Add(input);
      persistent_history_.Save();
    }

    RunCommand(input);
    free(input);
  }
}

void Console::addCommand(const CommandRegisterer::Command& command) {
  CHECK_NE(command.commands.size(), 0u);
  CHECK(command.callback);
  CHECK(!command.help_text.empty());
  command_registerer_ptr_->addCommand(command);
  auto_completion_.addCommandsToIndex(command.commands);
}

void Console::setConsoleName(const std::string& name) {
  CHECK(!name.empty());
  console_name_ = name;

  // Update displayed text and append selected map key to new console name.
  setSelectedMapKey(getSelectedMapKey());
}

void Console::addMapKeyToAutoCompletion(const std::string& map_key) {
  CHECK(!map_key.empty());
  auto_completion_.addFlagToIndex(map_key);
}

void Console::removeMapKeyFromAutoCompletion(const std::string& map_key) {
  CHECK(!map_key.empty());
  auto_completion_.removeFlagFromIndex(map_key);
}

void Console::addAllGFlagsToCompletion() {
  auto_completion_.addAllGFlags();
}

void Console::PersistentHistory::Add(const std::string& value) {
  persistent_history_.push_back(value);
  while (persistent_history_.size() > kMaxPersistentHistoryLength) {
    persistent_history_.pop_front();
  }
}

void Console::PersistentHistory::Save() const {
  std::ofstream ofs(FLAGS_command_history_filename);
  if (!ofs.is_open()) {
    LOG(WARNING) << "Could not write command history to file: "
                 << FLAGS_command_history_filename;
    return;
  }
  YAML::Save(persistent_history_, &ofs);
}

void Console::PersistentHistory::Load() {
  YAML::Load(FLAGS_command_history_filename, &persistent_history_);
  for (const std::string& command : persistent_history_) {
    add_history(command.c_str());
  }
}

Console::AutoCompletion* Console::AutoCompletion::backlink_this_ = nullptr;

Console::AutoCompletion::AutoCompletion() {
  // Add space to the completion indices that we won't get completions on space
  // characters.
  addCommandToIndex(" ");
  addFlagToIndex(" ");
}

Console::AutoCompletion::~AutoCompletion() {
  disableAutoCompletion();
}

void Console::AutoCompletion::addCommandToIndex(const std::string& command) {
  command_index_.insert(command);
}

void Console::AutoCompletion::addFlagToIndex(const std::string& flag) {
  flag_index_.insert(flag);
}

void Console::AutoCompletion::removeFlagFromIndex(const std::string& flag) {
  const CompletionIndex::const_iterator it = flag_index_.find(flag);
  if (it != flag_index_.end()) {
    flag_index_.erase(it);
  }
}

void Console::AutoCompletion::addCommandsToIndex(
    const std::vector<std::string>& commands) {
  for (const std::string& command : commands) {
    addCommandToIndex(command);
  }
}

void Console::AutoCompletion::addAllGFlags() {
  std::vector<google::CommandLineFlagInfo> flags;
  google::GetAllFlags(&flags);
  for (const google::CommandLineFlagInfo& flag : flags) {
    addFlagToIndex("-" + flag.name);
    addFlagToIndex("--" + flag.name);
  }
}

void Console::AutoCompletion::enableAutoCompletion() {
  CHECK(backlink_this_ == nullptr)
      << "There is already an auto-completer registered for this process";
  backlink_this_ = this;
  rl_attempted_completion_function =
      &AutoCompletion::readlineCompletionCallback;
  rl_bind_key('\t', rl_complete);
}

void Console::AutoCompletion::disableAutoCompletion() {
  backlink_this_ = nullptr;
  rl_attempted_completion_function = nullptr;
}

char** Console::AutoCompletion::readlineCompletionCallback(
    const char* text, int start, int /*end*/) {
  // Get a link to this, as C has no notion of classes.
  if (!backlink_this_) {
    return nullptr;
  }

  std::string word_to_complete(text);
  if (word_to_complete.empty()) {
    return nullptr;
  }

  AutoCompletion& that = *CHECK_NOTNULL(backlink_this_);
  std::string common_prefix_on_all_candidates;
  std::vector<std::string> candidates;
  if (start == 0) {
    // If the completed word starts at the beginning of the command line we look
    // for commands to
    // auto-complete.
    that.getCompletionCandidates(
        word_to_complete, that.command_index_, &common_prefix_on_all_candidates,
        &candidates);
  } else {
    // Otherwise we look for flags and map keys to auto-complete.
    that.getCompletionCandidates(
        word_to_complete, that.flag_index_, &common_prefix_on_all_candidates,
        &candidates);
  }
  rl_completion_append_character = ' ';

  if (candidates.empty()) {
    return nullptr;
  }
  return convertToCArray(common_prefix_on_all_candidates, candidates);
}

char** Console::AutoCompletion::convertToCArray(
    const std::string& common_prefix,
    const std::vector<std::string>& candidates) {
  CHECK(!candidates.empty());
  char** c_strings = new char*[candidates.size() + 2u];

  // Add the common prefix first.
  c_strings[0] = new char[common_prefix.size() + 1u];
  strcpy(c_strings[0], common_prefix.c_str());  // NOLINT

  // Add all candidates.
  for (size_t candidate_idx = 0u; candidate_idx < candidates.size();
       ++candidate_idx) {
    c_strings[candidate_idx + 1] =
        new char[candidates[candidate_idx].size() + 1u];
    strcpy(c_strings[candidate_idx + 1],  // NOLINT
           candidates[candidate_idx].c_str());
  }
  c_strings[candidates.size() + 1] = nullptr;
  return c_strings;
}

size_t Console::AutoCompletion::getCompletionCandidates(
    const std::string& word_to_complete, const CompletionIndex& word_index,
    std::string* common_prefix, std::vector<std::string>* candidates) const {
  CHECK_NOTNULL(common_prefix);
  CHECK_NOTNULL(candidates)->clear();

  if (word_to_complete.empty()) {
    common_prefix->clear();
    return 0;
  }

  for (const std::string& candidate : word_index) {
    // Check if the candidate starts with the word_start string.
    if (candidate.find(word_to_complete) == 0) {
      candidates->emplace_back(candidate);
    }
  }

  // Special case for if no candidates found.
  if (candidates->empty()) {
    *common_prefix = word_to_complete;
    return 0;
  }

  // Find the longest common prefix among all candidates. We can autocomplete up
  // to this point without any user interaction. If there is only one candidate
  // we can skip
  // the common prefix calculate as it is equal to this one candidate.
  std::string shared_prefix_string = *candidates->begin();
  if (candidates->size() > 1) {
    const std::string& top_string = *candidates->begin();
    const std::string& bottom_string = *candidates->rbegin();
    size_t char_idx = 0u;
    while (char_idx < top_string.length() &&
           top_string.at(char_idx) == bottom_string.at(char_idx)) {
      ++char_idx;
    }
    shared_prefix_string = top_string.substr(0, char_idx);
  }
  *common_prefix = shared_prefix_string;

  return candidates->size();
}

int Console::RunCommand(const std::string& command) {
  // FlagSaver stores the saves the current gflags configuration on construction
  // and restores this saved flag configuration when it's destructed.
  std::unique_ptr<google::FlagSaver> flag_saver_keep_flags;
  int command_result = 0;
  {
    const bool global_keep_gflags = FLAGS_keep_gflags;
    google::FlagSaver flag_saver_restore_flags;
    command_result = command_registerer_ptr_->processCommand(command);

    if (FLAGS_keep_gflags) {
      // If keep_gflags is set, construct another FlagSaver which will be
      // deleted after the intial one, so that the new flags (set by the current
      // command) will be kept persistent.
      if (!global_keep_gflags) {
        // If global_keep_gflags, then the flag was set to true before (as a
        // flag when the console was launched), so we don't disable it again.
        FLAGS_keep_gflags = false;
      }
      flag_saver_keep_flags.reset(new google::FlagSaver);
    }
  }
  return command_result;
}

void Console::installPlugin(ConsolePluginPtr plugin) {
  CHECK(plugin != nullptr);
  auto_completion_.addFlagToIndex(plugin->getPluginId());

  for (const CommandRegisterer::Command& command : plugin->commands_) {
    addCommand(command);
  }
  installed_plugins_.emplace(std::move(plugin));
}

void Console::uninstallAllPlugins() {
  command_registerer_ptr_->clear();
  installed_plugins_.clear();
}

void Console::getNamesOfInstalledPlugins(
    std::vector<std::string>* plugin_names) const {
  CHECK_NOTNULL(plugin_names)->clear();
  for (const ConsolePluginPtr& plugin : installed_plugins_) {
    plugin_names->emplace_back(plugin->getPluginId());
  }
}

void Console::setSelectedMapKey(const std::string& selected_map_key) {
  selected_map_key_ = selected_map_key;
  constexpr bool kUseReadlineEscapeCharacters = true;
  const std::string new_name =
      console_name_ + " " +
      common::formatText(
          std::string("<") + selected_map_key_ + ">",
          common::FormatOptions::kDefault, common::ForegroundColors::kYellow,
          common::BackgroundColors::kDefault, kUseReadlineEscapeCharacters);
  console_name_with_selected_key_ = new_name;
}

}  // namespace common
