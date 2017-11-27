#ifndef CONSOLE_COMMON_CONSOLE_H_
#define CONSOLE_COMMON_CONSOLE_H_

#include <list>
#include <memory>
#include <set>
#include <string>
#include <unordered_set>
#include <vector>

#include <maplab-common/unique-id.h>

#include "console-common/console-plugin-base.h"

/**
 * Shorthand for treating a command with one common::Id (e.g. mission id)
 * as argument
 */
bool idArg(const char* input, const char* command, common::Id* id);

namespace common {

class CommandRegisterer;
class ConsolePluginBase;

class Console {
 public:
  Console();
  explicit Console(const std::string& console_name);

  // The main purpose of this constructor is to supply your own extended
  // CommandRegisterer, like the visual_inertial_mapping::RpcCommandRegisterer.
  // Use the other constructors if the default common::CommandRegisterer
  // suffices.
  Console(
      const std::string& console_name,
      CommandRegisterer* command_registerer_ptr);

  virtual ~Console();
  void RunCommandPrompt();
  int RunCommand(const std::string& command);
  void addCommand(const CommandRegisterer::Command& command);
  void setConsoleName(const std::string& name);
  void addMapKeyToAutoCompletion(const std::string& map_key);
  void removeMapKeyFromAutoCompletion(const std::string& map_key);
  void addAllGFlagsToCompletion();

  void installPlugin(ConsolePluginPtr plugin);
  void getNamesOfInstalledPlugins(std::vector<std::string>* plugin_names) const;

  void setSelectedMapKey(const std::string& selected_map_key);
  const std::string& getSelectedMapKey() {
    return selected_map_key_;
  }

 protected:
  // This is used so that plugins can be destroyed before the dynamic library is
  // properly closed.
  void uninstallAllPlugins();

 private:
  class PersistentHistory {
   public:
    static constexpr int kMaxPersistentHistoryLength = 200;

    void Add(const std::string& value);
    void Save() const;
    void Load();

   private:
    std::list<std::string> persistent_history_;
  };

  class AutoCompletion {
   public:
    AutoCompletion();
    ~AutoCompletion();
    void addCommandToIndex(const std::string& command);
    void addFlagToIndex(const std::string& flag);
    void removeFlagFromIndex(const std::string& flag);
    void addCommandsToIndex(const std::vector<std::string>& commands);
    void addAllGFlags();
    void enableAutoCompletion();
    void disableAutoCompletion();

    static char** readlineCompletionCallback(
        const char* text, int start, int /*end*/);

    // This link is needed that the static C-compatible callback for readline
    // can access "this".
    static AutoCompletion* backlink_this_;

   private:
    typedef std::set<std::string> CompletionIndex;
    static char** convertToCArray(
        const std::string& common_prefix,
        const std::vector<std::string>& strings);
    size_t getCompletionCandidates(
        const std::string& word_to_complete, const CompletionIndex& word_index,
        std::string* common_prefix, std::vector<std::string>* candidates) const;
    CompletionIndex command_index_;
    CompletionIndex flag_index_;
  };

  bool sheep_enabled_;
  std::unique_ptr<CommandRegisterer> command_registerer_ptr_;
  std::string console_name_;
  std::string console_name_with_selected_key_;
  std::string selected_map_key_;
  PersistentHistory persistent_history_;
  AutoCompletion auto_completion_;

  std::unordered_set<ConsolePluginPtr> installed_plugins_;
};

}  // namespace common

#endif  // CONSOLE_COMMON_CONSOLE_H_
