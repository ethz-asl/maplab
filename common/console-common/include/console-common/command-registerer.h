#ifndef CONSOLE_COMMON_COMMAND_REGISTERER_H_
#define CONSOLE_COMMON_COMMAND_REGISTERER_H_

#include <functional>
#include <initializer_list>
#include <map>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <maplab-common/macros.h>

namespace common {
class Job;
enum class Processing { Sync, Async };

enum CommandStatus {
  kSuccess,
  kUnknownError,
  kStupidUserError,
  kCustomStatusOffset
};

class CommandRegisterer {
 public:
  CommandRegisterer();
  MAPLAB_DISALLOW_EVIL_CONSTRUCTORS(CommandRegisterer);

  int processCommand(const std::string& command);

  void getAllCommands(std::vector<std::string>* all_cmds) const;

  void listJobs() const;
  void waitForJobsToFinish() const;

  void clear();

  struct Command {
    Command(
        const std::initializer_list<std::string>& _commands,
        const std::function<int()>& _callback, const std::string& _help_text,
        const Processing _processing_model, const std::string& _plugin_name)
        : commands(_commands),
          callback(_callback),
          help_text(_help_text),
          processing_model(_processing_model),
          plugin_name(_plugin_name) {}

    std::vector<std::string> commands;
    std::function<int()> callback;
    std::string help_text;
    Processing processing_model;
    std::string plugin_name;
  };

  typedef std::vector<Command> Commands;

  void addCommand(const Command& command);
  const Command& getCommand(const std::string& command_name) const;

 private:
  void processCommandAsync(const std::string& command);

  Commands commands_;

  typedef std::map<std::string, size_t> CommandIndexMap;
  CommandIndexMap command_map_;

  std::unordered_map<int, std::shared_ptr<Job> > jobs_;
};
}  // namespace common
#endif  // CONSOLE_COMMON_COMMAND_REGISTERER_H_
