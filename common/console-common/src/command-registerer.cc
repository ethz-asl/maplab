#include "console-common/command-registerer.h"

#include <algorithm>
#include <chrono>
#include <exception>
#include <iostream>  // NOLINT
#include <map>
#include <sstream>  // NOLINT
#include <string>
#include <unordered_map>
#include <utility>
#include <wordexp.h>

#include <aslam/common/timer.h>
#include <glog/logging.h>
#include <maplab-common/accessors.h>

namespace common {
class Job {
 public:
  Job(const std::function<int()>& function, const std::string& description)
      : description_(description), running_(false), function_(function) {
    static int id = 0;
    id_ = id++;
    start_time_ = end_time_ = std::chrono::system_clock::now();
  }
  void startThread() {
    CHECK(function_);
    running_ = true;
    std::cout << "Starting job [" << id_ << "] " << description_ << "."
              << std::endl;
    thread_.reset(new std::thread([this]() {
      start_time_ = std::chrono::system_clock::now();
      function_();
      end_time_ = std::chrono::system_clock::now();
    }));
  }

  void joinThread() {
    if (running_) {
      std::cout << "Waiting for job [" << id_ << "] " << description_
                << " to finish." << std::endl;
      CHECK(thread_ != nullptr);
      thread_->join();
      running_ = false;
      end_time_ = std::chrono::system_clock::now();
    }
  }

  std::string printInfo() const {
    constexpr double kNumSecondsPerNanosecond = 1e-9;

    std::chrono::time_point<std::chrono::system_clock> end;
    if (running_) {
      end = std::chrono::system_clock::now();
    } else {
      end = end_time_;
    }

    double dt_seconds =
        static_cast<double>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                end - start_time_)
                .count()) *
        kNumSecondsPerNanosecond;
    std::stringstream ss;
    ss << "[" << id_ << "] " << description_ << ": "
       << (running_ ? "Running for " : "Stopped. Took ") << " " << dt_seconds
       << " secs. " << std::endl;
    return ss.str();
  }

  int getId() const {
    return id_;
  }

 private:
  std::string description_;
  int id_;
  bool running_;
  std::chrono::time_point<std::chrono::system_clock> start_time_;
  std::chrono::time_point<std::chrono::system_clock> end_time_;
  std::unique_ptr<std::thread> thread_;
  std::function<int()> function_;
};

CommandRegisterer::CommandRegisterer() {}

void CommandRegisterer::addCommand(const Command& command) {
  const size_t command_index = commands_.size();
  commands_.push_back(command);
  CHECK_EQ(commands_.size(), command_index + 1u);

  for (const std::string& command_string : command.commands) {
    if (command_map_.count(command_string) != 0u) {
      const std::string& owning_plugin =
          commands_[command_map_[command_string]].plugin_name;
      LOG(FATAL) << "The plugin \'" << command.plugin_name << "\' wants to "
                 << "register the command \'" << command_string << "\' but the "
                 << "command is already registered by plugin \'"
                 << owning_plugin << "\'.";
    }
    CHECK(command_map_.emplace(command_string, command_index).second);
  }
}

int CommandRegisterer::processCommand(const std::string& command) {
  if (command == "") {
    return kStupidUserError;
  }
  // Check if we have only whitespace
  if (command.find_first_not_of(' ') == std::string::npos) {
    return kStupidUserError;
  }

  wordexp_t result;
  if (wordexp(command.c_str(), &result, WRDE_SHOWERR)) {
    LOG(ERROR) << "Wordexp error";
    return kStupidUserError;
  }
  CHECK_GT(result.we_wordc, 0u);

  std::string command_without_flags = result.we_wordv[0];
  CommandIndexMap::const_iterator command_index_it =
      command_map_.find(command_without_flags);
  if (command_index_it == command_map_.end()) {
    LOG(ERROR) << "Unknown command: " << command_without_flags;
    std::cout << "Type help for more information.\n";
    wordfree(&result);
    return kStupidUserError;
  } else {
    // Go through all commands and check that the flags exist so we don't exit
    // in case we have a typo.
    int argc = result.we_wordc;
    for (int i = 1; i < argc; ++i) {
      std::string raw_command = result.we_wordv[i];
      // Process help here since gflags will shutdown the app otherwise.
      if (raw_command == "--help" || raw_command == "-help") {
        google::ShowUsageWithFlags(result.we_wordv[0]);
        wordfree(&result);
        return kSuccess;
      }
      bool is_a_flag = false;
      if (raw_command.size() > 1 && raw_command.substr(0, 2) == "--") {
        raw_command = raw_command.substr(2);
        is_a_flag = true;
      } else if (!raw_command.empty() && raw_command[0] == '-') {
        raw_command = raw_command.substr(1);
        is_a_flag = true;
      }
      size_t pos_of_equals = raw_command.find("=");
      if (pos_of_equals != std::string::npos) {
        raw_command = raw_command.substr(0, pos_of_equals);
      }
      std::string tmp;
      bool flag_exists =
          google::GetCommandLineOption(raw_command.c_str(), &tmp);
      if (is_a_flag && !flag_exists) {
        LOG(ERROR) << "Unknown command line flag '" << raw_command << "'";
        wordfree(&result);
        return kUnknownError;
      }
    }

    google::ParseCommandLineFlags(&argc, &result.we_wordv, false);

    CHECK_LT(command_index_it->second, commands_.size());
    const Command& command = commands_[command_index_it->second];
    if (command.processing_model == Processing::Async) {
      std::shared_ptr<Job> job(
          new Job(command.callback, command_without_flags));
      jobs_[job->getId()] = job;
      job->startThread();
      wordfree(&result);
      return kSuccess;
    } else {
      try {
        wordfree(&result);
        timing::Timer timer("exec - " + std::string(command_without_flags));
        int status = command.callback();
        timer.Stop();
        return status;
      } catch (const std::exception& e) {  // NOLINT
        LOG(ERROR) << "Caught exception while processing command "
                   << command_without_flags << ": " << e.what();
        wordfree(&result);
        return kUnknownError;
      }
    }
  }
}

void CommandRegisterer::listJobs() const {
  if (jobs_.empty()) {
    std::cout << "No jobs started so far." << std::endl;
    return;
  }
  std::cout << "Status of all known jobs:" << std::endl;
  for (const std::pair<int, std::shared_ptr<Job> > id_job : jobs_) {
    CHECK(id_job.second != nullptr);
    const Job& job = *id_job.second;
    std::cout << job.printInfo() << std::endl;
  }
}

void CommandRegisterer::waitForJobsToFinish() const {
  if (jobs_.empty()) {
    std::cout << "No jobs started so far." << std::endl;
    return;
  }
  std::cout << "Waiting for all known jobs to end:" << std::endl;
  for (const std::pair<int, std::shared_ptr<Job> >& id_job : jobs_) {
    CHECK(id_job.second != nullptr);
    Job& job = *id_job.second;
    job.joinThread();
  }
  std::cout << "All threads joined." << std::endl;
}

void CommandRegisterer::getAllCommands(
    std::vector<std::string>* all_cmds) const {
  CHECK_NOTNULL(all_cmds)->clear();
  all_cmds->reserve(commands_.size());
  for (const Command& command : commands_) {
    all_cmds->insert(
        all_cmds->end(), command.commands.begin(), command.commands.end());
  }
}

const CommandRegisterer::Command& CommandRegisterer::getCommand(
    const std::string& command_name) const {
  const CommandIndexMap::const_iterator it = command_map_.find(command_name);
  CHECK(it != command_map_.end());
  const size_t command_index = it->second;
  CHECK_LT(command_index, commands_.size());
  return commands_[command_index];
}

void CommandRegisterer::clear() {
  commands_.clear();
  command_map_.clear();
}

}  // namespace common
