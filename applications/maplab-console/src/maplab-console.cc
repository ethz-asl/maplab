#include "maplab-console/maplab-console.h"

#include <dlfcn.h>

#include <cstdlib>
#include <fstream>  // NOLINT
#include <string>
#include <unordered_set>

#include <gflags/gflags.h>
#include <maplab-common/file-system-tools.h>
#include <visualization/rviz-visualization-sink.h>

DEFINE_bool(ros_free, false, "Enable this flag to run on systems without ROS");

namespace maplab {

MapLabConsole::MapLabConsole(
    const std::string& console_name, int argc, char** argv)
    : common::Console(console_name) {
  setSelectedMapKey("");
  discoverAndInstallPlugins(argc, argv);
}

MapLabConsole::~MapLabConsole() {
  uninstallAllPlugins();
  for (void*& plugin_handle : plugin_handles_) {
    CHECK_NOTNULL(plugin_handle);
    CHECK_EQ(dlclose(plugin_handle), 0);
  }
}

void MapLabConsole::discoverAndInstallPlugins(int argc, char** argv) {
  // Plotter can be nullptr if --ros_free has been specified.
  const char* plugin_list_file_path =
      std::getenv("MAPLAB_CONSOLE_PLUGINS_LIBRARY_LIST");
  CHECK(plugin_list_file_path != nullptr && plugin_list_file_path[0] != '\0')
      << "$MAPLAB_CONSOLE_PLUGINS_LIBRARY_LIST isn't defined. Source your "
      << "catkin workspace:\n"
      << "    source ~/catkin_ws/devel/setup.bash";

  // Find plugins to install.
  std::unordered_set<std::string> discovered_plugins;
  {
    std::ifstream plugin_list_stream(plugin_list_file_path);
    if (plugin_list_stream.is_open()) {
      std::string plugin_library_path;
      while (std::getline(plugin_list_stream, plugin_library_path)) {
        if (common::fileExists(plugin_library_path)) {
          discovered_plugins.emplace(plugin_library_path);
        } else {
          LOG(WARNING)
              << "Plugin list mentions that there should be a library under \""
              << plugin_library_path
              << "\", but no such file exists. The entry will be automatically "
              << "removed from the file. If you think the plugin should be "
              << "available, please rerun cmake and recompile the package "
              << "containing the plugin by running\n"
              << "\tcatkin build --no-deps --force-cmake <plugin_package>\n"
              << "and then try again.";
        }
      }
    } else {
      LOG(WARNING) << "No plugin can be loaded because plugin list under \""
                   << plugin_list_file_path << "\" couldn't be opened.";
    }
  }
  {
    // Clean up plugins list: replace file with list of the plugins that have
    // been found and loaded. This should be without duplicates now.
    std::ofstream plugin_list_stream(plugin_list_file_path);
    if (plugin_list_stream.is_open()) {
      for (const std::string& plugin_library_path : discovered_plugins) {
        plugin_list_stream << plugin_library_path << "\n";
      }
    }
  }

  // Dynamically load found plugins.
  std::vector<std::pair</*plugin_handle=*/void*, /*plugin_file=*/std::string>>
      try_load_plugins_handle_path;
  for (const std::string& plugin_library_path : discovered_plugins) {
    void* handle = dlopen(plugin_library_path.c_str(), RTLD_LAZY);
    if (handle == nullptr) {
      LOG(ERROR) << "Failed to load library " << plugin_library_path
                 << ". Error message: " << dlerror();
      continue;
    }
    try_load_plugins_handle_path.emplace_back(handle, plugin_library_path);
  }

  // Now that all plugins are loaded we can parse the flags and add them to the
  // completion index.
  google::ParseCommandLineFlags(&argc, &argv, true);
  addAllGFlagsToCompletion();

  if (!FLAGS_ros_free) {
    visualization::RVizVisualizationSink::init();
    plotter_.reset(new visualization::ViwlsGraphRvizPlotter);
    LOG(INFO) << "RVIZ visualization initialized!";
  }

  for (const std::pair<void*, std::string>& handle_libpath :
       try_load_plugins_handle_path) {
    void* handle = handle_libpath.first;
    common::PluginCreateFunction create_function =
        common::PluginCreateFunction(dlsym(handle, "createConsolePlugin"));
    common::PluginDestroyFunction destroy_function =
        common::PluginDestroyFunction(dlsym(handle, "destroyConsolePlugin"));
    if (create_function == nullptr || destroy_function == nullptr) {
      LOG(ERROR) << "Error loading the functions from plugin "
                 << handle_libpath.second << ". Error message: " << dlerror()
                 << "\nMake sure that your plugin implements the functions "
                 << "\"ConsolePluginBase* "
                 << "createConsolePlugin(common::Console*, "
                 << "visualization::ViwlsGraphRvizPlotter)\" and \"void "
                 << "destroyConsolePlugin(common::ConsolePluginBase*)";
      CHECK_EQ(dlclose(handle), 0);
      continue;
    }
    plugin_handles_.emplace_back(handle);

    // Create and install plugin.
    common::ConsolePluginPtr plugin(
        create_function(this, plotter_.get()), destroy_function);
    VLOG(1) << "Installed plugin " << plugin->getPluginId() << " from "
            << handle_libpath.second << ".";
    installPlugin(std::move(plugin));
  }
}

}  // namespace maplab
