#include <cstdlib>
#include <string>

#include <benchmark_catkin/benchmark_entrypoint.h>

#include "vi-map/vi-map-serialization.h"
#include "vi-map/vi-map.h"

DECLARE_bool(show_progress_bar);

namespace vi_map {

class VIMapSerializationBenchmark : public ::benchmark::Fixture {
 public:
  void SetUp(const ::benchmark::State&) {
    FLAGS_show_progress_bar = false;

    // Can't parse gflags when using google benchmark, therefore we use an
    // environment variable instead.
    char* map_folder_env = std::getenv("BENCHMARK_MAP_FOLDER");
    if (map_folder_env == nullptr) {
      map_folder_load_ = "./test_maps/vi_app_test";
    } else {
      map_folder_load_ = map_folder_env;
    }
    CHECK(serialization::hasMapOnFileSystem(map_folder_load_))
        << "Map under path \"" << map_folder_load_ << "\" doesn't exist. "
        << "Use the environment variable BENCHMARK_MAP_FOLDER to select the "
        << "map folder.";
    map_folder_save_ = "./benchmark/save";

    map_ = aligned_unique<VIMap>();
    CHECK(serialization::loadMapFromFolder(map_folder_load_, map_.get()));
  }

  void TearDown() {
    map_.reset();
  }

 protected:
  std::string map_folder_load_;
  std::string map_folder_save_;
  VIMap::UniquePtr map_;
};

BENCHMARK_F(VIMapSerializationBenchmark, LoadMapFromFolder)
(benchmark::State& state) {  // NOLINT
  while (state.KeepRunning()) {
    VIMap deserialized_map;
    serialization::loadMapFromFolder(map_folder_load_, &deserialized_map);
  }
}

BENCHMARK_F(VIMapSerializationBenchmark, SaveMapToFolder)
(benchmark::State& state) {  // NOLINT
  backend::SaveConfig config;
  config.overwrite_existing_files = true;
  while (state.KeepRunning()) {
    serialization::saveMapToFolder(map_folder_save_, config, map_.get());
    CHECK(serialization::hasMapOnFileSystem(map_folder_save_));
  }
}

}  // namespace vi_map

BENCHMARKING_ENTRY_POINT
