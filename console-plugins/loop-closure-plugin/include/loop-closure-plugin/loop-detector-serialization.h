#ifndef LOOP_CLOSURE_PLUGIN_LOOP_DETECTOR_SERIALIZATION_H_
#define LOOP_CLOSURE_PLUGIN_LOOP_DETECTOR_SERIALIZATION_H_

#include <string>

namespace vi_map {
class VIMap;
}  // namespace vi_map

namespace loop_closure_plugin {
int generateLoopDetectorForVIMapAndSerialize(
    const std::string& map_folder, const vi_map::VIMap& vi_map);
}  // namespace loop_closure_plugin

#endif  // LOOP_CLOSURE_PLUGIN_LOOP_DETECTOR_SERIALIZATION_H_
