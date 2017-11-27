#ifndef LANDMARK_MANIPULATION_PLUGIN_LANDMARK_MANIPULATION_PLUGIN_H_
#define LANDMARK_MANIPULATION_PLUGIN_LANDMARK_MANIPULATION_PLUGIN_H_

#include <string>

#include <console-common/console-plugin-base-with-plotter.h>
#include <console-common/console.h>
#include <visualization/viwls-graph-plotter.h>

namespace landmark_manipulation_plugin {

class LandmarkManipulationPlugin : public common::ConsolePluginBaseWithPlotter {
 public:
  LandmarkManipulationPlugin(
      common::Console* console, visualization::ViwlsGraphRvizPlotter* plotter);

  virtual std::string getPluginId() const {
    return "landmark_manipulation";
  }

 private:
  int retriangulateLandmarks();
  int evaluateLandmarkQuality();
  int resetLandmarkQualityToUnknown();
  int initTrackLandmarks();
  int removeBadLandmarks();
};

}  // namespace landmark_manipulation_plugin

#endif  // LANDMARK_MANIPULATION_PLUGIN_LANDMARK_MANIPULATION_PLUGIN_H_
