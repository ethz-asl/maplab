#ifndef DENSE_RECONSTRUCTION_DENSE_RECONSTRUCTION_PLUGIN_H_
#define DENSE_RECONSTRUCTION_DENSE_RECONSTRUCTION_PLUGIN_H_

#include <string>

#include <console-common/console-plugin-base-with-plotter.h>
#include <console-common/console.h>

namespace dense_reconstruction {

class DenseReconstructionPlugin : public common::ConsolePluginBaseWithPlotter {
 public:
  explicit DenseReconstructionPlugin(
      common::Console* console, visualization::ViwlsGraphRvizPlotter* plotter);

  virtual std::string getPluginId() const override {
    return "dense_reconstruction";
  }
};

}  // namespace dense_reconstruction

#endif  // DENSE_RECONSTRUCTION_DENSE_RECONSTRUCTION_PLUGIN_H_
