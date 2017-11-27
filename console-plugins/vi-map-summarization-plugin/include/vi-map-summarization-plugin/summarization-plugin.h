#ifndef VI_MAP_SUMMARIZATION_PLUGIN_SUMMARIZATION_PLUGIN_H_
#define VI_MAP_SUMMARIZATION_PLUGIN_SUMMARIZATION_PLUGIN_H_

#include <string>

#include <console-common/console-plugin-base.h>
#include <console-common/console.h>
#include <vi-map/vi-map.h>

namespace summarization_plugin {

class SummarizationPlugin : public common::ConsolePluginBase {
 public:
  explicit SummarizationPlugin(common::Console* console);

  virtual std::string getPluginId() const {
    return "summarization";
  }

 private:
  int saveSummaryMapToDisk() const;
};

}  // namespace summarization_plugin

#endif  // VI_MAP_SUMMARIZATION_PLUGIN_SUMMARIZATION_PLUGIN_H_
