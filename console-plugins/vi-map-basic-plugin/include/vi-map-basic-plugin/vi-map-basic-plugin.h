#ifndef VI_MAP_BASIC_PLUGIN_VI_MAP_BASIC_PLUGIN_H_
#define VI_MAP_BASIC_PLUGIN_VI_MAP_BASIC_PLUGIN_H_

#include <string>

#include <console-common/console-plugin-base-with-plotter.h>
#include <console-common/console-plugin-base.h>
#include <console-common/console.h>
#include <maplab-common/map-manager-config.h>

namespace visualization {
class ViwlsGraphRvizPlotter;
}  // namespace visualization

namespace vi_map {

backend::SaveConfig parseSaveConfigFromGFlags();

class VIMapBasicPlugin : public common::ConsolePluginBaseWithPlotter {
 public:
  VIMapBasicPlugin(
      common::Console* console, visualization::ViwlsGraphRvizPlotter* plotter);

  std::string getPluginId() const override {
    return "vi_map_basic";
  }

 private:
  // ================
  // Console Commands
  // ================
  int selectMap();
  int listMaps();

  int createNewMap();
  int deleteMap();
  int clearStorage();
  int renameMap();

  int copyMap();
  int mergeMaps();
  int joinAllMaps();

  int loadMap();
  int loadAllMaps();
  int saveMap();
  int saveAllMaps();

  int loadMergeMap();
  int loadMergeAllMaps();

  int listMapsOnFileSystem();

  int getMapFolder();
  int setMapFolder();
  int cleanupResourceFolders();

  int listResourceFolders();
  int useMapResourceFolder();
  int useExternalResourceFolder();
  int printResourceStatistics();
  int printResourceCacheStatistics();

  int checkMapConsistency();

  int mapStatistics();
  int printMissionCoobservabilityStatistics() const;
  int printBaseframeTransformations() const;
  int printCameraCalibrations() const;
  int visualizeMap();
  int visualizeMapSequentially();

  int removeMission();
  int removeMissionInteractive();

  int spatiallyDistributeMissions();

  int convertMapToNewFormat();
};

}  // namespace vi_map

#endif  // VI_MAP_BASIC_PLUGIN_VI_MAP_BASIC_PLUGIN_H_
