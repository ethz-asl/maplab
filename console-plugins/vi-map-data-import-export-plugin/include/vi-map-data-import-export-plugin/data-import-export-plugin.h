#ifndef VI_MAP_DATA_IMPORT_EXPORT_PLUGIN_DATA_IMPORT_EXPORT_PLUGIN_H_
#define VI_MAP_DATA_IMPORT_EXPORT_PLUGIN_DATA_IMPORT_EXPORT_PLUGIN_H_

#include <string>

#include <console-common/console-plugin-base.h>
#include <console-common/console.h>

namespace data_import_export {

class DataImportExportPlugin : public common::ConsolePluginBase {
 public:
  explicit DataImportExportPlugin(common::Console* console);

  virtual std::string getPluginId() const override {
    return "data_import_export";
  }

 private:
  int exportPosesVelocitiesAndBiasesToCsv() const;
  int exportNCameraCalibration() const;
  int exportOptionalSensorExtrinsics() const;
  int importGpsDataFromRosbag() const;
  int exportGpsUtmToCsv() const;
  int exportGpsWgsToCsv() const;
};

}  // namespace data_import_export

#endif  // VI_MAP_DATA_IMPORT_EXPORT_PLUGIN_DATA_IMPORT_EXPORT_PLUGIN_H_
