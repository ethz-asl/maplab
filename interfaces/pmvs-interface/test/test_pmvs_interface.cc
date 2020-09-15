#include <string>

#include <glog/logging.h>
#include <maplab-common/test/testing-entrypoint.h>

#include "dense-reconstruction/pmvs-config.h"
#include "dense-reconstruction/pmvs-file-utils.h"
#include "dense-reconstruction/pmvs-interface.h"

namespace backend {

class PmvsImportExportTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    // TODO(mfehr): Implement
  }
};

TEST_F(PmvsImportExportTest, TestPmvsImportExport) {}

}  // namespace backend

MAPLAB_UNITTEST_ENTRYPOINT
