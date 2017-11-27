#include "maplab-common/test/testing-entrypoint.h"

#include "maplab-common/file-logger.h"
#include "maplab-common/file-system-tools.h"

namespace common {

TEST(FileLogger, WriteString) {
  const std::string kTestFile = "testfile.txt";
  const std::string kTestString1 = "blabla1";
  const std::string kTestString2 = "blabla2";
  {
    common::FileLogger logger(kTestFile);
    ASSERT_TRUE(logger.isOpen());
    logger << kTestString1 << kTestString2 << std::endl;
    logger << kTestString2 << kTestString1 << std::endl;
    logger.flushBuffer();
    logger.closeFile();
  }

  std::ifstream reader(kTestFile);
  ASSERT_TRUE(reader.good());

  std::string line_buffer;
  EXPECT_FALSE(reader.eof());
  std::getline(reader, line_buffer);
  EXPECT_EQ(kTestString1 + kTestString2, line_buffer);
  EXPECT_FALSE(reader.eof());
  std::getline(reader, line_buffer);
  EXPECT_EQ(kTestString2 + kTestString1, line_buffer);
  std::getline(reader, line_buffer);
  EXPECT_EQ("", line_buffer);
  EXPECT_TRUE(reader.eof());

  reader.close();
}

}  // namespace common

MAPLAB_UNITTEST_ENTRYPOINT
