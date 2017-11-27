#include "maplab-common/string-tools.h"
#include "maplab-common/test/testing-entrypoint.h"

namespace common {
TEST(MaplabCommon, TokenizeString) {
  std::string kInputString("1,2,,3");
  constexpr char kDelimiter = ',';

  bool remove_empty = true;
  std::vector<std::string> tokens;
  tokenizeString(kInputString, kDelimiter, remove_empty, &tokens);
  ASSERT_EQ(tokens.size(), 3u);
  EXPECT_EQ(tokens[0], "1");
  EXPECT_EQ(tokens[1], "2");
  EXPECT_EQ(tokens[2], "3");

  remove_empty = false;
  tokenizeString(kInputString, kDelimiter, remove_empty, &tokens);
  ASSERT_EQ(tokens.size(), 4u);
  EXPECT_EQ(tokens[0], "1");
  EXPECT_EQ(tokens[1], "2");
  EXPECT_EQ(tokens[2], "");
  EXPECT_EQ(tokens[3], "3");
}
}  // namespace common

MAPLAB_UNITTEST_ENTRYPOINT
