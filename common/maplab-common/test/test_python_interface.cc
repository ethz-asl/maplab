#include <limits.h>
#include <string>
#include <unistd.h>

#include "maplab-common/file-logger.h"
#include "maplab-common/python-interface.h"
#include "maplab-common/test/testing-entrypoint.h"

namespace common {

void createTestScript(const std::string& filename) {
  const std::string kTestScript(
      "def multiply_by_2(values):         \n"
      "    products = list()              \n"
      "    for value in values:           \n"
      "        products.append(value * 2) \n"
      "    return products                \n");

  FileLogger python_script(filename);
  python_script << kTestScript;
}

TEST(TestPythonInterface, SimpleTest) {
  const std::string kTestSciptFilenName("/tmp/test_module.py");
  createTestScript(kTestSciptFilenName);

  std::vector<double> input = {1, 2, 3};
  std::vector<double> output;

  PythonInterface interface(kTestSciptFilenName);
  EXPECT_TRUE(interface.callFunction("multiply_by_2", input, &output));
  ASSERT_EQ(output.size(), input.size());

  for (size_t i = 0u; i < input.size(); ++i) {
    EXPECT_EQ(input[i] * 2, output[i]);
  }
}

}  // namespace common

MAPLAB_UNITTEST_ENTRYPOINT
