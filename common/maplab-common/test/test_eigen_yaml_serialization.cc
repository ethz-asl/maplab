#include <string>
#include <vector>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include <maplab-common/eigen-yaml-serialization.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>
#include <maplab-common/yaml-serialization.h>

namespace common {
#define CHECK_TEST(testname, type, rows, cols, filename)    \
  do {                                                      \
    const double kApproxPrecision = 1e-5;                   \
    Eigen::Matrix<type, rows, cols> lhs;                    \
    Eigen::Matrix<type, rows, cols> rhs;                    \
    if (rows == Eigen::Dynamic && cols == Eigen::Dynamic) { \
      lhs.resize(8, 3);                                     \
    } else if (rows == Eigen::Dynamic) {                    \
      lhs.resize(8, Eigen::NoChange);                       \
    } else if (cols == Eigen::Dynamic) {                    \
      lhs.resize(Eigen::NoChange, 3);                       \
    }                                                       \
    lhs.setRandom();                                        \
    YAML::Save(lhs, filename);                              \
    bool ok_string = YAML::Load(filename, &rhs);            \
    EXPECT_TRUE(ok_string);                                 \
    EXPECT_NEAR_EIGEN(lhs, rhs, kApproxPrecision);          \
    std::ofstream ofs(filename);                            \
    YAML::Save(lhs, &ofs);                                  \
    ofs.close();                                            \
    bool ok_load = YAML::Load(filename, &rhs);              \
    EXPECT_TRUE(ok_load);                                   \
    EXPECT_NEAR_EIGEN(lhs, rhs, kApproxPrecision);          \
  } while (0);

TEST(MaplabCommon, EigenYamlSerialization_Fixed_SizeDouble) {
  CHECK_TEST("Fixed size double", double, 7, 6, "double_fixed.yaml");
}
TEST(MaplabCommon, EigenYamlSerialization_Dynamic_SizeDouble) {
  CHECK_TEST(
      "Dynamic size double", double, Eigen::Dynamic, Eigen::Dynamic,
      "double_dynamic.yaml");
}
TEST(MaplabCommon, EigenYamlSerialization_Dynamic_SizeVectorDouble) {
  CHECK_TEST(
      "Dynamic vector col double", double, 1, Eigen::Dynamic,
      "double_dynveccol.yaml");
}
TEST(MaplabCommon, EigenYamlSerialization_Dynamic_SizeVectorRowDouble) {
  CHECK_TEST(
      "Dynamic vector row double", double, Eigen::Dynamic, 1,
      "double_dynvecrow.yaml");
}
TEST(MaplabCommon, EigenYamlSerialization_Fixed_SizeInt) {
  CHECK_TEST("Fixed size int", int, 7, 6, "int_fixed.yaml");
}
TEST(MaplabCommon, EigenYamlSerialization_Dynamic_SizeInt) {
  CHECK_TEST(
      "Dynamic size int", int, Eigen::Dynamic, Eigen::Dynamic,
      "int_dynamic.yaml");
}
TEST(MaplabCommon, EigenYamlSerialization_Dynamic_SizeVectorColInt) {
  CHECK_TEST(
      "Dynamic vector col int", int, 1, Eigen::Dynamic, "int_dynveccol.yaml");
}
TEST(MaplabCommon, EigenYamlSerialization_Dynamic_SizeVectorRowInt) {
  CHECK_TEST(
      "Dynamic vector row int", int, Eigen::Dynamic, 1, "int_dynvecrow.yaml");
}

TEST(MaplabCommon, EigenYamlSerialization_DetectEmptyFile) {
  Eigen::Matrix<double, 5, 4> rhs;
  std::string filename = "destroyed_file.yaml";
  std::ofstream ofs(filename);
  ofs << "";
  ofs.close();

  bool parse_ok = YAML::Load(filename, &rhs);
  EXPECT_FALSE(parse_ok);
}

TEST(MaplabCommon, EigenYamlSerialization_DetectDestroyedFile) {
  Eigen::Matrix<double, 5, 4> rhs;
  std::string filename = "destroyed_file.yaml";
  std::ofstream ofs(filename);
  ofs << "Invalid data to test the YAML parser.";
  ofs.close();

  std::ifstream ifs(filename);
  YAML::Parser parser(ifs);
  bool parse_ok = YAML::Load(filename, &rhs);
  EXPECT_FALSE(parse_ok);
}

TEST(MaplabCommon, EigenYamlSerialization_DeathFromWrongMatrixType) {
  Eigen::Matrix<double, 5, 4> lhs;
  Eigen::Matrix<double, 5, 3> rhs;
  lhs.setRandom();
  std::string filename = "death_from_wrong_matrix_type.yaml";
  YAML::Save(lhs, filename);
  EXPECT_DEATH(YAML::Load(filename, &rhs), "^");
}

}  // namespace common

MAPLAB_UNITTEST_ENTRYPOINT
