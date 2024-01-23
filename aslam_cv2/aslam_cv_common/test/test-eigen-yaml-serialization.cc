#include <string>
#include <vector>

#include <Eigen/Core>
#include <eigen-checks/gtest.h>
#include <yaml-cpp/yaml.h>

#include <aslam/common/entrypoint.h>
#include <aslam/common/yaml-serialization.h>

namespace aslam {
#define CHECK_TEST(testname, type, rows, cols, filename)        \
  do {                                                          \
    const double kApproxPrecision = 1e-5;                       \
    Eigen::Matrix<type, rows, cols> lhs;                        \
    Eigen::Matrix<type, rows, cols> rhs;                        \
    if (rows == Eigen::Dynamic && cols == Eigen::Dynamic) {     \
      lhs.resize(8, 3);                                         \
    } else if (rows == Eigen::Dynamic) {                        \
      lhs.resize(8, Eigen::NoChange);                           \
    } else if (cols == Eigen::Dynamic) {                        \
      lhs.resize(Eigen::NoChange, 3);                           \
    }                                                           \
    const int num_rows = (rows == Eigen::Dynamic) ? 8 : rows;   \
    const int num_cols = (cols == Eigen::Dynamic) ? 3 : cols;   \
                                                                \
    for (int row_idx = 0; row_idx < num_rows; ++row_idx) {      \
      for (int col_idx = 0; col_idx < num_cols; ++col_idx) {    \
        lhs(row_idx, col_idx) =                                 \
            static_cast<type>(row_idx * num_cols + col_idx) *   \
            static_cast<type>(M_PI);                            \
      }                                                         \
    }                                                           \
    YAML::Save(lhs, filename);                                  \
    bool ok_string = YAML::Load(filename, &rhs);                \
    EXPECT_TRUE(ok_string);                                     \
    EXPECT_TRUE(EIGEN_MATRIX_NEAR(lhs, rhs, kApproxPrecision)); \
    std::ofstream ofs(filename);                                \
    YAML::Save(lhs, &ofs);                                      \
    ofs.close();                                                \
    bool ok_load = YAML::Load(filename, &rhs);                  \
    EXPECT_TRUE(ok_load);                                       \
    EXPECT_TRUE(EIGEN_MATRIX_NEAR(lhs, rhs, kApproxPrecision)); \
  } while (0);

TEST(EigenYamlSerialization, EigenYamlSerialization_Fixed_SizeDouble) {
  CHECK_TEST("Fixed size double", double, 7, 6, "double_fixed.yaml");
}
TEST(EigenYamlSerialization, EigenYamlSerialization_Dynamic_SizeDouble) {
  CHECK_TEST(
      "Dynamic size double", double, Eigen::Dynamic, Eigen::Dynamic,
      "double_dynamic.yaml");
}
TEST(EigenYamlSerialization, EigenYamlSerialization_Dynamic_SizeVectorDouble) {
  CHECK_TEST(
      "Dynamic vector col double", double, 1, Eigen::Dynamic,
      "double_dynveccol.yaml");
}
TEST(
    EigenYamlSerialization,
    EigenYamlSerialization_Dynamic_SizeVectorRowDouble) {
  CHECK_TEST(
      "Dynamic vector row double", double, Eigen::Dynamic, 1,
      "double_dynvecrow.yaml");
}
TEST(EigenYamlSerialization, EigenYamlSerialization_Fixed_SizeInt) {
  CHECK_TEST("Fixed size int", int, 7, 6, "int_fixed.yaml");
}
TEST(EigenYamlSerialization, EigenYamlSerialization_Dynamic_SizeInt) {
  CHECK_TEST(
      "Dynamic size int", int, Eigen::Dynamic, Eigen::Dynamic,
      "int_dynamic.yaml");
}
TEST(EigenYamlSerialization, EigenYamlSerialization_Dynamic_SizeVectorColInt) {
  CHECK_TEST(
      "Dynamic vector col int", int, 1, Eigen::Dynamic, "int_dynveccol.yaml");
}
TEST(EigenYamlSerialization, EigenYamlSerialization_Dynamic_SizeVectorRowInt) {
  CHECK_TEST(
      "Dynamic vector row int", int, Eigen::Dynamic, 1, "int_dynvecrow.yaml");
}

TEST(EigenYamlSerialization, EigenYamlSerialization_DetectEmptyFile) {
  Eigen::Matrix<double, 5, 4> rhs;
  std::string filename = "destroyed_file.yaml";
  std::ofstream ofs(filename);
  ofs << "";
  ofs.close();

  bool parse_ok = YAML::Load(filename, &rhs);
  EXPECT_FALSE(parse_ok);
}

TEST(EigenYamlSerialization, EigenYamlSerialization_DetectDestroyedFile) {
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

TEST(EigenYamlSerialization, EigenYamlSerialization_DeathFromWrongMatrixType) {
  Eigen::Matrix<double, 5, 4> lhs;
  Eigen::Matrix<double, 5, 3> rhs;
  lhs.setRandom();
  std::string filename = "death_from_wrong_matrix_type.yaml";
  YAML::Save(lhs, filename);
  EXPECT_FALSE(YAML::Load(filename, &rhs));
}

}  // namespace aslam

ASLAM_UNITTEST_ENTRYPOINT
