#ifndef MAPLAB_COMMON_FILE_LOGGER_INL_H_
#define MAPLAB_COMMON_FILE_LOGGER_INL_H_

#include <fstream>  // NOLINT
#include <memory>
#include <mutex>
#include <string>

#include <Eigen/Core>

namespace common {

template<typename DataType>
const FileLogger& FileLogger::operator<<(const DataType& object) const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  CHECK(isOpen());
  *file_handle_ << object;
  return *this;
}

template <typename DataType>
void FileLogger::writeDataWithDelimiterAndNewLine(
    const std::string& /*delimiter*/, const DataType& object) {
  // Base case -- don't need to print a delimiter.
  *file_handle_ << object << std::endl;
}

template <typename FirstDataType, typename... DataTypes>
void FileLogger::writeDataWithDelimiterAndNewLine(
    const std::string& delimiter, const FirstDataType& first_object,
    const DataTypes&... objects) {
  *file_handle_ << first_object << delimiter;
  writeDataWithDelimiterAndNewLine(delimiter, objects...);
}

// Eigen overloads.
template <typename Scalar, int Rows, int Cols, int C, int D, int E>
void FileLogger::writeDataWithDelimiterAndNewLine(
    const std::string& delimiter,
    const Eigen::Matrix<Scalar, Rows, Cols, C, D, E>& matrix) {
  const int rows = Rows == Eigen::Dynamic ? matrix.rows() : Rows;
  const int cols = Cols == Eigen::Dynamic ? matrix.cols() : Cols;
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      if (!(i == 0 && j == 0)) {
        // Don't need to put a delimiter before the first value and don't need
        // to put a delimiter after the last value.
        *file_handle_ << delimiter;
      }
      *file_handle_ << matrix(i, j);
    }
  }
  *file_handle_ << std::endl;
}

template <typename Scalar, int Rows, int Cols, int C, int D, int E,
          typename... DataTypes>
void FileLogger::writeDataWithDelimiterAndNewLine(
    const std::string& delimiter,
    const Eigen::Matrix<Scalar, Rows, Cols, C, D, E>& matrix,
    const DataTypes&... objects) {
  const int rows = Rows == Eigen::Dynamic ? matrix.rows() : Rows;
  const int cols = Cols == Eigen::Dynamic ? matrix.cols() : Cols;
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      *file_handle_ << matrix(i, j) << delimiter;
    }
  }
  writeDataWithDelimiterAndNewLine(delimiter, objects...);
}

template <typename Scalar, int Rows, int Cols, int C, int D, int E, int F,
          int G, bool H>
void FileLogger::writeDataWithDelimiterAndNewLine(
    const std::string& delimiter,
    const Eigen::Block<const Eigen::Matrix<Scalar, Rows, Cols, C, D, E>, G, F,
                       H>& matrix) {
  const int rows = Rows == Eigen::Dynamic ? matrix.rows() : Rows;
  const int cols = Cols == Eigen::Dynamic ? matrix.cols() : Cols;
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      if (!(i == 0 && j == 0)) {
        // Don't need to put a delimiter before the first value and don't need
        // to put a delimiter after the last value.
        *file_handle_ << delimiter;
      }
      *file_handle_ << matrix(i, j);
    }
  }
  *file_handle_ << std::endl;
}

template <typename Scalar, int Rows, int Cols, int C, int D, int E, int F,
          int G, bool H, typename... DataTypes>
void FileLogger::writeDataWithDelimiterAndNewLine(
    const std::string& delimiter,
    const Eigen::Block<const Eigen::Matrix<Scalar, Rows, Cols, C, D, E>, G, F,
                       H>& matrix,
    const DataTypes&... objects) {
  const int rows = Rows == Eigen::Dynamic ? matrix.rows() : Rows;
  const int cols = Cols == Eigen::Dynamic ? matrix.cols() : Cols;
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      *file_handle_ << matrix(i, j) << delimiter;
    }
  }
  writeDataWithDelimiterAndNewLine(delimiter, objects...);
}

template <typename Scalar, int Options>
void FileLogger::writeDataWithDelimiterAndNewLine(
    const std::string& delimiter,
    const Eigen::Quaternion<Scalar, Options>& quaternion) {
  writeDataWithDelimiterAndNewLine(
      delimiter, quaternion.x(), quaternion.y(), quaternion.z(),
      quaternion.w());
}

template <typename Scalar, int Options, typename... DataTypes>
void FileLogger::writeDataWithDelimiterAndNewLine(
    const std::string& delimiter,
    const Eigen::Quaternion<Scalar, Options>& quaternion,
    DataTypes... objects) {
  writeDataWithDelimiterAndNewLine(
      delimiter, quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w(),
      objects...);
}

}  // namespace common

#endif  // MAPLAB_COMMON_FILE_LOGGER_INL_H_
