#ifndef MAPLAB_COMMON_FILE_LOGGER_H_
#define MAPLAB_COMMON_FILE_LOGGER_H_

#include <fstream>  // NOLINT
#include <memory>
#include <mutex>
#include <string>

#include <Eigen/Core>
#include <glog/logging.h>

namespace common {

class FileLogger {
 public:
  explicit FileLogger(const std::string& filename);
  ~FileLogger();

  bool isOpen() const;
  void closeFile() const;
  void flushBuffer();

  template <typename DataType>
  const FileLogger& operator<<(const DataType& object) const;
  const FileLogger& operator<<(std::ostream& (*object)(std::ostream&)) const;
  const FileLogger& operator<<(std::ios_base& (*object)(std::ios_base&)) const;

  // Convenience functions to output multiple values separated by a provided
  // delimiter.
  template <typename DataType>
  void writeDataWithDelimiterAndNewLine(
      const std::string& delimiter, const DataType& object);
  template <typename FirstDataType, typename... DataTypes>
  void writeDataWithDelimiterAndNewLine(
      const std::string& delimiter, const FirstDataType& first_object,
      const DataTypes&... objects);

  // Eigen overloads.
  template <typename Scalar, int Rows, int Cols, int C, int D, int E>
  void writeDataWithDelimiterAndNewLine(
      const std::string& delimiter,
      const Eigen::Matrix<Scalar, Rows, Cols, C, D, E>& matrix);

  template <typename Scalar, int Rows, int Cols, int C, int D, int E,
            typename... DataTypes>
  void writeDataWithDelimiterAndNewLine(
      const std::string& delimiter,
      const Eigen::Matrix<Scalar, Rows, Cols, C, D, E>& matrix,
      const DataTypes&... objects);

  template <typename Scalar, int Rows, int Cols, int C, int D, int E, int F,
            int G, bool H>
  void writeDataWithDelimiterAndNewLine(
      const std::string& delimiter,
      const Eigen::Block<const Eigen::Matrix<Scalar, Rows, Cols, C, D, E>, G, F,
                         H>& matrix);

  template <typename Scalar, int Rows, int Cols, int C, int D, int E, int F,
            int G, bool H, typename... DataTypes>
  void writeDataWithDelimiterAndNewLine(
      const std::string& delimiter,
      const Eigen::Block<const Eigen::Matrix<Scalar, Rows, Cols, C, D, E>, G, F,
                         H>& matrix,
      const DataTypes&... objects);

  template <typename Scalar, int Options>
  void writeDataWithDelimiterAndNewLine(
      const std::string& delimiter,
      const Eigen::Quaternion<Scalar, Options>& quaternion);

  template <typename Scalar, int Options, typename... DataTypes>
  void writeDataWithDelimiterAndNewLine(
      const std::string& delimiter,
      const Eigen::Quaternion<Scalar, Options>& quaternion,
      DataTypes... objects);

 private:
  mutable std::recursive_mutex mutex_;
  std::unique_ptr<std::ofstream> file_handle_;
};

}  // namespace common
#include "maplab-common/file-logger-inl.h"
#endif  // MAPLAB_COMMON_FILE_LOGGER_H_
