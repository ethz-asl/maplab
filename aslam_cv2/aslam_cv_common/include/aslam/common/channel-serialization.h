#ifndef ASLAM_CHANNEL_SERIALIZATION_H_
#define ASLAM_CHANNEL_SERIALIZATION_H_

#include <cstdint>
#include <iostream>
#include <sstream>
#include <string>
#include <type_traits>

#include <glog/logging.h>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>

namespace aslam {
namespace internal {

struct HeaderInformation {
  uint32_t rows;
  uint32_t cols;
  uint32_t depth;
  uint32_t channels; ///< Needed for opencv support
  size_t size() const;
  bool serializeToBuffer(char* buffer, size_t offset) const;
  bool deSerializeFromBuffer(const char* const bufferIn, size_t offset);
};

template<typename Scalar>
void makeHeaderInformation(int rows, int cols, int channels,
                           HeaderInformation* headerInformation) {
  CHECK_NOTNULL(headerInformation);
  headerInformation->rows = rows;
  headerInformation->cols = cols;
  headerInformation->channels = channels;
  headerInformation->depth = cv::DataType<Scalar>::depth;
}

template<typename Scalar>
bool serializeToString(const char* const matrixData,
                       int rows, int cols, int channels, std::string* string) {
  CHECK_GE(rows, 0);
  CHECK_GE(cols, 0);
  CHECK_NOTNULL(string);
  HeaderInformation header;
  makeHeaderInformation<Scalar>(rows, cols, channels, &header);
  size_t matrixSize = sizeof(Scalar) * rows * cols * channels;
  size_t totalSize = matrixSize + header.size();

  CHECK_GT(totalSize, 0u);

  string->resize(totalSize);
  char* buffer = &(*string)[0];
  bool success = header.serializeToBuffer(buffer, 0);
  if (!success) {
    LOG(ERROR) << "Failed to serialize header";
    return false;
  }
  size_t offset = header.size();
  memcpy(buffer + offset, matrixData, matrixSize);
  return true;
}

template<typename Scalar>
bool serializeToBuffer(const char* const matrixData, int rows, int cols,
                       int channels, char** buffer, size_t* totalSize) {
  CHECK_NOTNULL(totalSize);
  CHECK_NOTNULL(buffer);
  HeaderInformation header;
  makeHeaderInformation<Scalar>(rows, cols, channels, &header);
  size_t matrixSize = sizeof(Scalar) * rows * cols * channels;
  *totalSize = matrixSize + header.size();

  *buffer = new char[*totalSize];
  bool success = header.serializeToBuffer(*buffer, 0);
  if (!success) {
    delete[] *buffer;
    *buffer = nullptr;
    return false;
  }
  size_t offset = header.size();
  memcpy(*buffer + offset, matrixData, matrixSize);
  return true;
}

template<typename Scalar, int ROWS, int COLS>
bool serializeToBuffer(const Eigen::Matrix<Scalar, ROWS, COLS>& matrix,
                       char** buffer, size_t* size) {
  const char* const matrixData = reinterpret_cast<const char*>(matrix.data());
  // Eigen matrices have only one channel
  constexpr int numChannels = 1;
  return serializeToBuffer<Scalar>(matrixData, matrix.rows(), matrix.cols(),
                                   numChannels, buffer, size);
}

template<typename Scalar, int ROWS, int COLS>
bool serializeToString(const Eigen::Matrix<Scalar, ROWS, COLS>& matrix,
                       std::string* string) {
  const char* const matrixData = reinterpret_cast<const char*>(matrix.data());
  // Eigen matrices have only one channel
  constexpr int numChannels = 1;
  return serializeToString<Scalar>(matrixData, matrix.rows(), matrix.cols(),
                                   numChannels, string);
}

template<typename Scalar, int ROWS, int COLS>
bool serializeToBuffer(const std::vector<Eigen::Matrix<Scalar, ROWS, COLS>>& matrices,
                       char** buffer, size_t* size) {
  CHECK_NOTNULL(size);
  CHECK_NOTNULL(buffer);

  // Eigen matrices have only one channel
  constexpr int channels = 1;

  // Calculate total size
  const size_t repeat = matrices.size();
  *size = sizeof(repeat);

  HeaderInformation header;
  for (size_t i = 0; i < matrices.size(); i++) {
    const int rows = matrices[i].rows();
    const int cols = matrices[i].cols();
    const size_t matrixSize = sizeof(Scalar) * rows * cols * channels;
    *size += matrixSize + header.size();
  }

  *buffer = new char[*size];

  memcpy(*buffer, &repeat, sizeof(repeat));
  size_t offset = sizeof(repeat);

  for (size_t i = 0; i < matrices.size(); i++) {
    const int rows = matrices[i].rows();
    const int cols = matrices[i].cols();

    makeHeaderInformation<Scalar>(rows, cols, channels, &header);
    bool success = header.serializeToBuffer(*buffer, offset);
    if (!success) {
      delete[] *buffer;
      *buffer = nullptr;
      return false;
    }
    offset += header.size();

    size_t matrixSize = sizeof(Scalar) * rows * cols * channels;
    const char* const matrixData = reinterpret_cast<const char*>(matrices[i].data());
    memcpy(*buffer + offset, matrixData, matrixSize);
    offset += matrixSize;
  }

  return true;
}

template<typename Scalar, int ROWS, int COLS>
bool serializeToString(const std::vector<Eigen::Matrix<Scalar, ROWS, COLS>>& matrices,
                       std::string* string) {
  CHECK_NOTNULL(string);

  // Eigen matrices have only one channel
  constexpr int channels = 1;

  // Calculate total size
  const size_t repeat = matrices.size();
  size_t size = sizeof(repeat);

  HeaderInformation header;
  for (size_t i = 0; i < matrices.size(); i++) {
    const int rows = matrices[i].rows();
    const int cols = matrices[i].cols();
    const size_t matrixSize = sizeof(Scalar) * rows * cols * channels;
    size += matrixSize + header.size();
  }

  string->resize(size);
  char* buffer = &(*string)[0];

  memcpy(buffer, &repeat, sizeof(repeat));
  size_t offset = sizeof(repeat);

  for (size_t i = 0; i < matrices.size(); i++) {
    const int rows = matrices[i].rows();
    const int cols = matrices[i].cols();

    makeHeaderInformation<Scalar>(rows, cols, channels, &header);
    bool success = header.serializeToBuffer(buffer, offset);
    if (!success) {
      LOG(ERROR) << "Failed to serialize header";
      return false;
    }
    offset += header.size();

    size_t matrixSize = sizeof(Scalar) * rows * cols * channels;
    const char* const matrixData = reinterpret_cast<const char*>(matrices[i].data());
    memcpy(buffer + offset, matrixData, matrixSize);
    offset += matrixSize;
  }

  return true;
}

template<typename Scalar, int ROWS, int COLS>
bool deSerializeFromBuffer(const char* const buffer, size_t size,
                           Eigen::Matrix<Scalar, ROWS, COLS>* matrix) {
  CHECK_NOTNULL(matrix);
  HeaderInformation header;
  CHECK_GE(size, header.size());
  bool success = header.deSerializeFromBuffer(buffer, 0);
  if (!success) {
    LOG(ERROR) << "Failed to deserialize header from string: " <<
        std::string(buffer, size);
    return false;
  }
  if (ROWS != Eigen::Dynamic) {
    CHECK_EQ(header.rows, static_cast<uint32_t>(ROWS));
  }
  if (COLS != Eigen::Dynamic) {
    CHECK_EQ(header.cols, static_cast<uint32_t>(COLS));
  }
  CHECK_EQ(header.depth, cv::DataType<Scalar>::depth);
  CHECK_EQ(1u, header.channels) << "Eigen matrices must have one channel.";

  if (ROWS == Eigen::Dynamic && COLS == Eigen::Dynamic) {
    matrix->resize(header.rows, header.cols);
  } else if (ROWS == Eigen::Dynamic && COLS != Eigen::Dynamic) {
    matrix->resize(header.rows, Eigen::NoChange);
  } else if (ROWS != Eigen::Dynamic && COLS == Eigen::Dynamic) {
    matrix->resize(Eigen::NoChange, header.cols);
  }

  size_t matrix_size = sizeof(Scalar) * matrix->rows() * matrix->cols();
  size_t total_size = matrix_size + header.size();
  CHECK_EQ(size, total_size);
  memcpy(matrix->data(), buffer + header.size(), matrix_size);
  return true;
}

template<typename Scalar, int ROWS, int COLS>
bool deSerializeFromString(const std::string& string,
                           Eigen::Matrix<Scalar, ROWS, COLS>* matrix) {
  CHECK_NOTNULL(matrix);
  return deSerializeFromBuffer(string.data(), string.size(), matrix);
}

template<typename Scalar, int ROWS, int COLS>
bool deSerializeFromBuffer(const char* const buffer, size_t size,
                           std::vector<Eigen::Matrix<Scalar, ROWS, COLS>>* matrices) {
  CHECK_NOTNULL(matrices);

  size_t repeat;
  memcpy(&repeat, buffer, sizeof(repeat));
  size_t offset = sizeof(repeat);

  matrices->resize(repeat);
  for (size_t i = 0; i < repeat; i++) {
    HeaderInformation header;
    bool success = header.deSerializeFromBuffer(buffer, offset);
    if (!success) {
      LOG(ERROR) << "Failed to deserialize header from string: " <<
          std::string(buffer, size);
      return false;
    }
    offset += header.size();

    if (ROWS != Eigen::Dynamic) {
      CHECK_EQ(header.rows, static_cast<uint32_t>(ROWS));
    }
    if (COLS != Eigen::Dynamic) {
      CHECK_EQ(header.cols, static_cast<uint32_t>(COLS));
    }
    CHECK_EQ(header.depth, cv::DataType<Scalar>::depth);
    CHECK_EQ(1u, header.channels) << "Eigen matrices must have one channel.";

    if (ROWS == Eigen::Dynamic && COLS == Eigen::Dynamic) {
      (*matrices)[i].resize(header.rows, header.cols);
    } else if (ROWS == Eigen::Dynamic && COLS != Eigen::Dynamic) {
      (*matrices)[i].resize(header.rows, Eigen::NoChange);
    } else if (ROWS != Eigen::Dynamic && COLS == Eigen::Dynamic) {
      (*matrices)[i].resize(Eigen::NoChange, header.cols);
    }

    size_t matrix_size = sizeof(Scalar) * (*matrices)[i].rows() * (*matrices)[i].cols();
    memcpy((*matrices)[i].data(), buffer + offset, matrix_size);
    offset += matrix_size;
  }

  return true;
}

template<typename Scalar, int ROWS, int COLS>
bool deSerializeFromString(const std::string& string,
                           std::vector<Eigen::Matrix<Scalar, ROWS, COLS>>* matrices) {
  CHECK_NOTNULL(matrices);
  return deSerializeFromBuffer(string.data(), string.size(), matrices);
}

bool serializeToString(const cv::Mat& image,
                       std::string* string);

bool deSerializeFromString(const std::string& string,
                           cv::Mat* image);

bool deSerializeFromBuffer(const char* const buffer, size_t size,
                           cv::Mat* image);

bool serializeToBuffer(const cv::Mat& matrix,
                       char** buffer, size_t* size);

template<typename Scalar>
bool serializeToString(const Scalar& value, std::string* string) {
  CHECK_NOTNULL(string);
  std::string value_string = std::to_string(value);
  string->swap(value_string);
  return true;
}

template<typename Scalar>
bool deSerializeFromString(const std::string& string,
                           Scalar* value,
                           typename std::enable_if<std::is_integral<Scalar>::value>::type* = 0) {
  CHECK_NOTNULL(value);
  CHECK(!string.empty());
  (*value) = static_cast<Scalar>(std::stoll(string));
  return true;
}

template<typename Scalar>
bool deSerializeFromString(const std::string& string,
                           Scalar* value,
                           typename std::enable_if<std::is_floating_point<Scalar>::value>::type*
                             = 0) {
  CHECK_NOTNULL(value);
  CHECK(!string.empty());
  (*value) = static_cast<Scalar>(std::stod(string));
  return true;
}

template<typename Scalar>
bool serializeToBuffer(const Scalar& value, char** buffer, size_t* size) {
  CHECK_NOTNULL(buffer);
  CHECK_NOTNULL(size);
  *size = sizeof(Scalar);
  *buffer = new char[*size];
  memcpy(*buffer, &value, *size);
  return true;
}
template<typename Scalar>
bool deSerializeFromBuffer(const char* const buffer, size_t size, Scalar* value) {
  CHECK_NOTNULL(value);
  CHECK_EQ(size, sizeof(Scalar));
  memcpy(value, buffer, size);
  return true;
}

}  // namespace internal
}  // namespace aslam

#endif  // ASLAM_CHANNEL_SERIALIZATION_H_
