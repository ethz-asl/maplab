#ifndef MAPLAB_COMMON_BINARY_SERIALIZATION_H_
#define MAPLAB_COMMON_BINARY_SERIALIZATION_H_
#include <fstream>  // NOLINT
#include <limits>
#include <list>
#include <map>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <aslam/common/hash-id.h>
#include <glog/logging.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <maplab-common/test/serialization-macros.h>

namespace sm {
class HashId;
}  // namespace sm

namespace common {
template <class TYPE>
void Serialize(
    const TYPE& value, std::ostream* out,
    typename std::enable_if<std::is_integral<TYPE>::value>::type* = 0) {
  CHECK_NOTNULL(out);
  out->write(reinterpret_cast<const char*>(&value), sizeof(value));
}

template <class TYPE>
void Deserialize(
    TYPE* value, std::istream* in,
    typename std::enable_if<std::is_integral<TYPE>::value>::type* = 0) {
  CHECK_NOTNULL(value);
  CHECK_NOTNULL(in);
  in->read(reinterpret_cast<char*>(value), sizeof(*value));
  CHECK_EQ(in->gcount(), static_cast<std::streamsize>(sizeof(*value)));
}

template <class TYPE>
void Serialize(
    const TYPE& value, std::ostream* out,
    typename std::enable_if<std::is_floating_point<TYPE>::value>::type* = 0) {
  CHECK_NOTNULL(out);
  out->write(reinterpret_cast<const char*>(&value), sizeof(value));
}

template <class TYPE>
void Deserialize(
    TYPE* value, std::istream* in,
    typename std::enable_if<std::is_floating_point<TYPE>::value>::type* = 0) {
  CHECK_NOTNULL(value);
  CHECK_NOTNULL(in);
  in->read(reinterpret_cast<char*>(value), sizeof(*value));
  CHECK_EQ(in->gcount(), static_cast<std::streamsize>(sizeof(*value)));
}

void Serialize(const uint32_t& value, std::ostream* out);

void Deserialize(uint32_t* value, std::istream* in);

void Serialize(const std::string& value, std::ostream* out);

void Deserialize(std::string* value, std::istream* in);

void Serialize(
    const char* memory_start, unsigned int memory_block_length_bytes,
    std::ostream* out);

void Deserialize(
    char* memory_start, unsigned int memory_block_length_bytes,
    std::istream* in);

void Serialize(
    const unsigned char* memory_start, unsigned int memory_block_length_bytes,
    std::ostream* out);

void Deserialize(
    unsigned char* memory_start, unsigned int memory_block_length_bytes,
    std::istream* in);

void Serialize(const aslam::HashId& value, std::ostream* out);

void Deserialize(aslam::HashId* value, std::istream* in);

template <typename TYPEA, typename TYPEB>
void Serialize(const std::pair<TYPEA, TYPEB>& value, std::ostream* out) {
  CHECK_NOTNULL(out);
  Serialize(value.first, out);
  Serialize(value.second, out);
}

template <typename TYPEA, typename TYPEB>
void Deserialize(std::pair<TYPEA, TYPEB>* value, std::istream* in) {
  CHECK_NOTNULL(value);
  CHECK_NOTNULL(in);
  Deserialize(&value->first, in);
  Deserialize(&value->second, in);
}

template <typename TYPEA, typename TYPEB>
void Serialize(const std::map<TYPEA, TYPEB>& value, std::ostream* out) {
  CHECK_NOTNULL(out);
  uint32_t length = value.size();
  Serialize(length, out);
  for (const std::pair<TYPEA, TYPEB>& entry : value) {
    Serialize(entry, out);
  }
}

template <typename TYPEA, typename TYPEB>
void Deserialize(std::map<TYPEA, TYPEB>* value, std::istream* in) {
  CHECK_NOTNULL(value);
  CHECK_NOTNULL(in);
  value->clear();
  uint32_t length;
  Deserialize(&length, in);
  for (uint32_t i = 0; i < length; ++i) {
    std::pair<TYPEA, TYPEB> entry;
    Deserialize(&entry, in);
    value->insert(entry);
  }
}

template <class Scalar, int Rows, int Cols, int C, int D, int E>
void Serialize(
    const Eigen::Matrix<Scalar, Rows, Cols, C, D, E>& M, std::ostream* out) {
  CHECK_NOTNULL(out);
  int rows = M.rows();
  int cols = M.cols();
  CHECK_NE(0, rows);
  CHECK_NE(0, cols);

  Serialize(rows, out);
  Serialize(cols, out);

  Serialize(
      reinterpret_cast<const char*>(M.data()), sizeof(Scalar) * rows * cols,
      out);
}

template <class Scalar, int Rows, int Cols, int C, int D, int E>
void Deserialize(
    Eigen::Matrix<Scalar, Rows, Cols, C, D, E>* M, std::istream* in) {
  CHECK_NOTNULL(M);
  CHECK_NOTNULL(in);
  int rows, cols;
  Deserialize(&rows, in);
  Deserialize(&cols, in);
  CHECK_NE(0, rows);
  CHECK_NE(0, cols);

  CHECK_EQ(rows, Rows) << "Unexpected number of rows for fixed-sized type.";
  CHECK_EQ(cols, Cols) << "Unexpected number of columns for fixed-sized type.";

  Deserialize(
      reinterpret_cast<char*>(M->data()), sizeof(Scalar) * rows * cols, in);
}

template <class Scalar, int Cols, int C, int D, int E>
void Deserialize(
    Eigen::Matrix<Scalar, Eigen::Dynamic, Cols, C, D, E>* M, std::istream* in) {
  CHECK_NOTNULL(M);
  CHECK_NOTNULL(in);
  int rows, cols;
  Deserialize(&rows, in);
  Deserialize(&cols, in);
  CHECK_NE(0, rows);
  CHECK_NE(0, cols);

  CHECK_EQ(cols, Cols) << "Unexpected number of columns for fixed-sized type.";
  M->resize(rows, Eigen::NoChange);

  Deserialize(
      reinterpret_cast<char*>(M->data()), sizeof(Scalar) * rows * cols, in);
}

template <class Scalar, int Rows, int C, int D, int E>
void Deserialize(
    Eigen::Matrix<Scalar, Rows, Eigen::Dynamic, C, D, E>* M, std::istream* in) {
  CHECK_NOTNULL(M);
  CHECK_NOTNULL(in);
  int rows, cols;
  Deserialize(&rows, in);
  Deserialize(&cols, in);
  CHECK_NE(0, rows);
  CHECK_NE(0, cols);

  CHECK_EQ(rows, Rows) << "Unexpected number of rows for fixed-sized type.";
  M->resize(Eigen::NoChange, cols);

  Deserialize(
      reinterpret_cast<char*>(M->data()), sizeof(Scalar) * rows * cols, in);
}

template <class Scalar, int C, int D, int E>
void Deserialize(
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, C, D, E>* M,
    std::istream* in) {
  CHECK_NOTNULL(M);
  CHECK_NOTNULL(in);
  int rows, cols;
  Deserialize(&rows, in);
  Deserialize(&cols, in);
  CHECK_NE(0, rows);
  CHECK_NE(0, cols);

  M->resize(rows, cols);

  Deserialize(
      reinterpret_cast<char*>(M->data()), sizeof(Scalar) * rows * cols, in);
}

template <template <typename, typename> class CONTAINER, typename TYPE,
          typename ALLOCATOR>
void Serialize(const CONTAINER<TYPE, ALLOCATOR>& value, std::ostream* out) {
  CHECK_NOTNULL(out);
  uint32_t length = value.size();
  Serialize(length, out);
  for (const TYPE& entry : value) {
    Serialize(entry, out);
  }
}

template <template <typename, typename> class CONTAINER, typename TYPE,
          typename ALLOCATOR>
void Deserialize(CONTAINER<TYPE, ALLOCATOR>* value, std::istream* in) {
  CHECK_NOTNULL(value);
  CHECK_NOTNULL(in);
  value->clear();
  uint32_t length;
  Deserialize(&length, in);
  value->resize(length);
  for (TYPE& entry : *value) {
    Deserialize(&entry, in);
  }
}

}  // namespace common

#endif  // MAPLAB_COMMON_BINARY_SERIALIZATION_H_
