#ifndef MAPLAB_COMMON_READABLE_SERIALIZATION_H_
#define MAPLAB_COMMON_READABLE_SERIALIZATION_H_

#include <string>
#include <vector>

#include <Eigen/Dense>

namespace common {
namespace readable_serialization {

// Ignores comment lines starting with a hash ("#").
template <typename ScalarType, int Cols, int StorageOrder>
void deserialize(
    const std::string& file_name,
    Eigen::Matrix<ScalarType, Eigen::Dynamic, Cols, StorageOrder>* matrix);

template <typename Type>
void deserialize(const std::string& file_name, std::vector<Type>* vector);

template <typename Type, int Cols, int Majority>
void deserializeIgnoringNFirstCharactersPerLine(
    const std::string& file_name, const size_t n_characters_to_ignore,
    Eigen::Matrix<Type, Eigen::Dynamic, Cols, Majority>* matrix);

template <typename Type>
void deserializeIgnoringNFirstCharactersPerLine(
    const std::string& file_name, const size_t n_characters_to_ignore,
    std::vector<std::vector<Type>>* file_contents);

}  // namespace readable_serialization
}  // namespace common

#include "./readable-serialization-inl.h"

#endif  // MAPLAB_COMMON_READABLE_SERIALIZATION_H_
