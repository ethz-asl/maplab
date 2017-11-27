#ifndef MAPLAB_COMMON_READABLE_SERIALIZATION_INL_H_
#define MAPLAB_COMMON_READABLE_SERIALIZATION_INL_H_

#include <fstream>  // NOLINT
#include <string>
#include <vector>

#include <glog/logging.h>

namespace common {
namespace readable_serialization {

template <typename Type, int Cols, int StorageOrder>
void deserialize(
    const std::string& file_name,
    Eigen::Matrix<Type, Eigen::Dynamic, Cols, StorageOrder>* matrix) {
  CHECK_NOTNULL(matrix);
  deserializeIgnoringNFirstCharactersPerLine(file_name, 0u, matrix);
}

template <typename Type>
void deserialize(const std::string& file_name, std::vector<Type>* vector) {
  CHECK_NOTNULL(vector)->clear();

  std::vector<std::vector<Type>> file_contents;
  deserializeIgnoringNFirstCharactersPerLine(file_name, 0u, &file_contents);

  for (const std::vector<Type> line : file_contents) {
    vector->reserve(vector->size() + line.size());
    vector->insert(vector->end(), line.begin(), line.end());
  }
}

template <typename Type, int Cols, int Majority>
void deserializeIgnoringNFirstCharactersPerLine(
    const std::string& file_name, const size_t n_characters_to_ignore,
    Eigen::Matrix<Type, Eigen::Dynamic, Cols, Majority>* matrix) {
  CHECK_NOTNULL(matrix);

  std::vector<std::vector<Type>> file_contents;
  deserializeIgnoringNFirstCharactersPerLine(
      file_name, n_characters_to_ignore, &file_contents);

  const int rows = file_contents.size();
  CHECK_GT(rows, 0);
  const int cols = file_contents[0].size();
  if (Cols == Eigen::Dynamic) {
    matrix->resize(rows, cols);
  } else {
    matrix->resize(rows, Eigen::NoChange);
    CHECK_EQ(cols, Cols);
  }
  for (int r = 0; r < rows; ++r) {
    CHECK_EQ(static_cast<int>(file_contents[r].size()), cols) << r;
    for (int c = 0; c < cols; ++c) {
      (*matrix)(r, c) = file_contents[r][c];
    }
  }
}

template <typename Type>
void deserializeIgnoringNFirstCharactersPerLine(
    const std::string& file_name, const size_t n_characters_to_ignore,
    std::vector<std::vector<Type>>* file_contents) {
  CHECK_NOTNULL(file_contents)->clear();
  std::ifstream in(file_name);
  CHECK(in.is_open()) << "Could not read from " << file_name << ".";

  while (!in.eof()) {
    std::string line;
    getline(in, line);
    if (in.eof()) {
      break;
    }
    if (line[0] == '#') {
      continue;
    }
    if (n_characters_to_ignore > 0u) {
      line = line.substr(n_characters_to_ignore);
    }
    std::istringstream iss(line);
    std::vector<Type> line_contents;
    while (!iss.eof()) {
      line_contents.push_back(0.);
      iss >> line_contents.back();
    }
    if (iss.fail()) {
      line_contents.pop_back();
    }
    file_contents->emplace_back(line_contents);
  }
}

}  // namespace readable_serialization
}  // namespace common

#endif  // MAPLAB_COMMON_READABLE_SERIALIZATION_INL_H_
