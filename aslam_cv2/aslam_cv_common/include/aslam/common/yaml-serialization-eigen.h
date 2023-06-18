#ifndef ASLAM_CV_COMMON_EIGEN_YAML_SERIALIZATION_H_
#define ASLAM_CV_COMMON_EIGEN_YAML_SERIALIZATION_H_

#include <Eigen/Core>
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

namespace YAML {  // This has to be in the same namespace as the Emitter.
// yaml serialization helper function for the Eigen3 Matrix object.
// The matrix is a base class for dense matrices.
// http://eigen.tuxfamily.org/dox-devel/TutorialMatrixClass.html
template <class Scalar_, int A_, int B_, int C_, int D_, int E_>
struct convert<Eigen::Matrix<Scalar_, A_, B_, C_, D_, E_> > {
  template <class Scalar, int A, int B, int C, int D, int E>
  static Node encode(const Eigen::Matrix<Scalar, A, B, C, D, E>& M) {
    Node node;
    typedef typename Eigen::Matrix<Scalar, A, B, C, D, E>::Index IndexType;
    IndexType rows = M.rows();
    IndexType cols = M.cols();
    node["rows"] = rows;
    node["cols"] = cols;
    CHECK_GT(rows, 0);
    CHECK_GT(cols, 0);
    for (IndexType i = 0; i < rows; ++i) {
      if (cols > 1) {
        YAML::Emitter ith_row;
        ith_row << YAML::Flow;
        ith_row << YAML::BeginSeq;
        for (IndexType j = 0; j < cols; ++j) {
          ith_row << M(i, j);
        }
        ith_row << YAML::EndSeq;
        CHECK(ith_row.good()) << "Emitter error: " << ith_row.GetLastError();
        node["data"].push_back(YAML::Load(ith_row.c_str()));
      } else {
        node["data"].push_back(M(i, 0));
      }
    }
    return node;
  }

  template <class Scalar, int A, int B, int C, int D, int E>
  static bool decodeEigenMatrixData(
      const Node& node, const int rows, const int cols,
      Eigen::Matrix<Scalar, A, B, C, D, E>& M) {
    typedef typename Eigen::Matrix<Scalar, A, B, C, D, E>::Index IndexType;
    if (!node.IsSequence()) {
      LOG(ERROR) << "The matrix data is not a sequence.";
      return false;
    }

    if (rows == 0 && cols == 0) {
      return true;
    }
    YAML::const_iterator it = node.begin();
    YAML::const_iterator it_end = node.end();

    // If the 2D matrix is stored as a linear array:
    if (static_cast<int>(node.size()) == rows * cols) {
      for (IndexType i = 0; i < rows; ++i) {
        for (IndexType j = 0; j < cols; ++j) {
          CHECK(it != it_end);
          if (it->IsSequence()) {
            CHECK(it->size() != 1u) << "Wrong dimension of Eigen-type matrix!";
            M(i, j) = it->begin()->as<Scalar>();
          } else {
            M(i, j) = it->as<Scalar>();
          }
          ++it;
        }
      }
    } else if (static_cast<int>(node.size()) == rows) {
      // If the 2D matrix is stored as a 2D, row-major matrix.
      for (IndexType i = 0; i < rows; ++i) {
        CHECK(it != it_end);

        // If there is only one column, the rows might not contain a sequence,
        // so we cannot initialize the iterators.

        if (it->IsSequence()) {
          YAML::const_iterator col_it = it->begin();
          YAML::const_iterator col_it_end = it->end();
          CHECK(static_cast<int>(it->size()) == cols)
              << "Wrong dimension of Eigen-type matrix! Expected " << cols
              << " columns, but provided " << it->size();

          for (IndexType j = 0; j < cols; ++j) {
            CHECK(col_it != col_it_end);
            M(i, j) = col_it->as<Scalar>();
            ++col_it;
          }
        } else {
          CHECK(cols == 1) << "Wrong dimension of Eigen-type matrix! Expected "
                           << cols << " columns, but provided " << 1;
          M(i, 0) = it->as<Scalar>();
        }
        ++it;
      }
    } else {
      LOG(ERROR) << "Wrong dimension (number of rows) of Eigen-type "
                    "matrix!";
      return false;
    }
    return true;
  }

  template <class Scalar, int A, int B, int C, int D, int E>
  static bool decode(
      const Node& node, Eigen::Matrix<Scalar, A, B, C, D, E>& M) {
    if (!node.IsMap()) {
      LOG(ERROR)
          << "Unable to get parse the matrix because the node is not a map.";
      return false;
    }

    typedef typename Eigen::Matrix<Scalar, A, B, C, D, E>::Index IndexType;
    IndexType rows = node["rows"].as<IndexType>();
    IndexType cols = node["cols"].as<IndexType>();

    if (rows != A || cols != B) {
      LOG(ERROR) << "The matrix is the wrong size (rows, cols). Wanted: (" << A
                 << "," << B << "), got (" << rows << ", " << cols << ")";
      return false;
    }

    return decodeEigenMatrixData(node["data"], rows, cols, M);
  }

  template <class Scalar, int B, int C, int D, int E>
  static bool decode(
      const Node& node, Eigen::Matrix<Scalar, Eigen::Dynamic, B, C, D, E>& M) {
    if (!node.IsMap()) {
      LOG(ERROR)
          << "Unable to get parse the matrix because the node is not a map.";
      return false;
    }

    typedef typename Eigen::Matrix<Scalar, Eigen::Dynamic, B, C, D, E>::Index
        IndexType;
    IndexType rows = node["rows"].as<IndexType>();
    IndexType cols = node["cols"].as<IndexType>();

    if (cols != B) {
      LOG(ERROR) << "The matrix is the wrong size (rows, cols). Wanted: ("
                 << rows << "," << B << "), got (" << rows << ", " << cols
                 << ")";
      return false;
    }

    M.resize(rows, Eigen::NoChange);

    return decodeEigenMatrixData(node["data"], rows, cols, M);
  }

  template <class Scalar, int A, int C, int D, int E>
  static bool decode(
      const Node& node, Eigen::Matrix<Scalar, A, Eigen::Dynamic, C, D, E>& M) {
    if (!node.IsMap()) {
      LOG(ERROR)
          << "Unable to get parse the matrix because the node is not a map.";
      return false;
    }

    typedef typename Eigen::Matrix<Scalar, A, Eigen::Dynamic, C, D, E>::Index
        IndexType;
    IndexType rows = node["rows"].as<IndexType>();
    IndexType cols = node["cols"].as<IndexType>();

    if (rows != A) {
      LOG(ERROR) << "The matrix is the wrong size (rows, cols). Wanted: (" << A
                 << "," << cols << "), got (" << rows << ", " << cols << ")";
      return false;
    }

    M.resize(Eigen::NoChange, cols);

    return decodeEigenMatrixData(node["data"], rows, cols, M);
  }

  template <class Scalar, int C, int D, int E>
  static bool decode(
      const Node& node,
      Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, C, D, E>& M) {
    if (!node.IsMap()) {
      LOG(ERROR)
          << "Unable to get parse the matrix because the node is not a map.";
      return false;
    }

    typedef typename Eigen::Matrix<
        Scalar, Eigen::Dynamic, Eigen::Dynamic, C, D, E>::Index IndexType;
    IndexType rows = node["rows"].as<IndexType>();
    IndexType cols = node["cols"].as<IndexType>();

    M.resize(rows, cols);

    return decodeEigenMatrixData(node["data"], rows, cols, M);
  }
};
}  // namespace YAML

#endif  // ASLAM_CV_COMMON_EIGEN_YAML_SERIALIZATION_H_
