#ifndef MAPLAB_COMMON_EIGEN_PROTO_INL_H_
#define MAPLAB_COMMON_EIGEN_PROTO_INL_H_
#include <glog/logging.h>
#include <map>

namespace common {
namespace eigen_proto {

template <typename Scalar, int Rows, int Cols, int C, int D, int E>
inline void deserialize(
    const google::protobuf::RepeatedField<Scalar>& proto,
    Eigen::Matrix<Scalar, Rows, Cols, C, D, E>* matrix) {
  CHECK_NOTNULL(matrix);
  static_assert(
      Rows != Eigen::Dynamic && Rows > 0 && Cols != Eigen::Dynamic && Cols > 0,
      "Matrix must be fully static and have meaningful dimensions!");
  CHECK_EQ(Rows * Cols, proto.size());
  Eigen::Map<const Eigen::Matrix<Scalar, Rows, Cols, C, D, E>> map(
      proto.data());
  *matrix = map;
}

template <typename Scalar, int Rows, int C, int D, int E>
inline void deserialize(
    const google::protobuf::RepeatedField<Scalar>& proto,
    Eigen::Matrix<Scalar, Rows, Eigen::Dynamic, C, D, E>* matrix) {
  CHECK_NOTNULL(matrix);
  static_assert(
      Rows != Eigen::Dynamic && Rows > 0,
      "Rows must be static and have meaningful dimensions!");
  CHECK_EQ(0, proto.size() % Rows);
  Eigen::Map<const Eigen::Matrix<Scalar, Rows, Eigen::Dynamic, C, D, E>> map(
      proto.data(), Rows, proto.size() / Rows);
  *matrix = map;
}

template <typename Scalar, int Cols, int C, int D, int E>
inline void deserialize(
    const google::protobuf::RepeatedField<Scalar>& proto,
    Eigen::Matrix<Scalar, Eigen::Dynamic, Cols, C, D, E>* matrix) {
  CHECK_NOTNULL(matrix);
  static_assert(
      Cols != Eigen::Dynamic && Cols > 0,
      "Cols must be static and have meaningful dimensions!");
  CHECK_EQ(0, proto.size() % Cols);
  Eigen::Map<const Eigen::Matrix<Scalar, Eigen::Dynamic, Cols, C, D, E>> map(
      proto.data(), proto.size() / Cols, Cols);
  *matrix = map;
}

template <typename Scalar, int Rows, int Cols, int C, int D, int E>
void serialize(
    const Eigen::Matrix<Scalar, Rows, Cols, C, D, E>& matrix,
    google::protobuf::RepeatedField<Scalar>* proto) {
  CHECK_NOTNULL(proto);
  proto->Clear();
  proto->Resize(matrix.size(), Scalar());
  Eigen::Map<Eigen::Matrix<Scalar, Rows, Cols, C, D, E>> map(
      proto->mutable_data(), matrix.rows(), matrix.cols());
  map = matrix;
}

template <typename Scalar, int Cols, int C, int D, int E>
void serialize(
    const Eigen::Matrix<Scalar, Eigen::Dynamic, Cols, C, D, E>& matrix,
    google::protobuf::RepeatedField<Scalar>* proto) {
  CHECK_NOTNULL(proto);
  proto->Resize(matrix.size(), Scalar());
  Eigen::Map<Eigen::Matrix<Scalar, Eigen::Dynamic, Cols, C, D, E>> map(
      proto->mutable_data(), matrix.rows(), matrix.cols());
  map = matrix;
}

template <int C, int D, int E>
void deserialize(
    const proto::MatrixXf& proto,
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, C, D, E>* matrix) {
  CHECK_NOTNULL(matrix);
  CHECK_EQ(static_cast<int>(proto.rows() * proto.cols()), proto.data_size());
  Eigen::Map<
      const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, C, D, E>>
      map(proto.data().data(), proto.rows(), proto.cols());
  *matrix = map;
}

template <int C, int D, int E>
void deserialize(
    const proto::MatrixXd& proto,
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, C, D, E>* matrix) {
  CHECK_NOTNULL(matrix);
  CHECK_EQ(static_cast<int>(proto.rows() * proto.cols()), proto.data_size());
  Eigen::Map<
      const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, C, D, E>>
      map(proto.data().data(), proto.rows(), proto.cols());
  *matrix = map;
}

template <int C, int D, int E>
void serialize(
    const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, C, D, E>& matrix,
    proto::MatrixXf* proto) {
  CHECK_NOTNULL(proto);
  google::protobuf::RepeatedField<float>* proto_data = proto->mutable_data();
  proto_data->Resize(matrix.size(), 0.f);
  Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, C, D, E>> map(
      proto_data->mutable_data(), matrix.rows(), matrix.cols());
  map = matrix;
  proto->set_rows(matrix.rows());
  proto->set_cols(matrix.cols());
}

template <int C, int D, int E>
void serialize(
    const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, C, D, E>&
        matrix,
    proto::MatrixXd* proto) {
  CHECK_NOTNULL(proto);
  google::protobuf::RepeatedField<double>* proto_data = proto->mutable_data();
  proto_data->Resize(matrix.size(), 0.);
  Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, C, D, E>>
      map(proto_data->mutable_data(), matrix.rows(), matrix.cols());
  map = matrix;
  proto->set_rows(matrix.rows());
  proto->set_cols(matrix.cols());
}

template <typename Scalar, int Options>
inline void deserialize(
    const google::protobuf::RepeatedField<Scalar>& proto,
    Eigen::Quaternion<Scalar, Options>* quaternion) {
  CHECK_NOTNULL(quaternion);
  CHECK_EQ(4, proto.size());
  Eigen::Map<const Eigen::Quaternion<Scalar, Options>> map(proto.data());
  *quaternion = map;
  CHECK_NEAR(1.0, quaternion->norm(), 1e-8);
}

template <typename Scalar, int Options>
inline void serialize(
    const Eigen::Quaternion<Scalar, Options>& quaternion,
    google::protobuf::RepeatedField<Scalar>* proto) {
  CHECK_NOTNULL(proto);
  CHECK_NEAR(1.0, quaternion.norm(), 1e-8);
  proto->Resize(4u, Scalar());
  Eigen::Map<Eigen::Quaternion<Scalar, Options>> map(proto->mutable_data());
  map = quaternion;
}

template <typename Scalar>
inline void deserialize(
    const google::protobuf::RepeatedField<Scalar>& proto,
    pose::Transformation* transformation) {
  CHECK_NOTNULL(transformation);
  CHECK_EQ(7, proto.size());
  constexpr static unsigned int kRotationBlockSize = 4;
  constexpr static unsigned int kPositionBlockSize = 3;
  Eigen::Map<const Eigen::Quaternion<Scalar>> quaternion_map(proto.data());
  Eigen::Map<const Eigen::Matrix<Scalar, kPositionBlockSize, 1>> position_map(
      proto.data() + kRotationBlockSize);
  transformation->getRotation().toImplementation() = quaternion_map;
  transformation->getPosition() = position_map;
  CHECK_NEAR(
      1.0, transformation->getRotation().toImplementation().norm(), 1e-8);
}

template <typename Scalar>
inline void serialize(
    const pose::Transformation& transformation,
    google::protobuf::RepeatedField<Scalar>* proto) {
  CHECK_NOTNULL(proto);
  CHECK_NEAR(1.0, transformation.getRotation().toImplementation().norm(), 1e-8);
  proto->Resize(7u, Scalar());
  Eigen::Map<Eigen::Matrix<Scalar, 7, 1>> map(proto->mutable_data());
  map << transformation.getRotation().toImplementation().coeffs(),
      transformation.getPosition();
}

}  // namespace eigen_proto
}  // namespace common

#endif  // MAPLAB_COMMON_EIGEN_PROTO_INL_H_
