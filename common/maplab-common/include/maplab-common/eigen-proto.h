#ifndef MAPLAB_COMMON_EIGEN_PROTO_H_
#define MAPLAB_COMMON_EIGEN_PROTO_H_

#include <Eigen/Dense>
#include <google/protobuf/repeated_field.h>

#include "maplab-common/eigen.pb.h"
#include "maplab-common/pose_types.h"

namespace common {
namespace eigen_proto {

//====================================================
// FULLY STATIC MATRICES OR JUST ONE DYNAMIC DIMENSION
//====================================================
template <typename Scalar, int Rows, int Cols, int C, int D, int E>
inline void deserialize(
    const google::protobuf::RepeatedField<Scalar>& proto,
    Eigen::Matrix<Scalar, Rows, Cols, C, D, E>* matrix);
template <typename Scalar, int Rows, int C, int D, int E>
inline void deserialize(
    const google::protobuf::RepeatedField<Scalar>& proto,
    Eigen::Matrix<Scalar, Rows, Eigen::Dynamic, C, D, E>* matrix);
template <typename Scalar, int Cols, int C, int D, int E>
inline void deserialize(
    const google::protobuf::RepeatedField<Scalar>& proto,
    Eigen::Matrix<Scalar, Eigen::Dynamic, Cols, C, D, E>* matrix);

template <typename Scalar, int Rows, int Cols, int C, int D, int E>
inline void serialize(
    const Eigen::Matrix<Scalar, Rows, Cols, C, D, E>& matrix,
    google::protobuf::RepeatedField<Scalar>* proto);

//=======================
// FULLY DYNAMIC MATRICES
//=======================
template <int C, int D, int E>
inline void deserialize(
    const proto::MatrixXf& proto,
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, C, D, E>* matrix);

template <int C, int D, int E>
inline void deserialize(
    const proto::MatrixXd& proto,
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, C, D, E>* matrix);

template <int C, int D, int E>
inline void serialize(
    const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, C, D, E>&
        matrix,
    proto::MatrixXd* proto);

template <int C, int D, int E>
inline void serialize(
    const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, C, D, E>& matrix,
    proto::MatrixXf* proto);

//============
// QUATERNIONS
//============
template <typename Scalar, int Options>
inline void deserialize(
    const google::protobuf::RepeatedField<Scalar>& proto,
    Eigen::Quaternion<Scalar, Options>* quaternion);

template <typename Scalar, int Options>
inline void serialize(
    const Eigen::Quaternion<Scalar, Options>& quaternion,
    google::protobuf::RepeatedField<Scalar>* proto);

//================
// TRANSFORMATIONS
//================
template <typename Scalar>
inline void deserialize(
    const google::protobuf::RepeatedField<Scalar>& proto,
    pose::Transformation* transformation);

template <typename Scalar>
inline void serialize(
    const pose::Transformation& transformation,
    google::protobuf::RepeatedField<Scalar>* proto);

}  // namespace eigen_proto
}  // namespace common

#include "./eigen-proto-inl.h"

#endif  // MAPLAB_COMMON_EIGEN_PROTO_H_
