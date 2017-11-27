#include <Eigen/Dense>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <eigen-checks/gtest.h>

#include "maplab-common/eigen-proto.h"
#include "maplab-common/pose_types.h"
#include "maplab-common/test/testing-entrypoint.h"
#include "maplab-common/test/testing-predicates.h"

namespace common {
namespace eigen_proto {

template <typename EigenType>
class EigenProtoSemiStaticTest : public ::testing::Test {
 protected:
  static EigenType referenceEigenObject();
  static proto::SemiStaticMatrixd referenceProto();
  static proto::SemiStaticMatrixd inconsistentProto() {
    proto::SemiStaticMatrixd result = referenceProto();
    result.mutable_data()->RemoveLast();
    return result;
  }
  // 1D dynamic vectors can't be death-checked against inconsistent protos.
  static bool needsTestWithInconsistentProto() {
    return true;
  }
  static proto::SemiStaticMatrixd generateProto(size_t n) {
    proto::SemiStaticMatrixd result;
    for (size_t i = 0; i < n; ++i) {
      result.add_data(i);
    }
    return result;
  }
  static proto::SemiStaticMatrixd overFilledProto() {
    proto::SemiStaticMatrixd result;
    for (int i = 0; i < 100; ++i) {
      result.add_data(0);
    }
    return result;
  }
  void checkProto() {
    proto::SemiStaticMatrixd proto = referenceProto();
    EXPECT_EQ(proto.data_size(), proto_.data_size());
    for (int i = 0; i < proto.data_size(); ++i) {
      EXPECT_EQ(proto.data(i), proto_.data(i));
    }
  }
  // Quaternion needs different specialization.
  void checkMatrix() {
    EXPECT_TRUE(EIGEN_MATRIX_EQUAL(eigen_object_, referenceEigenObject()));
  }
  static Eigen::Vector4d quaternionCoeffs() {
    Eigen::Vector4d result(0, 1, 2, 3);
    result.normalize();
    return result;
  }
  static Eigen::Vector3d positionCoords() {
    Eigen::Vector3d result(1, 2, 3);
    return result;
  }
  EigenType eigen_object_;
  proto::SemiStaticMatrixd proto_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template <>
Eigen::Vector3d
EigenProtoSemiStaticTest<Eigen::Vector3d>::referenceEigenObject() {
  return Eigen::Vector3d(0, 1, 2);
}
template <>
proto::SemiStaticMatrixd
EigenProtoSemiStaticTest<Eigen::Vector3d>::referenceProto() {
  return generateProto(3);
}

template <>
Eigen::Matrix<double, 2, 3>
EigenProtoSemiStaticTest<Eigen::Matrix<double, 2, 3>>::referenceEigenObject() {
  Eigen::Matrix<double, 2, 3> result;
  // serialization is col-major, while initialization is row-major,
  // thus this reordering:
  result << 0, 2, 4, 1, 3, 5;
  return result;
}
template <>
proto::SemiStaticMatrixd
EigenProtoSemiStaticTest<Eigen::Matrix<double, 2, 3>>::referenceProto() {
  return generateProto(6);
}

template <>
Eigen::VectorXd
EigenProtoSemiStaticTest<Eigen::VectorXd>::referenceEigenObject() {
  Eigen::VectorXd result;
  result.resize(4);
  result << 0, 1, 2, 3;
  return result;
}
template <>
proto::SemiStaticMatrixd
EigenProtoSemiStaticTest<Eigen::VectorXd>::referenceProto() {
  return generateProto(4);
}
template <>
bool EigenProtoSemiStaticTest<
    Eigen::VectorXd>::needsTestWithInconsistentProto() {
  return false;
}

template <>
Eigen::Matrix2Xd
EigenProtoSemiStaticTest<Eigen::Matrix2Xd>::referenceEigenObject() {
  Eigen::Matrix2Xd result;
  result.resize(2, 3);
  // serialization is col-major, while initialization is row-major,
  // thus this reordering:
  result << 0, 2, 4, 1, 3, 5;
  return result;
}
template <>
proto::SemiStaticMatrixd
EigenProtoSemiStaticTest<Eigen::Matrix2Xd>::referenceProto() {
  return generateProto(6);
}

template <>
Eigen::Quaterniond
EigenProtoSemiStaticTest<Eigen::Quaterniond>::referenceEigenObject() {
  Eigen::Vector4d coeffs = quaternionCoeffs();
  return Eigen::Quaterniond(coeffs[0], coeffs[1], coeffs[2], coeffs[3]);
}
template <>
proto::SemiStaticMatrixd
EigenProtoSemiStaticTest<Eigen::Quaterniond>::referenceProto() {
  proto::SemiStaticMatrixd result;
  Eigen::Vector4d coeffs = quaternionCoeffs();
  // Eigen Quaternion constructor takes wxyz, but it serializes to xyzw,
  // thus this reordering
  result.add_data(coeffs[1]);
  result.add_data(coeffs[2]);
  result.add_data(coeffs[3]);
  result.add_data(coeffs[0]);
  return result;
}
template <>
void EigenProtoSemiStaticTest<Eigen::Quaterniond>::checkMatrix() {
  EXPECT_TRUE(
      EIGEN_MATRIX_EQUAL(
          eigen_object_.coeffs(), referenceEigenObject().coeffs()));
}

template <>
pose::Transformation
EigenProtoSemiStaticTest<pose::Transformation>::referenceEigenObject() {
  Eigen::Vector4d coeffs = quaternionCoeffs();
  Eigen::Quaterniond quaternion(coeffs[0], coeffs[1], coeffs[2], coeffs[3]);
  Eigen::Vector3d position = positionCoords();
  return pose::Transformation(quaternion, position);
}
template <>
proto::SemiStaticMatrixd
EigenProtoSemiStaticTest<pose::Transformation>::referenceProto() {
  proto::SemiStaticMatrixd result;
  Eigen::Vector4d coeffs = quaternionCoeffs();
  // Eigen Quaternion constructor takes wxyz, but it serializes to xyzw,
  // thus this reordering.
  result.add_data(coeffs[1]);
  result.add_data(coeffs[2]);
  result.add_data(coeffs[3]);
  result.add_data(coeffs[0]);

  Eigen::Vector3d position = positionCoords();
  result.add_data(position[0]);
  result.add_data(position[1]);
  result.add_data(position[2]);
  return result;
}
template <>
void EigenProtoSemiStaticTest<pose::Transformation>::checkMatrix() {
  EXPECT_TRUE(
      EIGEN_MATRIX_EQUAL(
          eigen_object_.getPosition(), referenceEigenObject().getPosition()));
  EXPECT_TRUE(
      EIGEN_MATRIX_EQUAL(
          eigen_object_.getRotation().toImplementation().coeffs(),
          referenceEigenObject().getRotation().toImplementation().coeffs()));
}

typedef ::testing::Types<Eigen::Vector3d, Eigen::Matrix<double, 2, 3>,
                         Eigen::VectorXd, Eigen::Matrix2Xd, Eigen::Quaterniond,
                         pose::Transformation>
    MyTypes;

TYPED_TEST_CASE(EigenProtoSemiStaticTest, MyTypes);

TYPED_TEST(EigenProtoSemiStaticTest, serialize) {
  serialize(this->referenceEigenObject(), this->proto_.mutable_data());
  this->checkProto();
}

TYPED_TEST(EigenProtoSemiStaticTest, serializeIntoUnderfilled) {
  this->proto_ = this->inconsistentProto();
  serialize(this->referenceEigenObject(), this->proto_.mutable_data());
  this->checkProto();
}

TYPED_TEST(EigenProtoSemiStaticTest, serializeIntoOverfilled) {
  this->proto_ = this->overFilledProto();
  serialize(this->referenceEigenObject(), this->proto_.mutable_data());
  this->checkProto();
}

TYPED_TEST(EigenProtoSemiStaticTest, deserialize) {
  deserialize(this->referenceProto().data(), &this->eigen_object_);
  this->checkMatrix();
}

TYPED_TEST(EigenProtoSemiStaticTest, deserializeInconsistent) {
  if (this->needsTestWithInconsistentProto()) {
    EXPECT_DEATH(
        deserialize(this->inconsistentProto().data(), &this->eigen_object_),
        "^");
  }
}

class EigenProtoDynamicTest : public EigenProtoSemiStaticTest<Eigen::MatrixXd> {
 protected:
  static proto::MatrixXd referenceProto() {
    proto::MatrixXd result;
    result.set_rows(2);
    result.set_cols(3);
    for (int i = 0; i < 6; ++i) {
      result.add_data(i);
    }
    return result;
  }
  static proto::MatrixXd inconsistentProto() {
    proto::MatrixXd result;
    result.set_rows(2);
    result.set_cols(3);
    for (int i = 0; i < 5; ++i) {
      result.add_data(i);
    }
    return result;
  }
  static proto::MatrixXd overFilledProto() {
    proto::MatrixXd result;
    result.set_rows(2);
    result.set_cols(3);
    for (int i = 0; i < 100; ++i) {
      result.add_data(i);
    }
    return result;
  }
  void checkProto() {
    proto::MatrixXd proto = referenceProto();
    EXPECT_EQ(proto.data_size(), proto_.data_size());
    for (int i = 0; i < proto.data_size(); ++i) {
      EXPECT_EQ(proto.data(i), proto_.data(i));
    }
  }
  proto::MatrixXd proto_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template <>
Eigen::MatrixXd
EigenProtoSemiStaticTest<Eigen::MatrixXd>::referenceEigenObject() {
  Eigen::MatrixXd result;
  result.resize(2, 3);
  // serialization is col-major, while initialization is row-major,
  // thus this reordering:
  result << 0, 2, 4, 1, 3, 5;
  return result;
}

TEST_F(EigenProtoDynamicTest, serialize) {
  serialize(this->referenceEigenObject(), &this->proto_);
  this->checkProto();
}

TEST_F(EigenProtoDynamicTest, serializeIntoUnderfilled) {
  this->proto_ = this->inconsistentProto();
  serialize(this->referenceEigenObject(), this->proto_.mutable_data());
  this->checkProto();
}

TEST_F(EigenProtoDynamicTest, serializeIntoOverfilled) {
  this->proto_ = this->overFilledProto();
  serialize(this->referenceEigenObject(), this->proto_.mutable_data());
  this->checkProto();
}

TEST_F(EigenProtoDynamicTest, deserialize) {
  deserialize(this->referenceProto(), &this->eigen_object_);
  EXPECT_TRUE(
      EIGEN_MATRIX_EQUAL(this->eigen_object_, this->referenceEigenObject()));
}

TEST_F(EigenProtoDynamicTest, deserializeInsufficient) {
  EXPECT_DEATH(
      deserialize(this->inconsistentProto(), &this->eigen_object_), "^");
}

}  // namespace eigen_proto
}  // namespace common

MAPLAB_UNITTEST_ENTRYPOINT
