#ifndef SENSORS_SENSOR_EXTRINSICS_H_
#define SENSORS_SENSOR_EXTRINSICS_H_

#include <string>

#include <Eigen/Core>
#include <aslam/common/memory.h>
#include <aslam/common/pose-types.h>
#include <aslam/common/yaml-serialization.h>
#include <maplab-common/macros.h>
#include <maplab-common/yaml-file-serializable.h>

namespace vi_map {

enum class ExtrinsicsType : int { kPositionOnly, kTransformation, kInvalid };

constexpr char kYamlFieldNameExtrinsicsType[] = "extrinsics_type";
constexpr char kYamlFieldName_T_R_S[] = "T_R_S";

constexpr char kExtrinsicsTypePositionOnly[] = "position_only";
constexpr char kExtrinsicsTypeTransformation[] = "transformation";

inline std::string extrinsicsTypeToString(
    const ExtrinsicsType extrinsics_type) {
  switch (extrinsics_type) {
    case ExtrinsicsType::kPositionOnly:
      return static_cast<std::string>(kExtrinsicsTypePositionOnly);
      break;
    case ExtrinsicsType::kTransformation:
      return static_cast<std::string>(kExtrinsicsTypeTransformation);
      break;
    default:
      LOG(FATAL) << "Unknown extrinsics type: "
                 << static_cast<int>(extrinsics_type);
      break;
  }
  return "";
}

inline ExtrinsicsType stringToExtrinsicsType(const std::string& extrinsics) {
  if (extrinsics == static_cast<std::string>(kExtrinsicsTypePositionOnly)) {
    return ExtrinsicsType::kPositionOnly;
  } else if (
      extrinsics == static_cast<std::string>(kExtrinsicsTypeTransformation)) {
    return ExtrinsicsType::kTransformation;
  } else {
    LOG(FATAL) << "Unknown extrinsics type: " << extrinsics;
  }
  return ExtrinsicsType::kInvalid;
}

class Extrinsics final : public common::YamlFileSerializable {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MAPLAB_POINTER_TYPEDEFS(Extrinsics);
  // Coordinate frames used:
  // R: Platform reference frame (usually the IMU body frame).
  // S: Sensor body frame.
  Extrinsics(
      const ExtrinsicsType extrinsics_type, const aslam::Transformation& T_R_S)
      : extrinsics_type_(extrinsics_type), T_R_S_(T_R_S) {
    CHECK(extrinsics_type_ != ExtrinsicsType::kInvalid);
    if (extrinsics_type_ == ExtrinsicsType::kPositionOnly) {
      const aslam::Quaternion kIdentityQuaternion;
      CHECK_EQ(T_R_S.getRotation(), kIdentityQuaternion);
    }
  }
  explicit Extrinsics(const aslam::Transformation& T_R_S)
      : extrinsics_type_(ExtrinsicsType::kTransformation), T_R_S_(T_R_S) {}
  explicit Extrinsics(const aslam::Position3D& p_R_S)
      : extrinsics_type_(ExtrinsicsType::kPositionOnly) {
    T_R_S_.setIdentity();
    T_R_S_.getPosition() = p_R_S;
  }

  static Extrinsics::UniquePtr createFromYaml(const YAML::Node& yaml_node);
  static Extrinsics::UniquePtr createRandomExtrinsics();

  ~Extrinsics() = default;

  ExtrinsicsType getExtrinsicsType() const {
    return extrinsics_type_;
  }

  const aslam::Transformation& get_T_R_S() const {
    return T_R_S_;
  }

  const aslam::Position3D& get_p_R_S() const {
    return T_R_S_.getPosition();
  }

  void serialize(YAML::Node* yaml_node) const override {
    CHECK_NOTNULL(yaml_node);
    (*yaml_node)[static_cast<std::string>(kYamlFieldNameExtrinsicsType)] =
        extrinsicsTypeToString(extrinsics_type_);
    (*yaml_node)[static_cast<std::string>(kYamlFieldName_T_R_S)] =
        YAML::convert<Eigen::Matrix4d>::encode(
            T_R_S_.getTransformationMatrix());
  }

  bool deserialize(const YAML::Node& yaml_node) override;

  bool operator==(const Extrinsics& other) const {
    constexpr double kPrecision = 1e-12;
    return extrinsics_type_ == other.getExtrinsicsType() &&
           T_R_S_.getTransformationMatrix().isApprox(
               other.T_R_S_.getTransformationMatrix(), kPrecision);
  }

 private:
  Extrinsics();

  ExtrinsicsType extrinsics_type_;
  aslam::Transformation T_R_S_;
};


}  // namespace vi_map

#endif  // SENSORS_SENSOR_EXTRINSICS_H_
