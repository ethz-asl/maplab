#ifndef SENSORS_EXTERNAL_FEATURES_H_
#define SENSORS_EXTERNAL_FEATURES_H_

#include <aslam/common/sensor.h>
#include <aslam/frames/visual-frame.h>
#include <maplab-common/macros.h>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "sensors/measurement.h"
#include "sensors/sensor-types.h"

namespace vi_map {

enum class FeatureType : int {
  kInvalid = -1,
  kBinary = 0,  // Describes the standard maplab binary features which
                // are either BRISK / FREAK depending on the flag.
  kSuperPoint = 1,
  kR2D2 = 2,
  kSIFT = 3,
  // LiDAR versions of various features.
  kLIDARSuperPoint = 21
};

std::string FeatureTypeToString(FeatureType feature_type);
FeatureType StringToFeatureType(const std::string& feature_string);
bool isFloatFeature(FeatureType feature_type);
bool isLidarFeature(FeatureType feature_type);

class ExternalFeatures final : public aslam::Sensor {
 public:
  MAPLAB_POINTER_TYPEDEFS(ExternalFeatures);

  ExternalFeatures();
  explicit ExternalFeatures(
      const aslam::SensorId& sensor_id, const std::string& topic);

  Sensor::Ptr cloneAsSensor() const {
    return std::dynamic_pointer_cast<Sensor>(
        aligned_shared<ExternalFeatures>(*this));
  }

  ExternalFeatures* cloneWithNewIds() const {
    ExternalFeatures* cloned_external_features = new ExternalFeatures();
    *cloned_external_features = *this;
    aslam::SensorId new_id;
    aslam::generateId(&new_id);
    cloned_external_features->setId(new_id);
    return cloned_external_features;
  }

  uint8_t getSensorType() const override {
    return SensorType::kExternalFeatures;
  }

  std::string getSensorTypeString() const override {
    return static_cast<std::string>(kExternalFeaturesIdentifier);
  }

  const aslam::SensorId& getTargetSensorId() const {
    return target_sensor_id_;
  }

  const aslam::SensorId& getTargetNCameraId() const {
    CHECK(target_ncamera_id_.isValid())
        << "No associated ncamera id set, setTargetNCameraId "
           "has to be called before getTargetNCameraId.";
    return target_ncamera_id_;
  }

  void setTargetNCameraId(const aslam::SensorId& target_ncamera_id) {
    CHECK(target_ncamera_id.isValid());
    target_ncamera_id_ = target_ncamera_id;
  }

  size_t getTargetCameraIndex() const {
    CHECK(target_camera_index_set_)
        << "No associated camera index set, setTargetCameraIndex "
           "has to be called before getTargetCameraIndex.";
    return target_camera_index_;
  }

  void setTargetCameraIndex(size_t target_camera_index) {
    target_camera_index_ = target_camera_index;
    target_camera_index_set_ = true;
  }

  FeatureType getFeatureType() const {
    return feature_type_;
  }

 private:
  bool loadFromYamlNodeImpl(const YAML::Node& sensor_node) override;
  void saveToYamlNodeImpl(YAML::Node* sensor_node) const override;

  bool isValidImpl() const override {
    return true;
  }

  void setRandomImpl() override {}

  bool isEqualImpl(
      const Sensor& /*other*/, const bool /*verbose*/) const override {
    return true;
  }

  aslam::SensorId target_sensor_id_;
  aslam::SensorId target_ncamera_id_;
  size_t target_camera_index_;
  bool target_camera_index_set_;
  FeatureType feature_type_;
};

class ExternalFeaturesMeasurement : public Measurement {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MAPLAB_POINTER_TYPEDEFS(ExternalFeaturesMeasurement);

  explicit ExternalFeaturesMeasurement(
      const aslam::SensorId& sensor_id, const int64_t timestamp_nanoseconds,
      uint32_t num_keypoint_measurements, uint32_t descriptor_size,
      const std::vector<double>& keypoint_measurements_x,
      const std::vector<double>& keypoint_measurements_y,
      const std::vector<double>& keypoint_measurement_uncertainties,
      const std::vector<double>& keypoint_orientations,
      const std::vector<double>& keypoint_scores,
      const std::vector<double>& keypoint_scales,
      const std::vector<double> keypoint_3d_x,
      const std::vector<double> keypoint_3d_y,
      const std::vector<double> keypoint_3d_z,
      const std::vector<int32_t> keypoint_time_offsets,
      const std::vector<uint8_t>& descriptors,
      const std::vector<int32_t>& track_ids)
      : Measurement(sensor_id, timestamp_nanoseconds),
        num_keypoint_measurements_(num_keypoint_measurements),
        descriptor_size_(descriptor_size),
        keypoint_measurements_x_(keypoint_measurements_x),
        keypoint_measurements_y_(keypoint_measurements_y),
        keypoint_measurement_uncertainties_(keypoint_measurement_uncertainties),
        keypoint_orientations_(keypoint_orientations),
        keypoint_scores_(keypoint_scores),
        keypoint_scales_(keypoint_scales),
        keypoint_3d_x_(keypoint_3d_x),
        keypoint_3d_y_(keypoint_3d_y),
        keypoint_3d_z_(keypoint_3d_z),
        keypoint_time_offsets_(keypoint_time_offsets),
        descriptors_(descriptors),
        track_ids_(track_ids) {
    CHECK(keypoint_measurements_x_.size() == num_keypoint_measurements);
    CHECK(keypoint_measurements_y_.size() == num_keypoint_measurements);
    CHECK(descriptors_.size() == num_keypoint_measurements * descriptor_size);

    CHECK(
        keypoint_measurement_uncertainties_.size() == num_keypoint_measurements)
        << "Providing keypoint uncertainties is mandatory. If none are "
        << "computed set a constant value for all to weight them equally.";
    CHECK(
        keypoint_orientations_.size() == 0 ||
        keypoint_orientations_.size() == num_keypoint_measurements);
    CHECK(
        keypoint_scores_.size() == 0 ||
        keypoint_scores_.size() == num_keypoint_measurements);
    CHECK(
        keypoint_scales_.size() == 0 ||
        keypoint_scales_.size() == num_keypoint_measurements);
    CHECK(track_ids_.size() == num_keypoint_measurements)
        << "Providing keypoint tracks is mandatory. Please use one of the "
        << "externally provided trackes in maplab_features.";

    // 3D information is either not provided or is all there.
    CHECK(
        keypoint_3d_x_.size() == 0 ||
        keypoint_3d_x_.size() == num_keypoint_measurements);
    CHECK(
        keypoint_3d_x_.size() == keypoint_3d_y_.size() &&
        keypoint_3d_x_.size() == keypoint_3d_z_.size() &&
        keypoint_3d_x_.size() == keypoint_time_offsets_.size());
  }

  void getKeypointMeasurements(Eigen::Matrix2Xd* keypoint_measurements) const {
    CHECK_NOTNULL(keypoint_measurements);
    keypoint_measurements->resize(Eigen::NoChange, num_keypoint_measurements_);
    for (uint32_t i = 0; i < num_keypoint_measurements_; i++) {
      (*keypoint_measurements)(0, i) = keypoint_measurements_x_[i];
      (*keypoint_measurements)(1, i) = keypoint_measurements_y_[i];
    }
  }

  void getKeypointUncertainties(
      Eigen::VectorXd* keypoint_measurement_uncertainties) const {
    CHECK_NOTNULL(keypoint_measurement_uncertainties);
    keypoint_measurement_uncertainties->resize(num_keypoint_measurements_);
    for (uint32_t i = 0; i < num_keypoint_measurements_; i++) {
      (*keypoint_measurement_uncertainties)(i) =
          keypoint_measurement_uncertainties_[i];
    }
  }

  bool getKeypointOrientations(Eigen::VectorXd* keypoint_orientations) const {
    CHECK_NOTNULL(keypoint_orientations);
    if (keypoint_orientations_.size() == 0) {
      return false;
    }

    keypoint_orientations->resize(num_keypoint_measurements_);
    for (uint32_t i = 0; i < num_keypoint_measurements_; i++) {
      (*keypoint_orientations)(i) = keypoint_orientations_[i];
    }

    return true;
  }

  bool getKeypointScores(Eigen::VectorXd* keypoint_scores) const {
    CHECK_NOTNULL(keypoint_scores);
    if (keypoint_scores_.size() == 0) {
      return false;
    }

    keypoint_scores->resize(num_keypoint_measurements_);
    for (uint32_t i = 0; i < num_keypoint_measurements_; i++) {
      (*keypoint_scores)(i) = keypoint_scores_[i];
    }

    return true;
  }

  bool getKeypointScales(Eigen::VectorXd* keypoint_scales) const {
    CHECK_NOTNULL(keypoint_scales);
    if (keypoint_scales_.size() == 0) {
      return false;
    }

    keypoint_scales->resize(num_keypoint_measurements_);
    for (uint32_t i = 0; i < num_keypoint_measurements_; i++) {
      (*keypoint_scales)(i) = keypoint_scales_[i];
    }

    return true;
  }

  bool getKeypoint3DPositions(Eigen::Matrix3Xd* keypoint_3d_positions) const {
    CHECK_NOTNULL(keypoint_3d_positions);
    if (keypoint_3d_x_.size() == 0) {
      return false;
    }

    keypoint_3d_positions->resize(Eigen::NoChange, num_keypoint_measurements_);
    for (uint32_t i = 0; i < num_keypoint_measurements_; i++) {
      (*keypoint_3d_positions)(0, i) = keypoint_3d_x_[i];
      (*keypoint_3d_positions)(1, i) = keypoint_3d_y_[i];
      (*keypoint_3d_positions)(2, i) = keypoint_3d_z_[i];
    }

    return true;
  }

  bool getKeypointTimeOffsets(Eigen::VectorXi* keypoint_time_offsets) const {
    CHECK_NOTNULL(keypoint_time_offsets);
    if (keypoint_time_offsets_.size() == 0) {
      return false;
    }

    keypoint_time_offsets->resize(num_keypoint_measurements_);
    for (uint32_t i = 0; i < num_keypoint_measurements_; i++) {
      (*keypoint_time_offsets)(i) = keypoint_time_offsets_[i];
    }

    return true;
  }

  void getTrackIds(Eigen::VectorXi* track_ids) const {
    CHECK_NOTNULL(track_ids);
    track_ids->resize(num_keypoint_measurements_);
    for (uint32_t i = 0; i < num_keypoint_measurements_; i++) {
      (*track_ids)(i) = track_ids_[i];
    }
  }

  void getDescriptors(aslam::VisualFrame::DescriptorsT* descriptors) const {
    CHECK_NOTNULL(descriptors);
    (*descriptors) = Eigen::Map<const aslam::VisualFrame::DescriptorsT>(
        descriptors_.data(), descriptor_size_, num_keypoint_measurements_);
  }

  inline bool isEqual(
      const ExternalFeaturesMeasurement& other,
      const bool verbose = false) const {
    bool is_equal = true;
    if (other.getSensorId() != getSensorId()) {
      LOG_IF(WARNING, verbose)
          << "ExternalFeaturesMeasurements have different sensor id: "
          << other.getSensorId() << " vs " << getSensorId();
      is_equal = false;
    }

    if (other.getTimestampNanoseconds() != getTimestampNanoseconds()) {
      LOG_IF(WARNING, verbose)
          << "ExternalFeaturesMeasurements have different timestamps: "
          << other.getTimestampNanoseconds() << " vs "
          << getTimestampNanoseconds();
      is_equal = false;
    }

    return is_equal;
  }

  uint32_t getNumKeypointMeasurements() const {
    return num_keypoint_measurements_;
  }

 private:
  inline bool isValidImpl() const {
    return true;
  }

  inline void setRandomImpl() override {}

  uint32_t num_keypoint_measurements_;
  uint32_t descriptor_size_;
  std::vector<double> keypoint_measurements_x_;
  std::vector<double> keypoint_measurements_y_;
  std::vector<double> keypoint_measurement_uncertainties_;
  std::vector<double> keypoint_orientations_;
  std::vector<double> keypoint_scores_;
  std::vector<double> keypoint_scales_;
  std::vector<double> keypoint_3d_x_;
  std::vector<double> keypoint_3d_y_;
  std::vector<double> keypoint_3d_z_;
  std::vector<int32_t> keypoint_time_offsets_;
  std::vector<uint8_t> descriptors_;
  std::vector<int32_t> track_ids_;
};

DEFINE_MEASUREMENT_CONTAINERS(ExternalFeaturesMeasurement)

}  // namespace vi_map

DEFINE_MEASUREMENT_HASH(ExternalFeaturesMeasurement)

#endif  // SENSORS_EXTERNAL_FEATURES_H_
