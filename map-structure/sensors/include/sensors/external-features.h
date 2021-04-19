#ifndef SENSORS_EXTERNAL_FEATURES_H_
#define SENSORS_EXTERNAL_FEATURES_H_

#include <aslam/common/sensor.h>
#include <aslam/frames/visual-frame.h>
#include <maplab-common/macros.h>
#include <string>
#include <yaml-cpp/yaml.h>

#include "sensors/measurement.h"
#include "sensors/sensor-types.h"

namespace vi_map {

enum ExternalFeatureType : uint8_t {
  kVisualBinaryFeatures,
  kVisualFloatFeatures
};

// ...
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

  bool hasUncertainties() const {
    return has_uncertainties_;
  }

  bool hasOrientations() const {
    return has_orientations_;
  }

  bool hasScores() const {
    return has_scores_;
  }

  bool hasScales() const {
    return has_scales_;
  }

  bool hasTrackIds() const {
    return has_track_ids_;
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

  bool has_uncertainties_;
  bool has_orientations_;
  bool has_scores_;
  bool has_scales_;
  bool has_track_ids_;
  ExternalFeatureType feature_type_;
};

// ...
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
        descriptors_(descriptors),
        track_ids_(track_ids) {
    CHECK(keypoint_measurements_x_.size() == num_keypoint_measurements);
    CHECK(keypoint_measurements_y_.size() == num_keypoint_measurements);
    CHECK(descriptors_.size() == num_keypoint_measurements * descriptor_size);

    CHECK(
        keypoint_measurement_uncertainties_.size() == 0 ||
        keypoint_measurement_uncertainties_.size() ==
            num_keypoint_measurements);
    CHECK(
        keypoint_orientations_.size() == 0 ||
        keypoint_orientations_.size() == num_keypoint_measurements);
    CHECK(
        keypoint_scores_.size() == 0 ||
        keypoint_scores_.size() == num_keypoint_measurements);
    CHECK(
        keypoint_scales_.size() == 0 ||
        keypoint_scales_.size() == num_keypoint_measurements);
    CHECK(
        track_ids_.size() == 0 ||
        track_ids_.size() == num_keypoint_measurements);
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
    CHECK_EQ(
        keypoint_measurement_uncertainties_.size(), num_keypoint_measurements_)
        << "External keypoint uncertainty measurements requested but none "
           "provided.";
    CHECK_NOTNULL(keypoint_measurement_uncertainties);

    keypoint_measurement_uncertainties->resize(num_keypoint_measurements_);
    for (uint32_t i = 0; i < num_keypoint_measurements_; i++) {
      (*keypoint_measurement_uncertainties)(i) =
          keypoint_measurement_uncertainties_[i];
    }
  }

  void getKeypointOrientations(Eigen::VectorXd* keypoint_orientations) const {
    CHECK_EQ(keypoint_orientations_.size(), num_keypoint_measurements_)
        << "External keypoint orientations requested but none provided.";
    CHECK_NOTNULL(keypoint_orientations);

    keypoint_orientations->resize(num_keypoint_measurements_);
    for (uint32_t i = 0; i < num_keypoint_measurements_; i++) {
      (*keypoint_orientations)(i) = keypoint_orientations_[i];
    }
  }

  void getKeypointScores(Eigen::VectorXd* keypoint_scores) const {
    CHECK_EQ(keypoint_scores_.size(), num_keypoint_measurements_)
        << "External keypoint scores requested but none provided.";
    CHECK_NOTNULL(keypoint_scores);

    keypoint_scores->resize(num_keypoint_measurements_);
    for (uint32_t i = 0; i < num_keypoint_measurements_; i++) {
      (*keypoint_scores)(i) = keypoint_scores_[i];
    }
  }

  void getKeypointScales(Eigen::VectorXd* keypoint_scales) const {
    CHECK_EQ(keypoint_scales_.size(), num_keypoint_measurements_)
        << "External keypoint scales requested but none provided.";
    CHECK_NOTNULL(keypoint_scales);

    keypoint_scales->resize(num_keypoint_measurements_);
    for (uint32_t i = 0; i < num_keypoint_measurements_; i++) {
      (*keypoint_scales)(i) = keypoint_scales_[i];
    }
  }

  void getTrackIds(Eigen::VectorXi* track_ids) const {
    CHECK_EQ(track_ids_.size(), num_keypoint_measurements_)
        << "External keypoint track ids requested but none provided.";
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
  std::vector<uint8_t> descriptors_;
  std::vector<int32_t> track_ids_;
};

DEFINE_MEASUREMENT_CONTAINERS(ExternalFeaturesMeasurement)

}  // namespace vi_map

DEFINE_MEASUREMENT_HASH(ExternalFeaturesMeasurement)

#endif  // SENSORS_EXTERNAL_FEATURES_H_
