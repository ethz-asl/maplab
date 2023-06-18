#ifndef ASLAM_COMMON_SENSOR_H_
#define ASLAM_COMMON_SENSOR_H_

#include <string>

#include <aslam/common/macros.h>
#include <aslam/common/unique-id.h>
#include <aslam/common/yaml-file-serialization.h>
#include <aslam/common/yaml-serialization.h>

namespace aslam {

enum SensorType : uint8_t { kUnknown, kNCamera, kCamera };

constexpr const char kNCameraIdentifier[] = "NCAMERA";
constexpr const char kCameraIdentifier[] = "CAMERA";

constexpr const char kYamlFieldNameId[] = "id";
constexpr const char kYamlFieldNameSensorType[] = "sensor_type";
constexpr const char kYamlFieldNameTopic[] = "topic";
constexpr const char kYamlFieldNameDescription[] = "description";

class Sensor : public YamlFileSerializable {
 public:
  ASLAM_POINTER_TYPEDEFS(Sensor);

  Sensor();
  explicit Sensor(const SensorId& id);
  explicit Sensor(const SensorId& id, const std::string& topic);
  explicit Sensor(
      const SensorId& id, const std::string& topic,
      const std::string& description);

  virtual ~Sensor() = default;

  Sensor(const Sensor& other)
      : id_(other.id_),
        topic_(other.topic_),
        description_(other.description_) {}
  void operator=(const Sensor& other) {
    id_ = other.id_;
    topic_ = other.topic_;
    description_ = other.description_;
  }

  bool operator==(const Sensor& other) const;
  bool operator!=(const Sensor& other) const;
  bool isEqual(const Sensor& other, const bool verbose = false) const;

  virtual Sensor::Ptr cloneAsSensor() const = 0;

  // Set and get the sensor id.
  void setId(const SensorId& id) {
    CHECK(id.isValid());
    id_ = id;
  }
  const SensorId& getId() const {
    CHECK(id_.isValid());
    return id_;
  }

  // Set and get the topic
  void setTopic(const std::string& topic) {
    topic_ = topic;
  }
  const std::string& getTopic() const {
    return topic_;
  }

  // Set and get the description
  void setDescription(const std::string& description) {
    description_ = description;
  }
  const std::string& getDescription() const {
    return description_;
  }

  // Virtual
  virtual uint8_t getSensorType() const = 0;
  virtual std::string getSensorTypeString() const = 0;

  bool isValid() const;

  bool deserialize(const YAML::Node& sensor_node) override;
  void serialize(YAML::Node* sensor_node_ptr) const override;

  void setRandom();

 private:
  virtual bool isValidImpl() const = 0;
  virtual void setRandomImpl() = 0;
  virtual bool isEqualImpl(const Sensor& other, const bool verbose) const = 0;

  virtual bool loadFromYamlNodeImpl(const YAML::Node& sensor_node) = 0;
  virtual void saveToYamlNodeImpl(YAML::Node* sensor_node) const = 0;

 protected:
  // The id of this sensor.
  SensorId id_;
  std::string topic_;
  std::string description_;
};

}  // namespace aslam

#endif  // ASLAM_COMMON_SENSOR_H_
