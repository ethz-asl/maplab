#ifndef ROVIOLI_DATASOURCE_H_
#define ROVIOLI_DATASOURCE_H_

#include <functional>
#include <string>
#include <vector>

#include <glog/logging.h>
#include <maplab-common/macros.h>
#include <vio-common/vio-types.h>

namespace rovioli {

#define DECLARE_SENSOR_CALLBACK(SENSOR_NAME, MEASUREMENT_TYPE)             \
 public: /* NOLINT */                                                      \
  typedef std::function<void(MEASUREMENT_TYPE)> SENSOR_NAME##Callback;     \
  /* NOLINT */ void register##SENSOR_NAME##Callback(                       \
      const SENSOR_NAME##Callback& cb) { /* NOLINT */                      \
    CHECK(cb);                                                             \
    SENSOR_NAME##_callbacks_.emplace_back(cb);                             \
  }                                                                        \
  void invoke##SENSOR_NAME##Callbacks(const MEASUREMENT_TYPE& measurement) \
      const {                                                              \
    for (const SENSOR_NAME##Callback& callback /* NOLINT */ :              \
         SENSOR_NAME##_callbacks_) {                                       \
      callback(measurement);                                               \
    }                                                                      \
  }                                                                        \
                                                                           \
 private: /* NOLINT */                                                     \
  std::vector<SENSOR_NAME##Callback> SENSOR_NAME##_callbacks_;

class CallbackManager {
  DECLARE_SENSOR_CALLBACK(Image, vio::ImageMeasurement::Ptr);
  DECLARE_SENSOR_CALLBACK(Imu, vio::ImuMeasurement::Ptr);
};

class DataSource : public CallbackManager {
 public:
  MAPLAB_POINTER_TYPEDEFS(DataSource);
  MAPLAB_DISALLOW_EVIL_CONSTRUCTORS(DataSource);

  virtual ~DataSource() {}

  // Has all data been released to the output queues.
  virtual void startStreaming() = 0;
  virtual void shutdown() = 0;

  virtual bool allDataStreamed() const = 0;
  virtual std::string getDatasetName() const = 0;

  virtual void registerEndOfDataCallback(const std::function<void()>& cb) {
    CHECK(cb);
    end_of_data_callbacks_.emplace_back(cb);
  }
  void invokeEndOfDataCallbacks() const {
    for (const std::function<void()>& cb : end_of_data_callbacks_) {
      cb();
    }
  }

  // If this is the first timestamp we receive, we store it and shift all
  // subsequent timestamps. Will return false for any timestamps that are
  // smaller than the first timestamp received.
  bool shiftByFirstTimestamp(int64_t* timestamp_ns) {
    CHECK_NOTNULL(timestamp_ns);
    CHECK_GE(*timestamp_ns, 0);
    {
      std::lock_guard<std::mutex> lock(timestamp_mutex_);
      if (timestamp_at_start_ns_ == -1) {
        timestamp_at_start_ns_ = *timestamp_ns;
        *timestamp_ns = 0;
        VLOG(2)
            << "Set the first timestamp that was received to "
            << timestamp_at_start_ns_
            << "ns, all subsequent timestamp will be shifted by that amount.";
      } else {
        if (*timestamp_ns < timestamp_at_start_ns_) {
          LOG(WARNING) << "Received timestamp that is earlier than the first "
                       << "timestamp of the data source! First timestamp: "
                       << timestamp_at_start_ns_
                       << "ns, received timestamp: " << *timestamp_ns << "ns.";
          return false;
        }
        *timestamp_ns = *timestamp_ns - timestamp_at_start_ns_;
      }
    }

    CHECK_GE(timestamp_at_start_ns_, 0);
    CHECK_GE(*timestamp_ns, 0);
    return true;
  }

 protected:
  DataSource() = default;

 private:
  std::vector<std::function<void()>> end_of_data_callbacks_;

  std::mutex timestamp_mutex_;
  int64_t timestamp_at_start_ns_ = -1;
};

}  // namespace rovioli

#endif  // ROVIOLI_DATASOURCE_H_
