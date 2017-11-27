#include <iomanip>

#include <Eigen/Dense>
#include <glog/logging.h>
#include <maplab-common/file-logger.h>
#include <minkindr_conversions/kindr_tf.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_datatypes.h>

#include "simulation/magnetometer-simulator.h"

// This demo class subscribes to NavSatFix and global attitude topic and
// simulates
// a magnetometer measurement which can then be compared to real magnetometer
// measurements.
// Used to validate the magnetometer simulator with real data.
class MagnetometerSimulatorDemo {
 public:
  explicit MagnetometerSimulatorDemo(const simulation::YearMonthDay& utc_time) {
    magnetometer_simulator_.reset(
        new simulation::MagnetometerSimulator(utc_time));

    const std::string filename = "/tmp/simulated_magnetometer.txt";
    logger_.reset(new common::FileLogger(filename));
  }

  void runAndJoin() const {
    ros::spin();
    ros::waitForShutdown();
  }

  void registerSubscriberAndPublisher() {
    // Subscribe to NavSatFix topic.
    const std::string topic_name_llh = "/mavros/global_position/global";
    LOG(INFO) << "Subscribed to topic " << topic_name_llh;
    boost::function<void(const sensor_msgs::NavSatFixConstPtr& msg)>
        llh_callback =
            boost::bind(&MagnetometerSimulatorDemo::llhCallback, this, _1);
    sub_llh_ = node_handle_.subscribe(topic_name_llh, 1000, llh_callback);

    // Subscribe to global attitude topic.
    const std::string topic_name_attitude = "/mavros/imu/data";
    LOG(INFO) << "Subscribed to topic " << topic_name_attitude;
    boost::function<void(const sensor_msgs::ImuConstPtr& msg)>
        attitude_callback =
            boost::bind(&MagnetometerSimulatorDemo::attitudeCallback, this, _1);
    sub_attitude_ =
        node_handle_.subscribe(topic_name_attitude, 1000, attitude_callback);
  }

  void llhCallback(const sensor_msgs::NavSatFixConstPtr& msg) {
    aslam::Quaternion trafo_kindr;
    quaternionTFToKindr(quaternion_, &trafo_kindr);

    Eigen::Matrix3d R_B_G = trafo_kindr.getRotationMatrix().inverse();
    Eigen::Vector3d llh(msg->latitude, msg->longitude, msg->altitude);

    Eigen::Vector3d simulated_magnetic_field_tesla;
    magnetometer_simulator_->calculateMagneticField(
        llh, R_B_G, &simulated_magnetic_field_tesla);

    // Save to file.
    *logger_ << std::setprecision(15) << simulated_magnetic_field_tesla(0)
             << " " << simulated_magnetic_field_tesla(1) << " "
             << simulated_magnetic_field_tesla(2) << std::endl;
  }

  void attitudeCallback(const sensor_msgs::ImuConstPtr& msg) {
    quaternion_ = tf::Quaternion(
        msg->orientation.x, msg->orientation.y, msg->orientation.z,
        msg->orientation.w);
  }

 private:
  std::unique_ptr<simulation::MagnetometerSimulator> magnetometer_simulator_;
  std::unique_ptr<common::FileLogger> logger_;
  ros::NodeHandle node_handle_;
  ros::Subscriber sub_llh_;
  ros::Subscriber sub_attitude_;
  tf::Quaternion quaternion_;
};

int main(int argc, char** argv) {
  // Initialize google logging and flags.
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  // Initialize ros node handle.
  ros::init(argc, argv, "magnetometer_simulator_demo");

  // Set the time of the measurement.
  simulation::YearMonthDay utc_date;
  utc_date.year = 2015;
  utc_date.month = 5;
  utc_date.day = 1;

  MagnetometerSimulatorDemo magnetometer_simulator_demo(utc_date);
  magnetometer_simulator_demo.registerSubscriberAndPublisher();
  magnetometer_simulator_demo.runAndJoin();

  return 0;
}
