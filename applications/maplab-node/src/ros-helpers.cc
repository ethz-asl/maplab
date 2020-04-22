#include "maplab-node/ros-helpers.h"

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/image_encodings.h>
#pragma GCC diagnostic pop
#include <minkindr_conversions/kindr_msg.h>
#include <vio-common/vio-types.h>

DEFINE_bool(
    image_apply_clahe_histogram_equalization, false,
    "Apply CLAHE histogram equalization to image.");
DEFINE_int32(
    image_clahe_clip_limit, 2,
    "CLAHE histogram equalization parameter: clip limit.");
DEFINE_int32(
    image_clahe_grid_size, 8,
    "CLAHE histogram equalization parameter: grid size.");

DEFINE_double(
    image_16_bit_to_8_bit_scale_factor, 1,
    "Scale factor applied to 16bit images when converting them to 8bit "
    "images.");
DEFINE_double(
    image_16_bit_to_8_bit_shift, 0,
    "Shift applied to the scaled values when converting 16bit images to 8bit "
    "images.");

DEFINE_bool(
    set_imu_bias_to_zero, false,
    "Setting IMU biases for odometry estimation to zero.");
DECLARE_bool(map_builder_save_color_image_as_resources);

namespace maplab {

vio::ImuMeasurement::Ptr convertRosImuToMaplabImu(
    const sensor_msgs::ImuConstPtr& imu_msg) {
  CHECK(imu_msg);
  vio::ImuMeasurement::Ptr imu_measurement(new vio::ImuMeasurement);
  imu_measurement->timestamp = rosTimeToNanoseconds(imu_msg->header.stamp);
  imu_measurement->imu_data << imu_msg->linear_acceleration.x,
      imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z,
      imu_msg->angular_velocity.x, imu_msg->angular_velocity.y,
      imu_msg->angular_velocity.z;
  return imu_measurement;
}

void applyHistogramEqualization(
    const cv::Mat& input_image, cv::Mat* output_image) {
  CHECK_NOTNULL(output_image);
  CHECK(!input_image.empty());
  if (input_image.channels() == 1u) {
    static cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(
        FLAGS_image_clahe_clip_limit,
        cv::Size(FLAGS_image_clahe_grid_size, FLAGS_image_clahe_grid_size));
    clahe->apply(input_image, *output_image);
  } else if (input_image.channels() == 3u) {
    cv::Mat lab_image;
    cv::cvtColor(input_image, lab_image, CV_BGR2Lab);

    // Extract the L channel
    std::vector<cv::Mat> lab_planes(3);
    cv::split(
        lab_image, lab_planes);  // now we have the L image in lab_planes[0]

    // apply the CLAHE algorithm to the L channel
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(
        FLAGS_image_clahe_clip_limit,
        cv::Size(FLAGS_image_clahe_grid_size, FLAGS_image_clahe_grid_size));
    cv::Mat dst;
    clahe->apply(lab_planes[0], dst);

    // Merge the the color planes back into an Lab image
    dst.copyTo(lab_planes[0]);
    cv::merge(lab_planes, lab_image);

    // convert back to RGB
    cv::cvtColor(lab_image, *output_image, CV_Lab2BGR);
  } else {
    LOG(FATAL) << "Number of image channels unrecongized: "
               << input_image.channels();
  }
}

vio::ImageMeasurement::Ptr convertRosImageToMaplabImage(
    const sensor_msgs::ImageConstPtr& image_message, size_t camera_idx) {
  CHECK(image_message);
  cv_bridge::CvImageConstPtr cv_ptr;
  vio::ImageMeasurement::Ptr image_measurement(new vio::ImageMeasurement);
  try {
    // 16bit images are treated differently, we will apply histogram
    // equalization first and then convert them to MONO8;
    if (image_message->encoding == sensor_msgs::image_encodings::MONO16) {
      cv_ptr = cv_bridge::toCvShare(
          image_message, sensor_msgs::image_encodings::MONO16);
      CHECK(cv_ptr);

      cv::Mat processed_image;
      if (FLAGS_image_apply_clahe_histogram_equalization) {
        applyHistogramEqualization(cv_ptr->image, &processed_image);
      } else {
        processed_image = cv_ptr->image;
      }
      processed_image.convertTo(
          image_measurement->image, CV_8U,
          FLAGS_image_16_bit_to_8_bit_scale_factor,
          FLAGS_image_16_bit_to_8_bit_shift);
      image_measurement->encoding = sensor_msgs::image_encodings::MONO8;
    } else {
      if (image_message->encoding == sensor_msgs::image_encodings::TYPE_8UC1) {
        // NOTE: we assume all 8UC1 type images are monochrome images.
        cv_ptr = cv_bridge::toCvShare(
            image_message, sensor_msgs::image_encodings::TYPE_8UC1);
        image_measurement->encoding = sensor_msgs::image_encodings::MONO8;
      } else {
        if (FLAGS_map_builder_save_color_image_as_resources) {
          cv_ptr = cv_bridge::toCvShare(image_message);
          image_measurement->encoding = image_message->encoding;
        } else {
          cv_ptr = cv_bridge::toCvShare(
              image_message, sensor_msgs::image_encodings::MONO8);
          image_measurement->encoding = sensor_msgs::image_encodings::MONO8;
        }
      }
      CHECK(cv_ptr);

      if (FLAGS_image_apply_clahe_histogram_equalization) {
        cv::Mat processed_image;
        applyHistogramEqualization(cv_ptr->image, &processed_image);
        image_measurement->image = processed_image;
      } else {
        image_measurement->image = cv_ptr->image.clone();
      }
    }
  } catch (const cv_bridge::Exception& e) {  // NOLINT
    LOG(FATAL) << "cv_bridge exception: " << e.what();
  }
  CHECK(cv_ptr);

  image_measurement->timestamp =
      rosTimeToNanoseconds(image_message->header.stamp);
  image_measurement->camera_index = camera_idx;
  return image_measurement;
}

vi_map::RosLidarMeasurement::Ptr convertRosCloudToMaplabCloud(
    const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
    const aslam::SensorId& sensor_id) {
  CHECK(cloud_msg);

  vi_map::RosLidarMeasurement::Ptr lidar_measurement(
      new vi_map::RosLidarMeasurement(
          sensor_id, static_cast<int64_t>(cloud_msg->header.stamp.toNSec())));
  *(lidar_measurement->getPointCloudMutable()) = *cloud_msg;
  // pcl::fromROSMsg(*cloud_msg, *(lidar_measurement->getPointCloudMutable()));
  return lidar_measurement;
}

OdometryEstimate::Ptr convertRosOdometryMsgToOdometryEstimate(
    const maplab_msgs::OdometryWithImuBiasesConstPtr& msg,
    const aslam::Transformation& T_B_S,
    const vi_map::Odometry6DoF& /*sensor*/) {
  CHECK(msg);

  OdometryEstimate::Ptr odometry_estimate(new OdometryEstimate);

  // Header.
  odometry_estimate->timestamp_ns = msg->header.stamp.toNSec();

  CHECK_GE(odometry_estimate->timestamp_ns, 0);

  // Odometry Pose.
  aslam::Transformation T_O_S;
  tf::poseMsgToKindr(msg->pose.pose, &T_O_S);

  const aslam::Transformation T_M_B = T_O_S * T_B_S.inverse();
  odometry_estimate->vinode.set_T_M_I(T_M_B);

  odometry_estimate->vinode.setTimestamp(odometry_estimate->timestamp_ns);

  // Velocity.
  Eigen::Vector3d v_M_I;
  tf::vectorMsgToEigen(msg->twist.twist.linear, v_M_I);
  odometry_estimate->vinode.set_v_M_I(v_M_I);

  // IMU Biases.
  Eigen::Vector3d acc_bias = Eigen::Vector3d::Zero();
  if (!FLAGS_set_imu_bias_to_zero) {
    tf::vectorMsgToEigen(msg->accel_bias, acc_bias);
  }
  odometry_estimate->vinode.setAccBias(acc_bias);

  Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
  if (!FLAGS_set_imu_bias_to_zero) {
    tf::vectorMsgToEigen(msg->gyro_bias, gyro_bias);
  }
  odometry_estimate->vinode.setGyroBias(gyro_bias);

  // TODO(mfehr): We currently don't support this.
  odometry_estimate->has_T_G_M = false;

  return odometry_estimate;
}

vi_map::Absolute6DoFMeasurement::Ptr
convertPoseWithCovarianceToAbsolute6DoFConstraint(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg,
    const vi_map::Absolute6DoF& sensor) {
  CHECK(msg);

  const aslam::SensorId& sensor_id = sensor.getId();

  vi_map::Absolute6DoFMeasurement::Ptr absolute_constraint =
      aligned_shared<vi_map::Absolute6DoFMeasurement>();

  absolute_constraint->setSensorId(sensor_id);

  absolute_constraint->setTimestampNanoseconds(msg->header.stamp.toNSec());

  if (absolute_constraint->getTimestampNanoseconds() < 0) {
    LOG(ERROR) << "Received absolute 6DoF constraint with invalid timestamp!";
    return vi_map::Absolute6DoFMeasurement::Ptr();
  }

  Eigen::Quaternion<double> eigen_quaternion;
  tf::quaternionMsgToEigen(msg->pose.pose.orientation, eigen_quaternion);

  const double kTolerance = 1e-4;
  if (!aslam::Quaternion::isValidRotationMatrix(
          eigen_quaternion.toRotationMatrix(), kTolerance)) {
    LOG(ERROR)
        << "Received absolute 6DoF constraint with invalid rotation component!";
    return vi_map::Absolute6DoFMeasurement::Ptr();
  }

  aslam::Transformation T_G_S;
  tf::poseMsgToKindr(msg->pose.pose, &T_G_S);

  absolute_constraint->set_T_G_S(T_G_S);

  // From PoseWithCovariance definition:
  // Row-major representation of the 6x6 covariance matrix
  // The orientation parameters use a fixed-axis representation.
  // In order, the parameters are:
  // (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z
  // axis)
  const Eigen::Matrix<double, 6, 6, Eigen::RowMajor> T_G_S_cov(
      msg->pose.covariance.data());

  aslam::TransformationCovariance T_G_S_fixed_covariance;
  if (sensor.get_T_G_S_fixed_covariance(&T_G_S_fixed_covariance)) {
    absolute_constraint->set_T_G_S_covariance(T_G_S_fixed_covariance);
  } else {
    absolute_constraint->set_T_G_S_covariance(T_G_S_cov);
  }

  const aslam::TransformationCovariance& T_G_S_covariance =
      absolute_constraint->get_T_G_S_covariance();

  bool is_valid_covariance = true;
  is_valid_covariance &= !T_G_S_covariance.isZero();
  is_valid_covariance &= T_G_S_covariance.allFinite();
  is_valid_covariance &= !T_G_S_covariance.hasNaN();

  if (!is_valid_covariance) {
    LOG(ERROR) << "For the absolute 6DoF constraint at time "
               << absolute_constraint->getTimestampNanoseconds()
               << " no valid covariance was provided, neither as a "
               << "fixed covariance in the sensor calibration nor "
               << "as part of the ROS message! T_G_S_covariance: "
               << T_G_S_covariance;
    return vi_map::Absolute6DoFMeasurement::Ptr();
  }

  return absolute_constraint;
}

#ifdef VOXGRAPH
void convertVoxgraphEdgeListToLoopClosureConstraint(
    const voxgraph_msgs::LoopClosureEdgeListConstPtr& lc_edges_msg,
    const vi_map::LoopClosureSensor& sensor,
    std::vector<vi_map::LoopClosureMeasurement::Ptr>* lc_edges) {
  CHECK(lc_edges_msg);
  CHECK_NOTNULL(lc_edges)->clear();

  const aslam::SensorId& sensor_id = sensor.getId();

  const size_t num_lc_edges = lc_edges_msg->loop_closure_edges.size();
  lc_edges->reserve(num_lc_edges);

  for (size_t lc_edge_idx = 0u; lc_edge_idx < num_lc_edges; ++lc_edge_idx) {
    const voxgraph_msgs::LoopClosureEdge& lc_edge =
        lc_edges_msg->loop_closure_edges[lc_edge_idx];

    vi_map::LoopClosureMeasurement::Ptr loop_closure_constraint;
    loop_closure_constraint.reset(new vi_map::LoopClosureMeasurement());

    loop_closure_constraint->setSensorId(sensor_id);
    loop_closure_constraint->setTimestampNanosecondsA(
        lc_edge.timestamp_A.toNSec());
    loop_closure_constraint->setTimestampNanosecondsB(
        lc_edge.timestamp_B.toNSec());

    if (loop_closure_constraint->getTimestampNanosecondsA() < 0 ||
        loop_closure_constraint->getTimestampNanosecondsB() < 0) {
      LOG(ERROR) << "Received loop closure constraint with invalid timestamps!";
      continue;
    }

    CHECK_GE(loop_closure_constraint->getTimestampNanosecondsA(), 0);
    CHECK_GE(loop_closure_constraint->getTimestampNanosecondsB(), 0);

    aslam::Transformation T_A_B;
    tf::poseMsgToKindr(lc_edge.T_A_B.pose, &T_A_B);

    loop_closure_constraint->set_T_A_B(T_A_B);

    // From PoseWithCovariance definition:
    // Row-major representation of the 6x6 covariance matrix
    // The orientation parameters use a fixed-axis representation.
    // In order, the parameters are:
    // (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z
    // axis)
    const Eigen::Matrix<double, 6, 6, Eigen::RowMajor> T_A_B_cov(
        lc_edge.T_A_B.covariance.data());

    aslam::TransformationCovariance T_A_B_fixed_covariance;
    if (sensor.get_T_A_B_fixed_covariance(&T_A_B_fixed_covariance)) {
      loop_closure_constraint->set_T_A_B_covariance(T_A_B_fixed_covariance);
    } else {
      loop_closure_constraint->set_T_A_B_covariance(T_A_B_cov);
    }

    const aslam::TransformationCovariance& T_A_B_covariance =
        loop_closure_constraint->get_T_A_B_covariance();

    bool is_valid_covariance = true;
    is_valid_covariance &= !T_A_B_covariance.isZero();
    is_valid_covariance &= T_A_B_covariance.allFinite();
    is_valid_covariance &= !T_A_B_covariance.hasNaN();

    if (!is_valid_covariance) {
      LOG(ERROR) << "For the loop closure constraint between time "
                 << loop_closure_constraint->getTimestampNanosecondsA()
                 << loop_closure_constraint->getTimestampNanosecondsB()
                 << " no valid covariance was provided, neither as a "
                 << "fixed covariance in the sensor calibration nor "
                 << "as part of the ROS message! T_A_B_covariance: "
                 << T_A_B_covariance;
      continue;
    }

    lc_edges->emplace_back(loop_closure_constraint);
  }
}
#endif  // VOXGRAPH
#ifdef VOXGRAPH
vi_map::RosPointCloudMapSensorMeasurement::Ptr
convertVoxgraphMapToPointCloudMap(
    const voxgraph_msgs::MapSurfaceConstPtr& msg,
    const aslam::SensorId& sensor_id) {
  CHECK(msg);
  vi_map::RosPointCloudMapSensorMeasurement::Ptr pointcloud_map(
      new vi_map::RosPointCloudMapSensorMeasurement(
          sensor_id,
          static_cast<int64_t>(msg->pointcloud.header.stamp.toNSec())));

  *(pointcloud_map->getPointCloudMutable()) = msg->pointcloud;
  return pointcloud_map;
}
#else
vi_map::RosPointCloudMapSensorMeasurement::Ptr
convertRosPointCloudToPointCloudMap(
    const sensor_msgs::PointCloud2ConstPtr& msg,
    const aslam::SensorId& sensor_id) {
  CHECK(msg);
  vi_map::RosPointCloudMapSensorMeasurement::Ptr pointcloud_map(
      new vi_map::RosPointCloudMapSensorMeasurement(
          sensor_id, static_cast<int64_t>(msg->header.stamp.toNSec())));
  *(pointcloud_map->getPointCloudMutable()) = *msg;
  return pointcloud_map;
}
#endif  // VOXGRAPH

vi_map::WheelOdometryMeasurement::Ptr convertRosOdometryToMaplabWheelOdometry(
    const nav_msgs::OdometryConstPtr& odometry_msg, aslam::SensorId sensor_id) {
  CHECK(odometry_msg);
  aslam::Transformation T_S0_St;
  tf::poseMsgToKindr(odometry_msg->pose.pose, &T_S0_St);

  // TODO(ben): add support for covariances attached to msg if desired
  vi_map::WheelOdometryMeasurement::Ptr wheel_odometry_measurement(
      new vi_map::WheelOdometryMeasurement(
          sensor_id, rosTimeToNanoseconds(odometry_msg->header.stamp), T_S0_St,
          aslam::TransformationCovariance::Identity()));

  return wheel_odometry_measurement;
}

void odometryCovarianceToEigenMatrix(
    geometry_msgs::PoseWithCovariance::_covariance_type&
        odometry_msg_covariance,
    aslam::TransformationCovariance* covariance) {
  CHECK_NOTNULL(covariance);
  *covariance =
      Eigen::Map<Eigen::MatrixXd>(odometry_msg_covariance.data(), 6, 6);
}

void eigenMatrixToOdometryCovariance(
    const aslam::TransformationCovariance& covariance,
    double* odometry_msg_covariance_data) {
  CHECK_NOTNULL(odometry_msg_covariance_data);
  Eigen::Map<Eigen::MatrixXd>(odometry_msg_covariance_data, 6, 6) = covariance;
}

}  // namespace maplab
