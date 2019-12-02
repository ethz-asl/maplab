#include "rovioli/data-publisher-flow.h"
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <maplab-common/conversions.h>
#include <maplab-common/file-logger.h>
#include <maplab_msgs/OdometryWithImuBiases.h>
#include <minkindr_conversions/kindr_msg.h>
#include "rovioli/ros-helpers.h"

DEFINE_double(
    map_publish_interval_s, 2.0,
    "Interval of publishing the visual-inertial map to ROS [seconds].");

DEFINE_bool(
    publish_only_on_keyframes, false,
    "Publish frames only on keyframes instead of the IMU measurements. This "
    "means a lower frequency.");

DEFINE_bool(
    publish_debug_markers, true,
    "Publish debug sphere markers for T_M_I, T_G_I and localization frames.");

DEFINE_string(
    export_estimated_poses_to_csv, "",
    "If not empty, the map builder will export the estimated poses to a CSV "
    "file.");

DEFINE_bool(
    rovioli_visualize_map, true,
    "Set to false to disable map visualization. Note: map building needs to be "
    "active for the visualization.");

DECLARE_bool(rovioli_run_map_builder);


namespace rovioli {
namespace {
inline ros::Time createRosTimestamp(int64_t timestamp_nanoseconds) {
  static constexpr uint32_t kNanosecondsPerSecond = 1e9;
  const uint64_t timestamp_u64 = static_cast<uint64_t>(timestamp_nanoseconds);
  const uint32_t ros_timestamp_sec = timestamp_u64 / kNanosecondsPerSecond;
  const uint32_t ros_timestamp_nsec =
      timestamp_u64 - (ros_timestamp_sec * kNanosecondsPerSecond);
  return ros::Time(ros_timestamp_sec, ros_timestamp_nsec);
}
}  // namespace

DataPublisherFlow::DataPublisherFlow()
    : map_publisher_timeout_(common::TimeoutCounter(
          FLAGS_map_publish_interval_s * kSecondsToNanoSeconds)) {
  visualization::RVizVisualizationSink::init();
  plotter_.reset(new visualization::ViwlsGraphRvizPlotter);
}

void DataPublisherFlow::registerPublishers() {
  pub_pose_T_M_I_ =
      node_handle_.advertise<geometry_msgs::PoseStamped>(kTopicPoseMission, 1);
  pub_pose_T_G_I_ =
      node_handle_.advertise<geometry_msgs::PoseStamped>(kTopicPoseGlobal, 1);
  pub_transform_T_G_I_ =
      node_handle_.advertise<geometry_msgs::TransformStamped>(
          kTopicTransformGlobal, 1);
  pub_baseframe_T_G_M_ =
      node_handle_.advertise<geometry_msgs::PoseStamped>(kTopicBaseframe, 1);
  pub_velocity_I_ =
      node_handle_.advertise<geometry_msgs::Vector3Stamped>(kTopicVelocity, 1);
  pub_imu_acc_bias_ =
      node_handle_.advertise<geometry_msgs::Vector3Stamped>(kTopicBiasAcc, 1);
  pub_imu_gyro_bias_ =
      node_handle_.advertise<geometry_msgs::Vector3Stamped>(kTopicBiasGyro, 1);
  pub_extrinsics_T_C_Bs_ = node_handle_.advertise<geometry_msgs::PoseArray>(
      kCameraExtrinsicTopic, 1);
  pub_maplab_odom_T_M_I_ =
      node_handle_.advertise<maplab_msgs::OdometryWithImuBiases>(
          kTopicMaplabOdomMsg, 1);
}

void DataPublisherFlow::attachToMessageFlow(message_flow::MessageFlow* flow) {
  CHECK_NOTNULL(flow);
  registerPublishers();
  static constexpr char kSubscriberNodeName[] = "DataPublisherFlow";

  if (FLAGS_rovioli_run_map_builder && FLAGS_rovioli_visualize_map) {
    flow->registerSubscriber<message_flow_topics::RAW_VIMAP>(
        kSubscriberNodeName, message_flow::DeliveryOptions(),
        [this](const VIMapWithMutex::ConstPtr& map_with_mutex) {
          if (map_publisher_timeout_.reached()) {
            std::lock_guard<std::mutex> lock(map_with_mutex->mutex);
            visualizeMap(map_with_mutex->vi_map);
            map_publisher_timeout_.reset();
          }
        });
  }

  // Publish localization results.
  flow->registerSubscriber<message_flow_topics::LOCALIZATION_RESULT>(
      kSubscriberNodeName, message_flow::DeliveryOptions(),
      [this](const vio::LocalizationResult::ConstPtr& localization) {
        CHECK(localization != nullptr);
        localizationCallback(localization->T_G_B.getPosition());
      });

  if (FLAGS_publish_only_on_keyframes) {
    flow->registerSubscriber<message_flow_topics::MAP_UPDATES>(
        kSubscriberNodeName, message_flow::DeliveryOptions(),
        [this](const vio::MapUpdate::ConstPtr& vio_update) {
          CHECK(vio_update != nullptr);
          bool has_T_G_M =
              (vio_update->localization_state ==
                   common::LocalizationState::kLocalized ||
               vio_update->localization_state ==
                   common::LocalizationState::kMapTracking);
          publishVinsState(
              vio_update->timestamp_ns, vio_update->vinode, has_T_G_M,
              vio_update->T_G_M);
        });
  } else {
    flow->registerSubscriber<message_flow_topics::ROVIO_ESTIMATES>(
        kSubscriberNodeName, message_flow::DeliveryOptions(),
        [this](const RovioEstimate::ConstPtr& state) {
          CHECK(state != nullptr);
          publishVinsState(
              state->timestamp_ns, state->vinode, state->has_T_G_M,
              state->T_G_M);

          // Publish estimated camera-extrinsics.
          geometry_msgs::PoseArray T_C_Bs_message;
          T_C_Bs_message.header.frame_id = FLAGS_tf_imu_frame;
          T_C_Bs_message.header.stamp = createRosTimestamp(state->timestamp_ns);
          for (const auto& cam_idx_T_C_B :
               state->maplab_camera_index_to_T_C_B) {
            geometry_msgs::Pose T_C_B_message;
            tf::poseKindrToMsg(cam_idx_T_C_B.second, &T_C_B_message);
            T_C_Bs_message.poses.emplace_back(T_C_B_message);
          }
          pub_extrinsics_T_C_Bs_.publish(T_C_Bs_message);
        });
  }

  // CSV export for end-to-end test.
  if (!FLAGS_export_estimated_poses_to_csv.empty()) {
    // Lambda function will take ownership.
    std::shared_ptr<common::FileLogger> file_logger =
        std::make_shared<common::FileLogger>(
            FLAGS_export_estimated_poses_to_csv);
    constexpr char kDelimiter[] = ", ";
    file_logger->writeDataWithDelimiterAndNewLine(
        kDelimiter, "# Timestamp [s]", "t_G_M x [m]", "t_G_M y [m]",
        "t_G_M z [m]", "q_G_M x", "q_G_M y", "q_G_M z", "q_G_M w",
        "t_M_I x [m]", "t_M_I y [m]", "t_M_I z [m]", "q_M_I x", "q_M_I y",
        "q_M_I z", "q_M_I w", "has T_G_M");
    CHECK(file_logger != nullptr);
    flow->registerSubscriber<message_flow_topics::MAP_UPDATES>(
        kSubscriberNodeName, message_flow::DeliveryOptions(),
        [file_logger, kDelimiter,
         this](const vio::MapUpdate::ConstPtr& vio_update) {
          CHECK(vio_update != nullptr);
          const bool has_T_G_M = vio_update->localization_state ==
                                 common::LocalizationState::kLocalized;
          if (has_T_G_M) {
            latest_T_G_M_ = vio_update->T_G_M;
          }
          const aslam::Transformation T_M_I = vio_update->vinode.get_T_M_I();

          file_logger->writeDataWithDelimiterAndNewLine(
              kDelimiter,
              aslam::time::nanoSecondsToSeconds(vio_update->timestamp_ns),
              latest_T_G_M_.getPosition(), latest_T_G_M_.getEigenQuaternion(),
              T_M_I.getPosition(), T_M_I.getEigenQuaternion(), has_T_G_M);
        });
  }
}

void DataPublisherFlow::visualizeMap(const vi_map::VIMap& vi_map) const {
  static constexpr bool kPublishBaseframes = true;
  static constexpr bool kPublishVertices = true;
  static constexpr bool kPublishEdges = true;
  static constexpr bool kPublishLandmarks = true;
  static constexpr bool kPublishAbsolute6DofConstraints = true;
  plotter_->visualizeMap(
      vi_map, kPublishBaseframes, kPublishVertices, kPublishEdges,
      kPublishLandmarks, kPublishAbsolute6DofConstraints);
}

void DataPublisherFlow::publishVinsState(
    int64_t timestamp_ns, const vio::ViNodeState& vinode, const bool has_T_G_M,
    const aslam::Transformation& T_G_M) {
  ros::Time timestamp_ros = createRosTimestamp(timestamp_ns);

  // Check whether publishing is needed.
  const bool maplab_odom_should_publish 
    = pub_maplab_odom_T_M_I_.getNumSubscribers() > 0;
  const bool pose_T_M_I_should_publish 
    = pub_pose_T_M_I_.getNumSubscribers() > 0;
  const bool velocity_I_should_publish
    = pub_velocity_I_.getNumSubscribers() > 0;
  const bool imu_acc_bias_should_publish
    = pub_imu_acc_bias_.getNumSubscribers() > 0;
  const bool imu_gyro_bias_should_publish
    = pub_imu_gyro_bias_.getNumSubscribers() > 0;

  // Publish pose in mission frame.
  maplab_msgs::OdometryWithImuBiases maplab_odom_T_M_I;
  const aslam::Transformation& T_M_I = vinode.get_T_M_I();
  if (pose_T_M_I_should_publish || maplab_odom_should_publish) {
    geometry_msgs::PoseStamped T_M_I_message;
    tf::poseStampedKindrToMsg(
        T_M_I, timestamp_ros, FLAGS_tf_mission_frame, &T_M_I_message);
    if (pose_T_M_I_should_publish) {
      pub_pose_T_M_I_.publish(T_M_I_message);
    }  
    maplab_odom_T_M_I.header = T_M_I_message.header;
    maplab_odom_T_M_I.child_frame_id = FLAGS_tf_imu_frame;
    maplab_odom_T_M_I.pose.pose = T_M_I_message.pose;
    eigenMatrixToOdometryCovariance(
        vinode.getPoseCovariance(), maplab_odom_T_M_I.pose.covariance.data());
    maplab_odom_T_M_I.odometry_state = 0u;  //  = OK
  }
  visualization::publishTF(
      T_M_I, FLAGS_tf_mission_frame, FLAGS_tf_imu_frame, timestamp_ros);

  // Publish pose in global frame.
  aslam::Transformation T_G_I = T_G_M * T_M_I;
  if (pub_pose_T_G_I_.getNumSubscribers() > 0) {
    geometry_msgs::PoseStamped T_G_I_message;
    tf::poseStampedKindrToMsg(
        T_G_I, timestamp_ros, FLAGS_tf_map_frame, &T_G_I_message);
    pub_pose_T_G_I_.publish(T_G_I_message);
  }

  // Publish transform in global frame.
  if (pub_transform_T_G_I_.getNumSubscribers() > 0) {
    geometry_msgs::TransformStamped Transform_G_I_message;
    Transform_G_I_message.child_frame_id = FLAGS_tf_imu_frame;
    Transform_G_I_message.header.stamp = timestamp_ros;
    tf::transformKindrToMsg(T_G_I, &Transform_G_I_message.transform);
    pub_transform_T_G_I_.publish(Transform_G_I_message);
  }

  // Publish baseframe transformation.
  if (pub_baseframe_T_G_M_.getNumSubscribers() > 0) {
    geometry_msgs::PoseStamped T_G_M_message;
    tf::poseStampedKindrToMsg(
        T_G_M, timestamp_ros, FLAGS_tf_map_frame, &T_G_M_message);
    pub_baseframe_T_G_M_.publish(T_G_M_message);
  }
  visualization::publishTF(
      T_G_M, FLAGS_tf_map_frame, FLAGS_tf_mission_frame, timestamp_ros);

  // Publish velocity.
  if (velocity_I_should_publish || maplab_odom_should_publish) {
    const Eigen::Vector3d& v_M_I = vinode.get_v_M_I();
    geometry_msgs::Vector3Stamped velocity_msg;
    velocity_msg.header.stamp = timestamp_ros;
    velocity_msg.vector.x = v_M_I[0];
    velocity_msg.vector.y = v_M_I[1];
    velocity_msg.vector.z = v_M_I[2];
    // Also copy the velocity to the maplab odom message
    maplab_odom_T_M_I.twist.twist.linear = velocity_msg.vector;
    if (velocity_I_should_publish) {
      pub_velocity_I_.publish(velocity_msg);
    }
    // add the velocity covariance terms
    eigenMatrixToOdometryCovariance(
        vinode.getTwistCovariance(), maplab_odom_T_M_I.twist.covariance.data());
  }

  // Publish IMU bias.
  if (imu_acc_bias_should_publish || maplab_odom_should_publish 
       || imu_gyro_bias_should_publish) {
    geometry_msgs::Vector3Stamped bias_msg;
    bias_msg.header.stamp = timestamp_ros;
    Eigen::Matrix<double, 6, 1> imu_bias_acc_gyro = vinode.getImuBias();
    bias_msg.vector.x = imu_bias_acc_gyro[0];
    bias_msg.vector.y = imu_bias_acc_gyro[1];
    bias_msg.vector.z = imu_bias_acc_gyro[2];
    // Also copy the bias to the maplab odom message
    maplab_odom_T_M_I.accel_bias = bias_msg.vector;
    if (imu_acc_bias_should_publish) {
      pub_imu_acc_bias_.publish(bias_msg);
    }

    bias_msg.vector.x = imu_bias_acc_gyro[3];
    bias_msg.vector.y = imu_bias_acc_gyro[4];
    bias_msg.vector.z = imu_bias_acc_gyro[5];
    // Also copy the bias to the maplab odom message
    maplab_odom_T_M_I.gyro_bias = bias_msg.vector;
    if (imu_gyro_bias_should_publish) {
      pub_imu_gyro_bias_.publish(bias_msg);
    }
  }

  if (maplab_odom_should_publish) {
    pub_maplab_odom_T_M_I_.publish(maplab_odom_T_M_I);
  }

  if (FLAGS_publish_debug_markers) {
    stateDebugCallback(vinode, has_T_G_M, T_G_M);
  }
}

void DataPublisherFlow::stateDebugCallback(
    const vio::ViNodeState& vinode, const bool has_T_G_M,
    const aslam::Transformation& T_G_M) {
  constexpr size_t kMarkerId = 0u;
  visualization::Sphere sphere;
  const aslam::Transformation& T_M_I = vinode.get_T_M_I();
  sphere.position = T_M_I.getPosition();
  sphere.radius = 0.2;
  sphere.color = visualization::kCommonGreen;
  sphere.alpha = 0.8;
  T_M_I_spheres_.push_back(sphere);
  visualization::publishSpheres(
      T_M_I_spheres_, kMarkerId, FLAGS_tf_map_frame, "debug", "debug_T_M_I");

  // Publish ROVIO velocity
  const aslam::Position3D& t_M_I = T_M_I.getPosition();
  const Eigen::Vector3d& v_M_I = vinode.get_v_M_I();
  visualization::Arrow arrow;
  arrow.from = t_M_I;
  arrow.to = t_M_I + v_M_I;
  arrow.scale = 0.3;
  arrow.color = visualization::kCommonRed;
  arrow.alpha = 0.8;
  visualization::publishArrow(
      arrow, kMarkerId, FLAGS_tf_map_frame, "debug", "debug_v_M_I_");

  // Publish ROVIO global frame if it is available.
  if (has_T_G_M) {
    aslam::Transformation T_G_I = T_G_M * T_M_I;
    visualization::Sphere sphere;
    sphere.position = T_G_I.getPosition();
    sphere.radius = 0.2;
    sphere.color = visualization::kCommonWhite;
    sphere.alpha = 0.8;
    T_G_I_spheres_.push_back(sphere);

    visualization::publishSpheres(
        T_G_I_spheres_, kMarkerId, FLAGS_tf_map_frame, "debug", "debug_T_G_I");
  }
}

void DataPublisherFlow::localizationCallback(
    const Eigen::Vector3d& p_G_I_lc_pnp) {
  visualization::Sphere sphere;
  sphere.position = p_G_I_lc_pnp;
  sphere.radius = 0.2;
  sphere.color = visualization::kCommonRed;
  sphere.alpha = 0.8;
  T_G_I_loc_spheres_.push_back(sphere);

  constexpr size_t kMarkerId = 0u;
  visualization::publishSpheres(
      T_G_I_loc_spheres_, kMarkerId, FLAGS_tf_map_frame, "debug",
      "debug_T_G_I_raw_localizations");
}

}  //  namespace rovioli
