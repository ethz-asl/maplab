#include "voxblox_ros_interface/voxblox_bag_importer.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "voxblox_bag_importer");
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  voxblox::VoxbloxBagImporter node(nh, nh_private);

  // Get paths as ros params...
  // Can just as easily be gflags or input params, really doesn't matter.
  std::string map_path, calibration_path, bag_path, pointcloud_topic,
      pointcloud_camchain_namespace, tsdf_output_path, cam0_topic, cam1_topic,
      stereo_camchain_namespace;
  int integrate_every_nth_message = 1;
  bool generate_esdf = false;
  bool exit_at_end = false;
  nh_private.param("map_path", map_path, map_path);
  nh_private.param("calibration_path", calibration_path, calibration_path);
  nh_private.param("bag_path", bag_path, bag_path);
  nh_private.param("pointcloud_topic", pointcloud_topic, pointcloud_topic);
  nh_private.param("pointcloud_camchain_namespace",
                   pointcloud_camchain_namespace,
                   pointcloud_camchain_namespace);
  nh_private.param("cam0_topic", cam0_topic, cam0_topic);
  nh_private.param("cam1_topic", cam1_topic, cam1_topic);
  nh_private.param("stereo_camchain_namespace", stereo_camchain_namespace,
                   stereo_camchain_namespace);

  nh_private.param("generate_esdf", generate_esdf, generate_esdf);
  nh_private.param("tsdf_output_path", tsdf_output_path, tsdf_output_path);
  nh_private.param("integrate_every_nth_message", integrate_every_nth_message,
                   integrate_every_nth_message);
  nh_private.param("exit_at_end", exit_at_end, exit_at_end);

  node.setSubsampling(integrate_every_nth_message);
  node.setGenerateEsdf(generate_esdf);

  if (!node.setupRosbag(bag_path)) {
    ROS_ERROR_STREAM("Couldn't open bag " << bag_path);
    ros::shutdown();
    return 0;
  }

  if (!node.setupMap(map_path)) {
    ROS_ERROR_STREAM("Couldn't open map " << map_path << "!");
    ros::shutdown();
    return 0;
  }

  if (!pointcloud_topic.empty()) {
    node.setupPointcloudSensor(pointcloud_topic, pointcloud_camchain_namespace);
    ROS_INFO("Set up to use pointclouds!");
  } else if (!cam0_topic.empty() && !cam1_topic.empty()) {
    node.setupStereoSensor(cam0_topic, cam1_topic, stereo_camchain_namespace);
    ROS_INFO("Set up to use stereo!");
  } else {
    ROS_FATAL("Have to specify at least *ONE* sensor!");
    ros::shutdown();
    return 0;
  }

  ROS_INFO("Starting integrating data...");
  node.run();
  ROS_INFO("Finished integrating data! Integrated %llu messages out of %llu.",
           node.numMessages() / node.getSubsampling(), node.numMessages());

  node.visualize();
  ros::spinOnce();

  if (!tsdf_output_path.empty()) {
    node.save(tsdf_output_path);
    ROS_INFO_STREAM("Saved TSDF map to: " << tsdf_output_path);
  }

  if (exit_at_end) {
    ros::spinOnce();
    ros::shutdown();
  } else {
    ros::spin();
  }
  return 0;
}
