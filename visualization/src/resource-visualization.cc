#include "visualization/resource-visualization.h"

#include <chrono>
#include <thread>

#include <Eigen/Dense>
#include <aslam/cameras/camera-factory.h>
#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/camera-unified-projection.h>
#include <aslam/cameras/distortion-null.h>
#include <aslam/common/time.h>
#include <aslam/frames/visual-frame.h>
#include <aslam/frames/visual-nframe.h>
#include <depth-integration/depth-integration.h>
#include <glog/logging.h>
#include <maplab-common/progress-bar.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vi-map/vertex.h>
#include <vi-map/vi-map.h>
#include <visualization/common-rviz-visualization.h>
#include <visualization/point-cloud-filter.h>
#include <visualization_msgs/MarkerArray.h>
#include <voxblox/core/common.h>

DEFINE_bool(
    vis_pointcloud_accumulated_before_publishing, false,
    "Accumulate the point cloud resources before publishing them all at once. "
    "Use --vis_pointcloud_visualize_every_nth to sparsify the pointcloud, "
    "otherwise it will grow too large.");
DEFINE_bool(
    vis_pointcloud_publish_in_sensor_frame_with_tf, false,
    "If enabled, the point clouds are published in the sensor frame and the "
    "transformation between sensor frame and world frame is published to the "
    "tf tree. If disabled, the pointclouds are published in the world frame. "
    "This flag is not compatible with "
    "--vis_pointcloud_accumulated_before_publishing.");
DEFINE_bool(
    vis_pointcloud_reproject_depth_maps_with_undistorted_camera, false,
    "If enabled, the undistorted depth camera is used to reproject the depth "
    "maps to point cloud. The intended use case is if a depth map is created "
    "from an image that has been undistorted, hence the undistorted camera "
    "should be used to reproject it.");

DEFINE_string(
    vis_pointcloud_export_accumulated_pc_to_ply_path, "",
    "If a path is provided, the accumulated point cloud of the visualization "
    "command is also exported to PLY file. This export only makes sense if the "
    "visualization is accumulating the point cloud, therefore enable "
    "--vis_pointcloud_accumulated_before_publishing=true.");

DEFINE_int32(
    vis_pointcloud_sleep_between_point_clouds_ms, 1u,
    "Time the visualization sleeps between publishing point clouds.");
DEFINE_int32(
    vis_pointcloud_visualize_every_nth, 1,
    "Visualize only every nth pointcloud.");

DEFINE_double(
    vis_resource_visualization_frequency, 8,
    "Frequency of the image resources visualization in Hz.");

DEFINE_bool(
    vis_pointcloud_color_random, false,
    "If enabled, every point cloud receives a random color.");

DEFINE_bool(
    vis_pointcloud_filter_dense_map_before_publishing, false,
    "If enabled, the visualized dense map will be filtered before "
    "publishing.");

DEFINE_double(
    vis_pointcloud_filter_leaf_size_m, 0.3,
    "If point cloud filtering is enabled, defines the leaf size of the voxel "
    "grid filter. ");

DEFINE_bool(
    vis_pointcloud_filter_beautify_dense_map_before_publishing, false,
    "If point cloud filtering is enabled, beautifies the point cloud.");

namespace visualization {

template <typename T_input, typename T_output>
static void applyVoxelGridFilter(
    const T_input& cloud_in, T_output* cloud_filtered) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  CHECK_NOTNULL(pcl_cloud);

  backend::convertPointCloudType(cloud_in, &(*pcl_cloud));
  voxelGridPointCloud(FLAGS_vis_pointcloud_filter_leaf_size_m, pcl_cloud);
  backend::convertPointCloudType((*pcl_cloud), cloud_filtered);
}

template <typename T_input>
static void applyBeautification(
    const T_input& cloud_in, sensor_msgs::PointCloud2* cloud_filtered) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  CHECK_NOTNULL(pcl_cloud);

  backend::convertPointCloudType(cloud_in, &(*pcl_cloud));
  beautifyPointCloud(pcl_cloud, cloud_filtered);
  backend::convertPointCloudType(*pcl_cloud, cloud_filtered);
}

bool visualizeCvMatResources(
    const vi_map::VIMap& map, backend::ResourceType type) {
  CHECK_GT(FLAGS_vis_resource_visualization_frequency, 0.0);

  VLOG(1) << "INFO: Visualization will run at "
          << FLAGS_vis_resource_visualization_frequency
          << " Hz. Hold any key to speed up.";
  pose_graph::VertexIdList vertex_ids;
  map.getAllVertexIds(&vertex_ids);
  common::ProgressBar progress_bar(vertex_ids.size());
  vi_map::MissionVertexIdList mission_to_vertex_ids;
  map.getVertexIdsByMission(&mission_to_vertex_ids);

  if (mission_to_vertex_ids.empty()) {
    VLOG(1) << "No missions found!";
    return true;
  }

  int mission_num = 0;
  for (const vi_map::MissionVertexIdPair& mission_vertex_id_pair :
       mission_to_vertex_ids) {
    VLOG(1) << "## Mission " << (mission_num + 1) << " of "
            << mission_to_vertex_ids.size() << " ##";

    const aslam::NCamera& ncamera =
        map.getMissionNCamera(mission_vertex_id_pair.first);
    std::vector<std::string> cv_window_names;
    std::unordered_set<std::string> cv_active_window_names;
    getOpenCvWindowsForNCamera(ncamera, &cv_window_names);

    for (const pose_graph::VertexId& vertex_id :
         mission_vertex_id_pair.second) {
      const vi_map::Vertex& vertex = map.getVertex(vertex_id);
      const aslam::VisualNFrame& n_frame = vertex.getVisualNFrame();
      for (uint idx = 0u; idx < n_frame.getNumFrames(); ++idx) {
        cv::Mat image_resource;
        if (map.getFrameResource(vertex, idx, type, &image_resource)) {
          cv_active_window_names.insert(cv_window_names.at(idx));
          // Rescale depth and disparity maps to make them nice to look at.
          cv::Mat rescaled_image_resource;
          switch (type) {
            case backend::ResourceType::kRawImage:
              rescaled_image_resource = image_resource;
              break;
            case backend::ResourceType::kUndistortedImage:
              rescaled_image_resource = image_resource;
              break;
            case backend::ResourceType::kRawColorImage:
              rescaled_image_resource = image_resource;
              break;
            case backend::ResourceType::kUndistortedColorImage:
              rescaled_image_resource = image_resource;
              break;
            case backend::ResourceType::kRawDepthMap:
            // Fall through intended.
            case backend::ResourceType::kOptimizedDepthMap:
              cv::normalize(
                  image_resource, rescaled_image_resource, 0, 255,
                  cv::NORM_MINMAX, CV_8U);
              break;
            case backend::ResourceType::kDisparityMap:
              double min, max;
              cv::minMaxLoc(image_resource, &min, &max);
              image_resource.convertTo(
                  rescaled_image_resource, CV_8U, -255.0 / max, 255);
              break;
            default:
              LOG(FATAL) << "Non-compatible resource type found !";
          }
          cv::imshow(cv_window_names.at(idx), rescaled_image_resource);
        } else {
          if (cv_active_window_names.find(cv_window_names.at(idx)) ==
              cv_active_window_names.end()) {
            cv::destroyWindow(cv_window_names.at(idx));
          }
        }
      }
      cv::waitKey(1000.0 / FLAGS_vis_resource_visualization_frequency);
      progress_bar.increment();
    }
    destroyAllWindows(cv_window_names);
    ++mission_num;
  }
  return true;
}

void getOpenCvWindowsForNCamera(
    const aslam::NCamera& n_camera, std::vector<std::string>* named_windows) {
  CHECK_NOTNULL(named_windows);

  for (uint i = 0u; i < n_camera.getNumCameras(); ++i) {
    const aslam::Camera& camera = n_camera.getCamera(i);
    named_windows->push_back(
        "camera_" + std::to_string(i) + "_" + camera.getId().hexString());
    cv::namedWindow(named_windows->at(i), cv::WINDOW_NORMAL);
  }
  CHECK_EQ(named_windows->size(), n_camera.getNumCameras());
}

void destroyAllWindows(const std::vector<std::string>& windows_names) {
  for (uint i = 0; i < windows_names.size(); ++i) {
    cv::destroyWindow(windows_names.at(i));
    cv::waitKey(1);
  }
}

void publishPointCloudInLocalFrameWithTf(
    const aslam::Transformation& T_G_S,
    sensor_msgs::PointCloud2* point_cloud_S) {
  CHECK_NOTNULL(point_cloud_S);
  const std::string kGenericSensorFrame = "sensor";
  point_cloud_S->header.stamp = ros::Time::now();
  point_cloud_S->header.frame_id = kGenericSensorFrame;

  Eigen::Affine3d T_G_S_eigen;
  T_G_S_eigen.matrix() = T_G_S.getTransformationMatrix();
  tf::Transform T_G_S_tf;
  tf::transformEigenToTF(T_G_S_eigen, T_G_S_tf);
  static tf::TransformBroadcaster tf_broadcaster;
  tf_broadcaster.sendTransform(tf::StampedTransform(
      T_G_S_tf, point_cloud_S->header.stamp, FLAGS_tf_map_frame,
      kGenericSensorFrame));

  const std::string kPointCloudTopic = "point_cloud";
  RVizVisualizationSink::publish(kPointCloudTopic, *point_cloud_S);
  VLOG(2) << "Visualized point-cloud with "
          << point_cloud_S->width * point_cloud_S->height << " points on topic "
          << kPointCloudTopic;
}

void publishPointCloudInGlobalFrame(
    const std::string& topic_prefix, sensor_msgs::PointCloud2* point_cloud_G) {
  CHECK_NOTNULL(point_cloud_G);
  point_cloud_G->header.stamp = ros::Time::now();
  point_cloud_G->header.frame_id = FLAGS_tf_map_frame;
  const std::string kPointCloudTopic =
      ((topic_prefix.empty()) ? "" : (topic_prefix + "/")) + "point_cloud";
  RVizVisualizationSink::publish(kPointCloudTopic, *point_cloud_G);
  VLOG(2) << "Visualized point-cloud with "
          << point_cloud_G->width * point_cloud_G->height << " points on topic "
          << kPointCloudTopic;
}

void visualizeReprojectedDepthResource(
    const backend::ResourceType input_resource_type,
    const vi_map::MissionIdList& mission_ids, const vi_map::VIMap& vi_map) {
  CHECK(!mission_ids.empty());
  CHECK_GE(FLAGS_vis_pointcloud_sleep_between_point_clouds_ms, 0);
  CHECK(
      !(FLAGS_vis_pointcloud_accumulated_before_publishing &&
        FLAGS_vis_pointcloud_publish_in_sensor_frame_with_tf));

  resources::PointCloud accumulated_point_cloud_G;

  size_t point_cloud_counter = 0u;

  srand(time(NULL));

  depth_integration::IntegrationFunctionPointCloudMaplab integration_function =
      [&accumulated_point_cloud_G, &point_cloud_counter](
          const aslam::Transformation& T_G_S,
          const resources::PointCloud& points_S) {
        if (FLAGS_vis_pointcloud_visualize_every_nth > 0 &&
            (point_cloud_counter % FLAGS_vis_pointcloud_visualize_every_nth !=
             0u)) {
          ++point_cloud_counter;
          return;
        }

        uint8_t r = 0u, g = 0u, b = 0u;
        if (FLAGS_vis_pointcloud_color_random) {
          r = rand() % 256;
          g = rand() % 256;
          b = rand() % 256;
        }

        ++point_cloud_counter;

        // If we just accumulate, transform to global frame and append.
        if (FLAGS_vis_pointcloud_accumulated_before_publishing) {
          const size_t previous_size = accumulated_point_cloud_G.size();
          accumulated_point_cloud_G.appendTransformed(points_S, T_G_S);
          const size_t new_size = accumulated_point_cloud_G.size();

          if (FLAGS_vis_pointcloud_color_random) {
            accumulated_point_cloud_G.colorizePointCloud(
                previous_size, new_size, r, g, b);
          }
          return;
        }

        if (FLAGS_vis_pointcloud_filter_dense_map_before_publishing) {
          pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(
              new pcl::PointCloud<pcl::PointXYZI>);
          CHECK_NOTNULL(pcl_cloud);
          backend::convertPointCloudType(points_S, &(*pcl_cloud));
          /*
          voxelGridPointCloud(
              FLAGS_vis_pointcloud_filter_leaf_size_m, pcl_cloud);
          backend::convertPointCloudType((*pcl_cloud), &points_S);
          */
        }

        // Either publish in local frame with a tf or in global frame.
        if (FLAGS_vis_pointcloud_publish_in_sensor_frame_with_tf) {
          sensor_msgs::PointCloud2 ros_point_cloud_S;
          if (FLAGS_vis_pointcloud_color_random) {
            resources::PointCloud points_C_colorized = points_S;
            accumulated_point_cloud_G.colorizePointCloud(r, g, b);
            sensor_msgs::PointCloud2 ros_point_cloud_S;
            backend::convertPointCloudType(
                points_C_colorized, &ros_point_cloud_S);
          } else {
            backend::convertPointCloudType(points_S, &ros_point_cloud_S);
          }
          if (FLAGS_vis_pointcloud_filter_dense_map_before_publishing) {
            applyVoxelGridFilter(ros_point_cloud_S, &ros_point_cloud_S);
          } else if (
              FLAGS_vis_pointcloud_filter_beautify_dense_map_before_publishing) {
            applyBeautification(points_S, &ros_point_cloud_S);
          }

          publishPointCloudInLocalFrameWithTf(T_G_S, &ros_point_cloud_S);
        } else {
          resources::PointCloud points_G;
          points_G.appendTransformed(points_S, T_G_S);

          if (FLAGS_vis_pointcloud_color_random) {
            points_G.colorizePointCloud(r, g, b);
          }

          sensor_msgs::PointCloud2 ros_point_cloud_G;
          if (FLAGS_vis_pointcloud_filter_dense_map_before_publishing) {
            applyVoxelGridFilter(points_G, &ros_point_cloud_G);
          } else if (
              FLAGS_vis_pointcloud_filter_beautify_dense_map_before_publishing) {
            applyBeautification(points_G, &ros_point_cloud_G);
          } else {
            backend::convertPointCloudType(points_G, &ros_point_cloud_G);
          }
          publishPointCloudInGlobalFrame(
              "" /*topic prefix*/, &ros_point_cloud_G);
        }

        // Sleep for a bit to not publish the pointclouds too fast.
        std::this_thread::sleep_for(std::chrono::milliseconds(
            FLAGS_vis_pointcloud_sleep_between_point_clouds_ms));
      };

  depth_integration::integrateAllDepthResourcesOfType(
      mission_ids, input_resource_type,
      FLAGS_vis_pointcloud_reproject_depth_maps_with_undistorted_camera, vi_map,
      integration_function);

  // If we are done and we did not accumulate the point cloud there is nothing
  // left to do.
  if (!FLAGS_vis_pointcloud_accumulated_before_publishing ||
      accumulated_point_cloud_G.empty()) {
    return;
  }

  // Publish accumulated point cloud in global frame.
  sensor_msgs::PointCloud2 ros_point_cloud_G;
  if (FLAGS_vis_pointcloud_filter_dense_map_before_publishing) {
    applyVoxelGridFilter(accumulated_point_cloud_G, &ros_point_cloud_G);
  } else if (FLAGS_vis_pointcloud_filter_beautify_dense_map_before_publishing) {
    applyBeautification(accumulated_point_cloud_G, &ros_point_cloud_G);
  } else {
    backend::convertPointCloudType(
        accumulated_point_cloud_G, &ros_point_cloud_G);
  }
  publishPointCloudInGlobalFrame("" /*topic prefix*/, &ros_point_cloud_G);

  // Only continue if we want to export the accumulated point cloud to file.
  if (FLAGS_vis_pointcloud_export_accumulated_pc_to_ply_path.empty()) {
    return;
  }

  LOG(INFO) << "Writing accumulated point cloud to file: '"
            << FLAGS_vis_pointcloud_export_accumulated_pc_to_ply_path
            << "'...'";
  accumulated_point_cloud_G.writeToFile(
      FLAGS_vis_pointcloud_export_accumulated_pc_to_ply_path);
}

void createAndAppendAccumulatedPointCloudMessageForMission(
    const backend::ResourceType input_resource_type,
    const vi_map::MissionId& mission_id, const vi_map::VIMap& vi_map,
    resources::PointCloud* accumulated_point_cloud_G) {
  CHECK_NOTNULL(accumulated_point_cloud_G);
  CHECK(mission_id.isValid());
  CHECK(vi_map.hasMission(mission_id));

  uint32_t point_cloud_counter = 0u;

  srand(time(NULL));

  depth_integration::IntegrationFunctionPointCloudMaplab integration_function =
      [&accumulated_point_cloud_G, &point_cloud_counter](
          const aslam::Transformation& T_G_S,
          const resources::PointCloud& points_S) {
        if (FLAGS_vis_pointcloud_visualize_every_nth > 0 &&
            (point_cloud_counter % FLAGS_vis_pointcloud_visualize_every_nth !=
             0u)) {
          ++point_cloud_counter;
          return;
        }

        uint8_t r = 0u, g = 0u, b = 0u;
        if (FLAGS_vis_pointcloud_color_random) {
          r = rand() % 256;
          g = rand() % 256;
          b = rand() % 256;
        }

        ++point_cloud_counter;

        const size_t previous_size = accumulated_point_cloud_G->size();
        accumulated_point_cloud_G->appendTransformed(points_S, T_G_S);
        const size_t new_size = accumulated_point_cloud_G->size();

        if (FLAGS_vis_pointcloud_color_random) {
          accumulated_point_cloud_G->colorizePointCloud(
              previous_size, new_size, r, g, b);
        }
        return;
      };

  vi_map::MissionIdList mission_ids;
  mission_ids.emplace_back(mission_id);
  depth_integration::integrateAllDepthResourcesOfType(
      mission_ids, input_resource_type,
      FLAGS_vis_pointcloud_reproject_depth_maps_with_undistorted_camera, vi_map,
      integration_function);
}

void visualizeReprojectedDepthResourcePerRobot(
    const backend::ResourceType input_resource_type,
    const std::unordered_map<std::string, vi_map::MissionIdList>
        robot_name_to_mission_ids_map,
    const vi_map::VIMap& vi_map) {
  CHECK_GE(FLAGS_vis_pointcloud_sleep_between_point_clouds_ms, 0);
  CHECK(
      !(FLAGS_vis_pointcloud_accumulated_before_publishing &&
        FLAGS_vis_pointcloud_publish_in_sensor_frame_with_tf));

  // Accumulate all the point clouds of the robot missoins. If there are no
  // missions in the map, a empty point cloud will be published.
  for (const auto& kv : robot_name_to_mission_ids_map) {
    const std::string& robot_name = kv.first;
    const vi_map::MissionIdList& mission_ids = kv.second;

    if (robot_name.empty()) {
      LOG(ERROR) << "Cannot visualize point clouds for robot missions, because "
                 << "the robot name is empty (name: '" << robot_name << "')";
      continue;
    }

    resources::PointCloud accumulated_point_cloud_G;
    for (const vi_map::MissionId& mission_id : mission_ids) {
      if (!mission_id.isValid()) {
        LOG(ERROR) << "Cannot visualize one mission of robot '" << robot_name
                   << "' since the provided mission id is invalid!'";
        continue;
      }

      if (!vi_map.hasMission(mission_id)) {
        LOG(ERROR) << "Cannot visualize one mission of robot '" << robot_name
                   << "' since the provided mission id ('" << mission_id
                   << "') is not in the map!'";
        continue;
      }
      createAndAppendAccumulatedPointCloudMessageForMission(
          input_resource_type, mission_id, vi_map, &accumulated_point_cloud_G);
    }

    if (accumulated_point_cloud_G.empty()) {
      return;
    }
    // Publish accumulated point cloud in global frame.
    sensor_msgs::PointCloud2 ros_point_cloud_G;
    if (FLAGS_vis_pointcloud_filter_dense_map_before_publishing) {
      applyVoxelGridFilter(accumulated_point_cloud_G, &ros_point_cloud_G);
    } else if (
        FLAGS_vis_pointcloud_filter_beautify_dense_map_before_publishing) {
      applyBeautification(accumulated_point_cloud_G, &ros_point_cloud_G);
    } else {
      backend::convertPointCloudType(
          accumulated_point_cloud_G, &ros_point_cloud_G);
    }

    publishPointCloudInGlobalFrame(
        robot_name /*topic prefix*/, &ros_point_cloud_G);
  }
}

}  // namespace visualization
