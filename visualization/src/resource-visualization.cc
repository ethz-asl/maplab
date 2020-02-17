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

namespace visualization {

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

void transformAndAppendPointcloud(
    const voxblox::Pointcloud& points_S, const voxblox::Transformation& T_G_S,
    voxblox::Pointcloud* accumulated_points_G) {
  CHECK_NOTNULL(accumulated_points_G);

  const size_t start_idx = accumulated_points_G->size();
  const size_t new_size = points_S.size() + start_idx;
  accumulated_points_G->resize(new_size);

  for (size_t i = 0u; i < points_S.size(); ++i) {
    const size_t new_idx = start_idx + i;
    CHECK_LT(new_idx, new_size);
    (*accumulated_points_G)[new_idx] = T_G_S * points_S[i];
  }
}

void convertColorPointCloud(
    const voxblox::Pointcloud& point_cloud, const voxblox::Colors& colors,
    sensor_msgs::PointCloud2* ros_point_cloud) {
  CHECK_NOTNULL(ros_point_cloud);

  if (colors.empty()) {
    backend::convertPointCloudType(point_cloud, ros_point_cloud);
  } else {
    CHECK(point_cloud.size() == colors.size());
    backend::resizePointCloud(
        point_cloud.size(), true /*input_has_color*/,
        false /*input_has_normals*/, false /*input_has_scalars*/,
        ros_point_cloud);
    for (size_t i = 0u; i < point_cloud.size(); ++i) {
      backend::addPointToPointCloud(
          point_cloud[i].cast<double>(), i, ros_point_cloud);
      const voxblox::Color& color_in = colors[i];
      resources::RgbaColor color;
      color[0] = color_in.r;
      color[1] = color_in.g;
      color[2] = color_in.b;
      color[3] = color_in.a;
      backend::addColorToPointCloud(color, i, ros_point_cloud);
    }
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

  voxblox::Pointcloud accumulated_point_cloud_G;
  voxblox::Colors accumulated_colors;

  size_t point_cloud_counter = 0u;

  srand(time(NULL));

  depth_integration::IntegrationFunction integration_function =
      [&accumulated_point_cloud_G, &accumulated_colors, &point_cloud_counter](
          const voxblox::Transformation& voxblox_T_G_S,
          const voxblox::Pointcloud& points_S,
          const voxblox::Colors& original_colors) {
        if (FLAGS_vis_pointcloud_visualize_every_nth > 0 &&
            (point_cloud_counter % FLAGS_vis_pointcloud_visualize_every_nth !=
             0u)) {
          ++point_cloud_counter;
          return;
        }

        voxblox::Colors colors;
        if (FLAGS_vis_pointcloud_color_random) {
          const voxblox::Color new_color = voxblox::randomColor();
          colors = voxblox::Colors(points_S.size(), new_color);
        } else {
          colors = original_colors;
        }

        ++point_cloud_counter;

        const aslam::Transformation T_G_S = voxblox_T_G_S.cast<double>();

        // If we just accumulate, transform to global frame and append.
        if (FLAGS_vis_pointcloud_accumulated_before_publishing) {
          transformAndAppendPointcloud(
              points_S, voxblox_T_G_S, &accumulated_point_cloud_G);
          accumulated_colors.insert(
              accumulated_colors.end(), colors.begin(), colors.end());
          return;
        }

        // Either publish in local frame with a tf or in global frame.
        if (FLAGS_vis_pointcloud_publish_in_sensor_frame_with_tf) {
          sensor_msgs::PointCloud2 ros_point_cloud_S;
          convertColorPointCloud(points_S, colors, &ros_point_cloud_S);
          publishPointCloudInLocalFrameWithTf(T_G_S, &ros_point_cloud_S);
        } else {
          sensor_msgs::PointCloud2 ros_point_cloud_G;
          voxblox::Pointcloud points_G;
          voxblox::transformPointcloud(voxblox_T_G_S, points_S, &points_G);
          convertColorPointCloud(points_G, colors, &ros_point_cloud_G);

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
  convertColorPointCloud(
      accumulated_point_cloud_G, accumulated_colors, &ros_point_cloud_G);
  publishPointCloudInGlobalFrame("" /*topic prefix*/, &ros_point_cloud_G);

  // Only continue if we want to export the accumulated point cloud to file.
  if (FLAGS_vis_pointcloud_export_accumulated_pc_to_ply_path.empty()) {
    return;
  }

  // Convert to maplab format so we can write to PLY without external
  // dependencies, e.g. PCL.
  resources::PointCloud maplab_point_cloud_G;
  backend::convertPointCloudType(ros_point_cloud_G, &maplab_point_cloud_G);

  LOG(INFO) << "Writing accumulated point cloud to file: '"
            << FLAGS_vis_pointcloud_export_accumulated_pc_to_ply_path
            << "'...'";
  maplab_point_cloud_G.writeToFile(
      FLAGS_vis_pointcloud_export_accumulated_pc_to_ply_path);
}

void createAndAppendAccumulatedPointCloudMessageForMission(
    const backend::ResourceType input_resource_type,
    const vi_map::MissionId& mission_id, const vi_map::VIMap& vi_map,
    voxblox::Pointcloud* accumulated_point_cloud_G,
    voxblox::Colors* accumulated_colors) {
  CHECK_NOTNULL(accumulated_point_cloud_G);
  CHECK_NOTNULL(accumulated_colors);
  CHECK(mission_id.isValid());
  CHECK(vi_map.hasMission(mission_id));

  uint32_t point_cloud_counter = 0u;

  srand(time(NULL));

  depth_integration::IntegrationFunction integration_function =
      [&accumulated_point_cloud_G, &accumulated_colors, &point_cloud_counter](
          const voxblox::Transformation& voxblox_T_G_S,
          const voxblox::Pointcloud& points_S,
          const voxblox::Colors& original_colors) {
        if (FLAGS_vis_pointcloud_visualize_every_nth > 0 &&
            (point_cloud_counter % FLAGS_vis_pointcloud_visualize_every_nth !=
             0u)) {
          ++point_cloud_counter;
          return;
        }

        voxblox::Colors colors;
        if (FLAGS_vis_pointcloud_color_random) {
          const voxblox::Color new_color = voxblox::randomColor();
          colors = voxblox::Colors(points_S.size(), new_color);
        } else {
          colors = original_colors;
        }

        ++point_cloud_counter;

        const aslam::Transformation T_G_S = voxblox_T_G_S.cast<double>();

        transformAndAppendPointcloud(
            points_S, voxblox_T_G_S, accumulated_point_cloud_G);
        accumulated_colors->insert(
            accumulated_colors->end(), colors.begin(), colors.end());
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

    voxblox::Pointcloud accumulated_point_cloud_G;
    voxblox::Colors accumulated_colors;
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
          input_resource_type, mission_id, vi_map, &accumulated_point_cloud_G,
          &accumulated_colors);
    }

    // Publish accumulated point cloud in global frame.
    sensor_msgs::PointCloud2 ros_point_cloud_G;
    convertColorPointCloud(
        accumulated_point_cloud_G, accumulated_colors, &ros_point_cloud_G);
    publishPointCloudInGlobalFrame(
        robot_name /*topic prefix*/, &ros_point_cloud_G);
  }
}

}  // namespace visualization
