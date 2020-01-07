#ifndef VISUALIZATION_RESOURCE_VISUALIZATION_H_
#define VISUALIZATION_RESOURCE_VISUALIZATION_H_

#include <algorithm>
#include <chrono>
#include <fstream>  // NOLINT
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <aslam/cameras/camera-factory.h>
#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/camera-unified-projection.h>
#include <aslam/cameras/distortion-null.h>
#include <aslam/cameras/ncamera.h>
#include <aslam/common/time.h>
#include <aslam/frames/visual-frame.h>
#include <aslam/frames/visual-nframe.h>
#include <cv_bridge/cv_bridge.h>
#include <glog/logging.h>
#include <image_transport/image_transport.h>
#include <landmark-triangulation/pose-interpolator.h>
#include <map-resources/resource-conversion.h>
#include <maplab-common/progress-bar.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <vi-map/vertex.h>
#include <vi-map/vi-map.h>
#include <visualization/common-rviz-visualization.h>
#include <voxblox/core/common.h>

DECLARE_bool(vis_pointcloud_accumulated_before_publishing);

DECLARE_bool(vis_pointcloud_publish_in_sensor_frame_with_tf);

DECLARE_bool(vis_pointcloud_reproject_depth_maps_with_undistorted_camera);

DECLARE_double(vis_pointcloud_publishing_real_time_factor);

DECLARE_int32(vis_pointcloud_visualize_every_nth);

DECLARE_string(vis_pointcloud_export_accumulated_pc_to_ply_path);

namespace visualization {

bool visualizeCvMatResources(
    const vi_map::VIMap& map, backend::ResourceType type);
typedef std::unordered_map<aslam::CameraId, aslam::Camera::Ptr> CameraCache;

void visualizeReprojectedDepthResource(
    const backend::ResourceType type, const vi_map::MissionIdList& mission_ids,
    const vi_map::VIMap& map);

void getOpenCvWindowsForNCamera(
    const aslam::NCamera& n_camera, std::vector<std::string>* named_windows);

void destroyAllWindows(const std::vector<std::string>& windows_names);

}  // namespace visualization

#endif  // VISUALIZATION_RESOURCE_VISUALIZATION_H_
