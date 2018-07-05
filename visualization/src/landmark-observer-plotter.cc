#include "visualization/landmark-observer-plotter.h"

#include <Eigen/Core>
#include <maplab-common/sigint-breaker.h>
#include <ros/ros.h>
#include <vi-map-helpers/vi-map-geometry.h>
#include <vi-map/landmark.h>

#include "visualization/color-palette.h"
#include "visualization/common-rviz-visualization.h"

DECLARE_double(vis_scale);

namespace visualization {

const std::string LandmarkObserverPlotter::kClickedPointTopic =
    "/clicked_point";
const std::string LandmarkObserverPlotter::kLandmarkObserverRaysTopic =
    "landmark_observer_rays";

LandmarkObserverPlotter::LandmarkObserverPlotter(const vi_map::VIMap& vi_map)
    : vi_map_(vi_map), landmark_database_(vi_map) {}

void LandmarkObserverPlotter::visualizeClickedLandmarks() const {
  LOG(INFO) << "Visualizing selected landmarks. Use the \"Publish Point\" tool "
               "in Rviz to select landmarks. Stop plotting with Ctrl+C.";
  CHECK(ros::isInitialized());
  ros::NodeHandle node_handle("landmark_observer_plotter");
  constexpr uint32_t kQueueSize = 100u;
  // Circumvent the non-const [T* obj] argument in NodeHandle::subscribe(...).
  LandmarkObserverPlotter* mutable_object_pointer =
      const_cast<LandmarkObserverPlotter*>(this);
  const ros::Subscriber clicked_point_subscriber = node_handle.subscribe(
      kClickedPointTopic, kQueueSize,
      &LandmarkObserverPlotter::clickedPointCallback, mutable_object_pointer);
  common::SigintBreaker sigint_breaker;
  while (!sigint_breaker.isBreakRequested()) {
    ros::spinOnce();
  }
}

void LandmarkObserverPlotter::clickedPointCallback(
    const geometry_msgs::PointStamped::ConstPtr& clicked_point_msg) const {
  const std::string& map_frame = clicked_point_msg->header.frame_id;
  if (map_frame != kDefaultMapFrame) {
    LOG(ERROR) << "Clicked point given in frame " << map_frame << " instead of "
               << kDefaultMapFrame << ".";
    return;
  }
  const Eigen::Vector3d clicked_point_p_G = {clicked_point_msg->point.x,
                                             clicked_point_msg->point.y,
                                             clicked_point_msg->point.z};
  vi_map::LandmarkId nearest_landmark_id;
  if (!landmark_database_.getClosestDataItem(
          clicked_point_p_G, &nearest_landmark_id)) {
    LOG(ERROR) << "No closest landmark found.";
    return;
  } else {
    LOG(INFO) << "Got landmark " << nearest_landmark_id.hexString()
              << " at position (" << clicked_point_p_G.x() << ","
              << clicked_point_p_G.y() << "," << clicked_point_p_G.z() << ").";
  }
  CHECK(nearest_landmark_id.isValid());
  const vi_map::Landmark& landmark = vi_map_.getLandmark(nearest_landmark_id);
  if (landmark.getQuality() == vi_map::Landmark::Quality::kBad) {
    LOG(WARNING) << "The clicked landmark is of bad quality. This may happen "
                    "because the Rviz \"Publish Point\" method does not "
                    "publish the exact coordinates of a clicked landmark. Try "
                    "to zoom in to re-select the landmark.";
  }
  vi_map::VisualFrameIdentifierSet observer_frames;
  vi_map_.getVisualFrameIdentifiersForLandmark(
      nearest_landmark_id, &observer_frames);
  const size_t num_observers = observer_frames.size();
  CHECK_GT(num_observers, 0u);
  constexpr int kPositionDimensions = 3;
  Eigen::Matrix3Xd observer_rays_p_G_C(kPositionDimensions, num_observers);
  Eigen::Matrix3Xd observer_rays_p_G_l(kPositionDimensions, num_observers);
  const vi_map_helpers::VIMapGeometry vi_map_geometry(vi_map_);
  size_t col_idx = 0u;
  for (const vi_map::VisualFrameIdentifier& frame_id : observer_frames) {
    CHECK(frame_id.isValid());
    const pose::Transformation T_G_C =
        vi_map_geometry.getVisualFrame_T_G_C(frame_id);
    observer_rays_p_G_C.col(col_idx) = T_G_C.getPosition();
    observer_rays_p_G_l.col(col_idx) = vi_map_.get_p_G(nearest_landmark_id);
    ++col_idx;
  }
  const visualization::ColorList colors(
      num_observers, visualization::kCommonWhite);
  constexpr double kAlpha = 0.8;
  const double scale = 0.002 * FLAGS_vis_scale;
  constexpr size_t kMarkerId = 0u;
  publishLines(
      observer_rays_p_G_C, observer_rays_p_G_l, colors, kAlpha, scale,
      kMarkerId, kDefaultMapFrame, kDefaultNamespace,
      kLandmarkObserverRaysTopic);
}

}  // namespace visualization
