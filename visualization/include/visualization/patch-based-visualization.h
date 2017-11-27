#ifndef VISUALIZATION_PATCH_BASED_VISUALIZATION_H_
#define VISUALIZATION_PATCH_BASED_VISUALIZATION_H_

#include <string>
#include <vector>

#include <Eigen/Core>
#include <glog/logging.h>
#include <opencv2/core/core.hpp>
#include <vi-map/unique-id.h>
#include <vi-map/vertex.h>
#include <vi-map/vi-map.h>

namespace visualization {

inline void checkIfCoordinateValid(
    int x, int y, int upper_left_x, int upper_left_y, int lower_right_x,
    int lower_right_y) {
  CHECK_GE(x, upper_left_x);
  CHECK_GE(y, upper_left_y);
  CHECK_LT(x, lower_right_x);
  CHECK_LT(y, lower_right_y);
}

// Helper functions to convert a pixel coordinate to the coordinate on
// the bloated image.
template <typename T>
inline T getBloatedCoordinate(T x, int border_size_pixels) {
  CHECK_GT(border_size_pixels, 0);
  return (x + static_cast<T>(border_size_pixels));
}

bool getImagePatch(
    const cv::Mat& image, int patch_center_x, int patch_center_y,
    int half_patch_size_pixels, cv::Mat* image_patch);

void getImagePatchesForLandmark(
    const vi_map::LandmarkId& landmark_id,
    int half_patch_size_pixels, bool draw_bounding_boxes,
    bool draw_center_circle, const vi_map::VIMap& map,
    std::vector<cv::Mat>* patches);

void getImagePatchesForLandmark(
    const vi_map::LandmarkId& landmark_id,
    int half_patch_size_pixels, bool draw_bounding_boxes,
    bool draw_center_circle, const vi_map::VIMap& map, cv::Mat* patch_row);

// Wrapper to display an image.
void visualizeImage(const cv::Mat& image, const std::string& label);

// Retrieves the image belonging to the visual frame and returns a copy of it
// as a color image. Returns false if no image is available.
bool getCopyOfImageAsColorImage(
    const aslam::VisualFrame& frame, cv::Mat* image);

// Retrieves the image belonging to the visual frame of the given camera_index
// and vertex from the given VIMap and returns a copy of it as a color image.
// Returns false if no image is available.
bool getCopyOfImageAsColorImage(
    const vi_map::Vertex& vertex, size_t camera_index, const vi_map::VIMap& map,
    cv::Mat* image);

// Same as getCopyOfImageAsColorImage(...), but returns an image with extra
// black border of border_size_pixels on each side.
bool getCopyOfImageAsBloatedColorImage(
    const aslam::VisualFrame& frame, int border_size_pixels, cv::Mat* image);

// Same as getCopyOfImageAsColorImage(...), but returns an image with extra
// black border of border_size_pixels on each side.
bool getCopyOfImageAsBloatedColorImage(
    const vi_map::Vertex& vertex, size_t camera_index, int border_size_pixels,
    const vi_map::VIMap& map, cv::Mat* image);

// Visualizes the given landmarks as rows of patches of the query patch and
// all the matched landmark observation patches from the map.
void visualizePatchesOfLandmarkKeypointMatches(
    const aslam::VisualFrame& frame,
    const std::vector<size_t>& keypoint_indices,
    const vi_map::LandmarkIdList& landmark_ids,
    const std::vector<double>& scores, int half_patch_size_pixels,
    const vi_map::VIMap& map, cv::Mat* image);

// Projects the patches of the matched landmarks onto the given visual frame
// image. The matching keypoint location and the projection location of the
// landmark are connected by a blue line.
void visualizePatchesOfProjectedMatchedLandmarksInVisualFrameImage(
    const aslam::VisualFrame& frame,
    const std::vector<size_t>& keypoint_indices,
    const vi_map::LandmarkIdList& landmark_ids,
    const Eigen::Matrix3Xd& p_C_landmarks, int half_patch_size_pixels,
    const vi_map::VIMap& map, cv::Mat* image);

// Visualizes all landmarks seen by the given vertex frame as projections
// into the visual frame.
void visualizePatchesOfLandmarksOfVertexAsProjectionsOntoTheVisualFrame(
    const pose_graph::VertexId& vertex_id, size_t frame_index,
    int half_patch_size_pixels, const vi_map::VIMap& map, cv::Mat* image);

// Visualizes the patches of the given landmarks onto the query image
// at the image locations specified in p_C_landmarks.
void visualizePatchesOfLandmarksInVisualFrameImage(
    const aslam::VisualFrame& frame,
    const vi_map::LandmarkIdList& landmark_ids,
    const Eigen::Matrix3Xd& p_C_landmarks, int half_patch_size_pixels,
    const vi_map::VIMap& map, cv::Mat* image);
}  // namespace visualization

#endif  // VISUALIZATION_PATCH_BASED_VISUALIZATION_H_
