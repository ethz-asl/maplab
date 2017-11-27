#include "visualization/feature-matches-visualization.h"

#include <vector>

#include <Eigen/Dense>
#include <aslam/cameras/camera.h>
#include <aslam/matcher/match-helpers.h>
#include <aslam/matcher/match-visualization.h>
#include <glog/logging.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace visualization {

void saveOpenCvMatchesAndFeaturesAsImage(
    const aslam::VisualFrame& frame_A, const cv::Mat& image_A,
    const aslam::VisualFrame& frame_B, const cv::Mat& image_B,
    const aslam::OpenCvMatches& matches_A_B, const std::string& filename,
    aslam::FeatureVisualizationType visualization_type, vi_map::VIMap* map) {
  CHECK_NOTNULL(map);

  if (matches_A_B.empty()) {
    VLOG(1) << "No matches found.";
  }

  if (frame_A.getNumKeypointMeasurements() == 0 ||
      frame_B.getNumKeypointMeasurements() == 0) {
    VLOG(1) << "No features found.";
    return;
  }

  // Scale the keypoint score to sth we can visualize.
  const Eigen::Matrix2Xd& key_point_matrix_A(frame_A.getKeypointMeasurements());
  const Eigen::Matrix2Xd& key_point_matrix_B(frame_B.getKeypointMeasurements());
  const Eigen::VectorXd& key_point_scores_A = frame_A.getKeypointScores();
  const Eigen::VectorXd& key_point_scores_B = frame_B.getKeypointScores();
  const double max_score =
      std::max(key_point_scores_A.maxCoeff(), key_point_scores_B.maxCoeff());
  const double min_score =
      std::min(key_point_scores_A.minCoeff(), key_point_scores_B.minCoeff());
  const double score_range = std::max(max_score - min_score, 1.0);
  const double max_key_point_size = 30.0;
  const double min_key_point_size = 5.0;

  const Eigen::VectorXd& key_point_orientation_A =
      frame_A.getKeypointOrientations();
  const Eigen::VectorXd& key_point_orientation_B =
      frame_B.getKeypointOrientations();

  // Convert the keypoints to cv::KeyPoint.
  std::vector<cv::KeyPoint> key_points_A, key_points_B;
  for (uint i = 0; i < frame_A.getNumKeypointMeasurements(); ++i) {
    double key_point_size = std::max(
        (key_point_scores_A(i) - min_score) / score_range * max_key_point_size,
        min_key_point_size);
    key_points_A.emplace_back(
        cv::KeyPoint(
            key_point_matrix_A(0, i), key_point_matrix_A(1, i), key_point_size,
            key_point_orientation_A(i)));
  }
  for (uint i = 0; i < frame_B.getNumKeypointMeasurements(); ++i) {
    double key_point_size = std::max(
        (key_point_scores_B(i) - min_score) / score_range * max_key_point_size,
        min_key_point_size);
    key_points_B.emplace_back(
        cv::KeyPoint(
            key_point_matrix_B(0, i), key_point_matrix_B(1, i), key_point_size,
            key_point_orientation_B(i)));
  }

  // Convert color images to grayscale if necessary.
  cv::Mat grayscale_image_A, grayscale_image_B;
  bool is_color_image = image_A.channels() == 3;
  if (is_color_image) {
    cv::cvtColor(image_A, grayscale_image_A, CV_BGR2GRAY);
    cv::cvtColor(image_B, grayscale_image_B, CV_BGR2GRAY);
  } else {
    grayscale_image_A = image_A;
    grayscale_image_B = image_B;
  }

  // Write features and matches to image file.
  cv::Mat images_w_matches;
  aslam::drawKeyPointsAndMatches(
      grayscale_image_A, key_points_A, grayscale_image_B, key_points_B,
      matches_A_B, visualization_type, &images_w_matches);
  cv::imwrite(filename, images_w_matches);
}

void saveOpenCvMatchesAndFeaturesAsImage(
    const aslam::VisualFrame& frame_A, const aslam::VisualFrame& frame_B,
    const aslam::OpenCvMatches& matches_A_B, const std::string& filename,
    aslam::FeatureVisualizationType visualization_type, vi_map::VIMap* map) {
  CHECK_NOTNULL(map);

  if (!frame_A.hasRawImage() || !frame_B.hasRawImage()) {
    LOG(ERROR) << "One of the frames has no image!";
    return;
  }

  cv::Mat image_A = frame_A.getRawImage();
  cv::Mat image_B = frame_B.getRawImage();

  if (image_A.empty() || image_B.empty()) {
    LOG(ERROR) << "One of the images is empty!";
    return;
  }

  saveOpenCvMatchesAndFeaturesAsImage(
      frame_A, image_A, frame_B, image_B, matches_A_B, filename,
      visualization_type, map);
}

void saveOpenCvMatchesAndFeaturesAsImage(
    const aslam::VisualFrame& frame_A, const vi_map::Vertex& vertex_A,
    const aslam::VisualFrame& frame_B, const vi_map::Vertex& vertex_B,
    const aslam::OpenCvMatches& matches_A_B, const std::string& filename,
    aslam::FeatureVisualizationType visualization_type,
    const backend::ResourceType image_resource_type, vi_map::VIMap* map) {
  CHECK_NOTNULL(map);

  const unsigned int current_frame_idx =
      vertex_A.getVisualFrameIndex(frame_A.getId());
  const unsigned int next_frame_idx =
      vertex_B.getVisualFrameIndex(frame_B.getId());

  // Load image resources.
  cv::Mat image_A, image_B;
  if (!map->getFrameResource<cv::Mat>(
          vertex_A, current_frame_idx, image_resource_type, &image_A)) {
    LOG(ERROR) << "Frame " << current_frame_idx << " of vertex "
               << vertex_A.id() << " has no resource of type "
               << static_cast<int>(image_resource_type);
    return;
  }
  if (!map->getFrameResource<cv::Mat>(
          vertex_B, next_frame_idx, image_resource_type, &image_B)) {
    LOG(ERROR) << "Frame " << next_frame_idx << " of vertex " << vertex_B.id()
               << " has no resource of type "
               << static_cast<int>(image_resource_type);
    return;
  }

  saveOpenCvMatchesAndFeaturesAsImage(
      frame_A, image_A, frame_B, image_B, matches_A_B, filename,
      visualization_type, map);
}

void saveLandmarkMatchesAndFeaturesAsImage(
    const aslam::VisualFrame& frame_A, const vi_map::Vertex& vertex_A,
    const aslam::VisualFrame& frame_B, const vi_map::Vertex& vertex_B,
    const std::string& filename,
    aslam::FeatureVisualizationType visualization_type,
    const backend::ResourceType image_resource_type, vi_map::VIMap* map) {
  CHECK_NOTNULL(map);

  aslam::OpenCvMatches matches_A_B;
  int index_A = 0;
  int index_B = 0;
  float distance = 0.0;
  const unsigned int current_frame_idx =
      vertex_A.getVisualFrameIndex(frame_A.getId());
  const unsigned int next_frame_idx =
      vertex_B.getVisualFrameIndex(frame_B.getId());

  vi_map::LandmarkIdList landmark_ids_A;
  vi_map::LandmarkIdList landmark_ids_B;
  vertex_A.getFrameObservedLandmarkIds(current_frame_idx, &landmark_ids_A);
  vertex_B.getFrameObservedLandmarkIds(next_frame_idx, &landmark_ids_B);

  for (const vi_map::LandmarkId& landmark_id_A : landmark_ids_A) {
    for (const vi_map::LandmarkId& landmark_id_B : landmark_ids_B) {
      if (landmark_id_A.isValid() && landmark_id_B.isValid()) {
        if (landmark_id_A == landmark_id_B) {
          vi_map::Landmark landmark = map->getLandmark(landmark_id_A);
          const vi_map::KeypointIdentifierList& observations =
              landmark.getObservations();
          bool found_observation_for_current_vertex = false;
          bool found_observation_for_next_vertex = false;

          for (vi_map::KeypointIdentifier observation : observations) {
            if (observation.frame_id.vertex_id == vertex_A.id() &&
                observation.frame_id.frame_index == current_frame_idx) {
              found_observation_for_current_vertex = true;
              index_A = observation.keypoint_index;
              distance = static_cast<float>(frame_A.getKeypointScore(index_A));
            } else if (
                observation.frame_id.vertex_id == vertex_B.id() &&
                observation.frame_id.frame_index == next_frame_idx) {
              found_observation_for_next_vertex = true;
              index_B = observation.keypoint_index;
              distance = static_cast<float>(frame_B.getKeypointScore(index_B));
            }
          }
          if (found_observation_for_current_vertex &&
              found_observation_for_next_vertex) {
            matches_A_B.push_back(cv::DMatch(index_A, index_B, distance));
          }
        }
      }
    }
  }
  saveOpenCvMatchesAndFeaturesAsImage(
      frame_A, vertex_A, frame_B, vertex_B, matches_A_B, filename,
      visualization_type, image_resource_type, map);
}
}  // namespace visualization
