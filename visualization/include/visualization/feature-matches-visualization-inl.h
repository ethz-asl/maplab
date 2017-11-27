#ifndef VISUALIZATION_FEATURE_MATCHES_VISUALIZATION_INL_H_
#define VISUALIZATION_FEATURE_MATCHES_VISUALIZATION_INL_H_

#include <string>

#include <aslam/matcher/match-helpers.h>

namespace visualization {

template <typename MatchesWithScore>
void saveAslamMatchesAndFeaturesAsImage(
    const aslam::VisualFrame& frame_A, const cv::Mat& image_A,
    const aslam::VisualFrame& frame_B, const cv::Mat& image_B,
    const MatchesWithScore& matches_A_B, const std::string& filename,
    aslam::FeatureVisualizationType visualization_type, vi_map::VIMap* map) {
  CHECK_NOTNULL(map);

  aslam::OpenCvMatches cv_matches_A_B;
  aslam::convertMatchesWithScoreToOpenCvMatches<
      typename MatchesWithScore::value_type>(matches_A_B, &cv_matches_A_B);

  saveOpenCvMatchesAndFeaturesAsImage(
      frame_A, image_A, frame_B, image_B, cv_matches_A_B, filename,
      visualization_type, map);
}

template <typename MatchesWithScore>
void saveAslamMatchesAndFeaturesAsImage(
    const aslam::VisualFrame& frame_A, const aslam::VisualFrame& frame_B,
    const MatchesWithScore& matches_A_B, const std::string& filename,
    aslam::FeatureVisualizationType visualization_type, vi_map::VIMap* map) {
  aslam::OpenCvMatches cv_matches_A_B;
  aslam::convertMatchesWithScoreToOpenCvMatches<
      typename MatchesWithScore::value_type>(matches_A_B, &cv_matches_A_B);

  saveOpenCvMatchesAndFeaturesAsImage(
      frame_A, frame_B, cv_matches_A_B, filename, visualization_type, map);
}

template <typename MatchesWithScore>
void saveAslamMatchesAndFeaturesAsImage(
    const aslam::VisualFrame& frame_A, const vi_map::Vertex& vertex_A,
    const aslam::VisualFrame& frame_B, const vi_map::Vertex& vertex_B,
    const MatchesWithScore& matches_A_B, const std::string& filename,
    aslam::FeatureVisualizationType visualization_type,
    const backend::ResourceType image_resource_type, vi_map::VIMap* map) {
  CHECK_NOTNULL(map);

  aslam::OpenCvMatches cv_matches_A_B;
  aslam::convertMatchesWithScoreToOpenCvMatches<
      typename MatchesWithScore::value_type>(matches_A_B, &cv_matches_A_B);

  saveOpenCvMatchesAndFeaturesAsImage(
      frame_A, vertex_A, frame_B, vertex_B, cv_matches_A_B, filename,
      visualization_type, image_resource_type, map);
}

}  // namespace visualization

#endif  // VISUALIZATION_FEATURE_MATCHES_VISUALIZATION_INL_H_
