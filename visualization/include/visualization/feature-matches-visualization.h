#ifndef VISUALIZATION_FEATURE_MATCHES_VISUALIZATION_H_
#define VISUALIZATION_FEATURE_MATCHES_VISUALIZATION_H_

#include <string>
#include <vector>

#include <aslam/cameras/camera.h>
#include <aslam/matcher/match-visualization.h>
#include <aslam/matcher/match.h>
#include <map-resources/resource-common.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <vi-map/vertex.h>
#include <vi-map/vi-map.h>

namespace visualization {

void saveOpenCvMatchesAndFeaturesAsImage(
    const aslam::VisualFrame& frame_A, const cv::Mat& image_A,
    const aslam::VisualFrame& frame_B, const cv::Mat& image_B,
    const aslam::OpenCvMatches& matches_A_B, const std::string& filename,
    aslam::FeatureVisualizationType visualization_type, vi_map::VIMap* map);

void saveOpenCvMatchesAndFeaturesAsImage(
    const aslam::VisualFrame& frame_A, const vi_map::Vertex& vertex_A,
    const aslam::VisualFrame& frame_B, const vi_map::Vertex& vertex_B,
    const aslam::OpenCvMatches& matches_A_B, const std::string& filename,
    aslam::FeatureVisualizationType visualization_type,
    const backend::ResourceType image_resource_type, vi_map::VIMap* map);

void saveOpenCvMatchesAndFeaturesAsImage(
    const aslam::VisualFrame& frame_A, const aslam::VisualFrame& frame_B,
    const aslam::OpenCvMatches& matches_A_B, const std::string& filename,
    aslam::FeatureVisualizationType visualization_type, vi_map::VIMap* map);

template <typename MatchesWithScore>
void saveAslamMatchesAndFeaturesAsImage(
    const aslam::VisualFrame& frame_A, const cv::Mat& image_A,
    const aslam::VisualFrame& frame_B, const cv::Mat& image_B,
    const MatchesWithScore& matches_A_B, const std::string& filename,
    aslam::FeatureVisualizationType visualization_type, vi_map::VIMap* map);

template <typename MatchesWithScore>
void saveAslamMatchesAndFeaturesAsImage(
    const aslam::VisualFrame& frame_A, const aslam::VisualFrame& frame_B,
    const MatchesWithScore& matches_A_B, const std::string& filename,
    aslam::FeatureVisualizationType visualization_type, vi_map::VIMap* map);

template <typename MatchesWithScore>
void saveAslamMatchesAndFeaturesAsImage(
    const aslam::VisualFrame& frame_A, const vi_map::Vertex& vertex_A,
    const aslam::VisualFrame& frame_B, const vi_map::Vertex& vertex_B,
    const MatchesWithScore& matches_A_B, const std::string& filename,
    aslam::FeatureVisualizationType visualization_type,
    const backend::ResourceType image_resource_type, vi_map::VIMap* map);

void saveLandmarkMatchesAndFeaturesAsImage(
    const aslam::VisualFrame& frame_A, const vi_map::Vertex& vertex_A,
    const aslam::VisualFrame& frame_B, const vi_map::Vertex& vertex_B,
    const std::string& filename,
    aslam::FeatureVisualizationType visualization_type,
    const backend::ResourceType image_resource_type, vi_map::VIMap* map);

}  // namespace visualization

#include "./feature-matches-visualization-inl.h"

#endif  // VISUALIZATION_FEATURE_MATCHES_VISUALIZATION_H_
