#ifndef SEMANTIFY_BASIC_PLUGIN_SEMANTIFY_COMMON_H_
#define SEMANTIFY_BASIC_PLUGIN_SEMANTIFY_COMMON_H_

#include <iterator>
#include <random>
#include <vector>

#include <Eigen/Dense>
#include <gflags/gflags.h>

#include <map-resources/resource-typedefs.h>
#include <vi-map/vi-map.h>

namespace semantify {

DECLARE_double(semantify_tracking_confidence_threshold);
DECLARE_double(semantify_visualization_frequency);

extern std::vector<std::string> mask_rcnn_class_names;
extern std::string input_vis_window_name;

bool getSemanticResources(
    const vi_map::VIMap& map, const vi_map::Vertex& vertex, const size_t& idx,
    cv::Mat* image, cv::Mat* image_masks,
    resources::ObjectInstanceBoundingBoxes* boxes);

void visualizeInput(
    const cv::Mat& input_image,
    const resources::ObjectInstanceBoundingBoxes& input_boxes,
    const Eigen::Matrix2Xd& input_keypoints,
    const Eigen::VectorXi& input_track_ids, const cv::Mat& input_mask,
    bool look_up_class_name = false, bool visualize_mask = false);

void destroyVisualizationWindow();

void generateRandomColor(std::vector<cv::Vec3b>& colors);
void applyMask(cv::Mat& image, const cv::Mat& mask, const cv::Vec3b& color);

// random select an element from container
// Ref: https://gist.github.com/cbsmith/5538174
template <typename RandomGenerator = std::default_random_engine>
struct random_selector {
  // On most platforms, you probably want to use
  // std::random_device("/dev/urandom")()
  // random_selector(RandomGenerator g =
  // RandomGenerator(std::random_device()())) use this if you want to fix the
  // seed
  random_selector(RandomGenerator g = RandomGenerator(0u)) : gen(g) {}

  template <typename Iter>
  Iter select(Iter start, Iter end) {
    std::uniform_int_distribution<> dis(0, std::distance(start, end) - 1);
    std::advance(start, dis(gen));
    return start;
  }

  // convenience function
  template <typename Iter>
  Iter operator()(Iter start, Iter end) {
    return select(start, end);
  }

  // convenience function that works on anything with a sensible begin() and
  // end(), and returns with a ref to the value type
  template <typename Container>
  auto operator()(const Container& c) -> decltype(*begin(c))& {
    return *select(begin(c), end(c));
  }

 private:
  RandomGenerator gen;
};

// auxilary function
inline bool static sortBySec(
    const std::pair<unsigned int, float>& a,
    const std::pair<unsigned int, float>& b) {
  return (a.second < b.second);
}

}  // namespace semantify
#endif  // SEMANTIFY_BASIC_PLUGIN_SEMANTIFY_COMMON_H_
