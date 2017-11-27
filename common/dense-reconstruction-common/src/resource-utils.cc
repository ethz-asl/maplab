#include "dense-reconstruction/resource-utils.h"

namespace dense_reconstruction {

void generateColorMap(const cv::Mat& input_mat, cv::Mat* color_map) {
  CHECK_NOTNULL(color_map);

  double min, max;
  cv::minMaxIdx(input_mat, &min, &max);
  cv::Mat scaled_input_mat;
  input_mat.convertTo(scaled_input_mat, CV_8UC1, 255 / (max - min), -min);
  applyColorMap(scaled_input_mat, *color_map, cv::COLORMAP_RAINBOW);
}

}  // namespace dense_reconstruction
