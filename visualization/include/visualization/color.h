#ifndef VISUALIZATION_COLOR_H_
#define VISUALIZATION_COLOR_H_

#include <string>
#include <vector>

#include <glog/logging.h>
#include <opencv2/core/core.hpp>

namespace visualization {

struct Color {
  unsigned char red;
  unsigned char green;
  unsigned char blue;
  inline constexpr Color() : Color(0u, 0u, 0u) {}
  inline constexpr Color(
      unsigned char red, unsigned char green, unsigned char blue)
      : red(red), green(green), blue(blue) {}
};

static constexpr size_t kNumColors = 256u;

static constexpr Color kCommonRed(255u, 0u, 0u);
static constexpr Color kCommonBlue(0u, 0u, 255u);
static constexpr Color kCommonGreen(0u, 255u, 0u);
static constexpr Color kCommonDarkGreen(0u, 180u, 0u);
static constexpr Color kCommonYellow(255u, 255u, 0u);
static constexpr Color kCommonGray(100u, 100u, 100u);
static constexpr Color kCommonWhite(255u, 255u, 255u);
static constexpr Color kCommonBlack(0u, 0u, 0u);
static constexpr Color kCommonPink(255u, 100u, 255u);
static constexpr Color kCommonPurple(100u, 80u, 255u);
static constexpr Color kCommonDarkGray(60u, 60u, 60u);
static constexpr Color kCommonGolden(240u, 200u, 0u);

static const std::vector<Color> common_color_list = {
    kCommonRed, kCommonBlue, kCommonGreen, kCommonYellow};

static const cv::Scalar kCvRed(0.0, 0.0, 255.0);
static const cv::Scalar kCvBlue(255.0, 0.0, 0.0);
static const cv::Scalar kCvGreen(0.0, 255.0, 0.0);
static const cv::Scalar kCvGray(100.0, 100.0, 100.0);
static const cv::Scalar kCvDarkGray(50.0, 50.0, 50.0);
static const cv::Scalar kCvCyan(255.0, 255.0, 0.0);
static const cv::Scalar kCvYellow(0.0, 255.0, 255.0);
static const cv::Scalar kCvWhite(255.0, 255.0, 255.0);
static const cv::Scalar kCvBlack(0.0, 0.0, 0.0);

inline void convertStringToColor(
    const std::string& color_string, visualization::Color* color) {
  if (color_string == "red") {
    *color = kCommonRed;
  } else if (color_string == "blue") {
    *color = kCommonBlue;
  } else if (color_string == "green") {
    *color = kCommonGreen;
  } else if (color_string == "dark_green") {
    *color = kCommonDarkGreen;
  } else if (color_string == "yellow") {
    *color = kCommonYellow;
  } else if (color_string == "gray") {
    *color = kCommonGray;
  } else if (color_string == "white") {
    *color = kCommonWhite;
  } else if (color_string == "black") {
    *color = kCommonBlack;
  } else if (color_string == "pink") {
    *color = kCommonPink;
  } else if (color_string == "dark_gray" || color_string == "dark_grey") {
    *color = kCommonDarkGray;
  } else if (color_string == "golden") {
    *color = kCommonGolden;
  } else {
    LOG(FATAL) << "Unknown color: " << color_string;
  }
}

}  // namespace visualization

#endif  // VISUALIZATION_COLOR_H_
