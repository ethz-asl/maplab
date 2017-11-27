#ifndef VISUALIZATION_COLOR_PALETTE_H_
#define VISUALIZATION_COLOR_PALETTE_H_

#include <cmath>
#include <random>
#include <utility>
#include <vector>

#include <glog/logging.h>

#include "visualization/color.h"

namespace visualization {

typedef std::vector<visualization::Color> ColorList;

struct Palette {
  enum class PaletteTypes {
    kLinearRed,
    kGammaLogRed,
    kInversionRed,
    kLinearGray,
    kGammaLog,
    kInversion,
    kFalseColor1,
    kFalseColor2,
    kFalseColor3,
    kFalseColorJet
  };
  visualization::Color colors[kNumColors];
};

visualization::Palette GetPalette(visualization::Palette::PaletteTypes pal);

inline visualization::Color getPaletteColor(
    size_t index, const visualization::Palette& palette) {
  visualization::Color color;
  index = index % kNumColors;
  color.red = palette.colors[index].red;
  color.green = palette.colors[index].green;
  color.blue = palette.colors[index].blue;
  return color;
}

inline visualization::Color getPaletteColor(
    double value, const visualization::Palette& palette) {
  CHECK_GE(value, 0.0);
  CHECK_LE(value, 1.0);
  size_t index = static_cast<size_t>(value * 255);
  return getPaletteColor(index, palette);
}

inline void GetRandomRGBColor(visualization::Color* rgb) {
  CHECK_NOTNULL(rgb);
  visualization::Palette palette =
      GetPalette(visualization::Palette::PaletteTypes::kFalseColor1);
  std::random_device random_device;
  std::mt19937 random_number_generator(random_device());
  std::uniform_int_distribution<> distribution(0, 255);
  unsigned int random_color_idx = distribution(random_number_generator);
  *rgb = palette.colors[random_color_idx];
}
}  // namespace visualization

#endif  // VISUALIZATION_COLOR_PALETTE_H_
