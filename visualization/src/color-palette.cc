#include "visualization/color-palette.h"

#include <cmath>

#include <glog/logging.h>
#include <maplab-common/conversions.h>

namespace visualization {
Palette GetPalette(Palette::PaletteTypes palette_type) {
  Palette palette;

  switch (palette_type) {
    case Palette::PaletteTypes::kLinearRed:
      // Linear red palettes.
      for (size_t i = 0u; i < kNumColors; ++i) {
        palette.colors[i].blue = 0;
        palette.colors[i].green = 0;
        palette.colors[i].red = i;
      }
      break;
    case Palette::PaletteTypes::kGammaLogRed:
      // GammaLog red palettes.
      for (size_t i = 0u; i < kNumColors; ++i) {
        float f = log10(pow((i / 255.0), 1.0) * 9.0 + 1.0) * 255.0;
        palette.colors[i].blue = 0;
        palette.colors[i].green = 0;
        palette.colors[i].red = f;
      }
      break;
    case Palette::PaletteTypes::kInversionRed:
      // Inversion red palette.
      for (size_t i = 0u; i < kNumColors; ++i) {
        palette.colors[i].blue = 0;
        palette.colors[i].green = 0;
        palette.colors[i].red = 255 - i;
      }
      break;
    case Palette::PaletteTypes::kLinearGray:
      // Linear palettes.
      for (size_t i = 0u; i < kNumColors; ++i) {
        palette.colors[i].blue = palette.colors[i].green =
            palette.colors[i].red = i;
      }
      break;
    case Palette::PaletteTypes::kGammaLog:
      // GammaLog palettes.
      for (size_t i = 0u; i < kNumColors; ++i) {
        float f = log10(pow((i / 255.0), 1.0) * 9.0 + 1.0) * 255.0;
        palette.colors[i].blue = palette.colors[i].green =
            palette.colors[i].red = f;
      }
      break;
    case Palette::PaletteTypes::kInversion:
      // Inversion palette.
      for (size_t i = 0u; i < kNumColors; ++i) {
        palette.colors[i].blue = palette.colors[i].green =
            palette.colors[i].red = 255 - i;
      }
      break;
    case Palette::PaletteTypes::kFalseColor1:
      // False color palette #1.
      for (size_t i = 0u; i < kNumColors; ++i) {
        int red =
            (sin((i / 255.0 * 360.0 + 0.0) * kDegToRad) * 0.5 + 0.5) * 255.0;
        int green =
            (sin((i / 255.0 * 360.0 + 120.0) * kDegToRad) * 0.5 + 0.5) * 255.0;
        int blue =
            (sin((i / 255.0 * 360.0 + 240.0) * kDegToRad) * 0.5 + 0.5) * 255.0;
        palette.colors[i].blue = blue;
        palette.colors[i].green = green;
        palette.colors[i].red = red;
      }
      break;
    case Palette::PaletteTypes::kFalseColor2:
      // False color palette #2.
      for (size_t i = 0u; i < kNumColors; ++i) {
        int red =
            (sin((i / 255.0 * 360.0 + 120.0) * kDegToRad) * 0.5 + 0.5) * 255.0;
        int green =
            (sin((i / 255.0 * 360.0 + 240.0) * kDegToRad) * 0.5 + 0.5) * 255.0;
        int blue =
            (sin((i / 255.0 * 360.0 + 0.0) * kDegToRad) * 0.5 + 0.5) * 255.0;
        palette.colors[i].blue = blue;
        palette.colors[i].green = green;
        palette.colors[i].red = red;
      }
      break;
    case Palette::PaletteTypes::kFalseColor3:
      // False color palette #3.
      for (size_t i = 0u; i < kNumColors; ++i) {
        int red =
            (sin((i / 255.0 * 360.0 + 240.0) * kDegToRad) * 0.5 + 0.5) * 255.0;
        int green =
            (sin((i / 255.0 * 360.0 + 0.0) * kDegToRad) * 0.5 + 0.5) * 255.0;
        int blue =
            (sin((i / 255.0 * 360.0 + 120.0) * kDegToRad) * 0.5 + 0.5) * 255.0;
        palette.colors[i].blue = blue;
        palette.colors[i].green = green;
        palette.colors[i].red = red;
      }
      break;
    case Palette::PaletteTypes::kFalseColorJet:
      // False color palette #4. Matlab JET like.
      enum { kNSeparation = 64, kNValues = 192, kArraySize = kNumColors };
      std::vector<double> vals;
      vals.resize(kNValues, 0);
      size_t idx = 0;
      for (size_t i = 0u; i < kNSeparation; ++i) {
        vals.at(idx++) = (i / static_cast<double>(kNSeparation));
      }
      for (size_t i = 0u; i < kNSeparation; ++i) {
        vals.at(idx + i) = 1.;
      }

      idx += kNSeparation;
      for (int i = kNSeparation - 1; i >= 0; --i) {
        vals.at(idx++) = i / static_cast<double>(kNSeparation);
      }

      std::vector<size_t> r;
      r.resize(kNValues);
      std::vector<size_t> g;
      g.resize(kNValues);
      std::vector<size_t> b;
      b.resize(kNValues);
      for (size_t i = 0u; i < kNValues; ++i) {
        g.at(i) = ceil(kNSeparation / 2) - 1 + i;
        r.at(i) = g.at(i) + kNSeparation;
        b.at(i) = g.at(i) - kNSeparation;
      }
      int idxr, idxg, idxb, cntblue;
      idxr = idxg = idxb = cntblue = 0;
      for (size_t i = 0u; i < kNValues; ++i) {
        if (r.at(i) < kArraySize)
          palette.colors[r.at(i)].red = vals.at(idxr++) * 255.;
        if (g.at(i) < kArraySize)
          palette.colors[g.at(i)].green = vals.at(idxg++) * 255.;
        if (b.at(i) < kArraySize)
          cntblue++;
      }

      for (size_t i = 0u; i < kNValues; ++i) {
        if (b.at(i) < kArraySize)
          palette.colors[b.at(i)].blue =
              vals.at(kNValues - 1 - cntblue + idxb++) * 255.;
      }
      break;
  }
  return palette;
}

}  // namespace visualization
