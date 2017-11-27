#include "maplab-common/gnuplot-interface.h"

#include <cmath>
#include <sstream>  // NOLINT
#include <string>
#include <vector>

DEFINE_bool(use_gnuplot, true, "Toggle use of gnuplot.");
DEFINE_bool(
    save_gnuplot, false, "Output plots to a .png instead of the screen.");

namespace common {

GnuplotInterface::GnuplotInterface(bool persist, const std::string& title)
    : title_(title) {
  static int plot_i = 0;
  if (FLAGS_use_gnuplot) {
    if (FLAGS_save_gnuplot) {
      persist = false;
    }
    pipe_ = popen(persist ? "gnuplot --persist" : "gnuplot", "w");
    if (FLAGS_save_gnuplot) {
      operator<<("set term png");
      operator<<("set output \"" + std::to_string(plot_i++) + ".png\"");
    }
  }
  if (title != "") {
    setTitle(title);
  }
}
GnuplotInterface::GnuplotInterface(const std::string& title)
    : GnuplotInterface(true, title) {}
GnuplotInterface::GnuplotInterface() : GnuplotInterface(true, "") {}

GnuplotInterface::~GnuplotInterface() {
  if (FLAGS_use_gnuplot) {
    fclose(pipe_);
  }
}

FILE* GnuplotInterface::operator()() {
  CHECK(FLAGS_use_gnuplot);
  return pipe_;
}

void GnuplotInterface::operator<<(const std::string& string) {
  if (FLAGS_use_gnuplot) {
    fputs((string + std::string("\n")).c_str(), pipe_);
    fflush(pipe_);
  }
}
void GnuplotInterface::operator<<(const std::vector<size_t>& data) {
  if (FLAGS_use_gnuplot) {
    for (const size_t datum : data) {
      fprintf(pipe_, "%lu\n", datum);
    }
    operator<<("e");
  }
}
void GnuplotInterface::operator<<(const std::vector<int>& data) {
  if (FLAGS_use_gnuplot) {
    for (const int datum : data) {
      fprintf(pipe_, "%d\n", datum);
    }
    operator<<("e");
  }
}
void GnuplotInterface::operator<<(const std::vector<double>& data) {
  if (FLAGS_use_gnuplot) {
    for (const double datum : data) {
      fprintf(pipe_, "%lf\n", datum);
    }
    operator<<("e");
  }
}
void GnuplotInterface::operator<<(const Eigen::MatrixXd& data) {
  if (FLAGS_use_gnuplot) {
    for (int r = 0; r < data.rows(); ++r) {
      for (int c = 0; c < data.cols(); ++c) {
        const double value = data(r, c);
        if (std::isnan(value)) {
          LOG(WARNING) << "Replacing NaN with 0!";
          fputs("0 ", pipe_);
        } else {
          fprintf(pipe_, "%lf ", value);
        }
      }
      fputs("\n", pipe_);
    }
    operator<<("e");
    operator<<("e");
  }
}

void GnuplotInterface::plot(
    const std::vector<Eigen::Matrix2Xd>& data,
    const std::vector<std::string>& legends) {
  CHECK_EQ(data.size(), legends.size());
  fputs("plot ", pipe_);
  for (size_t i = 0u; i < legends.size(); ++i) {
    if (i > 0u) {
      fputs(", ", pipe_);
    }
    fprintf(pipe_, "'-' w l title '%s' lw 4", legends[i].c_str());
  }
  fputs("\n", pipe_);
  for (const Eigen::Matrix2Xd& datum : data) {
    operator<<(datum);
  }
}

void GnuplotInterface::plot3dLines(
    const std::initializer_list<const Eigen::Matrix3Xd* const>& data) {
  operator<<("set view equal xyz");
  std::ostringstream stream;
  for (size_t i = 0u; i < data.size(); ++i) {
    if (i == 0u) {
      stream << "splot '-' w l";
    } else {
      stream << ", '-' w l";
    }
  }
  operator<<(stream.str());
  for (const Eigen::Matrix3Xd* const matrix : data) {
    operator<<(*CHECK_NOTNULL(matrix));
  }
}

void GnuplotInterface::plot3dPoints(const Eigen::Matrix3Xd& data) {
  operator<<("set view equal xyz\nsplot '-'");
  operator<<(data);
}

void GnuplotInterface::plot2dHistogram(const Eigen::MatrixXd& histogram) {
  disableLegend();
  setEqualAxisRatio();
  setRange(-.5, histogram.cols() - .5, -.5, histogram.rows() - .5);
  operator<<("set view map");
  operator<<("unset colorbox");
  operator<<("splot '-' matrix with image");
  operator<<(histogram);
  setTitle();
}

void GnuplotInterface::setXLabel(const std::string& label) {
  operator<<("set xlabel \"" + label + "\"");
}

void GnuplotInterface::setYLabel(const std::string& label) {
  operator<<("set ylabel \"" + label + "\"");
}

void GnuplotInterface::setYLabels(
    const std::string& label_1, const std::string& label_2) {
  setYLabel(label_1);
  operator<<("set y2label \"" + label_2 + "\"");
  operator<<("set ytics nomirror");
  operator<<("set y2tics");
}

void GnuplotInterface::setTitle(const std::string& title) {
  operator<<("set title \"" + title + "\"");
}

void GnuplotInterface::setTitle() {
  CHECK_NE(title_, "");
  setTitle(title_);
}

void GnuplotInterface::setEqualAxisRatio() {
  operator<<("set size square");
}

void GnuplotInterface::disableLegend() {
  operator<<("set key off");
}

void GnuplotInterface::setLegendPosition(const std::string& position) {
  operator<<("set key " + position);
}

void GnuplotInterface::disableTics() {
  operator<<("unset xtics");
  operator<<("unset ytics");
}

void GnuplotInterface::useVerdanaFont(const size_t font_size_pt) {
  if (FLAGS_save_gnuplot) {
    return;
  }
  std::ostringstream stream;
  stream << "set terminal wxt enhanced font 'Verdana," << font_size_pt << "'";
  operator<<(stream.str());
}

void GnuplotInterface::plotDummy() {
  operator<<("plot '-' w p\n-1 -1\ne");
}

}  // namespace common
