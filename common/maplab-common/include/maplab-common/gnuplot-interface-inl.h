#ifndef MAPLAB_COMMON_GNUPLOT_INTERFACE_INL_H_
#define MAPLAB_COMMON_GNUPLOT_INTERFACE_INL_H_

#include <sstream>  // NOLINT
#include <string>
#include <vector>

#include <glog/logging.h>

namespace common {

template <int Rows>
void GnuplotInterface::operator<<(
    const Eigen::Matrix<double, Rows, Eigen::Dynamic>& data) {
  if (FLAGS_use_gnuplot) {
    for (int i = 0; i < data.cols(); ++i) {
      for (int d = 0; d < Rows; ++d) {
        fprintf(pipe_, "%lf ", data(d, i));
      }
      operator<<("");  // Adds newline.
    }
    operator<<("e");
  }
}

template <int Rows>
void GnuplotInterface::operator<<(
    const Eigen::Matrix<int, Rows, Eigen::Dynamic>& data) {
  if (FLAGS_use_gnuplot) {
    for (int i = 0; i < data.cols(); ++i) {
      for (int d = 0; d < Rows; ++d) {
        fprintf(pipe_, "%d ", data(d, i));
      }
      operator<<("");  // Adds newline.
    }
    operator<<("e");
  }
}

template <typename T1, typename T2, typename T3, typename T4>
void GnuplotInterface::drawRectangle(
    const T1 xmin, const T2 ymin, const T3 xmax, const T4 ymax,
    const std::string& color) {
  CHECK_LE(xmin, xmax);
  CHECK_LE(ymin, ymax);

  std::ostringstream stream;
  stream << "set object rect from " << xmin << "," << ymin << " to " << xmax
         << "," << ymax << " fc rgb \"" << color << "\"";
  operator<<(stream.str());
}

template <int Rows>
void GnuplotInterface::plot(
    const Eigen::Matrix<double, Rows, Eigen::Dynamic>& data) {
  static_assert(Rows < 3, "plot() only works with 1D or 2D data");
  disableLegend();
  operator<<("plot '-' w l");
  operator<<(data);
}

template <typename Type>
void GnuplotInterface::plotSteps(
    const std::unordered_map<
        std::string, Eigen::Matrix<Type, 2, Eigen::Dynamic>>& legend_data) {
  CHECK(!legend_data.empty());
  typedef std::unordered_map<std::string,
                             Eigen::Matrix<Type, 2, Eigen::Dynamic>>
      LegendDataMap;
  fputs("plot ", pipe_);
  bool first = true;
  for (const typename LegendDataMap::value_type& data : legend_data) {
    if (first) {
      first = false;
    } else {
      fputs(", ", pipe_);
    }
    fprintf(pipe_, "'-' w steps title '%s' lw 4", data.first.c_str());
  }
  fputs("\n", pipe_);
  for (const typename LegendDataMap::value_type& data : legend_data) {
    operator<<(data.second);
  }
}

template <typename T1, typename T2>
void GnuplotInterface::plotWithDifferentYAxes(
    const std::string& legend_1, const std::vector<T1>& data_1,
    const std::string& legend_2, const std::vector<T2>& data_2) {
  operator<<(
      "plot '-' w l title '" + legend_1 + "', '-' w l axes x1y2 title '" +
      legend_2 + "'");
  operator<<(data_1);
  operator<<(data_2);
}

template <typename T1, typename T2, typename T3, typename T4>
void GnuplotInterface::setRange(
    const T1 xmin, const T2 xmax, const T3 ymin, const T4 ymax) {
  CHECK_LT(xmin, xmax);
  CHECK_LT(ymin, ymax);

  std::ostringstream stream;
  stream << "set xrange [" << xmin << ":" << xmax << "]\nset yrange [" << ymin
         << ":" << ymax << "]";
  operator<<(stream.str());
}

}  // namespace common

#endif  // MAPLAB_COMMON_GNUPLOT_INTERFACE_INL_H_
