#ifndef MAPLAB_COMMON_GNUPLOT_INTERFACE_H_
#define MAPLAB_COMMON_GNUPLOT_INTERFACE_H_

#include <initializer_list>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>
#include <gflags/gflags.h>

DECLARE_bool(use_gnuplot);

namespace common {

class GnuplotInterface {
 public:
  GnuplotInterface(bool persist, const std::string& title);
  explicit GnuplotInterface(const std::string& title);
  GnuplotInterface();

  ~GnuplotInterface();

  // ================================
  // BASIC ACCESS FOR MANUAL COMMANDS
  // ================================
  FILE* operator()();

  // Adds a newline at the end of the string.
  void operator<<(const std::string& string);
  void operator<<(const std::vector<size_t>& data);
  void operator<<(const std::vector<int>& data);
  void operator<<(const std::vector<double>& data);
  // In data, columns are treated as data points and rows as dimensions.
  // Since gnuplot interprets rows as data points as columns as dimensions,
  // The data is transposed before being fed to gnuplot.
  template <int Rows>
  void operator<<(const Eigen::Matrix<double, Rows, Eigen::Dynamic>& data);
  template <int Rows>
  void operator<<(const Eigen::Matrix<int, Rows, Eigen::Dynamic>& data);
  // Unlike with Matrix3Xd, the data is not transposed for MatrixXd.
  void operator<<(const Eigen::MatrixXd& data);

  // =====================
  // PRE-IMPLEMENTED PLOTS
  // =====================
  template <typename T1, typename T2, typename T3, typename T4>
  void drawRectangle(
      const T1 xmin, const T2 ymin, const T3 xmax, const T4 ymax,
      const std::string& color);

  template <int Rows>
  void plot(const Eigen::Matrix<double, Rows, Eigen::Dynamic>& data);
  void plot(
      const std::vector<Eigen::Matrix2Xd>& data,
      const std::vector<std::string>& legends);
  template <typename Type>
  void plotSteps(
      const std::unordered_map<
          std::string, Eigen::Matrix<Type, 2, Eigen::Dynamic>>& legend_data);

  template <typename T1, typename T2>
  void plotWithDifferentYAxes(
      const std::string& legend_1, const std::vector<T1>& data_1,
      const std::string& legend_2, const std::vector<T2>& data_2);

  void plot3dLines(
      const std::initializer_list<const Eigen::Matrix3Xd* const>& data);
  void plot3dPoints(const Eigen::Matrix3Xd& data);

  void plot2dHistogram(const Eigen::MatrixXd& histogram);

  // ==============
  // PLOT COSMETICS
  // ==============
  void setXLabel(const std::string& label);

  void setYLabel(const std::string& label);

  void setYLabels(const std::string& label_1, const std::string& label_2);

  void setTitle(const std::string& title);
  void setTitle();  // Uses member "title_".

  template <typename T1, typename T2, typename T3, typename T4>
  void setRange(const T1 xmin, const T2 xmax, const T3 ymin, const T4 ymax);

  void setEqualAxisRatio();

  void disableLegend();

  // {left | right | center} {top | bottom | center}
  void setLegendPosition(const std::string& position);

  void disableTics();

  void useVerdanaFont(const size_t font_size_pt);

  void plotDummy();

 private:
  FILE* pipe_;
  // Because a title replot is sometimes required.
  const std::string title_;
};

}  // namespace common

#include "./gnuplot-interface-inl.h"

#endif  // MAPLAB_COMMON_GNUPLOT_INTERFACE_H_
