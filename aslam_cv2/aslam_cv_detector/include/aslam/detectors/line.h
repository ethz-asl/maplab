#ifndef ASLAM_CV_DETECTORS_LINE
#define ASLAM_CV_DETECTORS_LINE
#include <memory>

#include <Eigen/Core>
#include <aslam/common/channel.h>
#include <aslam/common/macros.h>
#include <lsd/lsd-opencv.h>

namespace aslam {

template<typename Type>
struct LineImpl {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef Eigen::Matrix<Type, 2, 1> PointType;

  LineImpl(Type x0, Type y0, Type x1, Type y1) :
    start_point(x0, y0), end_point(x1, y1) {};

  LineImpl(const PointType& start, const PointType& end) :
    start_point(start), end_point(end) {};

  PointType start_point;
  PointType end_point;
};
typedef LineImpl<double> Line;
typedef std::vector<Line> Lines;

}  // namespace aslam

#endif  // ASLAM_CV_DETECTORS_LINE
