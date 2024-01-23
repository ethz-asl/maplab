#include <glog/logging.h>

namespace aslam {

template<typename DerivedCamera, typename DerivedDistortion>
typename DerivedCamera::Ptr Camera::construct(
    const Eigen::VectorXd& intrinsics,
    uint32_t imageWidth,
    uint32_t imageHeight,
    const Eigen::VectorXd& distortionParameters) {
  aslam::Distortion::UniquePtr distortion(new DerivedDistortion(distortionParameters));
  typename DerivedCamera::Ptr camera(new DerivedCamera(intrinsics, imageWidth, imageHeight, distortion));
  return camera;
}

template<typename DerivedKeyPoint>
bool Camera::isKeypointVisible(
    const Eigen::MatrixBase<DerivedKeyPoint>& keypoint) const {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(DerivedKeyPoint, 2, 1);
  typedef typename DerivedKeyPoint::Scalar Scalar;
  return keypoint[0] >= static_cast<Scalar>(0.0)
      && keypoint[1] >= static_cast<Scalar>(0.0)
      && keypoint[0] < static_cast<Scalar>(imageWidth())
      && keypoint[1] < static_cast<Scalar>(imageHeight());
}

template<typename DerivedKeyPoint>
bool Camera::isKeypointVisibleWithMargin(
    const Eigen::MatrixBase<DerivedKeyPoint>& keypoint,
    typename DerivedKeyPoint::Scalar margin) const {
  typedef typename DerivedKeyPoint::Scalar Scalar;
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(DerivedKeyPoint, 2, 1);
  CHECK_LT(2 * margin, static_cast<Scalar>(imageWidth()));
  CHECK_LT(2 * margin, static_cast<Scalar>(imageHeight()));
  return keypoint[0] >= margin
      && keypoint[1] >= margin
      && keypoint[0] < (static_cast<Scalar>(imageWidth()) - margin)
      && keypoint[1] < (static_cast<Scalar>(imageHeight()) - margin);
}

inline std::ostream& operator<<(std::ostream& out, const Camera::Type& value) {
  static std::map<Camera::Type, std::string> names;
  if (names.size() == 0) {
    #define INSERT_ELEMENT(type, val) names[type::val] = #val
    INSERT_ELEMENT(Camera::Type, kPinhole);
    INSERT_ELEMENT(Camera::Type, kUnifiedProjection);
    #undef INSERT_ELEMENT
  }
  return out << names[value];
}

}  // namespace aslam
