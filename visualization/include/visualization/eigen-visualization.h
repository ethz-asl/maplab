/*

 Copyright (c) 2013, Markus Achtelik, ASL, ETH Zurich, Switzerland
 You can contact the author at <markus dot achtelik at mavt dot ethz dot ch>

 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of ETHZ-ASL nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL ETHZ-ASL BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */

#ifndef VISUALIZATION_EIGEN_VISUALIZATION_H_
#define VISUALIZATION_EIGEN_VISUALIZATION_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <visualization_msgs/MarkerArray.h>

namespace visualization {

/// converts any eigen vector with 3 elements to a geometry_msgs::Point
template <class Derived>
inline geometry_msgs::Point eigenToPoint(const Eigen::MatrixBase<Derived>& p) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);

  geometry_msgs::Point _p;
  _p.x = p[0];
  _p.y = p[1];
  _p.z = p[2];
  return _p;
}

/// converts any eigen vector with 3 elements to a geometry_msgs::Point
template <class Derived>
inline geometry_msgs::Vector3 eigenToVector3(
    const Eigen::MatrixBase<Derived>& p) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
  geometry_msgs::Vector3 _p;
  _p.x = p[0];
  _p.y = p[1];
  _p.z = p[2];
  return _p;
}

/// converts an Eigen::Quaterniond to a geometry_msgs::Quaternion
inline geometry_msgs::Quaternion eigenToQuaternion(
    const Eigen::Quaterniond& q) {
  geometry_msgs::Quaternion _q;
  _q.w = q.w();
  _q.x = q.x();
  _q.y = q.y();
  _q.z = q.z();
  return _q;
}

/// helper function to create a std_msgs::ColorRGBA
inline std_msgs::ColorRGBA createColorRGBA(float r, float g, float b, float a) {
  std_msgs::ColorRGBA c;
  c.r = r;
  c.g = g;
  c.b = b;
  c.a = a;
  return c;
}

/// helper function to create a geometry_msgs::Point
inline geometry_msgs::Point createPoint(double x, double y, double z) {
  geometry_msgs::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

/**
 * \brief Draws a covariance ellipsoid
 * \param[out] marker The marker in which the ellipsoid should be drawn
 * \param[in] mu static 3 element vector, specifying the center of the ellipsoid
 * \param[in] cov static 3x3 covariance matrix
 * \param[in] color RGBA color of the ellipsoid
 * \param[in] n_sigma confidence area / scale of the ellipsoid
 */
template <class DerivedMu, class DerivedCov>
void drawCovariance3D(
    const Eigen::MatrixBase<DerivedMu>& mu,
    const Eigen::MatrixBase<DerivedCov>& cov, const std_msgs::ColorRGBA& color,
    double n_sigma, visualization_msgs::Marker* marker_ptr) {
  CHECK_NOTNULL(marker_ptr);
  visualization_msgs::Marker& marker = *marker_ptr;
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(DerivedMu, 3);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(DerivedCov, 3, 3);

  const Eigen::Matrix3d _cov = (cov + cov.transpose()) * 0.5;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(
      _cov, Eigen::ComputeEigenvectors);
  Eigen::Matrix3d V = solver.eigenvectors();
  // make sure it's a rotation matrix
  V.col(2) = V.col(0).cross(V.col(1));
  const Eigen::Vector3d sigma = solver.eigenvalues().cwiseSqrt() * n_sigma;

  marker.pose.position = eigenToPoint(mu);
  marker.pose.orientation = eigenToQuaternion(Eigen::Quaterniond(V));
  marker.scale = eigenToVector3(sigma * 2.0);  // diameter, not half axis
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.color = color;
  marker.action = visualization_msgs::Marker::ADD;
}

/**
 * \brief Convenience function, behaves as drawCovariance3D()
 */
template <class DerivedMu, class DerivedCov>
inline visualization_msgs::Marker drawCovariance3D(
    const Eigen::MatrixBase<DerivedMu>& mu,
    const Eigen::MatrixBase<DerivedCov>& cov, const std_msgs::ColorRGBA& color,
    double n_sigma) {
  visualization_msgs::Marker marker;
  drawCovariance3D(marker, mu, cov, color, n_sigma);
  return marker;
}

template <class DerivedP, class DerivedQ>
void drawAxes(
    const Eigen::MatrixBase<DerivedP>& p,
    const Eigen::QuaternionBase<DerivedQ>& q, double scale, double line_width,
    double alpha, visualization::Color color,
    visualization_msgs::Marker* marker_ptr) {
  CHECK_NOTNULL(marker_ptr);
  visualization_msgs::Marker& marker = *marker_ptr;
  marker.colors.resize(6);
  marker.points.resize(6);
  marker.points[0] = createPoint(0, 0, 0);
  marker.points[1] = createPoint(1 * scale, 0, 0);
  marker.points[2] = createPoint(0, 0, 0);
  marker.points[3] = createPoint(0, 1 * scale, 0);
  marker.points[4] = createPoint(0, 0, 0);
  marker.points[5] = createPoint(0, 0, 1 * scale);

  marker.color = createColorRGBA(0, 0, 0, alpha);
  marker.colors[0] = createColorRGBA(1, 0, 0, alpha);
  marker.colors[1] = createColorRGBA(1, 0, 0, alpha);
  marker.colors[2] = createColorRGBA(0, 1, 0, alpha);
  marker.colors[3] = createColorRGBA(0, 1, 0, alpha);
  marker.colors[4] = createColorRGBA(0, 0, 1, alpha);
  marker.colors[5] = createColorRGBA(0, 0, 1, alpha);

  int blue = color.blue;
  int red = color.red;
  int green = color.green;

  if (blue + red + green != 0) {
    std_msgs::ColorRGBA colorRGBA;
    colorRGBA.r = static_cast<double>(red) / 255.0;
    colorRGBA.g = static_cast<double>(green) / 255.0;
    colorRGBA.b = static_cast<double>(blue) / 255.0;
    colorRGBA.a = alpha;

    marker.color = colorRGBA;
    marker.colors[0] = colorRGBA;
    marker.colors[1] = colorRGBA;
    marker.colors[2] = colorRGBA;
    marker.colors[3] = colorRGBA;
    marker.colors[4] = colorRGBA;
    marker.colors[5] = colorRGBA;
  }

  marker.scale.x = line_width;  // rest is unused
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position = eigenToPoint(p);
  marker.pose.orientation = eigenToQuaternion(q);
}

template <class DerivedP, class DerivedQ>
void drawArrow(
    const Eigen::MatrixBase<DerivedP>& p,
    const Eigen::QuaternionBase<DerivedQ>& q, const std_msgs::ColorRGBA& color,
    double length, double diameter, visualization_msgs::Marker* marker_ptr) {
  CHECK_NOTNULL(marker_ptr);
  visualization_msgs::Marker& marker = *marker_ptr;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.color = color;

  marker.pose.position = eigenToPoint(p);
  marker.pose.orientation = eigenToQuaternion(q);

  marker.scale.x = diameter;
  marker.scale.y = diameter;
  marker.scale.z = length;
}

template <class DerivedP1, class DerivedP2>
void drawArrow(
    const Eigen::MatrixBase<DerivedP1>& p1,
    const Eigen::MatrixBase<DerivedP2>& p2, const std_msgs::ColorRGBA& color,
    double diameter, visualization_msgs::Marker* marker_ptr) {
  CHECK_NOTNULL(marker_ptr);
  visualization_msgs::Marker& marker = *marker_ptr;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.color = color;

  marker.pose.position =
      visualization_msgs::Marker::_pose_type::_position_type();
  marker.pose.orientation =
      visualization_msgs::Marker::_pose_type::_orientation_type();

  marker.points.resize(2);
  marker.points[0] = eigenToPoint(p1);
  marker.points[1] = eigenToPoint(p2);

  marker.scale.x = diameter * 0.1;
  marker.scale.y = diameter * 2 * 0.1;
  marker.scale.z = 0;
}

template <class DerivedP, class DerivedQ>
void drawAxesArrows(
    const Eigen::MatrixBase<DerivedP>& p,
    const Eigen::QuaternionBase<DerivedQ>& q, double scale, double diameter,
    double alpha, visualization_msgs::MarkerArray* marker_array) {
  CHECK_NOTNULL(marker_array);
  visualization_msgs::MarkerArray::_markers_type& markers =
      marker_array->markers;
  markers.resize(3);
  Eigen::Vector3d origin(Eigen::Vector3d::Zero());

  drawArrow(
      markers[0], origin + p, q * Eigen::Vector3d::UnitX() * scale + p,
      createColorRGBA(1, 0, 0, alpha), diameter);
  drawArrow(
      markers[1], origin + p, q * Eigen::Vector3d::UnitY() * scale + p,
      createColorRGBA(0, 1, 0, alpha), diameter);
  drawArrow(
      markers[2], origin + p, q * Eigen::Vector3d::UnitZ() * scale + p,
      createColorRGBA(0, 0, 1, alpha), diameter);
}

}  // namespace visualization

#endif  // VISUALIZATION_EIGEN_VISUALIZATION_H_
