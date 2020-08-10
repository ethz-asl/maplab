#ifndef MINKINDR_CONVERSIONS_KINDR_MSG_H
#define MINKINDR_CONVERSIONS_KINDR_MSG_H

#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Vector3.h>
#include <glog/logging.h>
#include <kindr/minimal/quat-transformation.h>
#include <kindr/minimal/transform-2d.h>

namespace tf {

// A wrapper for the relevant functions in eigen_conversions.
template <typename Scalar>
void quaternionKindrToMsg(
    const kindr::minimal::RotationQuaternionTemplate<Scalar>& kindr,
    geometry_msgs::Quaternion* msg) {
  CHECK_NOTNULL(msg);
  quaternionEigenToMsg(kindr.toImplementation(), *msg);
}

template <typename Scalar>
void quaternionMsgToKindr(
    const geometry_msgs::Quaternion& msg,
    kindr::minimal::RotationQuaternionTemplate<Scalar>* kindr) {
  CHECK_NOTNULL(kindr);
  Eigen::Quaternion<Scalar> quat;
  quaternionMsgToEigen(msg, quat);
  *kindr = kindr::minimal::RotationQuaternionTemplate<Scalar>(quat);
}

// Also the Eigen implementation version of this.
template <typename Scalar>
void quaternionKindrToMsg(
    const Eigen::Quaternion<Scalar>& kindr, geometry_msgs::Quaternion* msg) {
  CHECK_NOTNULL(msg);
  quaternionEigenToMsg(kindr, *msg);
}

template <typename Scalar>
void quaternionMsgToKindr(
    const geometry_msgs::Quaternion& msg, Eigen::Quaternion<Scalar>* kindr) {
  CHECK_NOTNULL(kindr);
  Eigen::Quaternion<double> kindr_double;
  quaternionMsgToEigen(msg, kindr_double);
  *kindr = kindr_double.cast<Scalar>();
}

template <typename Scalar>
void rotationKindr2DToMsg(
    const Eigen::Rotation2D<Scalar>& kindr, geometry_msgs::Quaternion* msg) {
  CHECK_NOTNULL(msg);
  const kindr::minimal::RotationQuaternionTemplate<Scalar> quat(
      kindr::minimal::AngleAxisTemplate<Scalar>(
          kindr.angle(), static_cast<Scalar>(0.0), static_cast<Scalar>(0.0),
          static_cast<Scalar>(1.0)));
  quaternionEigenToMsg(quat.toImplementation(), *msg);
}

template <typename Scalar>
void quaternionMsgToKindr2D(
    const geometry_msgs::Quaternion& msg, Eigen::Rotation2D<Scalar>* kindr) {
  CHECK_NOTNULL(kindr);
  CHECK_LT(std::abs(msg.x), std::numeric_limits<double>::epsilon())
      << "No proper 2D rotation.";
  CHECK_LT(std::abs(msg.y), std::numeric_limits<double>::epsilon())
      << "No proper 2D rotation.";
  kindr::minimal::RotationQuaternionTemplate<Scalar> quat;
  quaternionMsgToEigen(msg, quat.toImplementation());
  const kindr::minimal::AngleAxisTemplate<Scalar> angle_axis(quat);
  kindr->angle() = angle_axis.angle();
}

// A wrapper for the relevant functions in eigen_conversions.
template <typename Scalar>
void pointKindrToMsg(
    const Eigen::Matrix<Scalar, 3, 1>& kindr, geometry_msgs::Point* msg) {
  CHECK_NOTNULL(msg);
  pointEigenToMsg(kindr, *msg);
}

template <typename Scalar>
void pointMsgToKindr(
    const geometry_msgs::Point& msg, Eigen::Matrix<Scalar, 3, 1>* kindr) {
  CHECK_NOTNULL(kindr);
  pointMsgToEigen(msg, *kindr);
}

template <typename Scalar>
void pointKindr2DToMsg(
    const Eigen::Matrix<Scalar, 2, 1>& kindr, geometry_msgs::Point* msg) {
  CHECK_NOTNULL(msg);
  msg->x = static_cast<double>(kindr.x());
  msg->y = static_cast<double>(kindr.y());
  msg->z = 0.0;
}

template <typename Scalar>
void pointMsgToKindr2D(
    const geometry_msgs::Point& msg, Eigen::Matrix<Scalar, 2, 1>* kindr) {
  CHECK_NOTNULL(kindr);
  // Verify that we got a proper 2D pose.
  CHECK_LT(std::abs(msg.z), std::numeric_limits<Scalar>::epsilon())
      << "No proper 2D position.";
  kindr->x() = static_cast<Scalar>(msg.x);
  kindr->y() = static_cast<Scalar>(msg.y);
}

template <typename Scalar>
void vectorKindrToMsg(
    const Eigen::Matrix<Scalar, 3, 1>& kindr, geometry_msgs::Vector3* msg) {
  CHECK_NOTNULL(msg);
  vectorEigenToMsg(kindr, *msg);
}

template <typename Scalar>
void vectorMsgToKindr(
    const geometry_msgs::Vector3& msg, Eigen::Matrix<Scalar, 3, 1>* kindr) {
  CHECK_NOTNULL(kindr);
  Eigen::Matrix<double, 3, 1> kindr_double;
  vectorMsgToEigen(msg, kindr_double);
  *kindr = kindr_double.cast<Scalar>();
}

// Convert a kindr::minimal::QuatTransformation to a 6 DoF geometry msgs pose.
template <typename Scalar>
void poseKindrToMsg(
    const kindr::minimal::QuatTransformationTemplate<Scalar>& kindr,
    geometry_msgs::Pose* msg) {
  CHECK_NOTNULL(msg);
  pointKindrToMsg(kindr.getPosition(), &msg->position);
  quaternionKindrToMsg(kindr.getRotation(), &msg->orientation);
}

template <typename Scalar>
void poseMsgToKindr(
    const geometry_msgs::Pose& msg,
    kindr::minimal::QuatTransformationTemplate<Scalar>* kindr) {
  CHECK_NOTNULL(kindr);
  Eigen::Matrix<Scalar, 3, 1> position;
  Eigen::Quaternion<Scalar> rotation;

  quaternionMsgToKindr(msg.orientation, &rotation);
  pointMsgToKindr(msg.position, &position);

  *kindr =
      kindr::minimal::QuatTransformationTemplate<Scalar>(rotation, position);
}

template <typename Scalar>
void poseStampedKindrToMsg(
    const kindr::minimal::QuatTransformationTemplate<Scalar>& kindr,
    const ros::Time& time, const std::string& reference_frame,
    geometry_msgs::PoseStamped* msg) {
  CHECK_NOTNULL(msg);
  msg->header.frame_id = reference_frame;
  msg->header.stamp = time;
  poseKindrToMsg(kindr, &msg->pose);
}

// Uses current time.
template <typename Scalar>
void poseStampedKindrToMsg(
    const kindr::minimal::QuatTransformationTemplate<Scalar>& kindr,
    const std::string& reference_frame, geometry_msgs::PoseStamped* msg) {
  poseStampedKindrToMsg(kindr, ros::Time(), reference_frame, msg);
}

// Convert a kindr::minimal::QuatTransformation to a geometry_msgs::Transform.
template <typename Scalar>
void transformKindrToMsg(
    const kindr::minimal::QuatTransformationTemplate<Scalar>& kindr,
    geometry_msgs::Transform* msg) {
  CHECK_NOTNULL(msg);
  vectorKindrToMsg(kindr.getPosition(), &msg->translation);
  quaternionKindrToMsg(kindr.getRotation(), &msg->rotation);
}

template <typename Scalar>
void transformMsgToKindr(
    const geometry_msgs::Transform& msg,
    kindr::minimal::QuatTransformationTemplate<Scalar>* kindr) {
  CHECK_NOTNULL(kindr);
  Eigen::Matrix<Scalar, 3, 1> position;
  Eigen::Quaternion<Scalar> rotation;

  quaternionMsgToKindr(msg.rotation, &rotation);
  vectorMsgToKindr(msg.translation, &position);

  *kindr =
      kindr::minimal::QuatTransformationTemplate<Scalar>(rotation, position);
}

template <typename Scalar>
void poseKindr2DToMsg(
    const kindr::minimal::Transformation2DTemplate<Scalar>& kindr,
    geometry_msgs::Pose* msg) {
  CHECK_NOTNULL(msg);
  pointKindr2DToMsg(kindr.getPosition(), &msg->position);
  rotationKindr2DToMsg(kindr.getRotation(), &msg->orientation);
}

// The input is assumed to be pseudo-2D: The z component of the position and
// the x/y components of the quaternion must be '0.0', otherwise this
// conversion will fail.
template <typename Scalar>
void poseMsgToKindr2D(
    const geometry_msgs::Pose& msg,
    kindr::minimal::Transformation2DTemplate<Scalar>* kindr) {
  CHECK_NOTNULL(kindr);
  Eigen::Matrix<Scalar, 2, 1> position;
  Eigen::Rotation2D<Scalar> rotation;

  quaternionMsgToKindr2D(msg.orientation, &rotation);
  pointMsgToKindr2D(msg.position, &position);

  *kindr = kindr::minimal::Transformation2DTemplate<Scalar>(rotation, position);
}

template <typename Scalar>
void poseStampedKindr2DToMsg(
    const kindr::minimal::Transformation2DTemplate<Scalar>& kindr,
    const ros::Time& time, const std::string& reference_frame,
    geometry_msgs::PoseStamped* msg) {
  CHECK_NOTNULL(msg);
  msg->header.frame_id = reference_frame;
  msg->header.stamp = time;
  poseKindr2DToMsg(kindr, &msg->pose);
}

}  // namespace tf

#endif  // MINKINDR_CONVERSIONS_KINDR_MSG_H
