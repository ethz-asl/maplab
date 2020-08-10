#pragma once

// ros
#include <ros/ros.h>

// geometry msgs
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>

// tf
#include <tf/tf.h>

// tf conversions
#include <tf_conversions/tf_eigen.h>

// pointmatcher_ros
#include "pointmatcher_ros/PmTf.h"
#include "pointmatcher_ros/usings.h"

namespace PointMatcher_ros {

/*!
 * Creates a pointmatcher transformation from a ROS transform msg.
 * @param tfMsg ROS transform msg.
 * @return Pointmatcher transformation.
 */
PmTf tfMsgToPmTf(const geometry_msgs::TransformStamped& tfMsg);

/*!
 * Creates a ROS transform msg from a pointmatcher transformation.
 * @param pmTf Pointmatcher transformation.
 * @return ROS transform msg.
 */
geometry_msgs::TransformStamped pmTfToTfMsg(const PmTf& pmTf);

/*!
 * Creates a pointmatcher transformation from a ROS transformation.
 * @param tf ROS transformation.
 * @return Pointmatcher transformation.
 */
PmTf tfToPmTf(const tf::StampedTransform& tf);

/*!
 * Creates a ROS transformation from a pointmatcher transformation.
 * @param pmTf Pointmatcher transformation.
 * @return ROS transformation.
 */
tf::StampedTransform pmTfToTf(const PmTf& pmTf);

/*!
 * Creates a pose from a ROS transformation.
 * @param tf ROS transformation.
 * @return Pose.
 */
geometry_msgs::PoseStamped tfToPose(const tf::StampedTransform& tf);

/*!
 * Creates a ROS transformation from a pose.
 * @param pose         Pose.
 * @param childFrameId Child frame id.
 * @return ROS transformation.
 */
tf::StampedTransform poseToTf(const geometry_msgs::PoseStamped& pose, const std::string& childFrameId);

/*!
 * Creates a ROS transformation from a pose with covariance.
 * @param pose         Pose with covariance.
 * @param childFrameId Child frame id.
 * @return transformation.
 */
tf::StampedTransform poseWithCovToTf(const geometry_msgs::PoseWithCovarianceStamped& pose, const std::string& childFrameId);

/*!
 * Creates a pointmatcher transformation from a pose with covariance.
 * @param pose         Pose with covariance.
 * @param childFrameId Child frame id.
 * @return Pointmatcher transformation.
 */
PmTf poseWithCovToPmTf(const geometry_msgs::PoseWithCovarianceStamped& pose, const std::string& childFrameId);

/*!
 * Creates a pose from a pointmatcher transformation.
 * @param pmTf Pointmatcher transformation.
 * @return Pose.
 */
geometry_msgs::PoseStamped pmTfToPose(const PmTf& pmTf);

/*!
 * Creates a pointmatcher transformation from a pose.
 * @param pose         Pose.
 * @param childFrameId Child frame id.
 * @return Pointmatcher transformation.
 */
PmTf poseToPmTf(const geometry_msgs::PoseStamped& pose, const std::string& childFrameId);

}  // namespace PointMatcher_ros
