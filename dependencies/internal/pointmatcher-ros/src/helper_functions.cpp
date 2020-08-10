// PointMatcher_ros
#include "pointmatcher_ros/helper_functions.h"

namespace PointMatcher_ros {

PmTf tfMsgToPmTf(const geometry_msgs::TransformStamped& tfMsg) {
  tf::StampedTransform tf;
  tf::transformStampedMsgToTF(tfMsg, tf);
  return PointMatcher_ros::tfToPmTf(tf);
}

geometry_msgs::TransformStamped pmTfToTfMsg(const PmTf& pmTf) {
  geometry_msgs::TransformStamped tfMsg;
  tf::transformStampedTFToMsg(PointMatcher_ros::pmTfToTf(pmTf), tfMsg);
  return tfMsg;
}

PmTf tfToPmTf(const tf::StampedTransform& tf) {
  PmTf pmTf;
  pmTf.sourceFrameId_ = tf.child_frame_id_;
  pmTf.targetFrameId_ = tf.frame_id_;
  pmTf.stamp_ = tf.stamp_;
  Eigen::Affine3d affineTf;
  tf::transformTFToEigen(tf, affineTf);
  pmTf.parameters_ = affineTf.matrix().cast<float>();
  return pmTf;
}

tf::StampedTransform pmTfToTf(const PmTf& pmTf) {
  tf::StampedTransform tf;
  tf.child_frame_id_ = pmTf.sourceFrameId_;
  tf.frame_id_ = pmTf.targetFrameId_;
  tf.stamp_ = pmTf.stamp_;
  tf::Matrix3x3 basis(pmTf.parameters_(0, 0), pmTf.parameters_(0, 1), pmTf.parameters_(0, 2), pmTf.parameters_(1, 0),
                      pmTf.parameters_(1, 1), pmTf.parameters_(1, 2), pmTf.parameters_(2, 0), pmTf.parameters_(2, 1),
                      pmTf.parameters_(2, 2));
  tf.setBasis(basis);
  tf::Vector3 origin(pmTf.parameters_(0, 3), pmTf.parameters_(1, 3), pmTf.parameters_(2, 3));
  tf.setOrigin(origin);
  return tf;
}

geometry_msgs::PoseStamped tfToPose(const tf::StampedTransform& tf) {
  geometry_msgs::PoseStamped poseStamped;
  poseStamped.header.frame_id = tf.frame_id_;
  poseStamped.header.seq = 0;
  poseStamped.header.stamp = tf.stamp_;
  poseStamped.pose.position.x = tf.getOrigin().getX();
  poseStamped.pose.position.y = tf.getOrigin().getY();
  poseStamped.pose.position.z = tf.getOrigin().getZ();
  poseStamped.pose.orientation.w = tf.getRotation().getW();
  poseStamped.pose.orientation.x = tf.getRotation().getX();
  poseStamped.pose.orientation.y = tf.getRotation().getY();
  poseStamped.pose.orientation.z = tf.getRotation().getZ();
  return poseStamped;
}

tf::StampedTransform poseToTf(const geometry_msgs::PoseStamped& pose, const std::string& childFrameId) {
  const tf::Vector3 origin(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
  const tf::Quaternion rotation(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
  tf::StampedTransform tf;
  tf.frame_id_ = pose.header.frame_id;
  tf.child_frame_id_ = childFrameId;
  tf.stamp_ = pose.header.stamp;
  tf.setOrigin(origin);
  tf.setRotation(rotation);
  return tf;
}

tf::StampedTransform poseWithCovToTf(const geometry_msgs::PoseWithCovarianceStamped& pose, const std::string& childFrameId) {
  const tf::Vector3 origin(pose.pose.pose.position.x, pose.pose.pose.position.y, pose.pose.pose.position.z);
  const tf::Quaternion rotation(pose.pose.pose.orientation.x, pose.pose.pose.orientation.y, pose.pose.pose.orientation.z,
                                pose.pose.pose.orientation.w);
  tf::StampedTransform tf;
  tf.frame_id_ = pose.header.frame_id;
  tf.child_frame_id_ = childFrameId;
  tf.stamp_ = pose.header.stamp;
  tf.setOrigin(origin);
  tf.setRotation(rotation);
  return tf;
}

PmTf poseWithCovToPmTf(const geometry_msgs::PoseWithCovarianceStamped& pose, const std::string& childFrameId) {
  return tfToPmTf(poseWithCovToTf(pose, childFrameId));
}

geometry_msgs::PoseStamped pmTfToPose(const PmTf& pmTf) { return tfToPose(pmTfToTf(pmTf)); }

PmTf poseToPmTf(const geometry_msgs::PoseStamped& pose, const std::string& childFrameId) { return tfToPmTf(poseToTf(pose, childFrameId)); }

}  // namespace PointMatcher_ros
