#!/usr/bin/env python

import rospy

from geometry_msgs.msg import PoseWithCovarianceStamped
from maplab_msgs.msg import OdometryWithImuBiases

odometry_counter = 0
pub = None
sub = None


def callback(odometry_estimate):
    global odometry_counter
    global pub

    rospy.loginfo(
        "[TEST-Absolute6DoFPosePublisher] received odometry estimate and publish a localization for it."
    )

    # Only publish localization every 10th odometry message.
    if odometry_counter < 10:
        odometry_counter = odometry_counter + 1
        return
    else:
        odometry_counter = 0

    #  Set timestamp
    absolute_constraint = PoseWithCovarianceStamped()
    absolute_constraint.header.stamp = odometry_estimate.header.stamp
    absolute_constraint.pose = odometry_estimate.pose

    # Add some constant offset that represents the transformation between World and Odometry frame.
    absolute_constraint.pose.pose.position.x = absolute_constraint.pose.pose.position.x + 5.0
    absolute_constraint.pose.pose.position.y = absolute_constraint.pose.pose.position.y + 10.0
    absolute_constraint.pose.pose.position.z = absolute_constraint.pose.pose.position.z + 20.0

    absolute_constraint.pose.pose.orientation.x = absolute_constraint.pose.pose.orientation.x
    absolute_constraint.pose.pose.orientation.y = absolute_constraint.pose.pose.orientation.y
    absolute_constraint.pose.pose.orientation.z = absolute_constraint.pose.pose.orientation.z
    absolute_constraint.pose.pose.orientation.w = absolute_constraint.pose.pose.orientation.w

    pub.publish(absolute_constraint)


def absolute_pose_test_publisher():
    global pub
    global sub

    pub = rospy.Publisher(
        'test_absolute_6dof', PoseWithCovarianceStamped, queue_size=10)
    sub = rospy.Subscriber(
        'odometry_subscriber', OdometryWithImuBiases, callback, queue_size=10)
    rospy.init_node('test_absolute_6dof', anonymous=True)
    rospy.spin()


if __name__ == '__main__':
    try:
        absolute_pose_test_publisher()
    except rospy.ROSInterruptException:
        print "Something went wrong!"
