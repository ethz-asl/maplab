#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from maplab_msgs.msg import OdometryWithImuBiases

class GPSToOdometry:
    def __init__(self):
        self.odom_sub = rospy.Subscriber(
            '/mavros/global_position/local', Odometry,
            self.odom_callback, queue_size=10)
        self.odom_pub = rospy.Publisher(
            '/mavros/odometry', OdometryWithImuBiases,
            queue_size=10)

    def odom_callback(self, msg):
        odom_msg = OdometryWithImuBiases()
        odom_msg.header = msg.header
        odom_msg.pose = msg.pose
        odom_msg.twist = msg.twist

        self.odom_pub.publish(odom_msg)

if __name__ == '__main__':
    rospy.init_node('gps_to_odom', anonymous=True)

    gps_to_odom = GPSToOdometry()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down.")
