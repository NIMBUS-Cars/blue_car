#!/usr/bin/env python

"""circle_drawing.py: Robot will draw a circle in a track"""

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from geometry_msgs.msg import TwistWithCovariance
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import rospy


class CircleDrawing:

    def __init__(self):
        self.odom_sub = rospy.Subscriber(
            '/vesc/odom', Odometry, self.odom_callback)
        self.drive = rospy.Publisher(rospy.get_param(
            '/nav_drive_topic'), AckermannDriveStamped, queue_size=10)
        self.drive_msg = AckermannDriveStamped()
        self.speed = 0
        self.steering = 0

    def odom_callback(self, odom_msg):
        rospy.loginfo("odom_callback")
        rospy.loginfo("odom_msg %s", odom_msg.pose.pose)
        rospy.loginfo("odom_msg %s", odom_msg.twist.twist)

        self.speed = 1
        self.steering = 1.5
        self.drive_msg.drive.speed = self.speed
        self.drive_msg.drive.steering_angle = self.steering
        self.drive.publish(self.drive_msg)


def main():
    rospy.init_node('circleDrawing')
    sn = CircleDrawing()
    rospy.spin()


if __name__ == '__main__':
    main()
