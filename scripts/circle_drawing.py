#!/usr/bin/env python

"""circle_drawing.py: Robot will draw a circle in a track"""

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from geometry_msgs.msg import TwistWithCovariance
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import rospy
import math


class CircleDrawing:

    def __init__(self):
        self.speed = 0
        self.steering = 0
        self.drive = rospy.Publisher(rospy.get_param('nav_drive_topic'), AckermannDriveStamped, queue_size=10)
        rospy.Subscriber('/vesc/odom', Odometry, self.odometryCallback)
        self.drive_msg = AckermannDriveStamped()


    def odometryCallback(self, odom_msg):
        rospy.loginfo("odom_msg %s", odom_msg.pose.pose)
        rospy.loginfo("odom_msg %s", odom_msg.twist.twist)

        self.speed = 0.5
        self.steering = 1
        self.drive_msg.drive.speed = self.speed
        self.drive_msg.drive.steering_angle = self.steering
        self.drive.publish(self.drive_msg)


def main():
    rospy.init_node('circleDrawing')
    sn = CircleDrawing()
    rospy.spin()


if __name__ == '__main__':
    main()
