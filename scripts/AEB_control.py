#!/usr/bin/env python

"""AEB_control.py: Robot will automatic apply brake when see obsticle"""

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from geometry_msgs.msg import TwistWithCovariance
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
import cv2
import rospy
import math


class AEB:

    def __init__(self):
        self.lidar_sub = rospy.Subscriber('/scan',
                                          Image, self.lidar_callback)
        self.drive = rospy.Publisher(rospy.get_param(
            '/nav_drive_topic'), AckermannDriveStamped, queue_size=10)
        self.drive_msg = AckermannDriveStamped()
        self.speed = 0.5

    def lidar_callback(self, msg):
        rospy.loginfo("lidar_callback")
        rospy.loginfo("lidar_msg %s", msg)


def main():
    rospy.init_node('AEB_node')
    sn = AEB()
    rospy.spin()


if __name__ == '__main__':
    main()
