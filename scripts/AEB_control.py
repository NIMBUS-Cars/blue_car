#!/usr/bin/env python

"""AEB_control.py: Robot will automatic apply brake when see obsticle"""

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from geometry_msgs.msg import TwistWithCovariance
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
import rospy
import math


def TTC_calc(r_i, v_x, theta):
    r_i_dot = 1 * v_x * math.cos(theta)
    if r_i_dot >= 0:
        TTC_i = float('inf')
    else:
        TTC_i = r_i / (-r_i_dot)
    return TTC_i


class AEB:
    def __init__(self):
        # speed 1.0, min_ttc , ttc_threshold 1.0
        # speed 1.75, min_ttc 0.7, ttc_threshold 0.5
        self.speed = 1.5
        self.ttc_threshold = 0.8
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.drive = rospy.Publisher(rospy.get_param(
            '/nav_drive_topic'), AckermannDriveStamped, queue_size=10)
        self.drive_msg = AckermannDriveStamped()

    def scan_callback(self, scan_msg):
        # Calculate the TTC around the vehicle, angle 0 is front
        min_ttc = float('inf')
        # increment by 2 to reduce calculation time
        # forward_angle = 0
        min_angle = scan_msg.angle_min
        for i in range(0, len(scan_msg.ranges), 2):
            # theta = abs(forward_angle - (min_angle + i * scan_msg.angle_increment))
            # rospy.loginfo("theta: %s", theta)
            ttc = TTC_calc(
                scan_msg.ranges[i], self.speed, i * scan_msg.angle_increment)
            min_ttc = min(min_ttc, ttc)
        rospy.loginfo("min_ttc: %s", min_ttc)

        if min_ttc <= self.ttc_threshold:
            rospy.loginfo("Min TTC below Threshhold, Apply brake here")
            self.speed = 0

        self.drive_msg.drive.speed = self.speed
        self.drive.publish(self.drive_msg)


def main():
    rospy.init_node('AEB_node')
    sn = AEB()
    rospy.spin()


if __name__ == '__main__':
    main()
