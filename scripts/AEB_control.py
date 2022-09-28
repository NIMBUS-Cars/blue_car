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
    r_i_dot = 1*v_x*math.cos(theta)
    if r_i_dot >= 0:
        TTC_i = float('inf')
    else:
        TTC_i = r_i / (-r_i_dot)
    return TTC_i

def calculate_ttc(r, v, theta):
    denominator = -1*v*math.cos(theta)
    if denominator <= 0: # this is basically doing the max(x,0) calculation, but helps us avoid math issues
        return float('inf')
    return r / denominator

class AEB:
    def __init__(self):
        self.speed = 1.5
        self.ttc_threshold = 1.0
        self.output_file = "5m.txt"
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.drive = rospy.Publisher(rospy.get_param('/nav_drive_topic'), AckermannDriveStamped, queue_size=10)
        self.drive_msg = AckermannDriveStamped()
        # self.ttc_data_file = open(self.output_file, "w") # This file is located in ~/.ros
        # self.ttc_data_file.write("ttc_threshold, min_ttc, speed\n")
        # self.ttc_data_file.close()

    def scan_callback(self, scan_msg):
        # Calculate the TTC around the vehicle (assuming ranges[0] is straight behind)
        min_ttc = float('inf')
        for i in range(0,len(scan_msg.ranges),2): # increment by 2 to reduce calculation time
            ttc = calculate_ttc(scan_msg.ranges[i], self.speed, i*scan_msg.angle_increment)
            min_ttc = min(min_ttc, ttc)

        if min_ttc <= self.ttc_threshold:
            self.speed = 0

        # if min_ttc != float('inf'):
        #     self.ttc_data_file = open(self.output_file, "a")
        #     self.ttc_data_file.write(str(self.ttc_threshold) + ", " + str(min_ttc) + ", " + str(self.speed) + "\n")
        #     self.ttc_data_file.close()

        self.drive_msg.drive.speed = self.speed
        self.drive.publish(self.drive_msg)


def main():
    rospy.init_node('AEB_node')
    sn = AEB()
    rospy.spin()


if __name__ == '__main__':
    main()
