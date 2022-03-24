#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

# This is based on a template obtained from https://github.com/f1tenth/f1tenth_labs/blob/main/lab2/code/scripts/safety_node.py
# Create a node to apply AEB based on sensor data

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from geometry_msgs.msg import TwistWithCovariance
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import math

def calculate_ttc(r, v, theta):
    denominator = -1*v*math.cos(theta)
    if denominator <= 0: # this is basically doing the max(x,0) calculation, but helps us avoid math issues
        return float('inf')
    return r / denominator

# This safety class is based on https://github.com/f1tenth/f1tenth_labs/blob/main/lab2/code/scripts/safety_node.py
class Safety(object):
    def __init__(self):
        self.speed = 0.5
        self.ttc_threshold = 1.0
        self.output_file = "2m.txt"
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.drive = rospy.Publisher(rospy.get_param('/nav_drive_topic'), AckermannDriveStamped, queue_size=10)
        self.drive_msg = AckermannDriveStamped()
        self.ttc_data_file = open(self.output_file, "w") # This file is located in ~/.ros
        self.ttc_data_file.write("ttc_threshold, min_ttc, speed\n")
        self.ttc_data_file.close()

    def scan_callback(self, scan_msg):
        # Calculate the TTC around the vehicle (assuming ranges[0] is straight behind)
        min_ttc = float('inf')
        for i in range(0,len(scan_msg.ranges),2): # increment by 2 to reduce calculation time
            ttc = calculate_ttc(scan_msg.ranges[i], self.speed, i*scan_msg.angle_increment)
            min_ttc = min(min_ttc, ttc)

        if min_ttc <= self.ttc_threshold:
            self.speed = 0

        if min_ttc != float('inf'):
            self.ttc_data_file = open(self.output_file, "a")
            self.ttc_data_file.write(str(self.ttc_threshold) + ", " + str(min_ttc) + ", " + str(self.speed) + "\n")
            self.ttc_data_file.close()

        self.drive_msg.drive.speed = self.speed
        self.drive.publish(self.drive_msg)


def main():
    rospy.init_node('safety')
    sn = Safety()
    rospy.spin()

if __name__ == '__main__':
    main()
