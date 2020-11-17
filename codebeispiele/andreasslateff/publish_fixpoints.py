#!/usr/bin/env python
# PointCloud2 as fix-pointis published
# https://answers.ros.org/question/289576/understanding-the-bytes-in-a-pcl2-message/
# Finn Linzer, TU Wien, 17.11.2020
# TODO: Evaluate possibility for dynamic reconfigure
# http://wiki.ros.org/dynamic_reconfigure

import sys
import struct

import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

import math
import numpy as np

# Load fix-point data
fixed_point_ptcl = np.loadtxt("fixed-points.txt")

rospy.init_node("fixed_point_publisher")
pub = rospy.Publisher("fixed_points", PointCloud2, queue_size=1)
points = []

rate = rospy.Rate(5.0)

while not rospy.is_shutdown():
    # Receive response

    for pt_from_list in fixed_point_ptcl:
        y_val = pt_from_list[1]
        x_val = pt_from_list[2]
        z_val = pt_from_list[3]

        pt = [x_val, y_val, z_val]
        points.append(pt)

    fields = [  PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
                # PointField('rgb', 12, PointField.UINT32, 1),
              ]

    header = Header()
    header.frame_id = "map"
    pc2 = point_cloud2.create_cloud(header, fields, points)

    pc2.header.stamp = rospy.Time.now()
    pub.publish(pc2)

    points.clear()

    rate.sleep()

