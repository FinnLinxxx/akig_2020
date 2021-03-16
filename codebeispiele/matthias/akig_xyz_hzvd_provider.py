
#!/usr/bin/env python
# PointCloud2 as fix-pointis published
# https://answers.ros.org/question/289576/understanding-the-bytes-in-a-pcl2-message/
# Matthias Rosa, TU Wien, 19.2.2021
# http://wiki.ros.org/dynamic_reconfigure

#TODO: Fixunkteliste durch Fixpunkte in TU austauschen.
#TODO: Punktauswahl?

import sys
import struct
import time

import rospy
from geometry_msgs.msg import PointStamped
import geometry_msgs.msg
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

import tf
import math
import numpy as np

# Load fix-point data
fixed_point_ptcl = np.loadtxt("fixpoints.txt")

rospy.init_node("fixed_point_publisher")
pub = rospy.Publisher("fixed_points", PointCloud2, queue_size=1)
points = []

def callback(data, pub):

    y = data.pose.position.y
    x = data.pose.position.x
    z = data.pose.position.z
    quat = (data.pose.orientation.x,
	    data.pose.orientation.y,
	    data.pose.orientation.z,
	    data.pose.orientation.w)

    euler = tf.transformations.euler_from_quaternion(quat)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]
    o = yaw

    for pt_from_list in fixed_point_ptcl:
        y_val = pt_from_list[1]
        x_val = pt_from_list[2]
        z_val = pt_from_list[3]
        hz = (-1)*math.atan2(y_val-y,x_val-x) + o
	
        if hz<0:
	   
            hz = hz + 2*math.pi
        d = math.sqrt(math.pow(y_val-y,2)+math.pow(x_val-x,2)+math.pow(z_val-z,2))
        v = math.acos((z_val-z)/d)
        pt = [x_val, y_val, z_val, hz, v, d]
        points.append(pt)
    fields = [  PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
                PointField('hz', 12, PointField.FLOAT32, 1),
		PointField('v', 16, PointField.FLOAT32, 1),
		PointField('d', 20, PointField.FLOAT32, 1)
              ]

    header = Header()
    header.frame_id = "tachy_aim_data"
    pc2 = point_cloud2.create_cloud(header, fields, points)

    pc2.header.stamp = rospy.Time.now()
    pub.publish(pc2)

    points.clear()

rospy.Subscriber("/husky/current_pose", geometry_msgs.msg.PoseStamped, callback, pub)

while not rospy.is_shutdown():

    time.sleep(10)

