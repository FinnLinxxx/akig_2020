#!/usr/bin/env python
"""
Andreas Bayr, TU Wien November 2020
"""

import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


class TachyData(geometry_msgs::PointStamped::ConstPtr& msg):
    def get_x(self):
        pass

    def get_y(self):
        pass

    def get_z(self):
        pass


class ImuData(sensor_msgs::Imu::ConstPtr&):
    def get_qx(self):
        pass

    def get_qy(self):
        pass

    def get_qz(self):
        pass

if __name__ == '__main__':
    rospy.init_node('tf_listen_node')
    rate = rospy.Rate(1.0)

    ImuData.get_qx()
    ImuData.get_qy()

    while not rospy.is_shutdown():
        rospy.sleep()

