#!/usr/bin/env python
"""
Andreas Bayr, TU Wien December 2020
listen_platform_tf2_talker is listening to IMU and Tachymeter data for restrictive orientation and position data
and publishes it on a tf object.
TODO: merge better via deeply coupled sensors?
"""
import sys
import rospy
from geometry_msgs.msg import PointStamped, TransformStamped
from sensor_msgs.msg import Imu
import tf2_ros


class Tachymeter(PointStamped):
    """
    inherits from rospy PointStamped Object
    Can be Positions given by eg. Tachymeter, Odometry or GPS
    additional Callback function for ros Subscriber
    """

    def callback(self, tachy_msg):
        rospy.loginfo("Received position [x]%5.2f  [y]%5.2f [z]%5.2f",
                      tachy_msg.x, tachy_msg.y, tachy_msg.z)

        self.set_x(tachy_msg.x)
        self.set_y(tachy_msg.y)
        self.set_z(tachy_msg.z)

    def get_x(self):
        return self.point.x

    def get_y(self):
        return self.point.y

    def get_z(self):
        return self.point.z

    def set_x(self, value):
        self.point.x = value

    def set_y(self, value):
        self.point.y = value

    def set_z(self, value):
        self.point.z = value

    # TODO: timestamp for combining! problem: imu and tachymeter publish at different frequencies


class IMU(Imu):
    """
    inherits from rospy Imu Object
    callback function gets orientation values x, y, z, q
    TODO: maybe also accel, velo from IMU?
    """
    def callback(self, imu_msg):
        # rospy.loginfo("Received orientation [x]%5.2f  [y]%5.2f [z]%5.2f",
        #               imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z)

        self.orientation.x = imu_msg.orientation.x
        self.orientation.y = imu_msg.orientation.y
        self.orientation.z = imu_msg.orientation.z
        self.orientation.w = imu_msg.orientation.w


def publish_frame(pos, imu_msg, target_frame):
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "map"
    t.child_frame_id = target_frame

    # if not /tachy_points are published, take (0,0,0)
    if pos.get_x() != 0:
        t.transform.translation.x = pos.get_x()
        t.transform.translation.y = pos.get_y()
        t.transform.translation.z = pos.get_z()
    else:
        t.transform.translation.x = 0
        t.transform.translation.y = 0
        t.transform.translation.z = 0

    rospy.loginfo("Received orientation [x]%5.2f  [y]%5.2f [z]%5.2f [w]%5.2f",
                  imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w)

    t.transform.rotation.x = imu_msg.orientation.x
    t.transform.rotation.y = imu_msg.orientation.y
    t.transform.rotation.z = imu_msg.orientation.z
    t.transform.rotation.w = imu_msg.orientation.w

    rospy.loginfo("Transform sent!")
    br.sendTransform(t)


if __name__ == '__main__':
    try:
        rospy.init_node('tf2_husky_broadcaster', anonymous=True)
        pos = Tachymeter()
        imu = IMU()

        rospy.set_param('ptcl2_local_frame', 'prisma_frame')  # TODO: move to launch file

        local_frame_ = rospy.get_param('ptcl2_local_frame')
        tachy_sub = rospy.Subscriber('/tachy_points', PointStamped, pos.callback, local_frame_)
        imu_sub = rospy.Subscriber('/imu/data', Imu, imu.callback)

        while not rospy.is_shutdown():
            publish_frame(pos, imu, local_frame_)

            rospy.sleep(0.1)
            # or: (execute when subscriber message comes in)
            # rospy.spin()

    except rospy.ROSInterruptException:
        pass
