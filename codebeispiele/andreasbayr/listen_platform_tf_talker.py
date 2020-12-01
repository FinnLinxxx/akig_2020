#!/usr/bin/env python
"""
Andreas Bayr, TU Wien November 2020
"""
import roslib
import rospy
import math
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PointStamped, Quaternion, TransformStamped
import sys

import tf
import tf2_ros

# roslib.load_manifest('listen_tf')  # Update the Python sys.path with package's dependencies


class TachyData:

    def __init__(self):
        self.get_x()
        self.get_y()
        self.get_z()

    def callback(self):
        return PointStamped(self.get_x(), self.get_y(), self.get_z())

    def get_x(self):
        pass

    def get_y(self):
        pass

    def get_z(self):
        pass

    def set_x(self):
        # how to retun only x ?
        return PointStamped.point[0]

    def set_y(self):
        return

    def set_z(self):
        return


class PlatformTFListener:
    def __init__(self, frame):
        # initialize rosnode
        # 'anonymous' let different instances of this node co-exist
        rospy.init_node('tf_listener', anonymous=True)

        # init setup
        self.point = PointStamped()
        self.point.point.x = 0
        self.point.point.y = 0
        self.point.point.z = 0

        # get position (x,y,z) from tachymeter
        self.sub_tachy = rospy.Subscriber("/tachydata", PointStamped, self.tachy_callback, frame)

        # init setup
        self.quaternion = Quaternion()
        self.quaternion.x = 0
        self.quaternion.y = 0
        self.quaternion.z = 0
        self.quaternion.w = 0

        # get orientation (x,y,z,q) from imu
        self.sub_imu = rospy.Subscriber("/imudata", Quaternion, frame)
        # no callback function for imu?

        # publish transformed, (queue_size: temporary storage)
        self.pub_tf = rospy.Publisher("/tf", tf.msg.tfMessage, queue_size=5)

    def tachy_callback(self, msg):
        rospy.loginfo("Received position [x]%5.2f  [y]%5.2f [z]%5.2f",
                      msg.x, msg.y, msg.z)
        self.point.point.x = msg.x
        self.point.point.y = msg.y
        self.point.point.z = msg.z

        # publish now here?

    def imu_callback(self):
        pass

    def transform(self):

        t = TransformStamped()
        t.header.frame_id = "frame1"
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = "frame2"

        rate = 0.1

        while not rospy.is_shutdown():

            t.transform.translation.x = self.point.point.x
            t.transform.translation.y = self.point.point.y
            t.transform.translation.z = self.point.point.z

            t.transform.rotation.x = self.quaternion.x
            t.transform.rotation.y = self.quaternion.y
            t.transform.rotation.z = self.quaternion.z
            t.transform.rotation.w = self.quaternion.w

            tfm = tf.msg.tfMessage([t])
            self.pub_tf.publish(tfm)

            # Run this loop at about 10Hz
            rospy.sleep(rate)


if __name__ == '__main__':
    try:
        # take frame_id as argument (?) tf Object from Tachy Pointcloud?
        frame_id = sys.argv[0]

        # create ros listener with buffer (?)
        # tfBuffer = tf2_ros.Buffer()
        # listener = tf2_ros.TransformListener(tfBuffer)
        # prisma_frame = tf.TransformListener()

        listener = PlatformTFListener(frame_id)
        listener.transform()
    except rospy.ROSInterruptException:
        pass
