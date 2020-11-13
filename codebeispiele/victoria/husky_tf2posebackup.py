#!/usr/bin/env python  
# TODO: Replace TwistStamped with PointStamped Data representation
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
import std_msgs.msg



if __name__ == '__main__':

    rospy.init_node('husky_tf2pose')
    listener = tf.TransformListener()
    husky_vel = rospy.Publisher('husky/cmd_vel', geometry_msgs.msg.TwistStamped,queue_size=1)
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        cmd = geometry_msgs.msg.TwistStamped()
        cmd.header = std_msgs.msg.Header()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "/map"

        angular = 4 * math.atan2(trans[1], trans[0])
        linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)

        cmd.twist.linear.x = linear
        cmd.twist.angular.z = angular
        husky_vel.publish(cmd)

        rate.sleep()
