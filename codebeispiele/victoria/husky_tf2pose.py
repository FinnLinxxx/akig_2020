#!/usr/bin/env python  
# TODO: Replace TwistStamped with PointStamped Data representation
import roslib
import rospy
import math
import tf
import tf2_ros
from tf2_msgs.msg import TFMessage
import geometry_msgs.msg
import std_msgs.msg
from tf import TransformListener

def callback(data, args):
#for available_transforms in data.transforms:
#    if (available_transforms.child_frame_id == "base_link"):

    tfListener  = args[0]
    husky_vel   = args[1]
    cmd         = args[2]

    if tfListener.frameExists("base_link"):
        #and tfListener.frameExists("map"): <--- why does this give: false all the time?
        t = tfListener.getLatestCommonTime("/base_link", "/map")
        translation, rotation = tfListener.lookupTransform("/base_link", "/map", t)
        ### translation and rotation --> ros api: 
        ### http://docs.ros.org/en/jade/api/geometry_msgs/html/msg/Transform.html

        print("Heard base_link tf")

        cmd.header = std_msgs.msg.Header()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "/huskypose"

        angular = 1337 #dummy values
        linear = 42

        cmd.twist.linear.x = linear
        cmd.twist.angular.z = angular
        husky_vel.publish(cmd)

if __name__ == '__main__':

    rospy.init_node('husky_tf2pose')

    tflistener = TransformListener()
    husky_vel = rospy.Publisher('husky/cmd_vel', geometry_msgs.msg.TwistStamped,queue_size=1)
    cmd = geometry_msgs.msg.TwistStamped()
    rospy.Subscriber("/tf", TFMessage, callback, (tflistener, husky_vel, cmd))

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        # do something besides the callback function

        rate.sleep()
