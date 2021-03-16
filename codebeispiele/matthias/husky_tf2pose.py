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


iii = 0

def callback(data, args):
    global iii
    iii = iii + 1
    if (iii % 100):
        return

    print(iii)
#for available_transforms in data.transforms:

#    if (available_transforms.child_frame_id == "base_link"):



    tfListener  = args[0]

    husky_vel   = args[1]

    cmd         = args[2]



    if tfListener.frameExists("base_link"):

        #and tfListener.frameExists("map"): <--- why does this give: false all the time?

        t = tfListener.getLatestCommonTime("/map", "/base_link")

        translation, rotation = tfListener.lookupTransform("/map", "/base_link", t)

        ### translation and rotation --> ros api: 

        ### http://docs.ros.org/en/jade/api/geometry_msgs/html/msg/Transform.html

        print(translation)

        #print(rotation)



        #print("Heard base_link tf")



        cmd.header = std_msgs.msg.Header()

        cmd.header.stamp = rospy.Time.now()

        cmd.header.frame_id = "/huskypose"



        #angular = 1337 #dummy values

        #linear = 42



        #cmd.twist.linear.x = linear

        #cmd.twist.angular.z = angular

        

        #pose or position?

        #https://de.wikipedia.org/wiki/Roll-Nick-Gier-Winkel

        #https://de.wikipedia.org/wiki/Eulersche_Winkel#Roll-,_Nick-_und_Gierwinkel:_z-y%E2%80%B2-x%E2%80%B3-Konvention

        

        #Quaternion?????



        cmd.pose.position.x = translation[0]
        cmd.pose.position.y = translation[1]
        cmd.pose.position.z = translation[2]
        cmd.pose.orientation.x = rotation[0]
        cmd.pose.orientation.y = rotation[1]
        cmd.pose.orientation.z = rotation[2]
        cmd.pose.orientation.w = rotation[3]

        husky_vel.publish(cmd)



if __name__ == '__main__':



    rospy.init_node('husky_tf2pose')



    tflistener = TransformListener()

    husky_vel = rospy.Publisher('husky/current_pose', geometry_msgs.msg.PoseStamped,queue_size=1)

    cmd = geometry_msgs.msg.PoseStamped()

    rospy.Subscriber("/tf", TFMessage, callback, (tflistener, husky_vel, cmd))



    rate = rospy.Rate(1.0)

    while not rospy.is_shutdown():

        # do something besides the callback function

        rate.sleep()
