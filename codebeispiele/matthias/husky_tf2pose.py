# AKIG 2020 Victoria Kostjak - 11771176
# Programm zum Auslesen der Pose des Huskeys
# Programm which publish current pose of the huskey

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

    # routine to slow down publishing rate
    global iii
    iii = iii + 1
    if (iii % 100):
        return

    print(iii)

    # get values from huskey
    tfListener  = args[0]

    husky_vel   = args[1]

    cmd         = args[2]


    # extract needed values
    if tfListener.frameExists("base_link"):

        t = tfListener.getLatestCommonTime("/map", "/base_link")

        translation, rotation = tfListener.lookupTransform("/map", "/base_link", t)


        # create posestamped
        cmd.header = std_msgs.msg.Header()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "/huskypose"

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

        rate.sleep()
