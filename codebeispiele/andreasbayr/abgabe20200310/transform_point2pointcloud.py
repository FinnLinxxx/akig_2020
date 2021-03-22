#!/usr/bin/env python
"""
Andreas Bayr, TU Wien December 2020
Listen to pointcloud, determine pointcloud-frame to global frame, transform pointcloud, advertise pointcloud
The measured laser scanner pointcloud needs to be transformed according to the pose measured by imu and tachymeter
in respect to the global frame.
# make sure you start this with python2!
"""
import rospy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


class TransformPoint2PointCloud:

    def __init__(self):
        self.cloud_in = PointCloud2()

        # get parameters

        self.ptcl2_global_frame_ = rospy.get_param('ptcl2_global_frame')
        self.ptcl2_local_frame_ = rospy.get_param('ptcl2_local_frame')
        self.ptcl2_input_topic_ = rospy.get_param('ptcl2_input_topic')
        self.ptcl2_output_topic_ = rospy.get_param('ptcl2_output_topic')
        self.drop_when_same_position_ = rospy.get_param('drop_when_same_position')

        self.sub_ptcl = rospy.Subscriber(self.ptcl2_input_topic_, PointCloud2, self.callback, queue_size=10)
        self.pub_trans_ptcl = rospy.Publisher(self.ptcl2_output_topic_, PointCloud2, queue_size=1)

    def callback(self, msg):
        # print(msg)
        self.cloud_in = msg

        # for point in pc2.read_points(msg):
        #     # rospy.logwarn("x, y, z: %.1f, %.1f, %.1f" % (point[0], point[1], point[2]))
        #     print("x, y, z: %.1f, %.1f, %.1f" % (point[0], point[1], point[2]))

    def reset(self):
        self.cloud_in.width = 0

    def transform(self):
        buffer = tf2_ros.Buffer()  # default 10 sec
        tfl = tf2_ros.TransformListener(buffer)
        tfb = tf2_ros.TransformBroadcaster()
        last_x = 0
        while not rospy.is_shutdown():
            try:

                ts = buffer.lookup_transform(self.ptcl2_global_frame_, self.ptcl2_local_frame_, rospy.Time())
                ts_x = ts.transform.rotation.x

                if self.drop_when_same_position_:
                    if self.cloud_in.width and ts_x != last_x:
                        last_x = ts_x
                        print(" trans_x: ", ts_x)

                        # make sure you start this with python2!
                        cloud_out = do_transform_cloud(self.cloud_in, ts)
                        self.pub_trans_ptcl.publish(cloud_out)
                        self.reset()

                    elif ts_x == last_x:
                        print(" trans_x dropped: ", ts_x)

                elif self.cloud_in.width:

                    print(" never drop -> trans_x: ", ts_x)
                    cloud_out = do_transform_cloud(self.cloud_in, ts)
                    self.pub_trans_ptcl.publish(cloud_out)
                    self.reset()

            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException,
                    tf2_ros.TransformException) as ex:

                rospy.logwarn("%s", ex)
                rospy.sleep(1)
                continue

        rospy.spin()


if __name__ == '__main__':

    # frames
    rospy.set_param('ptcl2_global_frame', 'map')  # frames do not start with /
    rospy.set_param('ptcl2_local_frame', 'prisma_frame')

    # topics
    rospy.set_param('ptcl2_input_topic', '/scan_cloud')
    rospy.set_param('ptcl2_output_topic', '/transformed_new_ptcl')

    # additional parameters
    rospy.set_param('drop_when_same_position', False)

    # print(rospy.get_param_names())

    try:
        # initalize node
        rospy.init_node('transform_point2pointcloud', anonymous=True)
        p2pcl = TransformPoint2PointCloud()
        p2pcl.transform()

    except rospy.ROSInterruptException:
        pass
