/**
 * @file   tf_points_global.cpp
 * @Author Finn Linzer, 4. Master Semester HafenCity University (finn.linzer@gmail.com)
 * @date   August, 2017
 * @brief  Listen to pointcloud, determine pointcloud-frame to global frame, transform pointcloud, advertise pointcloud
 * The measured velodyne pointcloud needs to be transformed according to the pose measured by imu and tachymeter in respect to the global frame.

* The velodyne Pointcloud is provided by the velodyne package

* The Frame-transformation is provided via ROS lookupTransform
--->	transformStamped = tfBuffer.lookupTransform("map", "prisma_frame", ros::Time(0));

* The Transformation is calculated via tf2 functionality 
--->    tf2::doTransform(cloud_in, cloud_out, transformStamped);

* The transformed pointcloud is published by
--->    ros::Publisher pub_trans_ptcl = nh_trans_ptcl.advertise<sensor_msgs::PointCloud2>("transformed_velodyne_points",1); The name is transformed_velodyne_points

The transformation and advertisement of the measured pointcloud in respect to the global scope is main goal of the following code
 */

/**
 * @brief   rosrun xio_imu_driver xio_imu_driver_node
 *          check wheather prisma_frame is advertised already with the correct orientation 
 * TODO: Check if transformed pointcloud is valid
 * @param none
 *
 * @retval TRUE   Successfully provide transformed pointcloud
 * @retval FALSE  Oops, did something.
 *
 * Example Usage:
 * @code
 *    rosrun xio_imu_driver xio_imu_driver_node
      --->    but only after prisma_frame is already advertised
 * @endcode
 */

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include "sensor_msgs/PointCloud2.h"
#include <tf2/convert.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

class Ptcl
{
  public:
  Ptcl()
  {
  }
  void ptclCallback(const sensor_msgs::PointCloud2ConstPtr&);
  sensor_msgs::PointCloud2 getPointCloud2();
  void ptclReset();
private:
  void setPointCloud2(sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2 ptcl_mass;
};

void Ptcl::ptclCallback(const sensor_msgs::PointCloud2ConstPtr& msg){
  //ROS_INFO("ptclCallback on the run");
  sensor_msgs::PointCloud2 msg0 = *msg;
  setPointCloud2(msg0);
}
sensor_msgs::PointCloud2 Ptcl::getPointCloud2(){
  return ptcl_mass;
}
void Ptcl::setPointCloud2(sensor_msgs::PointCloud2 setPtcl){
  ptcl_mass = setPtcl;
}
void Ptcl::ptclReset(){
  ptcl_mass.width = 0;
}

int main(int argc, char **argv)
{
  // launch-parameter definitions
  std::string ptcl2_global_frame_;
  std::string ptcl2_local_frame_;
  std::string ptcl2_input_topic_;
  std::string ptcl2_output_topic_;
  bool drop_when_same_position_;

  Ptcl ptcl_object;
  ros::init(argc,argv, "transform_pointcloud");

  ros::NodeHandle pnh_("~");
  pnh_.param<std::string>("ptcl2_global_frame", ptcl2_global_frame_, "map");
  pnh_.param<std::string>("ptcl2_local_frame", ptcl2_local_frame_, "velodyne");
  pnh_.param<std::string>("ptcl2_input_topic", ptcl2_input_topic_, "/velodyne_points");
  pnh_.param<std::string>("ptcl2_output_topic", ptcl2_output_topic_, "/transformed_ptcl");
  pnh_.param<bool>("drop_when_same_position", drop_when_same_position_, true);

  ros::NodeHandle nh_ptcl;
  ros::Subscriber sub_ptcl = nh_ptcl.subscribe<sensor_msgs::PointCloud2> (ptcl2_input_topic_, 10, &Ptcl::ptclCallback, &ptcl_object);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::NodeHandle nh_trans_ptcl;
  ros::Publisher pub_trans_ptcl = nh_trans_ptcl.advertise<sensor_msgs::PointCloud2>(ptcl2_output_topic_,1);
  double lastX = 0;

  while(ros::ok()){
    geometry_msgs::TransformStamped transformStamped;
    try{
      transformStamped = tfBuffer.lookupTransform(ptcl2_global_frame_, ptcl2_local_frame_, ros::Time(0));
      sensor_msgs::PointCloud2 cloud_in, cloud_out;
      cloud_in = ptcl_object.getPointCloud2();
      //check size of pointcloud > 0 before transform
      if(drop_when_same_position_) { 
        if(cloud_in.width && transformStamped.transform.translation.x != lastX){
          lastX = transformStamped.transform.translation.x;
          std::cout << " transX: " << transformStamped.transform.translation.x << std::endl;
          tf2::doTransform(cloud_in, cloud_out, transformStamped);
          pub_trans_ptcl.publish(cloud_out);
          ptcl_object.ptclReset();
        } else if(transformStamped.transform.translation.x == lastX) {
          std::cout << " transX dropped:" << transformStamped.transform.translation.x << std::endl;
        }
      } else {
        if(cloud_in.width){
          std::cout << "never drop ->transX: " << transformStamped.transform.translation.x << std::endl;
          tf2::doTransform(cloud_in, cloud_out, transformStamped);
          pub_trans_ptcl.publish(cloud_out);
          ptcl_object.ptclReset();
        }
      }
    } catch (tf2::TransformException &ex){
	ROS_WARN("%s", ex.what());
	ros::Duration(1.0).sleep();
	continue;
    }
  ros::spinOnce();
  }
return 0;
}
