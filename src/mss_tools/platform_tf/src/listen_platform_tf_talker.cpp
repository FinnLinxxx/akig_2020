/**
 * @file   listen_platform_tf_talker.cpp
 * @Author Finn Linzer, 4. Master Semester HafenCity University (finn.linzer@gmail.com)
 * @date   August, 2017
 * @brief  Listen to Tachymeter and IMU data, merge them into a TF - Frame
 * @brief  Indentation: Two spaces, no tabs

 * The Pose (orientation and position) of a mobile mapping system is to be detected. Therefore we need a Position (measured by tachymeter) and an orientation (measured by imu).

 * Tachymeter Data are measured continously by a modern Tachymeter. The data are distributed over a ros NodeHandle (ex. /tachy_points). These Data are already converted to a X,Y,Z Coordinate + Timestamp. This data is brought to you via a geometry_msgs::PointStamped object.

*  IMU Data are measured continously by a modern IMU. The data are distributed over a ros NodeHandle (ex. imu/data). This topic measures the orientation of a platform. This data is brought to you via a sensor_msgs::Imu object.

To have the correct pose in ros, we merge these information to a full functional TF2-Frame. The correct advertisement of this information is the main goal of the following code.
 */

/**
 * @name    TODO: differ between imu and tachy only datastream
 * @brief   rosrun platform_tf listen_platform_tf_talker 23
 * 	    check wheather imu and tachymeter data are available with correct names
23 is an example for the init angle of the imu, check which value fits best.
TODO: Determine angle over absolute method
 *
 * @param [in] degree . Degree in which the data should be tranformed in the Z-Axis
 *
 * @retval TRUE   Successfully provide Frame
 * @retval FALSE  Oops, did something.
 *
 * Example Usage:
 * @code
 *    rosrun platform_tf listen_platform_tf_talker 23  // turn incoming orientation 23 degree in Z-Axis
 * @endcode
 */


#include <ros/ros.h>
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PointStamped.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

class TachyInfo
{
public:
  TachyInfo()
  {
  }
  void tachyCallback(const geometry_msgs::PointStamped::ConstPtr&);
  double get_X();
  double get_Y();
  double get_Z();
  unsigned int get_sec();
  unsigned int get_nsec();
private:
  void set_X(double);
  void set_Y(double);
  void set_Z(double);
  double x_coord;
  double y_coord;
  double z_coord;
  void set_time(unsigned int, unsigned int);
  unsigned int nsec_last_message{0};
  unsigned int sec_last_message{0};
};

void TachyInfo::tachyCallback(const geometry_msgs::PointStamped::ConstPtr& msg){
  set_X(0);
  set_Y(0);
  set_Z(0);
  set_time(0,0);
  //ROS_INFO("I heard a tachymeter: [%s]", msg->header.frame_id.c_str());
  set_X(msg->point.x);
  set_Y(msg->point.y);
  set_Z(msg->point.z);
  //TODO: Ermitteln wie der Zeitoffset zwicshen Timestamp::ROS ist und der tatsächlichen Zeit (to be defined).
  set_time(msg->header.stamp.sec, msg->header.stamp.nsec);
}
void TachyInfo::set_X(double setX_fromTachy){
  x_coord = setX_fromTachy;
}
void TachyInfo::set_Y(double setY_fromTachy){
  y_coord = setY_fromTachy;
}
void TachyInfo::set_Z(double setZ_fromTachy){
  z_coord = setZ_fromTachy;
}
void TachyInfo::set_time(unsigned int sec_message_from_tachy, unsigned int nsec_message_from_tachy){
  sec_last_message = sec_message_from_tachy;
  nsec_last_message = nsec_message_from_tachy;
}
double TachyInfo::get_X(){
  return x_coord;
}
double TachyInfo::get_Y(){
  return y_coord;
}
double TachyInfo::get_Z(){
  return z_coord;
}
unsigned int TachyInfo::get_nsec(){
  return nsec_last_message;
}
unsigned int TachyInfo::get_sec(){
  return sec_last_message;
}


class ImuInfo
{
public:
  ImuInfo()
  {
  }
  void imuCallback(const sensor_msgs::Imu::ConstPtr&);
  double get_QX();
  double get_QY();
  double get_QZ();
  double get_QW();
  unsigned int get_sec();
  unsigned int get_nsec();
private:
  void set_QX(double);
  void set_QY(double);
  void set_QZ(double);
  void set_QW(double);
  double qx_rot;
  double qy_rot;
  double qz_rot;
  double qw_rot;
  void set_time(unsigned int, unsigned int);
  unsigned int nsec_last_message{0};
  unsigned int sec_last_message{0};
};

void ImuInfo::imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
  set_QX(0);
  set_QY(0);
  set_QZ(0);
  set_QW(1);
  //ROS_INFO("I heard an imu: [%s]", msg->header.frame_id.c_str());
  set_QX(msg->orientation.x);
  set_QY(msg->orientation.y);
  set_QZ(msg->orientation.z);
  set_QW(msg->orientation.w);
  set_time(msg->header.stamp.sec, msg->header.stamp.nsec);
}
void ImuInfo::set_QX(double setQX_fromImu){
  qx_rot = setQX_fromImu;
}
void ImuInfo::set_QY(double setQY_fromImu){
  qy_rot = setQY_fromImu;
}
void ImuInfo::set_QZ(double setQZ_fromImu){
  qz_rot = setQZ_fromImu;
}
void ImuInfo::set_QW(double setQW_fromImu){
  qw_rot = setQW_fromImu;
}
double ImuInfo::get_QX(){
  return qx_rot;
}
double ImuInfo::get_QY(){
  return qy_rot;
}
double ImuInfo::get_QZ(){
  return qz_rot;
}
double ImuInfo::get_QW(){
  return qw_rot;
}
void ImuInfo::set_time(unsigned int sec_message_from_imu, unsigned int nsec_message_from_imu){
  sec_last_message = sec_message_from_imu;
  nsec_last_message = nsec_message_from_imu;
}
unsigned int ImuInfo::get_nsec(){
  return nsec_last_message;
}
unsigned int ImuInfo::get_sec(){
  return sec_last_message;
}





int main(int argc, char **argv){


  // launch-parameter definitions
  std::string pose_global_frame_;
  std::string pose_local_frame_;
  double system_orientation_z_;
  std::string position_provider_;
  std::string orientation_provider_;

  TachyInfo tachy_object;
  ImuInfo imu_object;
  ros::init(argc, argv, "listen_platform_tf_talker");

  ros::NodeHandle pnh_("~");
  pnh_.param<std::string>("pose_global_frame", pose_global_frame_, "map");
  pnh_.param<std::string>("pose_local_frame", pose_local_frame_, "prisma_frame");
  pnh_.param<double>("system_orientation_z", system_orientation_z_, 0.0);
  pnh_.param<std::string>("position_provider", position_provider_, "/tachy_points");
  pnh_.param<std::string>("orientation_provider", orientation_provider_, "/imu/data_raw");

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe(position_provider_, 1000, &TachyInfo::tachyCallback, &tachy_object);

  ros::NodeHandle m;
  ros::Subscriber subm = m.subscribe(orientation_provider_, 1000, &ImuInfo::imuCallback, &imu_object);

  tf2_ros::TransformBroadcaster tfb;
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.frame_id = pose_global_frame_;
  transformStamped.child_frame_id = pose_local_frame_;
  tf2::Quaternion q_init(imu_object.get_QX(), imu_object.get_QY(), imu_object.get_QZ(), imu_object.get_QW());
 
  while(ros::ok()){
    ros::spinOnce();

    transformStamped.header.stamp = ros::Time::now();

    // Quaternion rotation is simply possible via multiplication
    // Measured imu quaternion gets rotated with argv rotation value
    // Set rotation value is set by user at programm start
    // This rotation value needs to be set as an offset 
    // TODO(linzer): Make determination of rot obsolete
    tf2::Quaternion q_rotate(imu_object.get_QX(), imu_object.get_QY(), imu_object.get_QZ(), imu_object.get_QW());
    tf2::Quaternion q_value2rotatequaternion;
      q_value2rotatequaternion.setRPY(0, 0, (system_orientation_z_*3.14159265359/180));
    q_rotate = q_value2rotatequaternion*q_rotate;
    q_rotate = q_rotate-q_init;
 
    // Acquired values may be near 0 or NaN.
    // TODO(linzer): Inform user via ROS_INFO
    transformStamped.transform.rotation.x = q_rotate.x(); 
    transformStamped.transform.rotation.y = q_rotate.y();
    transformStamped.transform.rotation.z = q_rotate.z();
    transformStamped.transform.rotation.w = q_rotate.w();

    transformStamped.transform.translation.x = tachy_object.get_X();
    transformStamped.transform.translation.y = tachy_object.get_Y();
    transformStamped.transform.translation.z = tachy_object.get_Z();
    
    // Time offset check between imu and tachymeter message may be considered. 
    // How to check for updated transform
    //unsigned int tachy_sec = tachy_object.get_sec();
    //unsigned int tachy_nsec = tachy_object.get_nsec();
    //unsigned int imu_sec = imu_object.get_sec();
    //unsigned int imu_nsec = imu_object.get_nsec();
    //unsigned int tachy_imu_diff_sec = imu_sec-tachy_sec;
    //unsigned int tachy_imu_diff_nsec = imu_nsec-tachy_nsec;
    //std::cout << "tachy_imu_diff: " << tachy_imu_diff_sec << "." << tachy_imu_diff_nsec << std::endl;
    tfb.sendTransform(transformStamped);
    ROS_INFO("platform_tf SEND!");
  }
return 0;
} 
