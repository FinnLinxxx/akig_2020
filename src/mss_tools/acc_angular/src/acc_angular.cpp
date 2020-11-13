#include <ros/ros.h>
#include <math.h>       /* cos */
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PointStamped.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
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
  double getIacc_X();
  double getIacc_Y();
  double getIacc_Z();
  double get_cummulative_x_dist();
  double get_cummulative_y_dist();
  double get_cummulative_z_dist();
  double get_cummulative_x_ang();
  double get_cummulative_y_ang();
  double get_cummulative_z_ang();
  void setZero_X();
  void setZero_Y();
private:
  void set_QX(double);
  void set_QY(double);
  void set_QZ(double);
  void set_QW(double);
  void setIacc_X(double);
  void setIacc_Y(double);
  void setIacc_Z(double);
  void add_ang_x(double,double);
  void add_ang_y(double,double);
  void add_ang_z(double,double);
  //void dist_x(double,double,double);
  //void dist_y(double,double,double);
  //void dist_z(double,double);
  //void spin_dead_reckoning(double, double, double, double, double, double, double, int, int);
  void spin_angular_vel(double, double, double, int, int);
  double prev_sec{0};
  double prev_nsec{0};
  // for moveInALi... x_dist{-1.277}, y_dist{2.11}
  double cummulative_x_ang{0};
  double cummulative_y_ang{0};
  double cummulative_z_ang{0};

  double cummulative_x_dist{0};
  double cummulative_y_dist{0};
  double cummulative_z_dist{0};
  double cummulative_x_vel{0};
  double cummulative_y_vel{0};
  double cummulative_z_vel{0};

  int bias_count{0};

  double bias_pitch_add{0};
  double bias_pitch{0};

  double bias_Iacc_X_add{0};
  double bias_Iacc_X{0};
  double bias_Iacc_Y_add{0};
  double bias_Iacc_Y{0};
  double bias_Iacc_Z_add{0};
  double bias_Iacc_Z{0};

  double qx_rot;
  double qy_rot;
  double qz_rot;
  double qw_rot;
  double iacc_lin_x;
  double iacc_lin_y;
  double iacc_lin_z;
  
};

void ImuInfo::imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
  set_QX(0);
  set_QY(0);
  set_QZ(0);
  set_QW(1);
  setIacc_X(0);
  setIacc_Y(0);
  setIacc_Z(0);
  //ROS_INFO("I heard an imu: [%s]", msg->header.frame_id.c_str());
  setIacc_X(msg->linear_acceleration.x);
  setIacc_Y(msg->linear_acceleration.y);
  setIacc_Z(msg->linear_acceleration.z);
  set_QX(msg->orientation.x);
  set_QY(msg->orientation.y);
  set_QZ(msg->orientation.z);
  set_QW(msg->orientation.w);
  //spin_dead_reckoning(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w, msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z, msg->header.stamp.sec, msg->header.stamp.nsec);
  spin_angular_vel(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z, msg->header.stamp.sec, msg->header.stamp.nsec);

}
void ImuInfo::setIacc_X(double setIaccX_fromImu){
  iacc_lin_x = setIaccX_fromImu;
}
void ImuInfo::setIacc_Y(double setIaccY_fromImu){
  iacc_lin_y = setIaccY_fromImu;
}
void ImuInfo::setIacc_Z(double setIaccZ_fromImu){
  iacc_lin_z = setIaccZ_fromImu;
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
void ImuInfo::setZero_X() {
  cummulative_x_dist = 0;
}
void ImuInfo::setZero_Y() {
  cummulative_y_dist = 0;
}
double ImuInfo::getIacc_X(){
  return iacc_lin_x;
}
double ImuInfo::getIacc_Y(){
  return iacc_lin_y;
}
double ImuInfo::getIacc_Z(){
  return iacc_lin_z;
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
double ImuInfo::get_cummulative_x_dist(){
  return cummulative_x_dist;
}
double ImuInfo::get_cummulative_y_dist(){
  return cummulative_y_dist;
}
double ImuInfo::get_cummulative_z_dist(){
  return cummulative_z_dist;
}

double ImuInfo::get_cummulative_x_ang(){
  return cummulative_x_ang;
}
double ImuInfo::get_cummulative_y_ang(){
  return cummulative_y_ang;
}
double ImuInfo::get_cummulative_z_ang(){ 
  return cummulative_z_ang; 
}

void ImuInfo::spin_angular_vel(double ang_x, double ang_y, double ang_z, int time_sec, int time_nsec){
double hz = 0;

  if( time_sec == prev_sec){
    //std::cout << "_nsec_vergangen: " << time_nsec - prev_nsec  << std::endl;
    hz = 1000000000 / (time_nsec - prev_nsec);
    prev_nsec = time_nsec;
  } else { 
    //std::cout << "jump prev_nsec: " << ((1000000000 - prev_nsec) + time_nsec) << std::endl;
    hz = 1000000000 / ((1000000000 - prev_nsec) + time_nsec);
    prev_nsec = time_nsec; 
  }
  prev_sec = time_sec;
  //std::cout << "Iacc_x: " << iacc_x << std::endl;
 // std::cout << "Iacc_y: " << iacc_y << std::endl;
  double freq = 0;
  freq = 1./hz;

  add_ang_x(ang_x, freq);
  add_ang_y(ang_y, freq);
  add_ang_z(ang_z, freq);
  //dist_y((cos(roll) * (iacc_y - bias_Iacc_Y)) ,freq, yaw); 
  ///dist_z(iacc_z - bias_Iacc_Z ,freq); 
  
} 


void ImuInfo::add_ang_x(double ang_x, double freq){
    cummulative_x_ang += ang_x * freq;
}
void ImuInfo::add_ang_y(double ang_y, double freq){
    cummulative_y_ang += ang_y * freq;
}
void ImuInfo::add_ang_z(double ang_z, double freq){
    cummulative_z_ang += ang_z * freq;
}
/*
void ImuInfo::dist_x(double iacc_x, double freq, double yaw){
  cummulative_x_dist = cummulative_x_dist + cos(-yaw) * (0.5 * iacc_x * freq * freq  + cummulative_x_vel * freq); 
  cummulative_x_vel = cummulative_x_vel + iacc_x * freq;
}
void ImuInfo::dist_y(double iacc_y, double freq, double yaw){
  cummulative_y_dist = cummulative_y_dist + sin(-yaw) * (0.5 * iacc_y * freq * freq  + cummulative_y_vel * freq); 
  cummulative_y_vel = cummulative_y_vel + iacc_y * freq;
}
//void ImuInfo::dist_y(double iacc_y, double freq, double yaw){
//  cummulative_y_dist = cummulative_y_dist + sin(-yaw) * (0.5 * iacc_y * freq * freq  + cummulative_y_vel * freq); 
//  cummulative_y_vel = cummulative_y_vel + iacc_y * freq;
//}
void ImuInfo::dist_z(double iacc_z, double freq){
  cummulative_z_dist = cummulative_z_dist + 0.5 * iacc_z * freq * freq  + cummulative_z_vel * freq; 
  cummulative_z_vel = cummulative_z_vel + iacc_z * freq;
}
*/


int main(int argc, char **argv)
{

  double rotation_value;

  if(argc != 2){
    ROS_INFO("No init angle for imu was set, just asume 0 degree");
    ROS_INFO("Usage: rosrun platform_tf listener 21    -   for 21 degree");
    sleep(3);
    rotation_value = 0;
  } else {
    //TODO(linzer): define PI in global scope
    rotation_value = (atof(argv[1]) /180) * 3.14159265359;
  }

  ImuInfo imu_object;
  TachyInfo tachy_object;

  ros::init(argc, argv, "acc_angular_listener");
  ros::NodeHandle m;
  ros::Subscriber subm = m.subscribe("imu/data_raw", 1000, &ImuInfo::imuCallback, &imu_object);


  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("tachy_points", 1000, &TachyInfo::tachyCallback, &tachy_object);

  //------- publish transform ---
  tf2_ros::TransformBroadcaster tfb;
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.frame_id = "map";
  transformStamped.child_frame_id = "acc_angular_frame";
  ros::Rate loop_rate(300); 
  ros::Publisher tf_pub;

  double check_prev_tachy_X = 0;
  double check_prev_tachy_Y = 0;

while(ros::ok())
{

  tf2::Quaternion q_value;
       q_value.setRPY(0, 0, rotation_value);
 
  transformStamped.header.stamp = ros::Time::now();

  tf2::Quaternion q_rotate_rpy(imu_object.get_cummulative_x_ang(), imu_object.get_cummulative_y_ang(), imu_object.get_cummulative_z_ang());

std::cout << "cum x_ang: " << imu_object.get_cummulative_x_ang() << std::endl;
std::cout << "cum y_ang: " << imu_object.get_cummulative_y_ang() << std::endl;
std::cout << "cum z_ang: " << imu_object.get_cummulative_z_ang() << std::endl << std::endl;

  q_rotate_rpy *= q_value;

  transformStamped.transform.rotation.x = q_rotate_rpy.x(); //bar_object.getQX();
  transformStamped.transform.rotation.y = q_rotate_rpy.y(); //bar_object.getQY();
  transformStamped.transform.rotation.z = q_rotate_rpy.z(); //bar_object.getQZ();
  transformStamped.transform.rotation.w = q_rotate_rpy.w(); //bar_object.getQW();

  transformStamped.transform.translation.x = 0; 
  transformStamped.transform.translation.y = 0;
  transformStamped.transform.translation.z = 0; 


  tfb.sendTransform(transformStamped);


  ros::spinOnce();
  loop_rate.sleep();



}
  return 0;
}
