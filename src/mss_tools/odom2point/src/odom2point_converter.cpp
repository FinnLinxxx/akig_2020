#include <ros/ros.h>
#include "std_msgs/String.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Odometry.h"

class OdomInfo
{
public:
  OdomInfo()
  {
  }
  void odomCallback(const nav_msgs::Odometry::ConstPtr&);
  double get_X();
  double get_Y();
  double get_Z();

  double get_X_ref();
  double get_Y_ref();
  double get_Z_ref();
  unsigned int get_sec();
  unsigned int get_nsec();
private:
  void set_X(double);
  void set_Y(double);
  void set_Z(double);

  void set_X_ref(double);
  void set_Y_ref(double);
  void set_Z_ref(double);

  double x_coord;
  double y_coord;
  double z_coord;
  void set_time(unsigned int, unsigned int);
  unsigned int nsec_last_message{0};
  unsigned int sec_last_message{0};

  int first_val_set{0};
  double x_coord_ref; //ref coordinates, first coordinate to reduce to 0
  double y_coord_ref;
  double z_coord_ref;

};

void OdomInfo::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
  set_X(0);
  set_Y(0);
  set_Z(0);
  set_time(0,0);
  //ROS_INFO("I heard a tachymeter: [%s]", msg->header.frame_id.c_str());
  set_X(msg->pose.pose.position.x);
  set_Y(msg->pose.pose.position.y);
  set_Z(msg->pose.pose.position.z);
  //TODO: Ermitteln wie der Zeitoffset zwicshen Timestamp::ROS ist und der tatsächlichen Zeit (to be defined).
  set_time(msg->header.stamp.sec, msg->header.stamp.nsec);
  if( !first_val_set && msg->pose.pose.position.x != 0 ) {
    first_val_set = 1;
    set_X_ref(msg->pose.pose.position.x);
    set_Y_ref(msg->pose.pose.position.y);
    set_Z_ref(msg->pose.pose.position.z);
  }

    printf("%7.4f %7.4f %7.4f\n", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    //std::cout << msg->pose.pose.position.x;
    //std::cout << " " << msg->pose.pose.position.y;
    //std::cout << " " << msg->pose.pose.position.z << std::endl;

}
void OdomInfo::set_X(double setX_fromTachy){
  x_coord = setX_fromTachy;
}
void OdomInfo::set_Y(double setY_fromTachy){
  y_coord = setY_fromTachy;
}
void OdomInfo::set_Z(double setZ_fromTachy){
  z_coord = setZ_fromTachy;
}


void OdomInfo::set_X_ref(double setX_fromTachy){
  x_coord_ref = setX_fromTachy;
}
void OdomInfo::set_Y_ref(double setY_fromTachy){
  y_coord_ref = setY_fromTachy;
}
void OdomInfo::set_Z_ref(double setZ_fromTachy){
  z_coord_ref = setZ_fromTachy;
}
void OdomInfo::set_time(unsigned int sec_message_from_tachy, unsigned int nsec_message_from_tachy){
  sec_last_message = sec_message_from_tachy;
  nsec_last_message = nsec_message_from_tachy;
}
double OdomInfo::get_X(){
  return x_coord;
}
double OdomInfo::get_Y(){
  return y_coord;
}
double OdomInfo::get_Z(){
  return z_coord;
}

double OdomInfo::get_X_ref(){
  return x_coord_ref;
}
double OdomInfo::get_Y_ref(){
  return y_coord_ref;
}
double OdomInfo::get_Z_ref(){
  return z_coord_ref;
}

unsigned int OdomInfo::get_nsec(){
  return nsec_last_message;
}
unsigned int OdomInfo::get_sec(){
  return sec_last_message;
}




int main(int argc, char **argv){

  OdomInfo odom_object;

  ros::init(argc, argv, "odom2point");

  ros::NodeHandle m;
  ros::Publisher point_pub = m.advertise<geometry_msgs::PointStamped>("gps_point_chatter", 1000);

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/odom", 1000, &OdomInfo::odomCallback, &odom_object);

  ros::Rate loop_rate(120);

  while(ros::ok()){
    ros::spinOnce();

     geometry_msgs::PointStamped gps_point;


    gps_point.header.frame_id = "gps_point";
    gps_point.header.stamp = ros::Time::now();

    gps_point.point.x = odom_object.get_X() - odom_object.get_X_ref();
    gps_point.point.y = odom_object.get_Y() - odom_object.get_Y_ref();
    gps_point.point.z = odom_object.get_Z() - odom_object.get_Z_ref();
    //std::cout << "Header" << std::endl;
    //std::cout << "X: " << gps_point.point.x << std::endl;
    //std::cout << gps_point.point.x;
    //std::cout << " " << gps_point.point.y;
    //std::cout << " " << gps_point.point.z << std::endl;
    point_pub.publish(gps_point);

    ros::spinOnce();
    loop_rate.sleep();
  }
return 0;
}
