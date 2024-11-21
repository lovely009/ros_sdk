#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Vector3.h"
#include <ins_pkg/ins622.h>
#include <cmath>   

ros::Publisher imu_pub;
ros::Publisher gps_pub;
ros::Publisher enu_vel_pub;

void Data2Pub_Callback(const ins_pkg::ins622& ins_msg);

/**************************************************************************
函数功能：主函数
入口参数：无
返回  值：无
**************************************************************************/
int main(int argc, char** argv)
{
    ros::init(argc, argv, "ins2ros");    //初始化ROS节点
    ROS_INFO("ins2ros::process: start");

    ros::NodeHandle node;    //创建句柄

    ros::Subscriber Insdata_sub = node.subscribe("/insData", 1, Data2Pub_Callback);
    enu_vel_pub = node.advertise<geometry_msgs::Vector3>("enu_vel", 10);
    imu_pub = node.advertise<sensor_msgs::Imu>("imu", 10);
    gps_pub = node.advertise<sensor_msgs::NavSatFix>("/gps/fix", 10);

  ros::spin();

  return 0;
}
/**************************************************************************
函数功能：数据转换
入口参数：/insData
返回  值：无
**************************************************************************/
void Data2Pub_Callback(const ins_pkg::ins622& ins_msg)	
{
  
  double roll = ins_msg.roll;
  double pitch = ins_msg.pitch;
  double yaw = ins_msg.yaw;

  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);

  sensor_msgs::Imu imuMsg;
  imuMsg.header.stamp = ros::Time(ins_msg.header.stamp.sec, ins_msg.header.stamp.nsec);
  imuMsg.header.frame_id = "gyro_link";

  imuMsg.angular_velocity.x = ins_msg.gyro_X;
  imuMsg.angular_velocity.y = ins_msg.gyro_Y;
  imuMsg.angular_velocity.z = ins_msg.gyro_Z;
  imuMsg.linear_acceleration.x = ins_msg.accel_X;
  imuMsg.linear_acceleration.y = ins_msg.accel_Y;
  imuMsg.linear_acceleration.z = ins_msg.accel_Z;

  imuMsg.orientation.w = cr * cp * cy + sr * sp * sy;
  imuMsg.orientation.x = sr * cp * cy - cr * sp * sy;
  imuMsg.orientation.y = cr * sp * cy + sr * cp * sy;
  imuMsg.orientation.z = cr * cp * sy - sr * sp * cy;

  imu_pub.publish(imuMsg);


  sensor_msgs::NavSatFix gps_data;
  gps_data.header.stamp = ros::Time(ins_msg.header.stamp.sec, ins_msg.header.stamp.nsec);
  gps_data.header.frame_id = "navsat_link";
  gps_data.latitude = ins_msg.latitude;
  gps_data.longitude = ins_msg.longitude;
  gps_data.altitude = ins_msg.altitude;
  gps_pub.publish(gps_data);
 
  geometry_msgs::Vector3 vel_msg;
  vel_msg.x = ins_msg.east_Vel;
  vel_msg.y = ins_msg.north_Vel;
  vel_msg.z = ins_msg.celestial_Vel;
  enu_vel_pub.publish(vel_msg);
}




