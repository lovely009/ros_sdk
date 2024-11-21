/*免责声明：本代码仅做调试参考，谨慎使用*/
/*小端:先读到的字节为低位字节*/
#include "ros/ros.h"
#include <string>
#include <vector>
#include <sstream>
#include <cmath>
#include <cstdlib> 
#include <unistd.h>
#include "serial.h"
#include <iostream>
#include <ins_pkg/ins622.h>
//关于串口的宏定义
//#define port       "/dev/ttyUSB0"
//#define Baudrate   921600
using namespace std;

string port;
int Baudrate;

typedef  struct poll_data_F
{
    uint16_t	data1;
    uint16_t 	data2;
    uint16_t	data3;
} POLL_DATA_F;


#pragma pack(push, 1)
/*
   该指令的目的是告诉编译器按照 1 字节对齐，
   这样可以确保结构体不会被填充额外的字节，
   从而强制使得 DATA_STREAM 保留原有的字节长度，即 65个字节。
*/
typedef struct
{
   //帧头:index 0~2
   uint8_t header[3] ;
   //欧拉角
   int16_t roll;//横滚角:index 3~4
   int16_t pitch;//俯仰角:index 5~6
   int16_t yaw;//航向角:index 7~8
   //陀螺仪
   int16_t gyro_X;//陀螺x轴: index 9~10
   int16_t gyro_Y;//陀螺y轴: index 11~12
   int32_t gyro_Z;//陀螺z轴: index 13~16
   //加速度
   int16_t accel_X;//x轴加速度: index 17~18
   int16_t accel_Y;//y轴加速度: index 19~20
   int16_t accel_Z;//z轴加速度: index 21~22
   //纬度、经度、海拔
   int32_t latitude;//纬度: index 23~26
   int32_t longitude;//经度: index 27~30
   int32_t altitude;//海拔: index 31~34
   //东北天方向速度 
   int16_t east_Vel;//东向速度: index 35~36
   int16_t north_Vel;//北向速度: index 37~38
   int16_t celestial_Vel;//天向速度: index 39~40
   //状态【bit0:位置 bit1:速度 bit2:姿态 bit3:航向角】
   int8_t status;//状态: index 41
   //保留位
   uint8_t reserved[6];//保留位: index 42~47
   //轮询数据
   POLL_DATA_F poll_Frame;//轮询数据: index 48~53
   //GPS时间
   uint32_t gps_Time;//GPS时间: index 54~57
   //轮询数据类型
   uint8_t type;//轮询数据类型: index 58
   //异或校验index为:0~58 的字节
   uint8_t xor_Verify_058;//异或校验: index 59
   //GPS周
   uint32_t gps_Week;//GPS周: index 60~63
   //异或校验index为:0~63 的字节
   uint8_t xor_Verify_063;//异或校验: index 64

}DATA_STREAM;
#pragma pack(pop)

typedef union
{

   DATA_STREAM data;
   uint8_t buf[sizeof(DATA_STREAM)];
}FRAME;

void dataProcess(serial::Serial & ser, ros::Publisher & ins_pub)
{
   while(ser.isOpen())
   {  
      
      //查找帧头
      uint8_t flag[1];
      ser.read(flag,1);
      if (flag[0] != 0xbd) continue;
      //printf("debug找到帧头 = %x\n",flag[0]);
           
      FRAME oneFrame;
      oneFrame.buf[0] = flag[0];
      ser.read(&oneFrame.buf[1],64);
      //1、异或校验字节序号为0~58的数据
      uint8_t xorResult = 0;
      for(int i = 0; i < 59; i++)
      {
         xorResult ^= oneFrame.buf[i];
      }
      //printf("本人校验 = %x\n",xorResult);
     // printf("正确值   = %x\n",oneFrame.data.xor_Verify_058);
      
      
      if(xorResult != oneFrame.data.xor_Verify_058)
      {
         std::cout<<"丢弃本帧"<<std::endl;
         continue;
      }
      //4、解析欧拉角（单位:deg）
      double roll  = (double)oneFrame.data.roll*360.0/32768.0;
      double pitch = (double)oneFrame.data.pitch*360.0/32768.0;
      double yaw   = (double)oneFrame.data.yaw*360.0/32768.0;
      //5、解析陀螺仪参数（单位:deg）
      double gyro_X = (double)oneFrame.data.gyro_X*360.0/32768.0;
      double gyro_Y = (double)oneFrame.data.gyro_Y*360.0/32768.0;
      double gyro_Z = (double)oneFrame.data.gyro_Z*600.0/2147483648.0;
      //6、加速度(单位:g)
      double accel_X = (double)oneFrame.data.accel_X*12.0/32768.0;
      double accel_Y = (double)oneFrame.data.accel_Y*12.0/32768.0;
      double accel_Z = (double)oneFrame.data.accel_Z*12.0/32768.0;
      //7、纬度(单位:deg)、经度(单位:deg)、海拔(单位:米)
      /*
      printf("byte23 = %02x\n",oneFrame.buf[23]);
      printf("byte24 = %02x\n",oneFrame.buf[24]);
      printf("byte25 = %02x\n",oneFrame.buf[25]);
      printf("byte26 = %02x\n",oneFrame.buf[26]);
      */
      
      double latitude  = (double)oneFrame.data.latitude*1.00e-07;
      double longitude = (double)oneFrame.data.longitude*1.00e-07;
      double altitude  = (double)oneFrame.data.altitude*1.00e-03;
      //8、东北天方向速度（m/s）
      double east_Vel      = (double)oneFrame.data.east_Vel*100.0/32768.0;
      double north_Vel     = (double)oneFrame.data.north_Vel*100.0/32768.0;;
      double celestial_Vel = (double)oneFrame.data.celestial_Vel*100.0/32768.0;
      //9、状态
      uint8_t status = oneFrame.data.status;
      //10、保留位(本例子不再做处理)
      //11、轮询数据
      POLL_DATA_F poll_Frame_;
      poll_Frame_.data1 = oneFrame.data.poll_Frame.data1;
      poll_Frame_.data2 = oneFrame.data.poll_Frame.data2;
      poll_Frame_.data3 = oneFrame.data.poll_Frame.data3;
      //12、GPS时间(单位:ms)
      double gps_Time = (double)oneFrame.data.gps_Time*0.25;
      //13、轮询数据类型（本例子不再做处理）
      //14、异或校验index为:0~58 的字节（本例子不再做处理）
      //15、GPS周（本例子不再做处理）
      //16、异或校验index为:0~63 的字节（本例子不再做处理）
      //17、发布ros消息
      ins_pkg::ins622 insMsg;
      insMsg.header.seq = 1;//设置该消息在当前话题中的序号
      insMsg.header.frame_id = "ins_frame";//设置ins惯导的坐标系名称
      insMsg.header.stamp = ros::Time::now();//将时间戳设置为ros时间
      //欧拉角
      insMsg.roll  = roll;
      insMsg.pitch = pitch;
      insMsg.yaw   = yaw;
      //陀螺仪参数
      insMsg.gyro_X = gyro_X;
      insMsg.gyro_Y = gyro_Y;
      insMsg.gyro_Z = gyro_Z;
      //加速度
      insMsg.accel_X = accel_X;
      insMsg.accel_Y = accel_Y;
      insMsg.accel_Z = accel_Z;
      //纬度(单位:deg)、经度(单位:deg)、海拔(单位:米)
      insMsg.latitude  = latitude;
      insMsg.longitude = longitude;
      insMsg.altitude  = altitude;
      //东北天方向速度
      insMsg.east_Vel = east_Vel;
      insMsg.north_Vel = north_Vel;
      insMsg.celestial_Vel = celestial_Vel;
      //轮询数据此处将不纳入ros消息，如有需要，请自行在msg文件夹中的ins622.msg文件中自行添加
      //GPS时间(单位:ms)
      insMsg.gps_Time = gps_Time; 
      ins_pub.publish(insMsg);
   }
  return;
}

int main(int argc, char** argv)
{
   ros::init(argc, argv, "insdata_deal");
   ros::NodeHandle private_nh("~");

   private_nh.param<int>("Baudrate", Baudrate, 921600);
   private_nh.param<std::string>("port", port, "/dev/ttyUSB0");

   serial::Serial ser; 
   //配置串口参数：1、波特率 2、串口号
   try
   {
   //串口设置
   ser.setPort(port);//设置串口号
   ser.setBaudrate(Baudrate);//设置串口波特率
   serial::Timeout to = serial::Timeout::simpleTimeout(1000);
   ser.setTimeout(to);
   ser.open();
   }
   catch (serial::IOException& e)
   {
     ROS_ERROR_STREAM("Unable to open port ");
     return -1;
   }
   if (ser.isOpen())
   {
      std::cout<<" ------ Serial port open successfully ------ "<<std::endl;
      std::cout<<" ------ Serial port = "<< port <<std::endl;
      std::cout<<" ------ Serial Baudrate = "<< Baudrate <<std::endl;
   }
   else//打开失败退出主函数
   {
      std::cout<<" ------ Serial port open fail: main will exit------ "<<std::endl;
      return -1;
   }

   ros::init(argc, argv, "insPub_node");
   ros::NodeHandle n;
   ros::Publisher  ins_pub = n.advertise<ins_pkg::ins622>("insData",20);
   dataProcess(ser, ins_pub);
   ros::spin();
}


//typedef  struct
//{
//0    uint8_t 			header[3];	//0xbd,0xdb,0x0b
//3    short 				roll;		//横滚角
//5    short 				pitch;		//俯仰角
//7    short				azimuth;	//方位角
//9    short 				gyroX;		//陀螺x轴
//11    short 				gyroY;		//陀螺y轴
//13		long				gyroZ;		//陀螺z轴
//17    short 				accelX;		//加表x轴
//19    short 				accelY;		//加表y轴
//21    short				accelZ;		//加表z轴
//23    long				latitude;	//纬度
//27    long				longitude;	//经度
//31    long				altitude;	//高度
//35    short				ve;			//东向速度
//37    short				vn;			//北向速度
//39    short				vu;			//天向速度
//41    uint8_t				status;		//bit0:位置 bit1:速度 bit2:姿态 bit3:航向角
//42    uint8_t				reserved[6];
//48    POLL_DATA_F			poll_frame;
//54    uint32_t			gps_time;
//58    uint8_t				type;
//59    uint8_t				xor_verify1;
//60    uint32_t			gps_week;
//64    uint8_t				xor_verify2;
//} DATA_STREAM;


