/*
* @file  : drone_lora_socket.cpp
* @brief : drone send/receive serial data through lora module and
           send/receive TCP/IP data by socket
* @date  : 2019.09.19
* @author: Danny
*
*/

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <iostream>
#include <mavros_msgs/GlobalPositionTarget.h> // 获取GPS信息
#include <netinet/in.h>
#include <ros/ros.h>
#include <serial/serial.h> //需要安装ROS官方的serial包,并且在Cmakelist文件中的find_package内加入serial
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/shm.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>
#include <vector>

using namespace std;

//要连接的服务器的的IP地址和端口号
string addr = "192.168.1.106";
int port = 8080;

//设置要使用的串口设备号和波特率
string serial_device = "/dev/ttyUSB0";
int baudrate = 9600;

// socket初始化，需要提供IP和端口号
//参数列表这么写是为了满足C++11标准与其他标准库的兼容性，C++11的string类型和以前不一样了。
int socket_init(int &sock_cli, const char *IP, const int port) {
  sock_cli = socket(AF_INET, SOCK_STREAM, 0);
  struct sockaddr_in servaddr;
  memset(&servaddr, 0, sizeof(servaddr));
  servaddr.sin_family = AF_INET;
  servaddr.sin_port = htons(port);
  servaddr.sin_addr.s_addr = inet_addr(IP);

  //连接服务器，成功返回0，错误返回-1
  while (connect(sock_cli, (struct sockaddr *)&servaddr, sizeof(servaddr)) <
         0) {
    ROS_INFO("Socket(%s : %d) connect fail, Wait to retry...", IP, port);
    sleep(1);
  }
  ROS_INFO("Socket(%s : %d) connect successful!", IP, port);
}

int serial_init(serial::Serial &ser, const string serial_device,
                const int baudrate = 9600) {
  serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
  ser.setPort(serial_device);
  ser.setBaudrate(baudrate);
  ser.setStopbits(serial::stopbits_one);
  ser.setParity(serial::parity_none);
  ser.setTimeout(timeout);

  while (!ser.isOpen()) {
    try {
      ser.open();
    } catch (serial::IOException &e) {
      ROS_INFO("open %s fail, wait to retry...", serial_device.c_str());
      sleep(1);
    }
  }
  ROS_INFO("open %s successful!", serial_device.c_str());
}

int main(int argc, char *argv[]) {
  uint8_t u_sendbuf[1024];
  uint8_t u_readbuf[1024];
  char c_sendbuf[1024];
  ros::init(argc, argv, "drone_lora_socket");
  ros::NodeHandle nh;

  //串口使用流程
  serial::Serial ser;
  serial_init(ser, serial_device, baudrate);

  while (cin >> u_sendbuf) {
    ser.write(u_sendbuf, sizeof(u_sendbuf));
    //接受或者发送完毕后把数组中的数据全部清空（置0）
    memset(u_sendbuf, 0, sizeof(u_sendbuf));
  }

  if (ser.available()) {
    ser.read(u_readbuf, ser.available());
  }

  /*
    //socket使用流程
    int sock_cli;
    //如果要将string转换为char*，可以使用string提供的函数c_str()
    //，或是函数data()，data除了返回字符串内容外，不附加结束符'\0'，而c_str()返回一个以‘\0’结尾的字符数组。
    socket_init(sock_cli, (char *)addr.c_str(), port);
    // sock_cli = socket_init((char *)addr.data(), port);

    while (fgets(c_sendbuf, sizeof(c_sendbuf), stdin) != NULL) {
      send(sock_cli, c_sendbuf, sizeof(c_sendbuf), 0);
      //接受或者发送完毕后把数组中的数据全部清空（置0）
      memset(c_sendbuf, 0, sizeof(c_sendbuf));
    }
  */
}