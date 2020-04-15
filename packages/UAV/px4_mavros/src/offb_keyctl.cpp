/*
* @file  : offboard_control.cpp
* @brief : offboard control
*          use keyboard to control position and yaw
* @date  : 2019.01.09
* @author: kumaron, Danny
*
*/
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <termio.h>

// for multithread
#include "pose_pos_control.h"
#include "pose_yaw_control.h"
#include <boost/thread.hpp>
#include <cmath>
#include <iostream>
#include <pthread.h>
#include <time.h>

using namespace std;

char key = 0;
static bool g_offboard_land_flag = false; // false: not land; true: land
pthread_mutex_t mutex; //互斥量，用于线程锁保护数据

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg) {
  current_state = *msg;
} //回调函数，储存无人机状态

//监听键盘，键入则立即读取，不需要按回车
int getch() {
  int input;
  struct termios new_settings;
  struct termios stored_settings;
  tcgetattr(0, &stored_settings);
  new_settings = stored_settings;
  new_settings.c_lflag &= (~ICANON);
  new_settings.c_cc[VTIME] = 0;
  tcgetattr(0, &stored_settings);
  new_settings.c_cc[VMIN] = 1;
  tcsetattr(0, TCSANOW, &new_settings);

  input = getchar();

  tcsetattr(0, TCSANOW, &stored_settings);
  return input;
}

// ctrl+c关闭进程。前提条件为无人机降落并已上锁
void shutdown_node(int sig) {
  if (!current_state.armed && g_offboard_land_flag) {
    ROS_INFO("Exit successuful!");
    ros::shutdown();
    exit(0);
  } else {
    ROS_INFO("It's dangerous to exit! Please wait drone to land and disarm");
  }
}

// key赋值线程
void *inputthread(void *arg) {
  int i;
  while (true) {
    pthread_mutex_lock(&mutex);
    key = getch();
    pthread_mutex_unlock(&mutex);
    pthread_yield(); //线程放弃对处理器的使用，并在再次调度之前在运行队列中等待
                     //它可以将执行切换到另一个线程。但是，如果没有更高优先级的线程需要在那时工作，线程可能会立即重新开始接着执行。
    usleep(100000);
    //无论什么情况，线程都会保证停止运行直到时间足够。此时一个键有效期为0.1s
  }
}

//
// Main
//
int main(int argc, char **argv) {

  // key赋值线程初始化
  int ret = 0;
  pthread_t id1;
  pthread_mutex_init(&mutex, NULL); //初始化互斥量
  ret = pthread_create(&id1, NULL, inputthread, NULL);
  if (ret) {
    ROS_INFO("create pthread error!\n");
    return -1;
  }

  //
  // Initiating position and orientation control
  // 初始化位置和方向控制

  // Initiating position control
  PositionData *set_position_data = new PositionData();
  // positionValueAssign(set_position_data, 0, 0, 2);             //修改过
  *set_position_data = {0, 0, 2};
  PositionControl position_control(set_position_data);

  // Initiating yaw control
  Quaternion *set_quater_data = new Quaternion(0, 0, 0, 1, 0);

  Quaternion *set_yaw_left = new Quaternion(0, 0, 0, 1, 0);
  Quaternion *set_yaw_right = new Quaternion(0, 0, 0, 1, 0);

  // 0
   YawControl yaw_control(set_yaw_left, set_yaw_right);            


  // theta = [0,360)
  int theta = 0;
  // change theta step
  const int theta_step = 1;
  for (theta = theta_step; theta < 360; theta += theta_step) {
    float z = sin(theta * M_PI / 360);
    float w = cos(theta * M_PI / 360);
    // quaterDataAssign(set_yaw_left, 0, 0, z, w, theta);             //修改过
    // quaterDataAssign(set_yaw_right, 0, 0, -z, -w, theta); //修改过
    *set_yaw_left = {0, 0, z, w, theta};    //修改过
    *set_yaw_right = {0, 0, -z, -w, theta}; //修改过
    yaw_control.insertAsLast(set_yaw_left, set_yaw_right); 

    // usleep(1000);
  }
  usleep(1000);

  //
  // Initiating mavros
  //

  ros::init(argc, argv, "offb_keyctl"); //初始化节点
  ros::NodeHandle nh;
  ros::Subscriber state_sub =
      nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(
      "mavros/setpoint_position/local", 10);
  ros::ServiceClient arming_client =
      nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  ros::ServiceClient land_client =
      nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
  ros::ServiceClient set_mode_client =
      nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

  // the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);
  // wait for FCU connection
  while (ros::ok() && current_state.connected) {
    ros::spinOnce();
    rate.sleep();
  }

  //
  // Offboard setpoint_position/local control
  //

  // Initiating position control data
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = set_position_data->x;
  pose.pose.position.y = set_position_data->y;
  pose.pose.position.z = set_position_data->z;

  // send a few setpoints before starting
  for (int i = 100; ros::ok() && i > 0; --i) {
    local_pos_pub.publish(pose);
    ros::spinOnce();
    rate.sleep();
  }

  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;
  ros::Time last_request = ros::Time::now();

  mavros_msgs::CommandTOL land_cmd;
  

  while (ros::ok()) {
    if (current_state.mode != "OFFBOARD" &&
        (ros::Time::now() - last_request > ros::Duration(3.0)) &&
        !g_offboard_land_flag) {
      if (set_mode_client.call(offb_set_mode) &&
          offb_set_mode.response.mode_sent) {
        ROS_INFO("Offboard enabled");
      }
      last_request = ros::Time::now();
    } else {
      if (!current_state.armed &&
          (ros::Time::now() - last_request > ros::Duration(3.0)) &&
          !g_offboard_land_flag) {
        if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
          ROS_INFO("Vehicle armed");
        }
        last_request = ros::Time::now();
      }
    }

    // get yaw theta
    float control_theta = yaw_control.getHandleTheta();

    switch (key) {
    // forward
    case 'w':
      set_position_data =
          position_control.positionControl(M_X_AXIS, M_FORWARD, control_theta);
      key = 0;
      break;

    // backward
    case 's':
      set_position_data =
          position_control.positionControl(M_X_AXIS, M_BACKWARD, control_theta);
      key = 0;
      break;

    // left
    case 'a':
      set_position_data =
          position_control.positionControl(M_Y_AXIS, M_LEFT, control_theta);
      key = 0;
      break;

    // right
    case 'd':
      set_position_data =
          position_control.positionControl(M_Y_AXIS, M_RIGHT, control_theta);
      key = 0;
      break;

    // 调整高度及航角
    case 0x1B:
      pthread_mutex_lock(&mutex);
      if (key = getch() == 0x5B) {
        switch (key = getch()) {
        case 0x41: // ↑键升高
          set_position_data =
              position_control.positionControl(M_Z_AXIS, M_UP, control_theta);
          key = 0;
          break;

        case 0x42: // ↓键降低
          set_position_data =
              position_control.positionControl(M_Z_AXIS, M_DOWN, control_theta);
          key = 0;
          break;

        case 0x44: // ←键逆时针旋转
          set_quater_data = yaw_control.yawControl(M_ANTICLOCKWISE);
          set_quater_data->showData();
          key = 0;
          break;

        case 0x43: // →键顺时针旋转
          set_quater_data = yaw_control.yawControl(M_CLOCKWISE);
          set_quater_data->showData();
          key = 0;
          break;
        }
      }
      pthread_mutex_unlock(&mutex);
      break;

    // land
    case ' ': // 空格键降落
      if (current_state.armed && !g_offboard_land_flag) {
        if (land_client.call(land_cmd) && land_cmd.response.success) {
          ROS_INFO("Landing");
          g_offboard_land_flag = true;
        }
      }
      break;

    // takeoff
    case 't': //按下t键后再按回车确认起飞
      pthread_mutex_lock(&mutex);
      if (key = getchar() == 0x0A) {
        if (!current_state.armed && g_offboard_land_flag)
          g_offboard_land_flag = false;
      }
      pthread_mutex_unlock(&mutex);
      break;

    default:
      break;
    }

    // 按下ctrl+c时候产生SIGINT信号，捕捉它并回调到信号处理函数
    if (SIG_ERR == signal(SIGINT, shutdown_node)) {
      ROS_INFO("install signal handler failed\n");
      return -1;
    }
    usleep(100);
    //
    // Publish pose data to realize control
    //
    pose.pose.position.x = set_position_data->x;
    pose.pose.position.y = set_position_data->y;
    pose.pose.position.z = set_position_data->z;
    pose.pose.orientation.x = set_quater_data->x;
    pose.pose.orientation.y = set_quater_data->y;
    pose.pose.orientation.z = set_quater_data->z;
    pose.pose.orientation.w = set_quater_data->w;

    local_pos_pub.publish(pose);
    ros::spinOnce();
    rate.sleep();

    // pthread_yield();
    // //线程放弃对处理器的使用，并在再次调度之前在运行队列中等待
    //它可以将执行切换到另一个线程。但是，如果没有更高优先级的线程需要在那时工作，线程可能会立即重新开始。
  }

  // pthread_join(id1, NULL);
  //调用该函数的线程将挂起等待,直到id为thread的线程终止,会阻塞调用它的线程（这里是主线程）
  return 0;
}
