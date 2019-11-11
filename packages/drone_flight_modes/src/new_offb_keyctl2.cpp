/*
* @file  : new_offb_keyctl.cpp
* @brief : offboard control
*          use keyboard to control position and yaw
* @date  : 2019.11.2
* @author: Danny
*
*/
#include <drone_flight_modes.hpp>

#include <ros/ros.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <termio.h>

// for multithread
#include <boost/thread.hpp>
#include <iostream>
#include <pthread.h>
#include <time.h>

using namespace std;
char key = 0;
static bool land_flag = true;      // false: not land; true: land
static bool set_home_flag = false; //每次起飞，都将起飞点设为home点
pthread_mutex_t mutex;             //互斥量，用于线程锁保护数据

// theta_step为偏航角变化步长，go_step为各方向变化步长
const int theta_step = 10;
const float go_step = 1;

mavros_msgs::State current_state;

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
  if (!current_state.armed && land_flag) {
    ROS_INFO("Exit successuful!");
    ros::shutdown();
    exit(0);
  } else {
    ROS_INFO("It's dangerous to exit! Please wait drone to land and disarm");
  }
}

// 监听键盘输入子线程，给变量key赋值
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

int main(int argc, char **argv) {
  ros::init(argc, argv, "new_offb_keyctl2");
  ros::NodeHandle nh;
  AeroDrone myDrone;

  // key赋值线程初始化
  int ret = 0;
  pthread_t id1;
  pthread_mutex_init(&mutex, NULL); //初始化互斥量
  ret = pthread_create(&id1, NULL, inputthread, NULL);
  if (ret) {
    ROS_INFO("create pthread error!\n");
    return -1;
  }

  // the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);

 
  // wait for FCU connection
  while (ros::ok() && !myDrone.currentState().connected) {
    ros::spinOnce();
    rate.sleep();
  }

  ros::Time last_request = ros::Time::now();

  while (ros::ok()) {
    if (myDrone.currentState().mode != "OFFBOARD" &&
        (ros::Time::now() - last_request > ros::Duration(5.0)) && !land_flag) {
      if (myDrone.setMode("OFFBOARD")) {
        ROS_INFO("Offboard Mode enabled");
      }
      last_request = ros::Time::now();
    }

    switch (key) {
    // forward
    case 'i': // x的正方向
      myDrone.moveENU(go_step, 0, 0);
      key = 0;
      break;

    // backward
    case 'k': // x的负方向
      myDrone.moveENU(-(go_step), 0, 0);
      key = 0;
      break;

    // left
    case 'j': // y的正方向
      myDrone.moveENU(0, go_step, 0);
      key = 0;
      break;

    // right
    case 'l': // y的负方向
      myDrone.moveENU(0, -(go_step), 0);
      key = 0;
      break;

    // forward
    case 'w': //机头的正方向
      myDrone.moveBody(go_step, 0, 0);
      key = 0;
      break;

    // backward
    case 's': //机头的负方向
      myDrone.moveBody(-(go_step), 0, 0);
      key = 0;
      break;

    // left
    case 'a': //机体的左侧
      myDrone.moveBody(0, go_step, 0);
      key = 0;
      break;

    // right
    case 'd': //机体的右侧
      myDrone.moveBody(0, -(go_step), 0);
      key = 0;
      break;

    // 调整高度及航角
    case 0x1B:
      pthread_mutex_lock(&mutex);
      if (key = getch() == 0x5B) {
        switch (key = getch()) {
        case 0x41: // ↑键升高
          myDrone.moveBody(0, 0, go_step);
          key = 0;
          break;

        case 0x42: // ↓键降低
          myDrone.moveBody(0, 0, -(go_step));
          key = 0;
          break;

        case 0x44: // ←键逆时针旋转
          myDrone.rotateAngle(theta_step);
          key = 0;
          break;

        case 0x43: // →键顺时针旋转
          myDrone.rotateAngle(-(theta_step));
          key = 0;
          break;
        }
      }
      pthread_mutex_unlock(&mutex);
      break;

    // land
    case ' ': // 空格键降落
      if (myDrone.currentState().armed && !land_flag) {
        if (myDrone.land()) {
          while (myDrone.currentState().armed) {
            sleep(1);
            ros::spinOnce();
            ROS_INFO("Landing...");
          }
          ROS_INFO("Vehicle disarmed");
          land_flag = true;
        }
      }
      break;

    // takeoff
    case 't': //按下t键后再按回车确认起飞
      pthread_mutex_lock(&mutex);
      if (key = getchar() == 0x0A) {
        if (!myDrone.currentState().armed && land_flag) {
          ROS_INFO("Ready to Takeoff");
          land_flag = false;
          if (myDrone.arm())
            ROS_INFO("Vehicle armed");
          myDrone.takeoff();
          sleep(5);
        }
      }
      pthread_mutex_unlock(&mutex);
      break;

    default:
      break;
    }

    // 按下ctrl+c时候产生SIGINT信号，捕捉它并回调到信号处理函数
    if (SIG_ERR == signal(SIGINT, shutdown_node)) {
      ROS_INFO("Install signal handler failed\n");
      return -1;
    }
    
    ros::spinOnce();
    rate.sleep();
    current_state = myDrone.currentState();
  }

  return 0;
}
