/*
* @file  : new_offb_keyctl.cpp
* @brief : offboard control
*          use keyboard to control position and yaw
* @date  : 2019.09.5
* @author: Danny
*
*/

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandHome.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <termio.h>

// for multithread
#include <boost/thread.hpp>
#include <cmath>
#include <iostream>
#include <pthread.h>
#include <time.h>

using namespace std;
char key = 0;
static bool land_flag = true;      // false: not land; true: land
static bool set_home_flag = false; //每次起飞，都将起飞点设为home点
pthread_mutex_t mutex;             //互斥量，用于线程锁保护数据

// theta为偏航角，theta_step为偏航角变化步长，go_step为各方向变化步长
int theta = 0;
const int theta_step = 10;
const float go_step = 0.2;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg) { current_state = *msg; }

geometry_msgs::PoseStamped local_position;
void local_position_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  local_position = *msg;
}

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
  ros::init(argc, argv, "new_offb_keyctl");
  ros::NodeHandle nh;

  // key赋值线程初始化
  int ret = 0;
  pthread_t id1;
  pthread_mutex_init(&mutex, NULL); //初始化互斥量
  ret = pthread_create(&id1, NULL, inputthread, NULL);
  if (ret) {
    ROS_INFO("create pthread error!\n");
    return -1;
  }

  ros::Subscriber state_sub =
      nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
  ros::Subscriber local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>(
      "mavros/local_position/pose", 10, local_position_cb);
  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(
      "mavros/setpoint_position/local", 10);
  // ros::ServiceClient set_home_client =
  //     nh.serviceClient<mavros_msgs::CommandHome>("mavros/cmd/set_home");
  ros::ServiceClient arming_client =
      nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  ros::ServiceClient land_client =
      nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
  // ros::ServiceClient takeoff_client =
  //     nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");
  ros::ServiceClient set_mode_client =
      nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

  // the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);

  mavros_msgs::CommandTOL land_cmd;
  // mavros_msgs::CommandTOL takeoff_cmd;

  // mavros_msgs::CommandHome set_home_cmd;
  // set_home_cmd.request.current_gps = true;

  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  mavros_msgs::CommandBool disarm_cmd;
  disarm_cmd.request.value = false;

  // wait for FCU connection
  while (ros::ok() && !current_state.connected) {
    ros::spinOnce();
    rate.sleep();
  }

  geometry_msgs::PoseStamped setpoint;
  setpoint.pose.position.x = 0;
  setpoint.pose.position.y = 0;
  setpoint.pose.position.z = 2.5; //自动起飞的默认高度

  // send a few setpoints before starting
  for (int i = 100; ros::ok() && i > 0; --i) {
    local_pos_pub.publish(setpoint);
    ros::spinOnce();
    rate.sleep();
  }

  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  ros::Time last_request = ros::Time::now();

  while (ros::ok()) {
    // // 无人机起飞时会自动设置home点
    // if (set_home_flag == false &&
    //     (ros::Time::now() - last_request > ros::Duration(5.0)) && !land_flag)
    //     {
    //   if (set_home_client.call(set_home_cmd) &&
    //   set_home_cmd.response.success) {
    //     ROS_INFO("Set home positin sucessful");
    //     set_home_flag = true;
    //   }
    //   last_request = ros::Time::now();
    // } else {
    if (current_state.mode != "OFFBOARD" &&
        (ros::Time::now() - last_request > ros::Duration(5.0)) && !land_flag) {
      if (set_mode_client.call(offb_set_mode) &&
          offb_set_mode.response.mode_sent) {
        ROS_INFO("Offboard Mode enabled");
      }
      last_request = ros::Time::now();
    } else {
      if (!land_flag && !current_state.armed &&
          (ros::Time::now() - last_request > ros::Duration(5.0)) &&
          !land_flag) {
        if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
          ROS_INFO("Vehicle armed");
        }
        last_request = ros::Time::now();
      }
    }
    // }

    switch (key) {
    // forward
    case 'i': // x的正方向
      setpoint.pose.position.x += go_step;
      ROS_INFO("X: %lf", setpoint.pose.position.x);
      key = 0;
      break;

    // backward
    case 'k': // x的负方向
      setpoint.pose.position.x -= go_step;
      ROS_INFO("X: %lf", setpoint.pose.position.x);
      key = 0;
      break;

    // left
    case 'j': // y的正方向
      setpoint.pose.position.y += go_step;
      ROS_INFO("Y: %lf", setpoint.pose.position.y);
      key = 0;
      break;

    // right
    case 'l': // y的负方向
      setpoint.pose.position.y -= go_step;
      ROS_INFO("Y: %lf", setpoint.pose.position.y);
      key = 0;
      break;

    // forward
    case 'w': //机头的正方向
      setpoint.pose.position.x += cos(theta * M_PI / 180) * go_step;
      setpoint.pose.position.y += sin(theta * M_PI / 180) * go_step;
      ROS_INFO("X: %lf", setpoint.pose.position.x);
      key = 0;
      break;

    // backward
    case 's': //机头的负方向
      setpoint.pose.position.x -= cos(theta * M_PI / 180) * go_step;
      setpoint.pose.position.y -= sin(theta * M_PI / 180) * go_step;
      ROS_INFO("X: %lf", setpoint.pose.position.x);
      key = 0;
      break;

    // left
    case 'a': //机体的左侧
      setpoint.pose.position.x -= sin(theta * M_PI / 180) * go_step;
      setpoint.pose.position.y += cos(theta * M_PI / 180) * go_step;
      ROS_INFO("Y: %lf", setpoint.pose.position.y);
      key = 0;
      break;

    // right
    case 'd': //机体的右侧
      setpoint.pose.position.x += sin(theta * M_PI / 180) * go_step;
      setpoint.pose.position.y -= cos(theta * M_PI / 180) * go_step;
      ROS_INFO("Y: %lf", setpoint.pose.position.y);
      key = 0;
      break;

    // 调整高度及航角
    case 0x1B:
      pthread_mutex_lock(&mutex);
      if (key = getch() == 0x5B) {
        switch (key = getch()) {
        case 0x41: // ↑键升高
          setpoint.pose.position.z += go_step;
          ROS_INFO("Z: %lf", setpoint.pose.position.z);
          key = 0;
          break;

        case 0x42: // ↓键降低
          setpoint.pose.position.z -= go_step;
          ROS_INFO("Z: %lf", setpoint.pose.position.z);
          key = 0;
          break;

        case 0x44: // ←键逆时针旋转
          theta += theta_step;
          ROS_INFO("Theta: %d", theta);
          setpoint.pose.orientation.w = cos(theta * M_PI / 360);
          setpoint.pose.orientation.z = sin(theta * M_PI / 360);
          key = 0;
          break;

        case 0x43: // →键顺时针旋转
          theta -= theta_step;
          ROS_INFO("Theta: %d", theta);
          setpoint.pose.orientation.w = cos(theta * M_PI / 360);
          setpoint.pose.orientation.z = sin(theta * M_PI / 360);
          key = 0;
          break;
        }
      }
      pthread_mutex_unlock(&mutex);
      break;

    // land
    case ' ': // 空格键降落
      if (current_state.armed && !land_flag) {
        if (land_client.call(land_cmd) && land_cmd.response.success) {
          while (current_state.armed) {
            sleep(1);
            ros::spinOnce();
            ROS_INFO("Landing...");
          }
          ROS_INFO("Vehicle disarmed");
          land_flag = true;
          // set_home_flag = false;
        }
      }
      break;

    // takeoff
    case 't': //按下t键后再按回车确认起飞
      pthread_mutex_lock(&mutex);
      if (key = getchar() == 0x0A) {
        if (!current_state.armed && land_flag) {
          // if (takeoff_client.call(takeoff_cmd) &&
          // takeoff_cmd.response.success) {
          ROS_INFO("Read to Takeoff");
          land_flag = false;

          setpoint.pose.position.x = local_position.pose.position.x;
          setpoint.pose.position.y = local_position.pose.position.y;
          setpoint.pose.position.z = local_position.pose.position.z + 2.5;
          // ROS_INFO("Read to Takeoff");
          // setpoint.pose.position.z = 2.5; //自动起飞的默认高度,可修改起飞高度

          // }
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
    usleep(100);
    local_pos_pub.publish(setpoint);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
