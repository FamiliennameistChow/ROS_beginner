/***************************************************************************************************************************
 * auto_landing.cpp
 *
 * Author: Danny
 *
 * Update Time: 2019.9.31
 *
 * 说明: 基于单目摄像头的自主降落程序
 *      1. 订阅目标相对位置(来自视觉的ros节点)
 *      2. 追踪算法及降落策略
***************************************************************************************************************************/
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandHome.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <signal.h>
#include <cmath>
#include <iostream>
#include <time.h>







int main(int argc, char **argv) {
}