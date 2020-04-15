#!/usr/bin/env python
# encoding: utf-8

from gazebo_msgs.msg import ModelState
import rospy
import math

def pose_publish():
    # 改变模型pose
    pose_pub = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=10)
    pose_msg = ModelState()
    pose_msg.model_name = 'mark_label_1'
    rate = rospy.Rate(30)
    angle = 0
    radius = 2
    vel = 0.3
    cout = 0
    while cout < 10:
        # 直线轨迹
        print("===")
        pose_msg.pose.position.x = 5
        pose_msg.pose.position.y =-5

        # # 圆形轨迹
        # pose_msg.pose.position.x = radius * math.cos(angle * math.pi / 180)
        # pose_msg.pose.position.y = radius * math.sin(angle * math.pi / 180)
        # angle = angle + (vel / radius) * 180 / (math.pi * 30)
        # if(angle == 360):
        #     angle = 0

        pose_msg.pose.position.z = 0
        pose_pub.publish(pose_msg)
        cout += 1
        rate.sleep()

def main():
    rospy.init_node('model_change', anonymous=True)
    try:
        pose_publish()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()