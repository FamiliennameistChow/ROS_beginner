#!/usr/bin/env python
# encoding: utf-8

"""
@author 
@desc 获取gazebo中小车真值构建
@date 20210507
说明：

"""
import rospy
from gazebo_msgs.srv import GetModelStateRequest, GetModelState
from geometry_msgs.msg import Pose
from  nav_msgs.msg  import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import os, sys

class MapBuilt:
    def __init__(self):
        rospy.init_node('change_pose', anonymous=True)

        rospy.Subscriber("/velodyne_points", PointCloud2, self.callback_pointcloud)

        get_state_service = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        model = GetModelStateRequest()
        model.model_name = 'scout/'


    def callback_pointcloud(self, msg):
        
