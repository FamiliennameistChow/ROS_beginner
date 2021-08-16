#!/usr/bin/env python
# encoding: utf-8

import rospy
import pcl
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import math
import copy

p_last = list()
p_last[0] = 0.0
p_last[1] = 0.0
p_last[2] = 0.0
leng = 0.0 

def on_new_point_cloud(data):
    pc = pc2.read_points(data, skip_nans=True, field_names=("x", "y", "z"))
    for p in pc:
        leng += math.pow(math.pow(p[0] - p_last[0] ,2) + math.pow(p[1] - p_last[1] ,2) , 0.5)
        p_last = copy.deepcopy(p)

    print("leng: " , leng)
    # p = pcl.PointCloud()
    # p.from_list(pc_list)
    # seg = p.make_segmenter()
    # seg.set_model_type(pcl.SACMODEL_PLANE)
    # seg.set_method_type(pcl.SAC_RANSAC)
    # indices, model = seg.segment()

rospy.init_node('listener', anonymous=True)
rospy.Subscriber("/key_pose_origin", PointCloud2, on_new_point_cloud)
rospy.spin()