#!/usr/bin/env python
# encoding: utf-8

"""
@author 
@desc 获取gazebo中小车真值,并在rviz中绘制
@date 20210510
说明：


"""
import rospy
from gazebo_msgs.srv import GetModelStateRequest, GetModelState
from geometry_msgs.msg import Pose, Point
from  nav_msgs.msg  import Odometry
import os, sys
from visualization_msgs.msg import Marker
import math
import copy


def point_dis(p1, p2):
    return math.sqrt(math.pow(p1.x - p2.x, 2) + math.pow(p1.y - p2.y, 2) + math.pow(p1.z - p2.z, 2) )



def main():
    rospy.init_node('draw_GT_pose', anonymous=True)
    rate = rospy.Rate(5)
    get_state_service = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    model = GetModelStateRequest()
    model.model_name = 'scout/'
    pub = rospy.Publisher('/gt_trajectory_vis', Marker, queue_size=10)
    vis_traj = Marker()

    vis_traj.header.frame_id = "world" # gazebo输出的坐标是全局world坐标系上的
    vis_traj.header.stamp = rospy.Time.now()

    vis_traj.type = vis_traj.SPHERE_LIST
    vis_traj.ns = "gt_traj"

    vis_traj.action = vis_traj.ADD
    vis_traj.id = 1
    vis_traj.scale.x = 0.15
    vis_traj.scale.y = 0.15
    vis_traj.scale.z = 0.15
    vis_traj.color.a = 0.5
    vis_traj.color.r = 0.0
    vis_traj.color.g = 1.0
    vis_traj.color.b = 0.0
    vis_traj.pose.orientation.x = 0.0
    vis_traj.pose.orientation.y = 0.0
    vis_traj.pose.orientation.z = 0.0
    vis_traj.pose.orientation.w = 1.0

    pt = Point()
    pt_last = Point()
    pt_last.x = 9999.0
    pt_last.y = 9999.0
    pt_last.z = 9999.0

    while not rospy.is_shutdown():
        objstate = get_state_service(model)
        pt.x = objstate.pose.position.x
        pt.y = objstate.pose.position.y
        pt.z = objstate.pose.position.z

        print("pt.x : %f, pt.y : %f, pt.z : %f, " % (pt.x, pt.y, pt.z))
        print("pt_last.x : %f, pt_last.y : %f, pt_last.z : %f, " % (pt_last.x, pt_last.y, pt_last.z))
        print(point_dis(pt, pt_last))
        print(point_dis(pt, pt_last) > 1.0)
        if point_dis(pt, pt_last) > 1.0:
            print("--------")
            vis_traj.points.append(pt)
            
            pub.publish(vis_traj)
            pt_last = copy.deepcopy(pt)
         
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass



